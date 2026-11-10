#!/usr/bin/env python3
# coding: UTF-8

from std_msgs.msg import String
from std_msgs.msg import Empty as EmptyMsg
from std_msgs.msg import Float64MultiArray
from collections import deque
import time
import rclpy
from rclpy.node import Node
from .CollaborationStateHeader import CollaborationCurrentData, CollaborationState
from .TransitionModule import Transition
from .CollaborationToolModule import CollaborationTool
from .CollaborationCommunicationModule import *
from geometry_msgs.msg import TransformStamped
from .MovePlannerModule import MovePlanner

#定数の定義
DEFAULT_VELOCITY = 1.0
RETRY_COUNT_MAX = 3
WORK_DETECT_COMMANDS = [1, 2]   #ワーク検出を行う作業指令ID
PREPAR_WAIT = 3
PAUSED_WAIT = 0.5
NON_OPERATING_WAIT = 1
COFFEE_WAIT_TIME = 37.0
HANDOVER_WAIT_TIME = 10.0 #受け渡しの待ち時間[s]. 時間内に引っ張り・合図がなければ離さずに元の位置へ戻す
HANDOVER_TOPIC = '/handover_ok' #受け渡し合図のトピック (std_msgs/msg/Empty)
#引っ張り検知（モータ電流）. COBOTTA を recv_format:=66 で起動したときに有効.
CURRENT_TOPIC = '/cobotta/Read_Current' #モータ電流のトピック (std_msgs/msg/Float64MultiArray)
PULL_DETECT_JOINTS = [3, 4, 5]  #引っ張り検知に使う関節番号
PULL_THRESHOLD = 20.0           #基準値からの変化がこの値以上で「引っ張られた」と判定
PULL_SETTLE_SEC = 0.5           #受け渡し位置に着いてから基準値を取り始めるまでの時間[s]
PULL_BASELINE_SEC = 0.5         #基準値を取る時間[s]
PULL_AVERAGE_SEC = 0.25         #判定に使う移動平均の時間[s]
HANDOVER_VELOCITY = 0.2   #受け渡し位置へ移動するときの速度・加速度スケール
IK_OFFSET = 0.03          #逆運動学動作の移動量[m]
IK_VELOCITY = 0.2         #逆運動学動作の速度・加速度スケール

#作業1, 2 の関節角度[deg] (J1, J2, J3, J4, J5, J6). ダイレクトティーチングで取得した値.
APPROACH_POSE    = (-141.41, 55.99, 137.76, 163.84, 90.91, 0.33)   #アプローチ（カップの手前）
ABOVE_PICK_POSE  = (-141.41, 55.45, 128.23, 163.84, 90.91, 0.33)   #ピック位置の真上（8cm上）
PICK_POSE        = (-141.38, 87.11, 105.96, 163.62, 99.93, -2.31)  #ピック位置（机の上のワーク）
LIFT_UP_POSE     = (-141.41, 51.83, 134.53, 163.84, 90.91, 0.33)   #持ち上げ
CARRY_POSE       = (76.76, 51.83, 134.53, 163.84, 90.91, 0.33)     #作業1 運ぶ（旋回）
ABOVE_PLACE_POSE = (85.68, 67.20, 93.86, 158.21, 71.82, 9.22)      #作業1 置く位置の上
PLACE_POSE       = (85.32, 91.101, 52.28, 160.68, 53.84, 11.70)    #作業1 置く
HANDOVER_POSE    = (76.76, 51.83, 134.53, 163.84, 90.91, 0.33)     #作業2 受け渡し位置

#作業1, 2 の速度・加速度スケール.
MOVE_VELOCITY    = 0.3    #通常の移動
CAREFUL_VELOCITY = 0.2    #把持位置への下降・把持中の持ち上げ・退避
CARRY_VELOCITY   = 0.5    #ワークを持って置き場へ運ぶ


###########################################
#MovePlanner共有クラス.                   #
###########################################
class ShareMovePlanner:
    move_plannner = None
    @classmethod
    def get_move_planner(cls):
        if cls.move_plannner is None:
            cls.move_plannner = MovePlanner()
        return cls.move_plannner
    
###########################################
#受け渡し合図受信クラス.                  #
###########################################
class HandoverSignal(Node):
    """人からの「受け取った」合図(HANDOVER_TOPIC)を受信する."""
    def __init__(self):
        super().__init__('handover_signal_node')
        self.received = False
        self.sub = self.create_subscription(EmptyMsg, HANDOVER_TOPIC, self.callback, 10)

    def callback(self, msg):
        self.received = True

    #待ち始める前に届いていた合図を捨てる.
    def clear(self):
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.0)
        self.received = False

    #合図を確認する(最大timeout_sec待つ).
    def check(self, timeout_sec):
        rclpy.spin_once(self, timeout_sec=timeout_sec)
        return self.received

###########################################
#モータ電流受信クラス（引っ張り検知用）.  #
###########################################
class HandoverCurrent(Node):
    """COBOTTA のモータ電流(CURRENT_TOPIC)を受信し, 基準値からの変化を求める."""
    def __init__(self):
        super().__init__('handover_current_node')
        self.samples = deque(maxlen=1000)  #(受信時刻, 関節1～6の電流)
        self.sub = self.create_subscription(Float64MultiArray, CURRENT_TOPIC, self.callback, 50)

    def callback(self, msg):
        self.samples.append((time.monotonic(), list(msg.data[:6])))

    #届いている電流データをすべて処理する.
    def update(self):
        for _ in range(200):
            before = len(self.samples)
            rclpy.spin_once(self, timeout_sec=0.0)
            if len(self.samples) == before:
                break

    def clear(self):
        self.update()
        self.samples.clear()

    #指定時刻以降の平均値(関節1～6). データがなければ None.
    def mean_since(self, since):
        data = [d for (t, d) in self.samples if t >= since]
        if 0 == len(data):
            return None
        return [sum(v[i] for v in data) / len(data) for i in range(6)]

###########################################
#初期処理クラス.                          #
###########################################
class INITIALISE(CollaborationState):
    def __init__(self,
                 label,
                 taskfin:TaskFinishServer,
                 tran:Transition,
                 outcomes):
        super().__init__(label, tran, outcomes)
        self.planner = ShareMovePlanner.get_move_planner()
        self.taskfin = taskfin
        self.counter = 0

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)
        
        if self.tran.get_event().is_workend():
            task_command_list = CollaborationCurrentData.GetCurrentCommandList()
            task_command = task_command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        
        if self.preempt('INIITIALISE is being preempted!!!'):
            return 'preempted'
        
        #初期処理を行う.
        return self.initialise()

    def initialise(self):
        ###ロボットごとの初期動作.
        #ここから.

        while not CollaborationTool.is_shutdown():
          #初期位置移動.
          ret = self.init_pose()
          if 'succeeded' == ret:
                break
          elif 'retry' == ret:
            continue
          else:
            return ret

        #ここまで.
        return 'succeeded'

    #初期位置移動.
    def init_pose(self):
        self.counter = 0

        #3回以上連続で失敗したら異常終了する.
        while not CollaborationTool.is_shutdown():
            self.counter += 1
            CollaborationTool.loginfo('Initialise_gripper')
            rc = self.planner.hand_open()
            ret =self.return_btos(rc)
            if 'succeeded' == ret:
                #正常完了の場合はエラーカウンターをリセット.
                self.counter = 0
                break
            else:
                if self.counter >= 3:
                    return 'aborted'
                else:
                    continue

        #3回以上連続で失敗したら異常終了する.
        while not CollaborationTool.is_shutdown():
            self.counter += 1
            CollaborationTool.loginfo('Initialise_pose')
            rc = self.planner.initial_pose()
            ret =self.return_btos(rc)
            if 'succeeded' == ret:
                #正常完了の場合はエラーカウンターをリセット.
                self.counter = 0
                break
            else:
                if self.counter >= 3:
                    return 'aborted'
                else:
                    continue

        return 'succeeded'

###########################################
#スタート前処理クラス.                    #
###########################################
class BEFORE_START(CollaborationState):
    def __init__(self, label, taskfin:TaskFinishServer, tran:Transition, outcomes):
        super().__init__(label, tran, outcomes)
        self.taskfin = taskfin

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)
        if self.tran.get_event().is_pause():
            CollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        
        CollaborationTool.loginfo('Before Start ...')
        if self.preempt('State BEFORE_START is being preempted!!!'):
            return 'preempted'
      
        return self.before_start()

    def before_start(self):
        ###ロボットごとのスタート前動作.
        #ここから.

        #ここまで.
        return 'succeeded'

###########################################
#準備中処理クラス.                        #
###########################################
class PREPAR_WORK(CollaborationState):
    def __init__(self, label, taskfin:TaskFinishServer, tran:Transition, area, disc, workd, outcomes):
        super().__init__(label, tran, outcomes)
        self.taskfin = taskfin
        self.area = area
        self.disc = disc
        self.workd = workd

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)

        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
      
        CollaborationTool.loginfo('Preparing ...')
        if self.preempt('State Prepar is being preempted!!!'):
            return 'preempted'
        
        #準備処理.
        return self.prepar()

    def prepar(self):
        ###ロボットごとの準備動作.
        #ここから.

        self.counter = 0

        #現在のコマンドリストを取得.
        command_list = CollaborationCurrentData.GetCurrentCommandList()
        command_id = command_list.get(0).task_command_id

        #ワークを認識する（ワーク検出サブシステム）. 作業1, 2 はワークを扱うので必須.
        if command_id in WORK_DETECT_COMMANDS:
            while not CollaborationTool.is_shutdown():
                ret = self.WorkDetect()
                if 'succeeded' == ret:
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret

            work_results = self.workd.work_detect_result.work_detection_results
            CollaborationTool.loginfo('Work detection : {} 個'.format(len(work_results)))
            if 0 == len(work_results):
                #ワークがない場合は自律動作を行わず, 作業結果を失敗として伝達する.
                CollaborationTool.logwarn('ワークが検出されませんでした. 作業を失敗として終了します.')
                CollaborationCurrentData.SetTaskResult(False)
                return 'succeeded'

        #ここまで.
        return 'succeeded'
    
    #周辺環境範囲を設定する.
    def PeripheralEnvironmentAreaSet(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        return 'succeeded'

    #排出位置を検出する.
    def PlaceDetect(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        
        CollaborationTool.loginfo("waiting place_position")
        self.disc.wait_for_service()
        CollaborationTool.loginfo("place_position comes up")

        event = self.check_event()
        if(None != event ):
            return event
        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State PlaceDetect is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            #排出位置を探索.
            self.set_place_area()
            rc = self.disc.execute(None)
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc) 
        else:
            return 'aborted'

    #ワークを検知する.
    def WorkDetect(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
    
        CollaborationTool.loginfo("waiting work_det_service")
        self.workd.wait_for_service()
        CollaborationTool.loginfo("work_det_service comes up")

        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State Work Detect is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            #ワーク検知範囲の設定.
            self.set_presence_area()
            rc = self.workd.execute(None)
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc) 
        else:
            return 'aborted'

    #排出位置を設定する.
    def set_place_area(self):
        #現在のコマンドリストを取得.
        command_list = CollaborationCurrentData.GetCurrentCommandList()
        task_command = command_list.get(0)
        place_area_list = task_command.get_PlaceLocationAreaList()
        print(place_area_list)
        place_area = place_area_list[0]
        print(place_area)
        self.disc.set_place_area(place_area.start_point.x,
                                     place_area.start_point.y,
                                     place_area.start_point.z,
                                     place_area.end_point.x,
                                     place_area.end_point.y,
                                     place_area.end_point.z,)

    #ワーク検知範囲を設定する.
    def set_presence_area(self):
        #現在のコマンドリストを取得.
        command_list = CollaborationCurrentData.GetCurrentCommandList()
        task_command = command_list.get(0)
        presence_area = task_command.work_presence_area
        self.workd.set_presence_area(task_command.task_command_id,
                                     task_command.work_type_id,
                                     presence_area.start_point.x,
                                     presence_area.start_point.y,
                                     presence_area.start_point.z,
                                     presence_area.end_point.x,
                                     presence_area.end_point.y,
                                     presence_area.end_point.z,)
    
    #作業途中で状態遷移するイベントの確認.
    def check_event(self):
        if self.tran.get_event().is_pause():
            CollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        return None
        
###########################################
#自律動作処理クラス.                      #
###########################################
class OPERATING_WORK(CollaborationState):
    def __init__(self,
                 label,
                 taskfin:TaskFinishServer,
                 tran:Transition,
                 outcomes):
        super().__init__(label, tran, outcomes)
        self.planner = ShareMovePlanner.get_move_planner()
        self.taskfin = taskfin
        self.counter = 0
        self.work_label = None
        self.handover_signal = HandoverSignal()
        self.handover_current = HandoverCurrent()
        self.handover_ok = False

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)
        #現在のコマンドリストを取得.
        command_list = CollaborationCurrentData.GetCurrentCommandList()
        CollaborationTool.loginfo(command_list.get(0).task_command_id)
        
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event

        #準備中にワークが見つからなかった場合は動作しない.
        if not CollaborationCurrentData.GetTaskResult():
            CollaborationTool.logwarn('準備中に失敗したため自律動作を行いません.')
            return 'succeeded'
        
        #自律動作.
        return self.operating_work()

    #自律動作処理.
    def operating_work(self):
        ###ロボットごとの自律動作.
        #ここから.

        #==========================================================================
        # 【講習課題 解答例】
        #   作業1 : ピック＆プレース（机の上のカップをトレイへ置く）
        #   作業2 : 人への受け渡し（引っ張られるか合図を受けたらハンドを開く. どちらもなければ元の位置へ戻す）
        #   作業3 : 逆運動学動作（初期姿勢から手先を上・横へ動かして戻る）
        # 関節角度は既存デモ（運搬）で教示済みの値を使用している.
        # ピック位置は既存デモの位置から手先姿勢を保ったまま8cm下げた値（運動学計算による, 実機未教示）.
        # 手前で机の高さまで下ろすと J4 が base_link と干渉判定されるため, 真上から下ろす.
        #==========================================================================

        #現在のコマンドリストを取得.
        command_list = CollaborationCurrentData.GetCurrentCommandList()
        command_id = command_list.get(0).task_command_id

        #作業1 ピック＆プレース.
        if command_id == 1:
            #ハンド開ける.
            if (None == self.work_label) or ('HandOpen1' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'HandOpen1'
                    ret = self.hand_open()
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #初期位置に戻る.
            if (None == self.work_label) or ('InitPose1' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'InitPose1'
                    ret = self.init_pose()
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #アプローチ（カップの手前）.
            if (None == self.work_label) or ('Approach1' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'Approach1'
                    ret = self.joint_manip(*APPROACH_POSE, MOVE_VELOCITY, MOVE_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #ピック位置の真上（8cm上）.
            if (None == self.work_label) or ('AbovePick1' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'AbovePick1'
                    ret = self.joint_manip(*ABOVE_PICK_POSE, MOVE_VELOCITY, MOVE_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #ピック位置（机の上のワーク. 真上から8cm下ろす）.
            if (None == self.work_label) or ('Pick1' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'Pick1'
                    ret = self.joint_manip(*PICK_POSE, CAREFUL_VELOCITY, CAREFUL_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #ハンド閉める（把持）.
            if (None == self.work_label) or ('HandClose1' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'HandClose1'
                    ret = self.hand_close()
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #持ち上げ.
            if (None == self.work_label) or ('Up1' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'Up1'
                    ret = self.joint_manip(*LIFT_UP_POSE, CAREFUL_VELOCITY, CAREFUL_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #運ぶ（旋回）.
            if (None == self.work_label) or ('Carry1' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'Carry1'
                    ret = self.joint_manip(*CARRY_POSE, CARRY_VELOCITY, CARRY_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #置く位置の上.
            if (None == self.work_label) or ('AbovePlace1' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'AbovePlace1'
                    ret = self.joint_manip(*ABOVE_PLACE_POSE, CARRY_VELOCITY, CARRY_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #置く.
            if (None == self.work_label) or ('Place1' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'Place1'
                    ret = self.joint_manip(*PLACE_POSE, MOVE_VELOCITY, MOVE_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #ハンド開ける（解放）.
            if (None == self.work_label) or ('Release1' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'Release1'
                    ret = self.hand_open()
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #置く位置の上へ戻る.
            if (None == self.work_label) or ('Retreat1' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'Retreat1'
                    ret = self.joint_manip(*ABOVE_PLACE_POSE, MOVE_VELOCITY, MOVE_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #初期位置に戻る.
            if (None == self.work_label) or ('InitPose1End' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'InitPose1End'
                    ret = self.init_pose()
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            self.work_label = None
            return 'succeeded'

        #作業2 人への受け渡し.
        if command_id == 2:
            #ハンド開ける.
            if (None == self.work_label) or ('HandOpen2' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'HandOpen2'
                    ret = self.hand_open()
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #初期位置に戻る.
            if (None == self.work_label) or ('InitPose2' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'InitPose2'
                    ret = self.init_pose()
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #アプローチ.
            if (None == self.work_label) or ('Approach2' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'Approach2'
                    ret = self.joint_manip(*APPROACH_POSE, MOVE_VELOCITY, MOVE_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #ピック位置の真上（8cm上）.
            if (None == self.work_label) or ('AbovePick2' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'AbovePick2'
                    ret = self.joint_manip(*ABOVE_PICK_POSE, MOVE_VELOCITY, MOVE_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #ピック位置（机の上のワーク. 真上から8cm下ろす）.
            if (None == self.work_label) or ('Pick2' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'Pick2'
                    ret = self.joint_manip(*PICK_POSE, CAREFUL_VELOCITY, CAREFUL_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #ハンド閉める（把持）.
            if (None == self.work_label) or ('HandClose2' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'HandClose2'
                    ret = self.hand_close()
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #持ち上げ.
            if (None == self.work_label) or ('Up2' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'Up2'
                    ret = self.joint_manip(*LIFT_UP_POSE, CAREFUL_VELOCITY, CAREFUL_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #受け渡し位置へ（人に近づくので低速）.
            if (None == self.work_label) or ('Handover2' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'Handover2'
                    ret = self.joint_manip(*HANDOVER_POSE, HANDOVER_VELOCITY, HANDOVER_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #人が受け取るのを待つ（引っ張られる, または合図を受けたら離す）.
            if (None == self.work_label) or ('HandoverWait2' == self.work_label):
                self.work_label = 'HandoverWait2'
                ret = self.handover_wait()
                if ('released' == ret) or ('timeout' == ret):
                    self.handover_ok = ('released' == ret)
                    self.work_label = None
                else:
                    return ret

            if self.handover_ok:
                #合図あり : ハンド開ける（受け渡し）.
                if (None == self.work_label) or ('Release2' == self.work_label):
                    while not CollaborationTool.is_shutdown():
                        self.work_label = 'Release2'
                        ret = self.hand_open()
                        if 'succeeded' == ret:
                            self.work_label = None
                            break
                        elif 'retry' == ret:
                            continue
                        else:
                            return ret

            else:
                #合図なし : 持ち上げ位置へ戻る.
                if (None == self.work_label) or ('ReturnUp2' == self.work_label):
                    while not CollaborationTool.is_shutdown():
                        self.work_label = 'ReturnUp2'
                        ret = self.joint_manip(*LIFT_UP_POSE, MOVE_VELOCITY, MOVE_VELOCITY)
                        if 'succeeded' == ret:
                            self.work_label = None
                            break
                        elif 'retry' == ret:
                            continue
                        else:
                            return ret

                #合図なし : ピック位置の真上へ.
                if (None == self.work_label) or ('ReturnAbove2' == self.work_label):
                    while not CollaborationTool.is_shutdown():
                        self.work_label = 'ReturnAbove2'
                        ret = self.joint_manip(*ABOVE_PICK_POSE, MOVE_VELOCITY, MOVE_VELOCITY)
                        if 'succeeded' == ret:
                            self.work_label = None
                            break
                        elif 'retry' == ret:
                            continue
                        else:
                            return ret

                #合図なし : 元の位置へ置く.
                if (None == self.work_label) or ('ReturnPlace2' == self.work_label):
                    while not CollaborationTool.is_shutdown():
                        self.work_label = 'ReturnPlace2'
                        ret = self.joint_manip(*PICK_POSE, CAREFUL_VELOCITY, CAREFUL_VELOCITY)
                        if 'succeeded' == ret:
                            self.work_label = None
                            break
                        elif 'retry' == ret:
                            continue
                        else:
                            return ret

                #合図なし : ハンド開ける.
                if (None == self.work_label) or ('ReturnOpen2' == self.work_label):
                    while not CollaborationTool.is_shutdown():
                        self.work_label = 'ReturnOpen2'
                        ret = self.hand_open()
                        if 'succeeded' == ret:
                            self.work_label = None
                            break
                        elif 'retry' == ret:
                            continue
                        else:
                            return ret

                #合図なし : 真上へ退避.
                if (None == self.work_label) or ('ReturnRetreat2' == self.work_label):
                    while not CollaborationTool.is_shutdown():
                        self.work_label = 'ReturnRetreat2'
                        ret = self.joint_manip(*ABOVE_PICK_POSE, CAREFUL_VELOCITY, CAREFUL_VELOCITY)
                        if 'succeeded' == ret:
                            self.work_label = None
                            break
                        elif 'retry' == ret:
                            continue
                        else:
                            return ret

            #初期位置に戻る.
            if (None == self.work_label) or ('InitPose2End' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'InitPose2End'
                    ret = self.init_pose()
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            self.work_label = None
            return 'succeeded'

        #作業3 逆運動学動作.
        if command_id == 3:
            #初期位置に戻る.
            if (None == self.work_label) or ('InitPose3' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'InitPose3'
                    ret = self.init_pose()
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #現在の手先位置・姿勢を記憶（基準位置）.
            if (None == self.work_label) or ('GetPose3' == self.work_label):
                self.work_label = 'GetPose3'
                current = self.planner.get_tool_pose()
                if current is None:
                    return 'aborted'
                self.base_pose = current.pose
                CollaborationTool.loginfo('Base tool pose : {}'.format(self.base_pose))
                self.work_label = None

            p = self.base_pose.position
            q = self.base_pose.orientation

            #上へ移動（z + IK_OFFSET）.
            if (None == self.work_label) or ('IKUp3' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'IKUp3'
                    ret = self.pose_manip(p.x, p.y, p.z + IK_OFFSET, q.x, q.y, q.z, q.w, IK_VELOCITY, IK_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #横へ移動（y + IK_OFFSET）.
            if (None == self.work_label) or ('IKSide3' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'IKSide3'
                    ret = self.pose_manip(p.x, p.y + IK_OFFSET, p.z + IK_OFFSET, q.x, q.y, q.z, q.w, IK_VELOCITY, IK_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #基準位置へ戻る.
            if (None == self.work_label) or ('IKReturn3' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'IKReturn3'
                    ret = self.pose_manip(p.x, p.y, p.z, q.x, q.y, q.z, q.w, IK_VELOCITY, IK_VELOCITY)
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            #初期位置に戻る.
            if (None == self.work_label) or ('InitPose3End' == self.work_label):
                while not CollaborationTool.is_shutdown():
                    self.work_label = 'InitPose3End'
                    ret = self.init_pose()
                    if 'succeeded' == ret:
                        self.work_label = None
                        break
                    elif 'retry' == ret:
                        continue
                    else:
                        return ret

            self.work_label = None
            return 'succeeded'

        #ここまで.
        self.work_label = None
        return 'succeeded'

    #受け渡し待ち.
    #  人がワークを引っ張る（モータ電流の変化）か, 合図(HANDOVER_TOPIC)を受けるまで待つ.
    #  戻り値 : 'released' 受け取られた / 'timeout' 時間切れ / それ以外は状態遷移（そのまま return する）
    def handover_wait(self):
        CollaborationTool.loginfo('HandoverWait : カップを引くか, 合図を送ってください（{} 秒以内）'.format(HANDOVER_WAIT_TIME))
        CollaborationTool.loginfo('  ros2 topic pub --once {} std_msgs/msg/Empty {{}}'.format(HANDOVER_TOPIC))
        #待ち始める前の古い合図・電流は捨てる.
        self.handover_signal.clear()
        self.handover_current.clear()
        #use_sim_time の影響を受けないようにシステム時刻で計測する.
        start = time.monotonic()
        baseline = None
        use_current = True
        while not CollaborationTool.is_shutdown():
            #作業途中で状態遷移するイベントの確認.
            event = self.check_event()
            if(None != event):
                return event

            #合図を確認.
            if self.handover_signal.check(0.0):
                CollaborationTool.loginfo('Handover : 合図を受信しました')
                return 'released'

            #電流による引っ張り検知.
            self.handover_current.update()
            now = time.monotonic()
            elapsed = now - start
            if use_current and (baseline is None) and (elapsed >= PULL_SETTLE_SEC + PULL_BASELINE_SEC):
                #静止直後の揺れが収まってから基準値を取る.
                baseline = self.handover_current.mean_since(start + PULL_SETTLE_SEC)
                if baseline is None:
                    CollaborationTool.logwarn('電流が取得できません（recv_format:=66 で起動しているか確認）. 合図のみで待機します.')
                    use_current = False
                else:
                    CollaborationTool.loginfo('Handover : 電流の基準値 {}'.format(['{:.1f}'.format(v) for v in baseline]))
            if use_current and (baseline is not None):
                average = self.handover_current.mean_since(now - PULL_AVERAGE_SEC)
                if average is not None:
                    diff = {j: average[j - 1] - baseline[j - 1] for j in PULL_DETECT_JOINTS}
                    if max(abs(v) for v in diff.values()) >= PULL_THRESHOLD:
                        CollaborationTool.loginfo('Handover : 引っ張りを検知しました 変化量 {}'.format({'J{}'.format(j): round(v, 1) for j, v in diff.items()}))
                        return 'released'

            #時間切れ.
            if elapsed >= HANDOVER_WAIT_TIME:
                CollaborationTool.logwarn('Handover timeout : 離さずに元の位置へ戻します')
                return 'timeout'
            time.sleep(0.01)
        return 'aborted'

    #ハンド開ける.
    def hand_open(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State HAND_OPEN is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            CollaborationTool.loginfo('HandOpen')
            rc = self.planner.hand_open()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
    
    #ハンド閉める.
    def hand_close(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State HAND_CLOSE is being preempted!!!'):
                return 'preempted'
            
            self.counter += 1
            CollaborationTool.loginfo('HandClose')
            rc = self.planner.hand_close()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
    

    #初期位置移動.
    def init_pose(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('InitPose')
            rc = self.planner.initial_pose()
           #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
    
    #順運動学動作
    def joint_manip(self, val_1, val_2, val_3, val_4, val_5, val_6, vel, acc):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State NORMAL_MANIP is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            CollaborationTool.loginfo('Manipulate at ({},{},{},{},{},{}) in scale velocity and acceleration {},{}'.format(val_1, val_2, val_3, val_4, val_5, val_6, vel, acc))
            rc = self.planner.joint_value(val_1, val_2, val_3, val_4, val_5, val_6, vel, acc)
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
    
    #逆運動学動作.
    def pose_manip(self, x, y, z, ox, oy, oz, ow, vel, acc):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State NORMAL_MANIP is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            CollaborationTool.loginfo('Manipulate at ({},{},{},{},{},{},{}) in scale velocity and acceleration {},{}'.format(x, y, z, ox, oy, oz, ow, vel, acc))
            rc = self.planner.tool_pose(x, y, z, ox, oy, oz, ow, vel, acc)
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'

    #作業途中で状態遷移するイベントの確認.
    def check_event(self):
        if self.tran.get_event().is_pause():
            CollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())

        elif self.tran.get_event().is_workend():
            command_list = CollaborationCurrentData.GetCurrentCommandList()
            task_command = command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        return None


###########################################
#非自律動作処理クラス.                    #
###########################################
class NON_OPERATING_WORK(CollaborationState):
    def __init__(self, label, taskfin:TaskFinishServer, tran:Transition, outcomes):
        super().__init__(label, tran, outcomes)
        self.taskfin = taskfin
        self.planner = ShareMovePlanner.get_move_planner()

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)
        #現在のコマンドリストを取得.
        command_list = CollaborationCurrentData.GetCurrentCommandList()
        CollaborationTool.loginfo(command_list.get(0).task_command_id)
        
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        
        #非自律動作処理.
        return self.non_operating_work()
    
    #非自律動作処理.
    def non_operating_work(self):
        ###ロボットごとの非自律動作.
        #ここから.

        CollaborationTool.wait_time(NON_OPERATING_WAIT)

        #初期位置に戻る.
        if (None == self.work_label) or ('InitPose1' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'InitPose1'
                ret = self.init_pose()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #ここまで.
        self.work_label = None
        return 'succeeded'
    
    #初期位置移動.
    def init_pose(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('InitPose')
            rc = self.planner.initial_pose()
           #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'

    #作業途中で状態遷移するイベントの確認.
    def check_event(self):
        if self.tran.get_event().is_pause():
            CollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())

        elif self.tran.get_event().is_workend():
            command_list = CollaborationCurrentData.GetCurrentCommandList()
            task_command = command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        return None

###########################################
#一時停止中処理クラス.                    #
###########################################
class PAUSED_WORK(CollaborationState):
    def __init__(self, label, taskfin:TaskFinishServer, tran:Transition, outcomes):
        super().__init__(label, tran, outcomes)
        self.taskfin = taskfin
        self.counter = 0
        self.planner = ShareMovePlanner.get_move_planner()

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)

        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event

        CollaborationTool.loginfo('Preparing ...')
        if self.preempt('State Prepar is being preempted!!!'):
            return 'preempted'
        
        #待機処理.
        return self.paused()

    def paused(self):
        ###ロボットごとの待機動作.
        #ここから.

        CollaborationTool.wait_time(PAUSED_WAIT)

        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        
        #初期位置移動.
        while not CollaborationTool.is_shutdown():
            ret = self.init_pose()
            if 'succeeded' == ret:
                    break
            elif 'retry' == ret:
                continue
            else:
                return ret


        #ここまで.
        return 'succeeded'

    #初期位置移動.
    def init_pose_upper_body(self):
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('Initialise')
            rc = self.planner.initial_pose_upper_body()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
        
    #初期位置移動.
    def init_pose(self):
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('init_pose')
            rc = self.planner.initial_pose()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'

    #作業途中で状態遷移するイベントの確認.
    def check_event(self):
        if self.tran.get_event().is_workend():
            command_list = CollaborationCurrentData.GetCurrentCommandList()
            task_command = command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        return None

###########################################
#終了処理クラス.                          #
###########################################        
class FINALISE(CollaborationState):
    def __init__(self,
                 label,
                 taskfin:TaskFinishServer,
                 tran:Transition,
                 outcomes):
        super().__init__(label, tran, outcomes)
        self.planner = ShareMovePlanner.get_move_planner()
        self.taskfin = taskfin
        self.counter = 0

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)

        if self.tran.get_event().is_workend():
            task_command_list = CollaborationCurrentData.GetCurrentCommandList()
            task_command = task_command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        
        if self.preempt('INIITIALISE_POSE is being preempted!!!'):
            return 'preempted'
        
        #終了処理を行う.
        return self.finalise()

    #終了処理.
    def finalise(self):
        ###ロボットごとの終了動作.
        #ここから.

        #カップを持っているときに軸を動かすと事故につながるので何もせず終了.

        #ここまで.
        return 'succeeded'
    
    def init_pose(self):
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('finalise')
            rc = self.planner.initial_pose()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'