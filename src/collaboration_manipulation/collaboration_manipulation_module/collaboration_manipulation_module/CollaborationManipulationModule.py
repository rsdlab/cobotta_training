#!/usr/bin/env python3
# coding: UTF-8

#####################################################################################################################
#このノードは．人協働マニピュレーションモジュールを機能単位で作成したモジュールです．　
#ハンド部分のモジュールと人検知部分の追加を行っています．　　　　　　　　　　　　　　 #
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/11/07
#08/30 排出位置検出システム編集
#09/21 排出位置検出システム　統合
#09/28 WS環境認識システム作成
#10/05 排出位置検出システム　統合確認　
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

from .CollaborationStateModule import HumanCollaborationStateMachine
from .CollaborationCommunicationModule import TaskCommandServer, TaskCancelServer, TaskFinishServer, GetStatusServer, SystemTerminate, PeripheralEnvironmentAreaSetClient
from .CollaborationCommunicationModule import PlacePositionClient, WorkDetectionClient, NotifyTaskResultClient, NotifyTaskCompleteClient, ServiceServerExecutor
from .CollaborationEventModule import CollaborationEventPublisher, CollaborationEventSubscriverWorkStart
from .CollaborationToolModule import CollaborationTool

##################################################################
#　　　　　　　　　　　 クラス定義　                            #
##################################################################
class HumanCollaboration:
    def __init__(self):
        area = PeripheralEnvironmentAreaSetClient()
        disc = PlacePositionClient()
        workd = WorkDetectionClient()
        workstart_event = CollaborationEventSubscriverWorkStart()
        self.recv_cmd = TaskCommandServer()
        self.sys_teminate = SystemTerminate()
        self.task_cancel = TaskCancelServer()
        self.task_finish = TaskFinishServer()
        self.get_status = GetStatusServer()
        self.statemachine = HumanCollaborationStateMachine(
              NotifyTaskResultClient(),
              NotifyTaskCompleteClient(),
              self.task_cancel,
              self.task_finish,
              self.get_status,
              area,
              disc,
              workd,
              workstart_event
              )
        CollaborationEventPublisher.register(self.statemachine)
        CollaborationEventPublisher.register(workstart_event)

        # サブスレッドにサービスサーバーを渡す.
        self.executor = ServiceServerExecutor()
        self.executor.add_service_server(self.recv_cmd)
        self.executor.add_service_server(self.sys_teminate)
        self.executor.add_service_server(self.task_cancel)
        self.executor.add_service_server(self.task_finish)
        self.executor.add_service_server(self.get_status)

    def execute(self):
        # サブスレッドでサービスサーバをspinする.
        self.executor.execute()

        # 状態遷移を開始.
        try:
            CollaborationTool.loginfo('State machine Start')
            self.statemachine.execute()
            CollaborationTool.loginfo(' State machine End')
        
        except KeyboardInterrupt:
            print('shutdown')

        finally:
            # マルチスレッドノードを破棄.
            self.executor.shutdown()
            CollaborationTool.signal_shutdown("sys_manage_halt")

def main():
    CollaborationTool.ros_init()
    hc = HumanCollaboration()
 
    #実行
    hc.execute()
##################################################################
#　　　　　　　　　　　 以下実行処理内容                         #
##################################################################   
if __name__ == '__main__':
    main()
 
