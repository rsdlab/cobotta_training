#!/usr/bin/env python3
# coding: UTF-8
#####################################################################################################################
#このノードは，ワーク検出モジュールを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/06/28
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

import rclpy
import yaml
import rospkg

from std_msgs.msg import *
from geometry_msgs.msg import *
from collaboration_manipulation_message.msg import WorkDetectionResult
from collaboration_manipulation_message.srv import DetectWorkpieces
from rclpy.node import Node

class WorkDetect(Node):
    def __init__(self):
        super().__init__('work_detection_node')
        #カメラ未接続時の仮実装. work_present:=false でワークなしを模擬できる.
        self.declare_parameter('work_present', True)
        self.setup_req = self.server = self.create_service(DetectWorkpieces, 'detect_workpieces_service', self.pose_request)
        print("Initialization done")

    def pose_request(self, req_, res):
        #ワーク検知処理.
        #ここから.
        
        # リクエストの確認.
        print("")
        print("Request Data:")
        print(req_)
        print("")

        # 仮実装 : センサは使わず, パラメータに従ってワークの有無だけを返す.
        # 位置・姿勢はカメラ等で認識した値を入れる（今は未設定）.
        if self.get_parameter('work_present').value:
            set_data = WorkDetectionResult()
            set_data.work_type_id = req_.work_type_id
            set_data.reliability = 1.0
            set_data.graspability = 1.0
            res.work_detection_results.work_detection_results.append(set_data)
        print("Detected : {}".format(len(res.work_detection_results.work_detection_results)))
        return res
        #ここまで.

def main():
    rclpy.init()
    wd = WorkDetect()
    rclpy.spin(wd)

if __name__ == "__main__":
    main()
