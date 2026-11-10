#!/usr/bin/env python3
# coding: UTF-8

#####################################################################################################################
#このノードは，上位アプリを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/06/28
#10/05 test_server.pyと単体検証を行うために，クラス部分をコメントアウト→統合試験でも成功
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

from abc import ABCMeta
from abc import abstractmethod
import rclpy
from collaboration_manipulation_message.msg import TargetArea, TaskCommandList, TaskCommand, PlaceAreaCandidatesList
from collaboration_manipulation_message.srv import TerminateSystem, SendTaskCommand, NotifyTaskResult, NotifyTaskCompletion, MoveCommunication
from std_msgs.msg import Empty
import sys
import os
import time
from rclpy.node import Node
import threading
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException

# 相対パスでhuman_collaboration/scriptsのパスを追加
sys.path.append(os.path.join(os.path.dirname(__file__), '../../collaboration_manipulation_module/collaboration_manipulation_module'))
# インポート
from EnumerateModule import EnumCommandReceiveState

class SystemManagementBase(Node, metaclass=ABCMeta):
    """
    Base class that SystemManagement communication
    Define machine-dependent packages
    """
    def __init__(self, node_name):
        super().__init__(node_name)
    
    @abstractmethod
    def execute(self, data):
        pass


class SystemManagementPublisher(SystemManagementBase):
    def __init__(self, topic_name, class_type, node_name):
        super().__init__(node_name)
        self.pub = self.create_publisher(class_type, topic_name, queue_size = 10)

    def execute(self, data):
        self.pub.publish(data)

class SystemManagementClient(SystemManagementBase):
    def __init__(self, service_name, service_class, node_name):
        super().__init__(node_name)
        self.proxy = self.create_client(service_class, service_name)

    def execute(self, data):
        future = self.proxy.call_async(data)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        print(result)
        return result

class SystemManagementServer(SystemManagementBase):
    def __init__(self, service_name, service_class, callback_impl, node_name):
        super().__init__(node_name)
        self.server = self.create_service(service_class, service_name, callback_impl)

    def service_delete(self):
        self.server.destroy()

    def execute(self, object):
        pass

    def spin(self):
        rclpy.spin(self)

#作業結果コマンド受信クラス.
class TaskResultServer(SystemManagementServer):
    def __init__(self):
        super().__init__('notify_task_result', NotifyTaskResult, self.recv_task_result, 'task_result_server_node')
        self.task_failed = 0

    def service_delete(self):
        super().service_delete()

    #作業結果コマンド受信.
    def recv_task_result(self, data, res):
        print("TaskResult:", data.task_result.task_result)
        if(False == data.task_result.task_result):
            self.task_failed = 1

        res.return_code = 1
        return res
    
    def get_task_failed(self):
        return self.task_failed
    
    def reset_task_result(self):
        self.task_failed = 0

#作業完了コマンド受信クラス.
class TaskCompleteServer(SystemManagementServer):
    def __init__(self):
        super().__init__('notify_task_completion', NotifyTaskCompletion, self.recv_task_complete, 'task_complete_server_node')
        self.task_complete = 0

    def service_delete(self):
        super().service_delete()

    #作業完了コマンド受信.
    def recv_task_complete(self, data, res):
        print("TaskComplete")
        self.task_complete = 1
        res.return_code = 1
        return res
    
    def get_task_complete(self):
        return self.task_complete
    
    def reset_task_complete(self):
        self.task_complete = 0

#システム終了指令.
class TerminateSystemClient(SystemManagementClient):
    def __init__(self,):
        super().__init__('terminate_system', TerminateSystem, 'terminate_system_client_node')

    def execute(self):
        msg = TerminateSystem.Request()
        msg.empty = Empty()
        super().execute(msg)

#作業開始指令.
class SendTaskCommandClient(SystemManagementClient):
    def __init__(self):
        super().__init__('send_command_service', SendTaskCommand, 'send_task_command_client_node')

    def execute(self, command_id):
        print('{} start'.format(self.__class__.__name__))

        while not self.proxy.wait_for_service(timeout_sec=1.0):
            print("Waiting for send_command_service to become available...")
        print("send_command_service fined!")
        ###作業コマンド作成 ここから.

        #作業コマンドリスト作成.
        set_task_command_list = TaskCommandList()
        set_task_command = TaskCommand()
        set_task_command.task_command_id = command_id
        set_task_command.work_type_id = 1
        set_task_command.picking_count= 1
        
        #ワーク検知エリア設定.
        set_task_command.work_presence_area = TargetArea()
        set_task_command.work_presence_area.start_point.x = -2.0
        set_task_command.work_presence_area.start_point.y = -2.0
        set_task_command.work_presence_area.start_point.z = 0.0
        set_task_command.work_presence_area.end_point.x = 2.0
        set_task_command.work_presence_area.end_point.y = 2.0
        set_task_command.work_presence_area.end_point.z = 2.0

        #廃棄候補エリアリスト作成.
        set_task_command.place_area_candidates = PlaceAreaCandidatesList()

        #廃棄候補エリア作成.
        dis_area = TargetArea()
        dis_area.start_point.x = -2.0
        dis_area.start_point.y = -2.0
        dis_area.start_point.z = 0.0
        dis_area.end_point.x = 2.0
        dis_area.end_point.y = 2.0
        dis_area.end_point.z = 2.0
        set_task_command.place_area_candidates.place_area_candidates.append(dis_area)

        print(set_task_command)
        set_task_command_list.task_commands.append(set_task_command)

        #ここまで.
        srv = SendTaskCommand.Request()
        srv.task_commands = set_task_command_list
        ret = super().execute(srv)
        print('reslt ', ret)
        return ret
    
def wait(result:TaskResultServer, comp:TaskCompleteServer):
    while ((False == result.get_task_failed()) and (False == comp.get_task_complete()) and (rclpy.ok())):
        time.sleep(0.05)
    return

class ServiceServerExecutor:
    def __init__(self):
        self.executor = SingleThreadedExecutor()
        self.node_list = []

    def add_service_server(self, node):
        self.executor.add_node(node)
        self.node_list.append(node)

    def execute(self):
        # サブスレッドでexecutorをspin.
        self.executor_thread = threading.Thread(target=self.spin_executor, args=(), daemon=True)
        self.executor_thread.start()

    def spin_executor(self):
        # サブスレッドでノードのspinを継続
        try:
            self.executor.spin()
        except ExternalShutdownException:
                print("Executor spin stopped by external shutdown.")
        except Exception as e:
                print(f"Executor spin error: {e}")
        finally:
                print("Executor stopped")
    
    def shutdown(self):
        self.executor.shutdown()
        if self.executor_thread is not None:
            self.executor_thread.join()
        for node in self.node_list:
            node.service_delete()
            node.destroy_node()

#システム始動通信受信クラス.
class MoveCommunicationServer(SystemManagementServer):
    def __init__(self):
        super().__init__('move_cobotta', MoveCommunication, self.move_command, 'move_communication_server_node')

    def service_delete(self):
        super().service_delete()
    
    def move_command(self, request, response):
        response.empty = carry()
        return response
    

def halt():
    cmd = TerminateSystemClient()
    #終了処理
    print("halt")
    time.sleep(0.1)
    print("wait TerminateSystem Server")
    cmd.proxy.wait_for_service()
    print("TerminateSystem Server OK")
    cmd.execute()
    time.sleep(0.1)

def carry(data = None):
    cmd = SendTaskCommandClient()
    result = TaskResultServer()
    complete = TaskCompleteServer()
    executor = ServiceServerExecutor()
    executor.add_service_server(result)
    executor.add_service_server(complete)
    executor.execute()

    #一人撮影用タイマー
    #time.sleep(10.0)
    
    #ボタン押す.
    print("Push")
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(1).response):
        print("command_retry")
        time.sleep(1)

    #動作完了まで待つ.
    wait(result, complete)
    if(True == result.get_task_failed()):
        executor.shutdown()
        return Empty()

    #待機.
    print("Wait")
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(2).response):
        print("command_retry")
        time.sleep(1)

    #動作完了まで待つ.
    wait(result, complete)
    if(True == result.get_task_failed()):
        executor.shutdown()
        return Empty()

    #運ぶ.
    print("Carry")
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(3).response):
        print("command_retry")
        time.sleep(1)

    #動作完了まで待つ.
    wait(result, complete)
    if(True == result.get_task_failed()):
        executor.shutdown()
        return Empty()

    executor.shutdown()
    return Empty()

def push(data = None):
    cmd = SendTaskCommandClient()
    result = TaskResultServer()
    complete = TaskCompleteServer()
    executor = ServiceServerExecutor()
    executor.add_service_server(result)
    executor.add_service_server(complete)
    executor.execute()
    
    #ボタン押す.
    print("Push")
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(1).response):
        print("command_retry")
        time.sleep(1)

    #動作完了まで待つ.
    wait(result, complete)
    if(True == result.get_task_failed()):
        executor.shutdown()
        return Empty()

    executor.shutdown()
    return Empty()

def carry_only(data = None):
    cmd = SendTaskCommandClient()
    result = TaskResultServer()
    complete = TaskCompleteServer()
    executor = ServiceServerExecutor()
    executor.add_service_server(result)
    executor.add_service_server(complete)
    executor.execute()

    #運ぶ.
    print("Carry")
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(3).response):
        print("command_retry")
        time.sleep(1)

    #動作完了まで待つ.
    wait(result, complete)
    if(True == result.get_task_failed()):
        executor.shutdown()
        return Empty()

    executor.shutdown()
    return Empty()

#指定した作業指令IDのみ実行.
def single(command_id):
    cmd = SendTaskCommandClient()
    result = TaskResultServer()
    complete = TaskCompleteServer()
    executor = ServiceServerExecutor()
    executor.add_service_server(result)
    executor.add_service_server(complete)
    executor.execute()

    print("Task {}".format(command_id))
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(command_id).response):
        print("command_retry")
        time.sleep(1)

    #動作完了まで待つ.
    wait(result, complete)

    executor.shutdown()
    return Empty()

def main(args=None):
    rclpy.init(args=args)
    argv = sys.argv
    print("args:", argv)

    #引数なしで起動された場合は一連動作を行う.
    if 1 < len(argv):
        #システム終了指令.
        if 'halt' == argv[1]:
            halt()
        elif 'push' == argv[1] or 'Push' == argv[1]:
            push()
        elif 'carry' == argv[1]:
            carry_only()
        elif argv[1].isdigit():
            single(int(argv[1]))
        elif'comm' == argv[1]:
            comm = MoveCommunicationServer()
            rclpy.spin(comm)
        else:
            carry()
    else:
        carry()
    
    rclpy.shutdown() 

if __name__ == '__main__':
    main()