#!/usr/bin/env python3
# coding: UTF-8

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from action_msgs.msg import GoalStatus
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from sensor_msgs.msg import JointState
import math
import time
from moveit_msgs.msg import PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
from tf2_ros import Buffer, TransformListener

MOVE_GROUP_COMMAND_NAME = 'arm'
MOVE_FRAME_NAME = 'base_link'
MOVE_LINK_NAME = 'tool'
MOVE_PLANNER_TIMEOUT = 7.0

INITIALIZE_JOINT_1 = -90.0
INITIALIZE_JOINT_2 = -20.0
INITIALIZE_JOINT_3 = 90.0
INITIALIZE_JOINT_4 = 90.0
INITIALIZE_JOINT_5 = 90.0
INITIALIZE_JOINT_6 = -90.0

HAND_OPEN_VALUE= 0.015
HAND_CLOSE_VALUE= 0.0032

#MovePlannerクラス.
class MovePlanner(Node):
    """
    Classes that plan and control movement
    Define machine-dependent packages
    """
    def __init__(self):
        super().__init__('move_Planner_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Waiting for move_action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Connected to move_action server.')
        self._gripper_client = ActionClient(self, GripperCommand, '/denso_hand_controller/gripper_cmd')
        self.get_logger().info('Waiting for /denso_hand_controller/gripper_cmd action server...')
        self._gripper_client.wait_for_server()
        self.get_logger().info('Connected to gripper action server.')
    
    #順運動学動作呼び出し口.
    def joint_value(self, val_1, val_2, val_3, val_4, val_5, val_6, vel = 1.0, acc = 1.0):
        joint_goal = {
        'joint_1': math.radians(val_1),
        'joint_2': math.radians(val_2),
        'joint_3': math.radians(val_3),
        'joint_4': math.radians(val_4),
        'joint_5': math.radians(val_5),
        'joint_6': math.radians(val_6)}
        return self.send_goal_fk(joint_goal, vel, acc)
    
    #順運動学動作.
    def send_goal_fk(self, joint_goal, vel, acc):
        self.get_logger().info(f"Moving arm to goal1: {joint_goal}")

        goal_msg = MoveGroup.Goal()
        goal_msg.planning_options.plan_only = False
        # 目標ジョイント状態をセット
        joint_constraints = []
        for joint_name, position in joint_goal.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            joint_constraints.append(jc)
        constraints = Constraints()
        constraints.joint_constraints = joint_constraints
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = MOVE_GROUP_COMMAND_NAME  # MoveIt設定に合わせて変更
        motion_plan_request.goal_constraints.append(constraints)
        motion_plan_request.max_velocity_scaling_factor = vel  # 速度
        motion_plan_request.max_acceleration_scaling_factor = acc  # 加速度
        goal_msg.request = motion_plan_request
        self.get_logger().info('Sending goal...')
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return False
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Motion succeeded!')
            #動作完了を待つ
            return self.wait_until_joint_goal_reached(joint_goal)
        else:
            self.get_logger().error(f'Motion failed with error code {result.result.error_code.val}')
            return False
    
    #動作完了待ち.
    def wait_until_joint_goal_reached(self, target_joints: dict, tolerance=0.015, timeout=60.0):
        """
        target_joints: {joint_name: radian_value, ...}
        tolerance: 許容誤差（ラジアン）
        timeout: タイムアウト秒数
        """
        current_joint_states = {}

        def joint_state_callback(msg):
            for name, position in zip(msg.name, msg.position):
                current_joint_states[name] = position

        sub = self.create_subscription(
            JointState,
            '/joint_states',
            joint_state_callback,
            10)

        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            # 全てのtarget_jointsの角度が取得できているか確認
            if not all(j in current_joint_states for j in target_joints):
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            # 誤差判定
            all_within_tolerance = True
            for joint_name, target_pos in target_joints.items():
                current_pos = current_joint_states[joint_name]
                if abs(current_pos - target_pos) > tolerance:
                    all_within_tolerance = False
                    break

            if all_within_tolerance:
                self.get_logger().info("Target joint positions reached.")
                log_str = "Current joint states:"
                for joint_name in sorted(current_joint_states.keys()):
                    log_str += f"\n  {joint_name}: {math.degrees(current_joint_states[joint_name]):.2f} deg"
                self.get_logger().info(log_str)
                self.destroy_subscription(sub)
                return True

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn("Timeout waiting for joints to reach target.")
        log_str = "Current joint states:\n"
        for joint_name in sorted(current_joint_states.keys()):
            log_str += f"  {joint_name}: {math.degrees(current_joint_states[joint_name]):.2f} deg\n"
        self.get_logger().info(log_str)
        self.destroy_subscription(sub)
        return False
    
    def initial_pose(self):
        return self.joint_value(INITIALIZE_JOINT_1, INITIALIZE_JOINT_2, INITIALIZE_JOINT_3, INITIALIZE_JOINT_4, INITIALIZE_JOINT_5, INITIALIZE_JOINT_6)

    #ハンド制御.
    def send_hand_goal(self, position: float, max_effort: float = 5.0):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position  # [m]
        goal_msg.command.max_effort = max_effort  # 最大力[N]

        self.get_logger().info(f'Sending gripper goal: position={position}, max_effort={max_effort}')
        send_goal_future = self._gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False

        self.get_logger().info('Gripper goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Gripper motion succeeded')
            return True
        else:
            self.get_logger().error(f'Gripper motion failed with status code: {result.status}')
            return False
    
    #ハンド開ける.
    def hand_open(self):
        return self.send_hand_goal(HAND_OPEN_VALUE)
    
    #ハンド閉める.
    def hand_close(self):
        return self.send_hand_goal(HAND_CLOSE_VALUE)
    
    # 逆運動学動作呼び出し口.
    def tool_pose(self,x ,y, z, ox, oy, oz, ow, vel = 1.0, acc = 1.0):
        target_pose = PoseStamped()
        target_pose.header.frame_id = MOVE_FRAME_NAME  # cobottaの基準フレームに合わせる
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = ox
        target_pose.pose.orientation.y = oy
        target_pose.pose.orientation.z = oz
        target_pose.pose.orientation.w = ow
        return self.send_goal_ik(target_pose, vel, acc)
    
    # 逆運動学動作.
    def send_goal_ik(self, target_pose: PoseStamped, vel, acc):
        goal = MoveGroup.Goal()

        goal.request.group_name = MOVE_GROUP_COMMAND_NAME
        goal.request.goal_constraints.append(self.create_goal_constraints_ik(target_pose))
        goal.request.allowed_planning_time = MOVE_PLANNER_TIMEOUT
        goal.planning_options.plan_only = False
        goal.request.max_velocity_scaling_factor = vel
        goal.request.max_acceleration_scaling_factor = acc

        self.get_logger().info('Sending goal to move_action...')
        send_goal_future = self._action_client.send_goal_async(goal, self.feedback_callback_ik)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')

        # 結果受信のFutureを取得して完了まで待つ
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()

        # GoalStatus の状態を確認する
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if result.result.error_code.val == 1:
                self.get_logger().info('Motion succeeded!')
                #動作完了を待つ.
                return self.wait_until_tool_pose_reached(target_pose)
            else:
                self.get_logger().error(f'Motion failed with error code {result.result.error_code.val}')
                return False
        else:
            self.get_logger().error(f'Motion did not succeed. Status: {result.status}')
            return False

    # 逆運動学用msg作成.
    def create_goal_constraints_ik(self, target_pose: PoseStamped) -> Constraints:
        constraints = Constraints()
        # 位置制約
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = MOVE_LINK_NAME  # cobottaのエンドエフェクタ名に合わせる
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 許容範囲
        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)

        constraints.position_constraints.append(pos_constraint)

        # 姿勢制約
        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = MOVE_LINK_NAME  # 同じくエンドエフェクタ名
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        constraints.orientation_constraints.append(ori_constraint)

        return constraints

    # フィードバックコールバック
    def feedback_callback_ik(self, feedback_msg):
        # feedback_msgはmoveit_msgs/MoveGroup_Feedback型
        self.get_logger().info(f"Current state: {feedback_msg.feedback.state}")

    # toolの現在地取得.
    def get_tool_pose(self):
        try:
            # 親フレーム, 子フレーム（例: base_link→tool）
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                MOVE_FRAME_NAME,
                MOVE_LINK_NAME,
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f'Could not get tool pose: {e}')
            return None
    
    # 位置確認.
    def pose_distance(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    # 姿勢確認.
    def orientation_distance(self, pose1, pose2):
        # 四元数の内積で角度差を計算
        import numpy as np
        q1 = pose1.orientation
        q2 = pose2.orientation

        dot_product = q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w
        dot_product = max(min(dot_product, 1.0), -1.0)  # Clamp

        angle = 2 * math.acos(abs(dot_product))
        return angle
    
    #動作完了待ち.
    def wait_until_tool_pose_reached(self, target_pose_stamped, pos_threshold=0.015, ori_threshold=0.15, timeout=10.0):
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            current_pose = self.get_tool_pose()
            if current_pose is None:
                self.get_logger().info('Waiting for valid tool pose...')
                time.sleep(0.1)
                continue
            
            pos_err = self.pose_distance(current_pose.pose, target_pose_stamped.pose)
            ori_err = self.orientation_distance(current_pose.pose, target_pose_stamped.pose)
            
            if pos_err < pos_threshold and ori_err < ori_threshold:
                self.get_logger().info('Target pose reached.')
                self.get_logger().info(f'Pos err: {pos_err:.4f} m, Ori err: {ori_err:.4f} rad')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().warn('Timeout waiting for tool pose to reach target.')
        self.get_logger().info(f'Pos err: {pos_err:.4f} m, Ori err: {ori_err:.4f} rad')
        return False
