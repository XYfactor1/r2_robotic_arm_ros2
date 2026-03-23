#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class R2PurePosition(Node):
    def __init__(self):
        super().__init__('r2_pure_position')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.target_x = -0.016
        self.target_y = 0.692
        self.target_z = 0.259

    def send_goal(self):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'r2_arm'
        goal_msg.planning_options.plan_only = False
        goal_msg.request.start_state.is_diff = True
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = self.target_x
        target_pose.pose.position.y = self.target_y
        target_pose.pose.position.z = self.target_z

        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'suction' 
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.01]  # 放宽到 5 厘米误差，闭着眼都能算出来
        pc.constraint_region.primitives.append(s)
        pc.constraint_region.primitive_poses.append(target_pose.pose)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'suction'
        
        oc.orientation.x = -0.046 
        oc.orientation.y = 0.706
        oc.orientation.z = 0.031
        oc.orientation.w = 0.706

        oc.absolute_x_axis_tolerance = 0.01 # Yaw 容差 (约 5.7 度)
        oc.absolute_z_axis_tolerance = 0.01 # Pitch 容差
        oc.absolute_y_axis_tolerance = 3.14 # Roll 容差 (放开自转)
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)

        goal_msg.request.goal_constraints.append(c)
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        self.get_logger().info('等待 MoveIt 服务器...')
        self._action_client.wait_for_server()
        
        self.get_logger().info(f'🚀 发送纯位置指令: X={self.target_x}, Y={self.target_y}, Z={self.target_z}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 规划被拒绝！(坐标依旧不可达)')
            return
        self.get_logger().info('✅ 规划成功，机械臂开始运动...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('🎉 完美到达 XYZ 位置！纯位置 IK 测试成功！')
        else:
            self.get_logger().warn(f'⚠️ 执行失败，错误码: {result.error_code.val}')

def main():
    rclpy.init()
    node = R2PurePosition()
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()