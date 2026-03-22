#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from r2_interfaces.srv import GetTarget
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener


class R2GoalTest(Node):
    def __init__(self):
        super().__init__("r2_goal_test")

        # Defaults aligned to current MoveIt config
        self.declare_parameter("action_name", "move_action")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ee_link", "suction")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("plan_only", False)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("planning_attempts", 10)
        self.declare_parameter("max_velocity_scaling", 0.4)
        self.declare_parameter("max_acceleration_scaling", 0.4)

        # Vision integration (service-based)
        self.declare_parameter("use_vision", False)
        self.declare_parameter("vision_service", "/get_target_pose")
        self.declare_parameter("vision_frame", "camera")
        self.declare_parameter("vision_offset_x", 0.0)
        self.declare_parameter("vision_offset_y", 0.0)
        self.declare_parameter("vision_offset_z", 0.0)
        self.declare_parameter("min_send_interval", 1.0)

        # Competition logic: lock target once, then execute with bounded retries
        self.declare_parameter("one_shot_mode", True)
        self.declare_parameter("max_goal_attempts", 3)

        # # Target pose
        # self.declare_parameter("target_x", 0.009)
        # self.declare_parameter("target_y", 0.698)
        # self.declare_parameter("target_z", 0.478)
        # self.declare_parameter("target_qx", 0.434)
        # self.declare_parameter("target_qy", 0.558)
        # self.declare_parameter("target_qz", 0.474)
        # self.declare_parameter("target_qw", 0.525)

        # Constraints
        self.declare_parameter("use_orientation", False)
        self.declare_parameter("position_tolerance", 0.01)  # meters (sphere radius)
        self.declare_parameter("orientation_tol_x", 0.1)  # radians
        self.declare_parameter("orientation_tol_y", 0.1)
        self.declare_parameter("orientation_tol_z", 3.14)  # allow free roll by default

        self._goal_in_progress = False
        self._last_goal_time_ns = 0
        self._service_call_in_flight = False
        self._last_service_warn_ns = 0

        self._use_vision = bool(self.get_parameter("use_vision").value)
        self._one_shot_mode = bool(self.get_parameter("one_shot_mode").value)
        self._max_goal_attempts = max(1, int(self.get_parameter("max_goal_attempts").value))

        self._locked_target_pose: Optional[PoseStamped] = None
        self._attempt_count = 0
        self._mission_done = False
        self._current_goal_is_locked = False

        self._tf_buffer = None
        self._tf_listener = None
        self._vision_client = None
        self._vision_timer = None

        action_name = self.get_parameter("action_name").get_parameter_value().string_value
        self._action_client = ActionClient(self, MoveGroup, action_name)

        if self._use_vision:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            service_name = self.get_parameter("vision_service").value
            self._vision_client = self.create_client(GetTarget, service_name)
            self._vision_timer = self.create_timer(
                float(self.get_parameter("min_send_interval").value),
                self._vision_timer_cb,
            )
            self.get_logger().info(f"使用视觉服务: {service_name}")
            if self._one_shot_mode:
                self.get_logger().info(
                    f"one_shot_mode 已启用：锁定一次目标，最多尝试 {self._max_goal_attempts} 次"
                )
            else:
                self.get_logger().info("连续模式：将持续根据视觉目标进行规划")

    def _clone_pose(self, source: PoseStamped) -> PoseStamped:
        target = PoseStamped()
        target.header.frame_id = source.header.frame_id
        target.header.stamp = source.header.stamp
        target.pose.position.x = source.pose.position.x
        target.pose.position.y = source.pose.position.y
        target.pose.position.z = source.pose.position.z
        target.pose.orientation.x = source.pose.orientation.x
        target.pose.orientation.y = source.pose.orientation.y
        target.pose.orientation.z = source.pose.orientation.z
        target.pose.orientation.w = source.pose.orientation.w
        return target

    def _build_goal(self, target_pose: PoseStamped) -> MoveGroup.Goal:
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.get_parameter("group_name").value
        goal_msg.planning_options.plan_only = bool(self.get_parameter("plan_only").value)
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.allowed_planning_time = float(self.get_parameter("planning_time").value)
        goal_msg.request.num_planning_attempts = int(self.get_parameter("planning_attempts").value)
        goal_msg.request.max_velocity_scaling_factor = float(
            self.get_parameter("max_velocity_scaling").value
        )
        goal_msg.request.max_acceleration_scaling_factor = float(
            self.get_parameter("max_acceleration_scaling").value
        )

        ee_link = self.get_parameter("ee_link").value
        qx = float(self.get_parameter("target_qx").value)
        qy = float(self.get_parameter("target_qy").value)
        qz = float(self.get_parameter("target_qz").value)
        qw = float(self.get_parameter("target_qw").value)
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-6:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw

        constraints = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = target_pose.header.frame_id
        pc.link_name = ee_link
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [float(self.get_parameter("position_tolerance").value)]
        pc.constraint_region.primitives.append(s)
        pc.constraint_region.primitive_poses.append(target_pose.pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        if bool(self.get_parameter("use_orientation").value):
            oc = OrientationConstraint()
            oc.header.frame_id = target_pose.header.frame_id
            oc.link_name = ee_link
            oc.orientation = target_pose.pose.orientation
            oc.absolute_x_axis_tolerance = float(self.get_parameter("orientation_tol_x").value)
            oc.absolute_y_axis_tolerance = float(self.get_parameter("orientation_tol_y").value)
            oc.absolute_z_axis_tolerance = float(self.get_parameter("orientation_tol_z").value)
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)

        goal_msg.request.goal_constraints.append(constraints)
        return goal_msg

    def _send_move_goal(self, target_pose: PoseStamped, reason: str, count_attempt: bool):
        if self._mission_done or self._goal_in_progress:
            return

        attempt_prefix = ""
        if count_attempt:
            self._attempt_count += 1
            attempt_prefix = f"[尝试 {self._attempt_count}/{self._max_goal_attempts}] "

        goal_msg = self._build_goal(self._clone_pose(target_pose))

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            err = "MoveIt action server 未就绪，请检查 move_group 是否启动"
            self.get_logger().error(err)
            if count_attempt:
                self._handle_goal_failure(err)
            return

        self.get_logger().info(
            attempt_prefix
            + f"{reason}: "
            + "pos=("
            + f"{target_pose.pose.position.x:.4f}, "
            + f"{target_pose.pose.position.y:.4f}, "
            + f"{target_pose.pose.position.z:.4f}), "
            + f"group={goal_msg.request.group_name}, "
            + f"ee_link={self.get_parameter('ee_link').value}"
        )

        self._goal_in_progress = True
        self._current_goal_is_locked = count_attempt
        self._last_goal_time_ns = self.get_clock().now().nanoseconds
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def _handle_goal_failure(self, reason: str):
        if not (self._use_vision and self._one_shot_mode and self._locked_target_pose is not None):
            return

        if self._mission_done:
            return

        if self._attempt_count >= self._max_goal_attempts:
            self._mission_done = True
            self.get_logger().error(
                f"{reason}，已达到最大尝试次数({self._max_goal_attempts})，停止任务"
            )
            return

        self.get_logger().warning(f"{reason}，将对锁定目标进行重试")
        self._send_move_goal(self._locked_target_pose, "锁定目标重试", count_attempt=True)

    def send_goal(self):
        base_frame = self.get_parameter("base_frame").value

        target_pose = PoseStamped()
        target_pose.header.frame_id = base_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = float(self.get_parameter("target_x").value)
        target_pose.pose.position.y = float(self.get_parameter("target_y").value)
        target_pose.pose.position.z = float(self.get_parameter("target_z").value)

        self._send_move_goal(target_pose, "固定参数目标", count_attempt=False)

    def _vision_timer_cb(self):
        if self._mission_done:
            return

        if self._goal_in_progress or self._service_call_in_flight:
            return

        if self._one_shot_mode and self._locked_target_pose is not None:
            return

        now_ns = self.get_clock().now().nanoseconds
        min_interval = float(self.get_parameter("min_send_interval").value)
        if now_ns - self._last_goal_time_ns < int(min_interval * 1e9):
            return

        if not self._vision_client.wait_for_service(timeout_sec=0.0):
            if now_ns - self._last_service_warn_ns > int(5e9):
                self.get_logger().warning("视觉服务未就绪: /get_target_pose")
                self._last_service_warn_ns = now_ns
            return

        req = GetTarget.Request()
        self._service_call_in_flight = True
        future = self._vision_client.call_async(req)
        future.add_done_callback(self._vision_response_cb)

    def _vision_response_cb(self, future):
        self._service_call_in_flight = False
        if self._mission_done:
            return

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"视觉服务调用失败: {exc}")
            return

        if not response.success:
            return

        base_frame = self.get_parameter("base_frame").value
        vision_frame = self.get_parameter("vision_frame").value

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = vision_frame
        msg.point.x = float(response.x)
        msg.point.y = float(response.y)
        msg.point.z = float(response.z)

        try:
            transform = self._tf_buffer.lookup_transform(
                base_frame,
                vision_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:
            self.get_logger().warning(f"TF 变换失败: {exc}")
            return

        point_in_base = do_transform_point(msg, transform)

        dx = float(self.get_parameter("vision_offset_x").value)
        dy = float(self.get_parameter("vision_offset_y").value)
        dz = float(self.get_parameter("vision_offset_z").value)

        target_pose = PoseStamped()
        target_pose.header.frame_id = base_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = point_in_base.point.x + dx
        target_pose.pose.position.y = point_in_base.point.y + dy
        target_pose.pose.position.z = point_in_base.point.z + dz

        if self._one_shot_mode:
            if self._locked_target_pose is not None:
                return
            self._locked_target_pose = self._clone_pose(target_pose)
            if self._vision_timer is not None:
                self._vision_timer.cancel()
            self.get_logger().info(
                "已锁定视觉目标，后续不再刷新视觉数据，将基于该目标进行规划"
            )
            self._send_move_goal(self._locked_target_pose, "锁定视觉目标", count_attempt=True)
            return

        self._send_move_goal(target_pose, "视觉目标", count_attempt=False)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"发送规划请求失败: {exc}")
            self._goal_in_progress = False
            self._handle_goal_failure(f"发送规划请求失败: {exc}")
            return

        if not goal_handle.accepted:
            reason = "规划被拒绝，请检查可达性或约束设置"
            self.get_logger().error(reason)
            self._goal_in_progress = False
            self._handle_goal_failure(reason)
            return

        self.get_logger().info("规划接受，开始执行")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            result = future.result().result
            code = result.error_code.val
        except Exception as exc:
            self.get_logger().error(f"获取执行结果失败: {exc}")
            self._goal_in_progress = False
            self._handle_goal_failure(f"获取执行结果失败: {exc}")
            return

        if code == 1:
            self.get_logger().info("执行成功")
            if self._current_goal_is_locked:
                self._mission_done = True
                self.get_logger().info("one-shot任务完成，停止后续规划")
        else:
            self.get_logger().warning(f"执行失败，MoveItErrorCodes={code}")
            self._handle_goal_failure(f"执行失败，MoveItErrorCodes={code}")

        self._goal_in_progress = False


def main():
    rclpy.init()
    node = R2GoalTest()
    if not bool(node.get_parameter("use_vision").value):
        node.send_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
