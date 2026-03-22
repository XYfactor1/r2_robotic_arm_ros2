#pragma once

#include <chrono>
#include <memory>
#include <r2_robotic_arm/can/socket/r2_robotic_arm.hpp>
#include <r2_robotic_arm/damiao_motor/dm_motor_constants.hpp>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "r2_robotic_arm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace r2_robotic_arm_hardware {

class R2RoboticArmHardware : public hardware_interface::SystemInterface {
public:
  R2RoboticArmHardware();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // 5-DOF arm
  static constexpr size_t ARM_DOF = 5;
  static constexpr size_t PRISMATIC_JOINT_INDEX = 2;  // joint_3

  // Motor configuration (按你的型号顺序)
  const std::vector<r2_robotic_arm::damiao_motor::MotorType> DEFAULT_MOTOR_TYPES = {
      r2_robotic_arm::damiao_motor::MotorType::DM8009,  // Joint 1
      r2_robotic_arm::damiao_motor::MotorType::DM6248,  // Joint 2
      r2_robotic_arm::damiao_motor::MotorType::DM3519,  // Joint 3
      r2_robotic_arm::damiao_motor::MotorType::DM4340,  // Joint 4
      r2_robotic_arm::damiao_motor::MotorType::DM4310,  // Joint 5
  };

  const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x01, 0x02, 0x03, 0x04,
                                                      0x05};
  const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x11, 0x12, 0x13, 0x14,
                                                      0x15};

  // Gains (可在 URDF 中通过 kp1..kp5 / kd1..kd5 覆盖)
  std::vector<double> kp_ = {70.0, 70.0, 70.0, 60.0, 10.0};
  std::vector<double> kd_ = {2.75, 2.5, 2.0, 2.0, 0.7};

  // Configuration
  std::string can_interface_;
  bool can_fd_;

  // joint_3 transmission (screw lead in meters per screw revolution,
  // gear ratio defined as motor_turns / screw_turns)
  double joint3_lead_m_ = 0.012;
  double joint3_gear_ratio_ = 1.0;
  double joint3_rad_to_m_ = 1.0;

  // r2_robotic_arm instance
  std::unique_ptr<r2_robotic_arm::can::socket::R2RoboticArm> r2_robotic_arm_;

  // Joint names
  std::vector<std::string> joint_names_;

  // ROS2 control state and command vectors
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  void enable_motors();
  bool parse_config(const hardware_interface::HardwareInfo& info);
  void generate_joint_names();
};

}  // namespace r2_robotic_arm_hardware
