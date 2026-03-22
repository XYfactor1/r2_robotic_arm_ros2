#include "r2_robotic_arm_hardware/r2_robotic_arm_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace r2_robotic_arm_hardware {

namespace {
constexpr double kTwoPi = 6.283185307179586;
}  // namespace

R2RoboticArmHardware::R2RoboticArmHardware() = default;

bool R2RoboticArmHardware::parse_config(const hardware_interface::HardwareInfo &info)
{
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  it = info.hardware_parameters.find("can_fd");
  can_fd_ = (it != info.hardware_parameters.end()) ?
    (std::string(it->second) == "true" || std::string(it->second) == "True") :
    true;

  for (size_t i = 1; i <= ARM_DOF; ++i) {
    it = info.hardware_parameters.find("kp" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kp_[i - 1] = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kd_[i - 1] = std::stod(it->second);
    }
  }

  it = info.hardware_parameters.find("joint3_lead_m");
  if (it != info.hardware_parameters.end()) {
    joint3_lead_m_ = std::stod(it->second);
  }

  it = info.hardware_parameters.find("joint3_gear_ratio");
  if (it != info.hardware_parameters.end()) {
    joint3_gear_ratio_ = std::stod(it->second);
  }

  if (joint3_lead_m_ <= 0.0 || joint3_gear_ratio_ <= 0.0) {
    RCLCPP_ERROR(
        rclcpp::get_logger("R2RoboticArmHardware"),
        "Invalid joint_3 transmission params: lead_m=%.6f, gear_ratio=%.6f",
        joint3_lead_m_, joint3_gear_ratio_);
    return false;
  }
  joint3_rad_to_m_ = joint3_lead_m_ / (kTwoPi * joint3_gear_ratio_);
  if (joint3_rad_to_m_ <= 0.0) {
    RCLCPP_ERROR(
        rclcpp::get_logger("R2RoboticArmHardware"),
        "Computed joint_3 rad_to_m conversion is invalid: %.12f",
        joint3_rad_to_m_);
    return false;
  }

  RCLCPP_INFO(
      rclcpp::get_logger("R2RoboticArmHardware"),
      "joint_3 transmission: lead=%.6f m/rev, gear_ratio=%.6f, rad_to_m=%.12f",
      joint3_lead_m_, joint3_gear_ratio_, joint3_rad_to_m_);

  return true;
}

void R2RoboticArmHardware::generate_joint_names()
{
  joint_names_.clear();
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    joint_names_.push_back("joint_" + std::to_string(i));
  }
}

hardware_interface::CallbackReturn R2RoboticArmHardware::on_init(
    const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  if (!parse_config(info))
  {
    return CallbackReturn::ERROR;
  }

  generate_joint_names();

  if (joint_names_.size() != ARM_DOF)
  {
    RCLCPP_ERROR(rclcpp::get_logger("R2RoboticArmHardware"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 ARM_DOF);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("R2RoboticArmHardware"),
              "Initializing R2RoboticArm on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  r2_robotic_arm_ =
      std::make_unique<r2_robotic_arm::can::socket::R2RoboticArm>(can_interface_, can_fd_);

  r2_robotic_arm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
                                   DEFAULT_RECV_CAN_IDS);

  const size_t total_joints = joint_names_.size();
  pos_commands_.assign(total_joints, 0.0);
  vel_commands_.assign(total_joints, 0.0);
  tau_commands_.assign(total_joints, 0.0);
  pos_states_.assign(total_joints, 0.0);
  vel_states_.assign(total_joints, 0.0);
  tau_states_.assign(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("R2RoboticArmHardware"),
              "R2 Robotic Arm Hardware initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R2RoboticArmHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  r2_robotic_arm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  r2_robotic_arm_->recv_all();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
R2RoboticArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
R2RoboticArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn R2RoboticArmHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("R2RoboticArmHardware"), "Activating R2 Robotic Arm Hardware...");
  r2_robotic_arm_->get_arm().set_control_mode_all(r2_robotic_arm::damiao_motor::ControlMode::MIT);
  r2_robotic_arm_->set_callback_mode_all(r2_robotic_arm::damiao_motor::CallbackMode::STATE);
  r2_robotic_arm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  r2_robotic_arm_->recv_all();

  enable_motors();

  RCLCPP_INFO(rclcpp::get_logger("R2RoboticArmHardware"), "R2 Robotic Arm Hardware activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R2RoboticArmHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("R2RoboticArmHardware"),
              "Deactivating R2 Robotic Arm Hardware...");

  r2_robotic_arm_->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  r2_robotic_arm_->recv_all();

  RCLCPP_INFO(rclcpp::get_logger("R2RoboticArmHardware"), "R2 Robotic Arm Hardware deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type R2RoboticArmHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  r2_robotic_arm_->refresh_all();
  r2_robotic_arm_->recv_all();

  const auto &arm_motors = r2_robotic_arm_->get_arm().get_motors();

  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i)
  {
    if (i == PRISMATIC_JOINT_INDEX) {
      pos_states_[i] = arm_motors[i].get_position() * joint3_rad_to_m_;
      vel_states_[i] = arm_motors[i].get_velocity() * joint3_rad_to_m_;
    } else {
      pos_states_[i] = arm_motors[i].get_position();
      vel_states_[i] = arm_motors[i].get_velocity();
    }
    // tau_states_[i] = arm_motors[i].get_torque();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type R2RoboticArmHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<r2_robotic_arm::damiao_motor::MITParam> arm_params;
  arm_params.reserve(ARM_DOF);

  for (size_t i = 0; i < ARM_DOF; ++i)
  {
    if (i == PRISMATIC_JOINT_INDEX) {
      const double motor_pos = pos_commands_[i] / joint3_rad_to_m_;
      const double motor_vel = vel_commands_[i] / joint3_rad_to_m_;
      arm_params.push_back({kp_[i], kd_[i], motor_pos, motor_vel, tau_commands_[i]});
    } else {
      arm_params.push_back({kp_[i], kd_[i], pos_commands_[i], vel_commands_[i], tau_commands_[i]});
    }
  }
  r2_robotic_arm_->get_arm().mit_control_all(arm_params);
  r2_robotic_arm_->recv_all(1000);
  return hardware_interface::return_type::OK;
}

void R2RoboticArmHardware::enable_motors()
{
  RCLCPP_INFO(rclcpp::get_logger("R2RoboticArmHardware"),
              "Reading current positions and enabling in place...");
  const auto &arm_motors = r2_robotic_arm_->get_arm().get_motors();

  std::vector<r2_robotic_arm::damiao_motor::MITParam> arm_params;
  arm_params.reserve(ARM_DOF);

  for (size_t i = 0; i < ARM_DOF; ++i)
  {
    double current_pos = 0.0;
    if (i < arm_motors.size())
    {
      current_pos = arm_motors[i].get_position();
    }

    if (i == PRISMATIC_JOINT_INDEX) {
      const double joint_pos = current_pos * joint3_rad_to_m_;
      pos_states_[i] = joint_pos;
      pos_commands_[i] = joint_pos;
      arm_params.push_back({kp_[i], kd_[i], current_pos, 0.0, 0.0});
    } else {
      pos_states_[i] = current_pos;
      pos_commands_[i] = current_pos;
      arm_params.push_back({kp_[i], kd_[i], current_pos, 0.0, 0.0});
    }
  }
  r2_robotic_arm_->get_arm().mit_control_all(arm_params);

  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  r2_robotic_arm_->recv_all();
}

}  // namespace r2_robotic_arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(r2_robotic_arm_hardware::R2RoboticArmHardware,
                       hardware_interface::SystemInterface)
