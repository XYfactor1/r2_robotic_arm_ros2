// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <r2_robotic_arm/can/socket/r2_robotic_arm.hpp>
#include <r2_robotic_arm/damiao_motor/dm_motor_constants.hpp>
#include <thread>

int main() {
    try {
        std::cout << "=== R2RoboticArm CAN Example ===" << std::endl;
        std::cout << "This example demonstrates the R2RoboticArm API functionality" << std::endl;

        // Initialize R2RoboticArm with CAN interface and enable CAN-FD
        std::cout << "Initializing R2RoboticArm CAN..." << std::endl;
        r2_robotic_arm::can::socket::R2RoboticArm r2_robotic_arm("can0", true);  // Use CAN-FD on can0 interface

        // Initialize arm motors
        std::vector<r2_robotic_arm::damiao_motor::MotorType> motor_types = {
            r2_robotic_arm::damiao_motor::MotorType::DM4310, r2_robotic_arm::damiao_motor::MotorType::DM4310};
        std::vector<uint32_t> send_can_ids = {0x01, 0x02};
        std::vector<uint32_t> recv_can_ids = {0x11, 0x12};
        r2_robotic_arm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);

        // Set callback mode to ignore and enable all motors
        r2_robotic_arm.set_callback_mode_all(r2_robotic_arm::damiao_motor::CallbackMode::IGNORE);

        // Enable all motors
        std::cout << "\n=== Enabling Motors ===" << std::endl;
        r2_robotic_arm.enable_all();
        // Allow time (2ms) for the motors to respond for slow operations like enabling
        r2_robotic_arm.recv_all(2000);

        // Set device mode to param and query motor id
        std::cout << "\n=== Querying Motor Recv IDs ===" << std::endl;
        r2_robotic_arm.set_callback_mode_all(r2_robotic_arm::damiao_motor::CallbackMode::PARAM);
        r2_robotic_arm.query_param_all(static_cast<int>(r2_robotic_arm::damiao_motor::RID::MST_ID));
        // Allow time (2ms) for the motors to respond for slow operations like querying
        // parameter from register
        r2_robotic_arm.recv_all(2000);

        // Access motors through components
        for (const auto& motor : r2_robotic_arm.get_arm().get_motors()) {
            std::cout << "Arm Motor: " << motor.get_send_can_id() << " ID: "
                      << motor.get_param(static_cast<int>(r2_robotic_arm::damiao_motor::RID::MST_ID))
                      << std::endl;
        }

        // Set device mode to state and control motor
        std::cout << "\n=== Controlling Motors ===" << std::endl;
        r2_robotic_arm.set_callback_mode_all(r2_robotic_arm::damiao_motor::CallbackMode::STATE);

        // Control arm motors with position control
        r2_robotic_arm.get_arm().mit_control_all({r2_robotic_arm::damiao_motor::MITParam{2, 1, 0, 0, 0},
                                           r2_robotic_arm::damiao_motor::MITParam{2, 1, 0, 0, 0}});
        r2_robotic_arm.recv_all(500);

        // Control arm motors with torque control
        r2_robotic_arm.get_arm().mit_control_all({r2_robotic_arm::damiao_motor::MITParam{0, 0, 0, 0, 0.1},
                                           r2_robotic_arm::damiao_motor::MITParam{0, 0, 0, 0, 0.1}});
        r2_robotic_arm.recv_all(500);

        for (int i = 0; i < 10; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            r2_robotic_arm.refresh_all();
            r2_robotic_arm.recv_all(300);

            // Display arm motor states
            for (const auto& motor : r2_robotic_arm.get_arm().get_motors()) {
                std::cout << "Arm Motor: " << motor.get_send_can_id()
                          << " position: " << motor.get_position() << std::endl;
            }
        }

        r2_robotic_arm.disable_all();
        r2_robotic_arm.recv_all(1000);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
