# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(r2_robotic_arm_can VERSION 1.2.8)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Testing support
include(CTest)

include(GNUInstallDirs)

# Create the main library
add_library(
  r2_robotic_arm_can
  src/r2_robotic_arm/can/socket/arm_component.cpp
  src/r2_robotic_arm/can/socket/r2_robotic_arm.cpp
  src/r2_robotic_arm/canbus/can_device_collection.cpp
  src/r2_robotic_arm/canbus/can_socket.cpp
  src/r2_robotic_arm/damiao_motor/dm_motor.cpp
  src/r2_robotic_arm/damiao_motor/dm_motor_control.cpp
  src/r2_robotic_arm/damiao_motor/dm_motor_device.cpp
  src/r2_robotic_arm/damiao_motor/dm_motor_device_collection.cpp)
set_target_properties(
  r2_robotic_arm_can
  PROPERTIES POSITION_INDEPENDENT_CODE ON
             VERSION ${PROJECT_VERSION}
             SOVERSION ${PROJECT_VERSION_MAJOR})
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.23)
  target_sources(
    r2_robotic_arm_can
    PUBLIC FILE_SET
           HEADERS
           TYPE
           HEADERS
           BASE_DIRS
           include
           FILES
           include/r2_robotic_arm/can/socket/arm_component.hpp
           include/r2_robotic_arm/can/socket/r2_robotic_arm.hpp
           include/r2_robotic_arm/canbus/can_device.hpp
           include/r2_robotic_arm/canbus/can_device_collection.hpp
           include/r2_robotic_arm/canbus/can_socket.hpp
           include/r2_robotic_arm/damiao_motor/dm_motor.hpp
           include/r2_robotic_arm/damiao_motor/dm_motor_constants.hpp
           include/r2_robotic_arm/damiao_motor/dm_motor_control.hpp
           include/r2_robotic_arm/damiao_motor/dm_motor_device.hpp
           include/r2_robotic_arm/damiao_motor/dm_motor_device_collection.hpp)
  install(
    TARGETS r2_robotic_arm_can
    EXPORT r2_robotic_arm_can_export
    FILE_SET HEADERS)
else()
  target_include_directories(
    r2_robotic_arm_can PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
  install(TARGETS r2_robotic_arm_can EXPORT r2_robotic_arm_can_export)
  install(DIRECTORY include/r2_robotic_arm TYPE INCLUDE)
endif()

# CMake package
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/R2RoboticArmCAN)
install(
  EXPORT r2_robotic_arm_can_export
  DESTINATION ${INSTALL_CMAKE_DIR}
  NAMESPACE R2RoboticArmCAN::
  FILE R2RoboticArmCANTargets.cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  R2RoboticArmCANConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/R2RoboticArmCANConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_DIR})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/R2RoboticArmCANConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/R2RoboticArmCANConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/R2RoboticArmCANConfigVersion.cmake
        DESTINATION ${INSTALL_CMAKE_DIR})

# pkg-config package
if(IS_ABSOLUTE "${CMAKE_INSTALL_INCLUDEDIR}")
  set(PKG_CONFIG_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}")
else()
  set(PKG_CONFIG_INCLUDEDIR "\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}")
endif()
if(IS_ABSOLUTE "${CMAKE_INSTALL_LIBDIR}")
  set(PKG_CONFIG_LIBDIR "${CMAKE_INSTALL_LIBDIR}")
else()
  set(PKG_CONFIG_LIBDIR "\${prefix}/${CMAKE_INSTALL_LIBDIR}")
endif()
configure_file(r2_robotic_arm-can.pc.in ${CMAKE_CURRENT_BINARY_DIR}/r2_robotic_arm-can.pc
               @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/r2_robotic_arm-can.pc
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

add_executable(r2_robotic_arm-can-motor-check setup/motor_check.cpp)
target_link_libraries(r2_robotic_arm-can-motor-check r2_robotic_arm_can)
install(TARGETS r2_robotic_arm-can-motor-check DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(r2_robotic_arm-can-diagnosis setup/r2_robotic_arm_can_diagnosis.cpp)
target_link_libraries(r2_robotic_arm-can-diagnosis r2_robotic_arm_can)
install(TARGETS r2_robotic_arm-can-diagnosis DESTINATION ${CMAKE_INSTALL_BINDIR})

# Add motor control example executable
add_executable(r2_robotic_arm-can-demo examples/demo.cpp)
target_link_libraries(r2_robotic_arm-can-demo r2_robotic_arm_can)
install(TARGETS r2_robotic_arm-can-demo DESTINATION ${CMAKE_INSTALL_BINDIR})

# Install scripts
install(
  PROGRAMS setup/r2_robotic_arm-can-change-baudrate
           setup/r2_robotic_arm-can-configure-socketcan
           setup/r2_robotic_arm-can-configure-socketcan-4-arms
           setup/r2_robotic_arm-can-set-zero
           setup/r2_robotic_arm-can-zero-position-calibration
  DESTINATION ${CMAKE_INSTALL_BINDIR})

# Add tests
if(BUILD_TESTING)
  # add_subdirectory(test)
endif()
