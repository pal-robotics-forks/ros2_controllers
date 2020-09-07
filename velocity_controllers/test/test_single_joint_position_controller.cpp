// Copyright 2020 PAL Robotics SL.
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

#include <stddef.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/joint_handle.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_single_joint_position_controller.hpp"
#include "test_robot_hardware/test_robot_hardware.hpp"

using CallbackReturn = velocity_controllers::SingleJointPositionController::CallbackReturn;

void SingleJointPositionControllerTest::SetUpTestCase()
{
  rclcpp::init(0, nullptr);
}

void SingleJointPositionControllerTest::TearDownTestCase()
{
  rclcpp::shutdown();
}

void SingleJointPositionControllerTest::SetUp()
{
  // initialize robot
  test_robot_ = std::make_shared<test_robot_hardware::TestRobotHardware>();
  test_robot_->init();

  // initialize controller
  controller_ = std::make_unique<FriendSingleJointPositionController>();
}

void SingleJointPositionControllerTest::TearDown()
{
  controller_.reset(nullptr);
}

void SingleJointPositionControllerTest::SetUpController()
{
  const auto result = controller_->init(test_robot_, "test_single_joint_position_controller");
  ASSERT_EQ(result, controller_interface::return_type::SUCCESS);
}

void SingleJointPositionControllerTest::SetUpHandles()
{
  // get handles from test_robot_hardware
  joint1_vel_cmd_handle_ = std::make_shared<hardware_interface::JointHandle>(
    "joint1",
    "velocity_command");

  ASSERT_EQ(
    test_robot_->get_joint_handle(
      *joint1_vel_cmd_handle_), hardware_interface::hardware_interface_ret_t::OK);
}

TEST_F(SingleJointPositionControllerTest, PidParamsTest)
{
  SetUpController();

  controller_->lifecycle_node_->declare_parameter("joint", "joint1");

  // configure failed, PID parameters not declared
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);

  controller_->lifecycle_node_->declare_parameter("pid.p", 0.0);
  controller_->lifecycle_node_->declare_parameter("pid.i", 0.0);
  controller_->lifecycle_node_->declare_parameter("pid.d", 0.0);
  controller_->lifecycle_node_->declare_parameter("pid.i_clamp_max", 0.0);
  controller_->lifecycle_node_->declare_parameter("pid.i_clamp_min", 0.0);

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}
