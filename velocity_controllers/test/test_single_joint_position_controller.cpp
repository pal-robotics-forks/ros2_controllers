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

  joint1_pos_state_handle_ = std::make_shared<hardware_interface::JointHandle>(
    "joint1",
    "position");

  ASSERT_EQ(
    test_robot_->get_joint_handle(
      *joint1_vel_cmd_handle_), hardware_interface::hardware_interface_ret_t::OK);

  ASSERT_EQ(
    test_robot_->get_joint_handle(
      *joint1_pos_state_handle_), hardware_interface::hardware_interface_ret_t::OK);
}

TEST_F(SingleJointPositionControllerTest, ConfigureTest)
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

  // configure failed, 'period' param not set
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  controller_->lifecycle_node_->declare_parameter("period", 0.0);

  // configure failed, 'period' must be positive and bigger than zero
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  auto result = controller_->lifecycle_node_->set_parameter(rclcpp::Parameter("period", 0.1));
  ASSERT_TRUE(result.successful);

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(SingleJointPositionControllerTest, CommandTest)
{
  SetUpController();
  SetUpHandles();

  // configure controller
  controller_->lifecycle_node_->declare_parameter("joint", "joint1");
  controller_->lifecycle_node_->declare_parameter("period", 0.1);
  controller_->lifecycle_node_->declare_parameter("pid.p", 1.0);
  controller_->lifecycle_node_->declare_parameter("pid.i", 0.0);
  controller_->lifecycle_node_->declare_parameter("pid.d", 0.0);
  controller_->lifecycle_node_->declare_parameter("pid.i_clamp_max", 0.0);
  controller_->lifecycle_node_->declare_parameter("pid.i_clamp_min", 0.0);

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // check default velocity command and position state
  ASSERT_EQ(joint1_vel_cmd_handle_->get_value(), 1.2);
  ASSERT_EQ(joint1_pos_state_handle_->get_value(), 1.1);

  // update successful, no command received yet
  ASSERT_EQ(controller_->update(), controller_interface::return_type::SUCCESS);

  // check velocity command and position state are unchanged
  ASSERT_EQ(joint1_vel_cmd_handle_->get_value(), 1.2);
  ASSERT_EQ(joint1_pos_state_handle_->get_value(), 1.1);

  // command joint to current position
  auto command_ptr = std::make_shared<FriendSingleJointPositionController::CmdType>();
  command_ptr->data = 1.1;
  controller_->rt_command_ptr_.writeFromNonRT(command_ptr);

  // update successful
  ASSERT_EQ(controller_->update(), controller_interface::return_type::SUCCESS);

  // velocity should be zero
  ASSERT_EQ(joint1_vel_cmd_handle_->get_value(), 0.0);

  // command joint to different position
  command_ptr->data = 2.0;
  controller_->rt_command_ptr_.writeFromNonRT(command_ptr);

  // update successful
  ASSERT_EQ(controller_->update(), controller_interface::return_type::SUCCESS);

  // velocity should greater than zero
  ASSERT_GT(joint1_vel_cmd_handle_->get_value(), 0.0);
}
