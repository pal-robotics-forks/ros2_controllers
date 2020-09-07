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
#include "test_joint_effort_controller.hpp"
#include "test_robot_hardware/test_robot_hardware.hpp"

using CallbackReturn = effort_controllers::JointEffortController::CallbackReturn;

void JointEffortControllerTest::SetUpTestCase()
{
  rclcpp::init(0, nullptr);
}

void JointEffortControllerTest::TearDownTestCase()
{
  rclcpp::shutdown();
}

void JointEffortControllerTest::SetUp()
{
  // initialize robot
  test_robot_ = std::make_shared<test_robot_hardware::TestRobotHardware>();
  test_robot_->init();

  // initialize controller
  controller_ = std::make_unique<FriendJointEffortController>();
}

void JointEffortControllerTest::TearDown()
{
  controller_.reset(nullptr);
}

void JointEffortControllerTest::SetUpController()
{
  const auto result = controller_->init(test_robot_, "test_joint_effort_controller");
  ASSERT_EQ(result, controller_interface::return_type::SUCCESS);
}

void JointEffortControllerTest::SetUpHandles()
{
  // get handles from test_robot_hardware
  joint1_eff_cmd_handle_ = std::make_shared<hardware_interface::JointHandle>(
    "joint1",
    "effort_command");

  ASSERT_EQ(
    test_robot_->get_joint_handle(
      *joint1_eff_cmd_handle_), hardware_interface::hardware_interface_ret_t::OK);
}

TEST_F(JointEffortControllerTest, ConfigureParamsTest)
{
  // joint handle not initialized yet
  ASSERT_FALSE(controller_->joint_cmd_handle_);

  SetUpController();

  // configure failed, 'joint' paremeter not set
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  controller_->lifecycle_node_->declare_parameter("joint", "");

  // configure failed, 'joint' is empty
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  auto result = controller_->lifecycle_node_->set_parameter(rclcpp::Parameter("joint", "joint1"));
  ASSERT_TRUE(result.successful);

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // joint handle initialized
  ASSERT_TRUE(controller_->joint_cmd_handle_);
  ASSERT_EQ(controller_->joint_cmd_handle_->get_name(), "joint1");
}

TEST_F(JointEffortControllerTest, CheckParamsTest)
{
  // joint handle not initialized yet
  ASSERT_FALSE(controller_->joint_cmd_handle_);

  SetUpController();

  // configure failed, interface name has already been set, with the wrong interface
  controller_->lifecycle_node_->declare_parameter("interface_name", "position_command");
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);

  auto result = controller_->lifecycle_node_->set_parameter(
    rclcpp::Parameter(
      "interface_name",
      rclcpp::ParameterValue("effort_command")));
  ASSERT_TRUE(result.successful);

  controller_->lifecycle_node_->declare_parameter("joint", "joint1");

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // joint handle initialized
  ASSERT_TRUE(controller_->joint_cmd_handle_);
  ASSERT_EQ(controller_->joint_cmd_handle_->get_name(), "joint1");
}

TEST_F(JointEffortControllerTest, StopJointOnDeactivateTest)
{
  SetUpController();
  SetUpHandles();

  controller_->lifecycle_node_->declare_parameter("joint", "joint1");

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // check joint commands are still the default ones
  ASSERT_EQ(joint1_eff_cmd_handle_->get_value(), 1.3);

  // stop the controller
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // check joint commands are now zero
  ASSERT_EQ(joint1_eff_cmd_handle_->get_value(), 0.0);
}
