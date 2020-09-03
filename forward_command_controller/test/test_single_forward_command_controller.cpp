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
#include "forward_command_controller/single_forward_command_controller.hpp"
#include "hardware_interface/joint_handle.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/wait_set.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "test_single_forward_command_controller.hpp"
#include "test_robot_hardware/test_robot_hardware.hpp"

using CallbackReturn = forward_command_controller::SingleForwardCommandController::CallbackReturn;

namespace
{
rclcpp::WaitResultKind wait_for(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  const auto timeout = std::chrono::seconds(10);
  return wait_set.wait(timeout).kind();
}
}

void SingleForwardCommandControllerTest::SetUpTestCase()
{
  rclcpp::init(0, nullptr);
}

void SingleForwardCommandControllerTest::TearDownTestCase()
{
  rclcpp::shutdown();
}

void SingleForwardCommandControllerTest::SetUp()
{
  // initialize robot
  test_robot_ = std::make_shared<test_robot_hardware::TestRobotHardware>();
  test_robot_->init();

  // initialize controller
  controller_ = std::make_unique<FriendSingleForwardCommandController>();
}

void SingleForwardCommandControllerTest::TearDown()
{
  controller_.reset(nullptr);
}

void SingleForwardCommandControllerTest::SetUpController()
{
  const auto result = controller_->init(test_robot_, "single_forward_command_controller");
  ASSERT_EQ(result, controller_interface::return_type::SUCCESS);
}

void SingleForwardCommandControllerTest::SetUpHandles()
{
  // get handles from test_robot_hardware
  joint1_pos_cmd_handle_ = std::make_shared<hardware_interface::JointHandle>(
    "joint1",
    "position_command");

  ASSERT_EQ(
    test_robot_->get_joint_handle(
      *joint1_pos_cmd_handle_), hardware_interface::hardware_interface_ret_t::OK);
}

TEST_F(SingleForwardCommandControllerTest, ConfigureParamsTest)
{
  // joint handle not initialized yet
  ASSERT_FALSE(controller_->joint_cmd_handle_);

  SetUpController();

  // configure failed, 'joint' paremeter not set
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  controller_->lifecycle_node_->declare_parameter("joint", "");

  // configure failed, 'interface_name' paremeter not set
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  controller_->lifecycle_node_->declare_parameter("interface_name", "");

  // configure failed, 'joint' is empty
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  auto result = controller_->lifecycle_node_->set_parameter(rclcpp::Parameter("joint", "joint1"));
  ASSERT_TRUE(result.successful);

  // configure failed, 'interface_name' is empty
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  result = controller_->lifecycle_node_->set_parameter(
    rclcpp::Parameter(
      "interface_name",
      rclcpp::ParameterValue("position_command")));
  ASSERT_TRUE(result.successful);

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // joint handle initialized
  ASSERT_TRUE(controller_->joint_cmd_handle_);
  ASSERT_EQ(controller_->joint_cmd_handle_->get_name(), "joint1");
}

TEST_F(SingleForwardCommandControllerTest, ConfigureJointsChecksTest)
{
  // joint handle not initialized yet
  ASSERT_FALSE(controller_->joint_cmd_handle_);

  SetUpController();

  controller_->lifecycle_node_->declare_parameter("joint", "joint4");
  controller_->lifecycle_node_->declare_parameter("interface_name", "acceleration_command");

  // configure failed, 'joint4' not in robot_hardware
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  auto result = controller_->lifecycle_node_->set_parameter(
    rclcpp::Parameter("joint", "joint1"));
  ASSERT_TRUE(result.successful);

  // configure failed, 'joint1' does not support 'acceleration_command' interface
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  result = controller_->lifecycle_node_->set_parameter(
    rclcpp::Parameter(
      "interface_name",
      rclcpp::ParameterValue("position_command")));
  ASSERT_TRUE(result.successful);

  // configure successful
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // joint handle initialized
  ASSERT_TRUE(controller_->joint_cmd_handle_);
  ASSERT_EQ(controller_->joint_cmd_handle_->get_name(), "joint1");
}

TEST_F(SingleForwardCommandControllerTest, CommandSuccessTest)
{
  SetUpController();
  SetUpHandles();

  // configure controller
  controller_->lifecycle_node_->declare_parameter("joint", "joint1");
  controller_->lifecycle_node_->declare_parameter("interface_name", "position_command");
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // update successful though no command has been send yet
  ASSERT_EQ(controller_->update(), controller_interface::return_type::SUCCESS);

  // check joint command is still the default one
  ASSERT_EQ(joint1_pos_cmd_handle_->get_value(), 1.1);

  // send command
  FriendSingleForwardCommandController::CmdType::SharedPtr command_ptr =
    std::make_shared<FriendSingleForwardCommandController::CmdType>();
  command_ptr->data = 10.0;
  controller_->rt_command_ptr_.writeFromNonRT(command_ptr);

  // update successful, command received
  ASSERT_EQ(controller_->update(), controller_interface::return_type::SUCCESS);

  // check joint command has been modified
  ASSERT_EQ(joint1_pos_cmd_handle_->get_value(), 10.0);
}

TEST_F(SingleForwardCommandControllerTest, NoCommandCheckTest)
{
  SetUpController();
  SetUpHandles();

  // configure controller
  controller_->lifecycle_node_->declare_parameter("joint", "joint1");
  controller_->lifecycle_node_->declare_parameter("interface_name", "position_command");
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // update successful, no command received yet
  ASSERT_EQ(controller_->update(), controller_interface::return_type::SUCCESS);

  // check joint command is still the default one
  ASSERT_EQ(joint1_pos_cmd_handle_->get_value(), 1.1);
}

TEST_F(SingleForwardCommandControllerTest, CommandCallbackTest)
{
  SetUpController();
  SetUpHandles();

  // configure controller
  controller_->lifecycle_node_->declare_parameter("joint", "joint1");
  controller_->lifecycle_node_->declare_parameter("interface_name", "position_command");

  // default value
  ASSERT_EQ(joint1_pos_cmd_handle_->get_value(), 1.1);

  auto node_state = controller_->get_lifecycle_node()->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = controller_->get_lifecycle_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // send a new command
  rclcpp::Node test_node("test_node");
  auto command_pub = test_node.create_publisher<std_msgs::msg::Float64>(
    "command", rclcpp::SystemDefaultsQoS());
  std_msgs::msg::Float64 command_msg;
  command_msg.data = 10.0;
  command_pub->publish(command_msg);

  // wait for command message to be passed
  ASSERT_EQ(wait_for(controller_->joint_command_subscriber_), rclcpp::WaitResultKind::Ready);

  // process callbacks
  rclcpp::spin_some(controller_->get_lifecycle_node()->get_node_base_interface());

  // update successful
  ASSERT_EQ(controller_->update(), controller_interface::return_type::SUCCESS);

  // check command in handle was set
  ASSERT_EQ(joint1_pos_cmd_handle_->get_value(), 10.0);
}
