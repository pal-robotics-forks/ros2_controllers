// Copyright 2020 PAL Robotics S.L.
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

#ifndef VELOCITY_CONTROLLERS__SINGLE_JOINT_POSITION_CONTROLLER_HPP_
#define VELOCITY_CONTROLLERS__SINGLE_JOINT_POSITION_CONTROLLER_HPP_

#include <memory>

#include "velocity_controllers/single_joint_velocity_controller.hpp"
#include "velocity_controllers/visibility_control.h"

// forward declaration
namespace control_toolbox
{
class PidROS;
}

namespace velocity_controllers
{

/**
 * \brief TODO
 *
 * This class forwards the computed velocity down to a joint.
 *
 * \param joint Name of the joint to control.
 * \param pid Contains the gains for the PID loop around position.
 *
 * Subscribes to:
 * - \b command (std_msgs::msg::Float64) : The position command to apply.
 */
class SingleJointPositionController : public SingleJointVelocityController
{
public:
  VELOCITY_CONTROLLERS_PUBLIC
  SingleJointPositionController();

  VELOCITY_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::shared_ptr<control_toolbox::PidROS> pid_;
};

}  // namespace velocity_controllers

#endif  // VELOCITY_CONTROLLERS__SINGLE_JOINT_POSITION_CONTROLLER_HPP_
