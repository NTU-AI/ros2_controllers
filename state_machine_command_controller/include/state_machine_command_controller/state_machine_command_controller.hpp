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

#ifndef STATE_MACHINE_COMMAND_CONTROLLER__STATE_MACHINE_COMMAND_CONTROLLER_HPP_
#define STATE_MACHINE_COMMAND_CONTROLLER__STATE_MACHINE_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "state_machine_command_controller/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/int32.hpp"

namespace state_machine_command_controller
{
using CmdType = std_msgs::msg::Int32;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * \brief State machine command controller for a set of joints.
 *
 * This class forwards the command signal down to a set of joints
 * on the specified interface.
 *
 * \param joints Names of the joints to control.
 * \param interface_name Name of the interface to command.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Int32) : The commands to apply.
 */
class StateMachineCommandController : public controller_interface::ControllerInterface
{
public:
  STATE_MACHINE_COMMAND_CONTROLLER_PUBLIC
  StateMachineCommandController();

  STATE_MACHINE_COMMAND_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  STATE_MACHINE_COMMAND_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  STATE_MACHINE_COMMAND_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  STATE_MACHINE_COMMAND_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  STATE_MACHINE_COMMAND_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  STATE_MACHINE_COMMAND_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  STATE_MACHINE_COMMAND_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

protected:
  int state_;
  std::string actuator_name_;
  std::string interface_name_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr state_command_subscriber_;

  std::string logger_name_;
};

}  // namespace state_machine_command_controller

#endif  // STATE_MACHINE_COMMAND_CONTROLLER__STATE_MACHINE_COMMAND_CONTROLLER_HPP_
