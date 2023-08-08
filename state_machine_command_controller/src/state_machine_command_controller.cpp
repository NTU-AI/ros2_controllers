// Copyright 2021 PAL Robotics SL.
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

/*
 * Authors: Subhas Das, Denis Stogl, Victor Lopez
 */

#include "state_machine_command_controller/state_machine_command_controller.hpp"
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"

namespace state_machine_command_controller
{
  using hardware_interface::LoanedCommandInterface;

  StateMachineCommandController::StateMachineCommandController()
  : controller_interface::ControllerInterface(),
    rt_command_ptr_(nullptr),
    state_command_subscriber_(nullptr)
  {
  }

  controller_interface::CallbackReturn StateMachineCommandController::on_init()
  {   
    try
    {
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn StateMachineCommandController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    
    try
    {
      state_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/state", rclcpp::SystemDefaultsQoS(),
      [this](const CmdType::SharedPtr msg) { 
        this->rt_command_ptr_.writeFromNonRT(msg); 
      }
      );
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'state' parameter was empty");
      return CallbackReturn::ERROR;
    }
    RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration StateMachineCommandController::command_interface_configuration()
    const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;

    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interfaces_config.names.push_back(this->params_.actuator_name+"/"+this->params_.interface_name);
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration StateMachineCommandController::state_interface_configuration()
    const
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::CallbackReturn StateMachineCommandController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++)
    {
      std::cout << si->get_name() << std::endl;
      std::cout << si->get_interface_name() << std::endl;
    }
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn StateMachineCommandController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {

    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type StateMachineCommandController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    auto state_command = this->rt_command_ptr_.readFromRT();
    if((*state_command) == nullptr)
    {
      return controller_interface::return_type::OK;
    }
    else
    {
      command_interfaces_[0].set_value((double)(*state_command)->data);
    }
    return controller_interface::return_type::OK;
  }

}  

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  state_machine_command_controller::StateMachineCommandController, controller_interface::ControllerInterface)
