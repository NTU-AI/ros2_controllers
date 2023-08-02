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

controller_interface::return_type StateMachineCommandController::init(
  const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  try
  {
    auto_declare<int>("state", 0);

    auto_declare<std::string>("interface_name", "");
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn StateMachineCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try{
    state_ = node_->get_parameter("state").as_int();
  }
  catch(const std::exception & e){
    RCLCPP_ERROR(get_node()->get_logger(), "'state' parameter was empty");
    return CallbackReturn::ERROR;
  }

  // Specialized, child controllers set interfaces before calling configure function.
  node_->set_parameter({"interface_name", "state"});
  // if (interface_name_.empty())
  // {
  this->actuator_name_ = node_->get_parameter("actuator_name").as_string();
  this->interface_name_ = node_->get_parameter("interface_name").as_string();
  // }

  // if (interface_name_.empty())
  // {
  //   RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
  //   return CallbackReturn::ERROR;
  // }

  state_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/state", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { 
      this->rt_command_ptr_.writeFromNonRT(msg); 
    }
    );

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
StateMachineCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  //for (const auto & joint : joint_names_)
  //{
  
  command_interfaces_config.names.push_back(this->actuator_name_+"/"+this->interface_name_);
  //}

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
StateMachineCommandController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
// template <typename T>
// bool get_ordered_interfaces(
//   std::vector<T> & unordered_interfaces,
//   const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
// {
//   //for (const auto & joint_name : joint_names)
//   //{
//   for (auto & command_interface : unordered_interfaces)
//   {
//     if ((command_interface.get_interface_name() == interface_type))
//     {
//       ordered_interfaces.push_back(std::ref(command_interface));
//     }
//   }

//   return true;
// }

CallbackReturn StateMachineCommandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++)
  {
    std::cout << si->get_name() << std::endl;
    std::cout << si->get_interface_name() << std::endl;
  }
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  // if (!get_ordered_interfaces(command_interfaces_, interface_name_, ordered_interfaces) ||  command_interfaces_.size() != ordered_interfaces.size())
  // {
  //   //RCLCPP_ERROR(
  //   //  node_->get_logger(), "Expected %zu position command interfaces, got %zu", joint_names_.size(),
  //   //  ordered_interfaces.size());
  //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  // }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  return CallbackReturn::SUCCESS;
}

CallbackReturn StateMachineCommandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type StateMachineCommandController::update()
{
  auto state_command = this->rt_command_ptr_.readFromRT();

  //std::cout << (*state_command)->data << std::endl;

  if((*state_command) == nullptr){
    return controller_interface::return_type::OK;
  }else{
    //std::cout << "recebi" << std::endl;

    //std::cout << "TESTE: " << (*state_command)->data << std::endl;
    command_interfaces_[0].set_value((double)(*state_command)->data);
  }

 

  // if ((*enable_commands)->data.size() != command_interfaces_.size())
  // {
  //   RCLCPP_ERROR_THROTTLE(
  //     get_node()->get_logger(), *node_->get_clock(), 1000,
  //     "command size (%zu) does not match number of interfaces (%zu)",
  //     (*enable_commands)->data.size(), command_interfaces_.size());
  //   return controller_interface::return_type::ERROR;
  // }

  // for (auto index = 0ul; index < command_interfaces_.size(); ++index)
  // {
  //   command_interfaces_[index].set_value((*enable_commands)->data[index]);
  // }

  return controller_interface::return_type::OK;
}

}  // namespace state_machine_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  state_machine_command_controller::StateMachineCommandController, controller_interface::ControllerInterface)
