#include "onoff_command_controller/onoff_command_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"

namespace onoff_command_controller
{
using hardware_interface::LoanedCommandInterface;

OnOffCommandController::OnOffCommandController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  enable_command_subscriber_(nullptr)
{
}

controller_interface::return_type OnOffCommandController::init(
  const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  try
  {
    //auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<bool>("enabled", false);

    auto_declare<std::string>("interface_name", "");
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn OnOffCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //auto enabled_param_ = node_->get_parameter("enabled");
  try{
    enabled_ = node_->get_parameter("enabled").as_bool();
  }
  catch(const std::exception & e){
    RCLCPP_ERROR(get_node()->get_logger(), "'enabled' parameter was empty");
    return CallbackReturn::ERROR;
  }

  // Specialized, child controllers set interfaces before calling configure function.
  if (interface_name_.empty())
  {
    interface_name_ = node_->get_parameter("interface_name").as_string();
  }

  if (interface_name_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return CallbackReturn::ERROR;
  }

  enable_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
OnOffCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // for (const auto & joint : joint_names_)
  // {
  //   command_interfaces_config.names.push_back(joint + "/" + interface_name_);
  // }
  command_interfaces_config.names.push_back(interface_name_);

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
OnOffCommandController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
// template <typename T>
// bool get_ordered_interfaces(
//   std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
//   const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
// {
//     for (auto & command_interface : unordered_interfaces)
//     {
//       if (
//         (command_interface.get_name() == joint_name) &&
//         (command_interface.get_interface_name() == interface_type))
//       {
//         ordered_interfaces.push_back(std::ref(command_interface));
//       }
//     }

//   //return joint_names.size() == ordered_interfaces.size();
//   return true;
// }

CallbackReturn OnOffCommandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  // if (
  //   !get_ordered_interfaces(
  //     command_interfaces_, joint_names_, interface_name_, ordered_interfaces) ||
  //   command_interfaces_.size() != ordered_interfaces.size())
  // {
  //   RCLCPP_ERROR(
  //     node_->get_logger(), "Expected %zu position command interfaces, got %zu", joint_names_.size(),
  //     ordered_interfaces.size());
  //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  // }

  //reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  return CallbackReturn::SUCCESS;
}

CallbackReturn OnOffCommandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type OnOffCommandController::update()
{
  auto enable_command = rt_command_ptr_.readFromRT();

  if(!enable_command){
    return controller_interface::return_type::OK;
  }else{
    return controller_interface::return_type::ERROR;
  }

  // no command received yet
  if (!enable_command || !(*enable_command))
  {
    return controller_interface::return_type::OK;
  }

  command_interfaces_[0].set_value((*enable_command)->data);

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

}  // namespace onoff_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  onoff_command_controller::OnOffCommandController, controller_interface::ControllerInterface)
