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

#ifndef SEMANTIC_COMPONENTS__TIME_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__TIME_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "semantic_components/semantic_component_interface.hpp"
//#include "sensor_msgs/msg/time_reference.hpp"
#include "std_msgs/msg/float32.hpp"

namespace semantic_components
{
class TimeSensor : public SemanticComponentInterface<std_msgs::msg::Float32>
{
public:
  explicit TimeSensor(const std::string & name) : SemanticComponentInterface(name, 1)
  {
    interface_names_.emplace_back(name_ + "/" + "seconds");
    
    // Set default values to NaN
    seconds_ = std::numeric_limits<double>::quiet_NaN();
  }

  virtual ~TimeSensor() = default;

  /// Return seconds
  /**
   * Return seconds reported by an Time
   *
   * \return seconds
   */
  double get_seconds()
  {
    size_t interface_offset = 0;
    seconds_ = state_interfaces_[interface_offset].get().get_value();
    return seconds_;
  }

  /// Return Camera message with height and width
  /**
   * Constructs and return a Camera message from the current values.
   * \return Camera message from values;
   */
  bool get_values_as_message(std_msgs::msg::Float32 & message)
  {
    // call functions to update with the latest values
    get_seconds();

    // update the message values
    message.data = seconds_;

    return true;
  }

protected:
  // Order is: seconds
  double seconds_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__CAMERA_SENSOR_HPP_
