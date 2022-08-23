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

#ifndef SEMANTIC_COMPONENTS__RANGE_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__RANGE_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "semantic_components/semantic_component_interface.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/Vector3"

namespace semantic_components
{
class RangeSensorDetectedPoint : public SemanticComponentInterface<geometry_msgs::msg::Vector3>
{
public:
  explicit RangeSensorDetectedPoint(const std::string & name) : SemanticComponentInterface(name, 5)
  {
    interface_names_.emplace_back(name_ + "/" + "detected_point");
    
    detected_point_ = std::numeric_limits<float>::quiet_NaN();
  }

  virtual ~RangeSensorDetectedPoint() = default;

  std::vector<float> get_detected_point(){
    size_t interface_offset = 5;
    auto arrayData = state_interfaces_[interface_offset].get().get_array_value();
    detected_point_.assign(arrayData.begin(), arrayData.end());
    return detected_point_;
  }

  bool get_values_as_message(geometry_msgs::msg::Vector3 & message)
  {
    get_detected_point();

    // update the message values
    message = detected_point_;

    return true;
  }

protected:
  // Order is: radiation_type, field_of_view, min_range, max_range, range
  std::vector<float> detected_point_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__CAMERA_SENSOR_HPP_
