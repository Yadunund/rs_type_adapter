// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <cinttypes>

#include "rs_type_adapter_example/rs_type_adapt_subscriber.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace rs_type_adapt_example
{

RsTypeAdaptIntraSub::RsTypeAdaptIntraSub(const rclcpp::NodeOptions& options)
: rclcpp::Node("image_sub_type_adapt_intra", options)
{
  sub_ = create_subscription<cv_bridge::ROSCvMatContainer>(
    "color/image_raw",
    10,
    [this](const cv_bridge::ROSCvMatContainer& container) -> void
    {
      const cv::Mat& mat = container.cv_mat();
      const auto& publish_time = rclcpp::Time(container.header().stamp);
      const auto now = this->get_clock()->now();
      const double latency = (now - publish_time).seconds();
      
      RCLCPP_INFO(
        this->get_logger(),
        "Type adapter sub accessed! glass-to-glass latency: %.6f seconds", latency      
      );
      // RCLCPP_INFO(this->get_logger(), "Received image with address: 0x%" PRIXPTR "\n", reinterpret_cast<std::uintptr_t>(msg.get()));
    });
}

RsTypeAdaptIntraSub::~RsTypeAdaptIntraSub(){}

}  // namespace rs_type_adapt_example

RCLCPP_COMPONENTS_REGISTER_NODE(rs_type_adapt_example::RsTypeAdaptIntraSub)
