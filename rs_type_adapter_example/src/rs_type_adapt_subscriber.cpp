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

#include "rs_type_adapter_example/rs_type_adapt_subscriber.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace rs_type_adapt_example
{

RsTypeAdaptIntraSub::RsTypeAdaptIntraSub(rclcpp::NodeOptions options)
: rclcpp::Node("image_sub_type_adapt_intra", options.use_intra_process_comms(true))
{
  auto callback =
    [this](std::unique_ptr<cv_type_adapt::ROSCvMatContainer> msg) -> void
    {
      (void)msg;
      RCLCPP_INFO(this->get_logger(), "Image received");
    };

  sub_ = create_subscription<cv_type_adapt::ROSCvMatContainer>("color/image_raw", 10, callback);
}

RsTypeAdaptIntraSub::~RsTypeAdaptIntraSub(){}

}  // namespace rs_type_adapt_example

RCLCPP_COMPONENTS_REGISTER_NODE(rs_type_adapt_example::RsTypeAdaptIntraSub)

