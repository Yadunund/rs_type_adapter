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

#ifndef TYPE_ADAPT_EXAMPLE__IMAGE_SUB_TYPE_ADAPT_INTRA_NODE_HPP_
#define TYPE_ADAPT_EXAMPLE__IMAGE_SUB_TYPE_ADAPT_INTRA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/visibility_control.h>

namespace rs_type_adapt_example
{

class RsIntraSub : public rclcpp::Node
{
public:
  CV_BRIDGE_PUBLIC
  explicit RsIntraSub(rclcpp::NodeOptions options);

  virtual ~RsIntraSub();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace type_adapt_example

#endif  // TYPE_ADAPT_EXAMPLE__IMAGE_SUB_TYPE_ADAPT_INTRA_NODE_HPP_
