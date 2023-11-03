// Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
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

#include "hri/voice.hpp"
#include "std_msgs/msg/bool.hpp"

namespace hri
{

Voice::Voice(
  ID id,
  rclcpp::Node::SharedPtr node,
  tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: FeatureTracker{id}
  , node_(node)
  , _reference_frame(reference_frame)
  , tf_buffer_(tf_buffer)
{
}

Voice::~Voice()
{
  // executor_->cancel();
  // dedicated_listener_thread_->join();
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Deleting voice " << id_);
}

void Voice::init()
{
  rclcpp::NodeOptions node_options;

  node_options.start_parameter_event_publisher(false);
  node_options.start_parameter_services(false);
  auto node_params = node_->get_node_parameters_interface();
  auto node_topics = node_->get_node_topics_interface();
  auto qos = rclcpp::SystemDefaultsQoS();

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, true);
  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
  options.callback_group = callback_group_;

  ns_ = "/humans/voices/" + id_;
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "New voice detected: " << ns_);

  is_speaking_subscriber_ = rclcpp::create_subscription<std_msgs::msg::Bool>(
    node_params, node_topics, ns_ + "/is_speaking", qos,
    [&](std_msgs::msg::Bool::SharedPtr msg) {
      _is_speaking = msg->data;
      for (auto & cb : is_speaking_callbacks) {
        cb(msg->data);
      }
    });

  speech_subscriber_ = rclcpp::create_subscription<hri_msgs::msg::LiveSpeech>(
    node_params, node_topics, ns_ + "/speech", qos,
    bind(&Voice::_onSpeech, this, std::placeholders::_1));
}

void Voice::_onSpeech(const hri_msgs::msg::LiveSpeech::ConstSharedPtr msg)
{
  if (msg->incremental.size() > 0) {
    _incremental_speech = msg->incremental;
    for (auto cb : incremental_speech_callbacks) {
      cb(_incremental_speech);
    }
  }

  if (msg->final.size() > 0) {
    _speech = msg->final;
    for (auto cb : speech_callbacks) {
      cb(_speech);
    }
  }
}

std::optional<geometry_msgs::msg::TransformStamped> Voice::transform() const
{
  try {
    auto transform = tf_buffer_.lookupTransform(
      _reference_frame, frame(),
      tf2::TimePointZero);

    return transform;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(
      node_->get_logger(),
      "failed to transform person frame " << frame()
                                          << " to " << _reference_frame <<
        ex.what());
    return std::optional<geometry_msgs::msg::TransformStamped>();
  }
}
}  // namespace hri
