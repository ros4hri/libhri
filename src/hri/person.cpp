// Copyright 2022 PAL Robotics
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the PAL Robotics S.L. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <geometry_msgs/msg/transform_stamped.hpp>
#include "hri/person.hpp"
#include "hri/hri.hpp"
#include "hri_msgs/msg/engagement_level.hpp"
#include <std_msgs/msg/float32.hpp>

namespace hri
{

Person::Person(
  ID id,
  rclcpp::Node::SharedPtr node,
  const HRIListener * listener,
  tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: FeatureTracker{id}
  , listener_(listener)
  , _alias("")
  , _engagement_status(nullptr)
  , _loc_confidence(0.)
  , _reference_frame(reference_frame)
  , node_(node)
  , tf_buffer_(tf_buffer)
{
}


Person::~Person()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Deleting person " << id_);
  executor_->cancel();
  dedicated_listener_thread_->join();
}

void Person::init()
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

  ns_ = "/humans/persons/" + id_;
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "New person detected: " << ns_);

  face_id_subscriber_ = rclcpp::create_subscription<std_msgs::msg::String>(
    node_params, node_topics, ns_ + "/face_id", qos, [&](
      const std_msgs::msg::String::SharedPtr msg) {
      face_id = msg->data;
    }, options);

  body_id_subscriber_ = rclcpp::create_subscription<std_msgs::msg::String>(
    node_params, node_topics, ns_ + "/body_id", qos, [&](
      const std_msgs::msg::String::SharedPtr msg) {
      body_id = msg->data;
    }, options);

  voice_id_subscriber_ = rclcpp::create_subscription<std_msgs::msg::String>(
    node_params, node_topics, ns_ + "/voice_id", qos,
    [&](const std_msgs::msg::String::SharedPtr msg) {voice_id = msg->data;}, options);

  anonymous_subscriber_ = rclcpp::create_subscription<std_msgs::msg::Bool>(
    node_params, node_topics, ns_ + "/anonymous", qos,
    [&](const std_msgs::msg::Bool::SharedPtr msg) {_anonymous = msg->data;}, options);

  alias_subscriber_ = rclcpp::create_subscription<std_msgs::msg::String>(
    node_params, node_topics, ns_ + "/alias", qos, [&](const std_msgs::msg::String::SharedPtr msg) {
      _alias = msg->data;
    }, options);

  engagement_subscriber_ = rclcpp::create_subscription<hri_msgs::msg::EngagementLevel>(
    node_params, node_topics, ns_ + "/engagement_status", qos,
    [&](const hri_msgs::msg::EngagementLevel::SharedPtr msg) {_engagement_status = msg;}, options);

  loc_confidence_subscriber_ = rclcpp::create_subscription<std_msgs::msg::Float32>(
    node_params, node_topics, ns_ + "/location_confidence", qos,
    [&](const std_msgs::msg::Float32::SharedPtr msg) {_loc_confidence = msg->data;}, options);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, node_->get_node_base_interface());
  dedicated_listener_thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});
}

FaceWeakConstPtr Person::face() const
{
  if (listener_->getFaces().count(face_id) != 0) {
    return listener_->getFaces()[face_id];
  } else {
    return FaceWeakConstPtr();
  }
}

BodyWeakConstPtr Person::body() const
{
  if (listener_->getBodies().count(body_id) != 0) {
    return listener_->getBodies()[body_id];
  } else {
    return BodyWeakConstPtr();
  }
}

VoiceWeakConstPtr Person::voice() const
{
  if (listener_->getVoices().count(voice_id) != 0) {
    return listener_->getVoices()[voice_id];
  } else {
    return VoiceWeakConstPtr();
  }
}

boost::optional<EngagementLevel> Person::engagement_status() const
{
  if (!_engagement_status) {
    return boost::optional<EngagementLevel>();
  }

  switch (_engagement_status->level) {
    case hri_msgs::msg::EngagementLevel::UNKNOWN:
      return boost::optional<EngagementLevel>();
    case hri_msgs::msg::EngagementLevel::ENGAGING:
      return EngagementLevel::ENGAGING;
    case hri_msgs::msg::EngagementLevel::ENGAGED:
      return EngagementLevel::ENGAGED;
    case hri_msgs::msg::EngagementLevel::DISENGAGING:
      return EngagementLevel::DISENGAGING;
    case hri_msgs::msg::EngagementLevel::DISENGAGED:
      return EngagementLevel::DISENGAGED;
    default:
      // we should handle all the possible engagement values
      assert(false);
      return boost::optional<EngagementLevel>();
  }
}

boost::optional<geometry_msgs::msg::TransformStamped> Person::transform() const
{
  if (abs(_loc_confidence) < 1e-2) {
    return boost::optional<geometry_msgs::msg::TransformStamped>();
  }

  try {
    auto transform = tf_buffer_.lookupTransform(
      _reference_frame, frame(),
      tf2::TimePointZero);
    return transform;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(
      node_->get_logger(),
      "failed to transform person frame " << frame()
                                          << " to " << _reference_frame << ". " <<
        ex.what());
    return boost::optional<geometry_msgs::msg::TransformStamped>();
  }
}

}  // namespace hri
