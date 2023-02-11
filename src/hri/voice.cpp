// Copyright 2022 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "hri/voice.hpp"

namespace hri
{

Voice::Voice(
  ID id,
  rclcpp::Node::SharedPtr node,
  tf2::BufferCore &tf_buffer,
  const std::string& reference_frame)
  : FeatureTracker{id}
  , tf_buffer_(tf_buffer)
  , _reference_frame(reference_frame)
  ,node_(node)
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
}

boost::optional<geometry_msgs::msg::TransformStamped> Voice::transform() const
{
  try
  {
    auto transform = tf_buffer_.lookupTransform(_reference_frame, frame(),
                                                     tf2::TimePointZero);

    return transform;
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "failed to transform person frame " << frame()
                                << " to " << _reference_frame << ex.what());
    return boost::optional<geometry_msgs::msg::TransformStamped>();
  }
}
}  // namespace hri