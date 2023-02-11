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

#include "hri/body.hpp"

#include <cv_bridge/cv_bridge.h>


namespace hri
{

Body::Body(
  ID id,
  rclcpp::Node::SharedPtr node,
  tf2::BufferCore &tf_buffer,
  const std::string& reference_frame)
  : FeatureTracker{ id }
  , tf_buffer_(tf_buffer)
  , _reference_frame(reference_frame)
  , node_(node)
{
}


Body::~Body()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Deleting body " << id_);
  executor_->cancel();
  dedicated_listener_thread_->join();
}

void Body::init()
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

  ns_ = "/humans/bodies/" + id_;
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "New body detected: " << ns_);

  roi_subscriber_ = rclcpp::create_subscription<sensor_msgs::msg::RegionOfInterest>(
      node_params, node_topics, ns_ + "/roi", qos, bind(&Body::onRoI, this, std::placeholders::_1), options);

  cropped_subscriber_ = rclcpp::create_subscription<sensor_msgs::msg::Image>(
      node_params, node_topics, ns_ + "/cropped", qos, bind(&Body::onCropped, this, std::placeholders::_1), options);

  skeleton_subscriber_ = rclcpp::create_subscription<hri_msgs::msg::Skeleton2D>(
      node_params, node_topics, ns_ + "/skeleton2d", qos, bind(&Body::onSkeleton, this, std::placeholders::_1), options);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, node_->get_node_base_interface());  
  dedicated_listener_thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});
}

void Body::onRoI(const sensor_msgs::msg::RegionOfInterest::ConstSharedPtr roi)
{
  roi_ = cv::Rect(roi->x_offset, roi->y_offset, roi->width, roi->height);
}

cv::Rect Body::roi() const
{
  return roi_;
}

void Body::onCropped(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cropped_ = cv_bridge::toCvShare(msg)->image;
}

cv::Mat Body::cropped() const
{
  return cropped_;
}

void Body::onSkeleton(const hri_msgs::msg::Skeleton2D::ConstSharedPtr msg)
{
  skeleton_ = msg->skeleton;
}

std::vector<SkeletonPoint> Body::skeleton() const
{
  return skeleton_;
}

boost::optional<geometry_msgs::msg::TransformStamped> Body::transform() const
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