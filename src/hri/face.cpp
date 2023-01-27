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

#include "hri/face.hpp"

#include <cv_bridge/cv_bridge.h>
#include <hri_msgs/msg/soft_biometrics.hpp>


namespace hri
{

Face::Face(
  ID id,
  rclcpp::Node::SharedPtr node,
  tf2::BufferCore* tf_buffer_ptr,
  const std::string& reference_frame)
  : FeatureTracker{ id, node }
  , softbiometrics_(nullptr)
  , _tf_buffer_ptr()
  , _reference_frame(reference_frame)

{
}

Face::~Face()
{
    executor_->cancel();
    dedicated_listener_thread_->join();
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Deleting face " << id_);
  // roi_subscriber_.shutdown();
}

void Face::init()
{
  rclcpp::NodeOptions node_options;

  // node_options.arguments({"--ros-args", "-r", "__node:=" + id_});
  node_options.start_parameter_event_publisher(false);
  node_options.start_parameter_services(false);
  optional_default_node_ = rclcpp::Node::make_shared("_", node_options);

  callback_group_ = optional_default_node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
  options.callback_group = callback_group_;

  ns_ = "/humans/faces/" + id_;
  // RCLCPP_DEBUG_STREAM(node_->get_logger(), "New face detected: " << ns_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "New face detected: " << ns_);

  auto roi_subscriber_ = optional_default_node_->create_subscription<sensor_msgs::msg::RegionOfInterest>(
      ns_ + "/roi", 1, bind(&Face::onRoI, this, std::placeholders::_1), options);

  auto cropped_subscriber_ = optional_default_node_->create_subscription<sensor_msgs::msg::Image>(
      "/humans/faces/bdfab/cropped", 1, bind(&Face::onCropped, this, std::placeholders::_1), options);

  auto aligned_subscriber_ = optional_default_node_->create_subscription<sensor_msgs::msg::Image>(
      ns_ + "/aligned", 1, bind(&Face::onAligned, this, std::placeholders::_1), options);

  auto landmarks_subscriber_ = optional_default_node_->create_subscription<hri_msgs::msg::FacialLandmarks>(
      ns_ + "/landmarks", 1, bind(&Face::onLandmarks, this, std::placeholders::_1), options);

  auto softbiometrics_subscriber_ = optional_default_node_->create_subscription<hri_msgs::msg::SoftBiometrics>(
      ns_ + "/softbiometrics", 1, bind(&Face::onSoftBiometrics, this, std::placeholders::_1), options);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, optional_default_node_->get_node_base_interface());  
  dedicated_listener_thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});
  
}

void Face::onRoI(sensor_msgs::msg::RegionOfInterest::SharedPtr roi)
{
  roi_ = cv::Rect(roi->x_offset, roi->y_offset, roi->width, roi->height);
}

cv::Rect Face::roi() const
{
  return roi_;
}

void Face::onCropped(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "Got to cropped");
  RCLCPP_INFO(node_->get_logger(), "I heard: '%u'",msg->height);
  cropped_ = cv_bridge::toCvCopy(msg)->image;  // if using toCvShare, the image ends up shared with aligned_!
}

cv::Mat Face::cropped() const
{
  return cropped_;
}

void Face::onAligned(sensor_msgs::msg::Image::SharedPtr msg)
{
  aligned_ = cv_bridge::toCvCopy(msg)->image;  // if using toCvShare, the image ends up shared with cropped_!
}

cv::Mat Face::aligned() const
{
  return aligned_;
}

void Face::onLandmarks(hri_msgs::msg::FacialLandmarks::SharedPtr msg)
{
  int i = 0;

  for (auto landmark : msg->landmarks)
  {
    facial_landmarks_[i].x = landmark.x;
    facial_landmarks_[i].y = landmark.y;
    facial_landmarks_[i].c = landmark.c;
    ++i;
  }
}

void Face::onSoftBiometrics(hri_msgs::msg::SoftBiometrics::SharedPtr biometrics)
{
  softbiometrics_ = biometrics;
}


boost::optional<float> Face::age() const
{
  if (!softbiometrics_)
    return boost::optional<float>();

  return softbiometrics_->age;
}

boost::optional<Gender> Face::gender() const
{
  if (!softbiometrics_)
    return boost::optional<Gender>();
  if (softbiometrics_->gender == 0)  // UNDEFINED
    return boost::optional<Gender>();

  return static_cast<Gender>(softbiometrics_->gender);
}

boost::optional<geometry_msgs::msg::TransformStamped> Face::transform() const
{
  try
  {
    auto transform = _tf_buffer_ptr->lookupTransform(_reference_frame, frame(),
                                                     rclcpp::Time(0), FACE_TF_TIMEOUT);

    return transform;
  }
  catch (tf2::LookupException)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "failed to transform the face frame " << frame() << " to " << _reference_frame
                                                          << ". Are the frames published?");
    return boost::optional<geometry_msgs::msg::TransformStamped>();
  }
}

boost::optional<geometry_msgs::msg::TransformStamped> Face::gazeTransform() const
{
  try
  {
    auto transform = _tf_buffer_ptr->lookupTransform(_reference_frame, gazeFrame(),
                                                     rclcpp::Time(0), FACE_TF_TIMEOUT);

    return transform;
  }
  catch (tf2::LookupException)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "failed to transform the gaze frame " << frame() << " to " << _reference_frame
                                                          << ". Are the frames published?");
    return boost::optional<geometry_msgs::msg::TransformStamped>();
  }
}

}  // namespace hri