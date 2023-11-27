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

#ifndef HRI__BODY_HPP_
#define HRI__BODY_HPP_

#include <memory>
#include <optional>
#include <string>

#include "hri/feature_tracker.hpp"
#include "hri/types.hpp"
#include "hri_msgs/msg/normalized_region_of_interest2_d.hpp"
#include "hri_msgs/msg/skeleton2_d.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"

namespace hri
{
// TODO(LJU): possibly subscribe also to the /joint_states, gesture and /posture sub-topics
class Body : public FeatureTracker
{
  friend class HRIListener;  // for invalidate()

public:
  Body(
    ID id,
    NodeInterfaces & node_interfaces,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    const tf2::BufferCore & tf_buffer,
    const std::string & reference_frame);

  virtual ~Body();

  /** \brief If available, returns the normalized 2D region of interest (RoI) of the body.
   *
   * The coordinates are provided in the original camera's image coordinate
   * space.
   */
  std::optional<cv::Rect2f> roi() const {return roi_;}

  /** \brief Returns the body image, cropped from the source image.
   */
  std::optional<cv::Mat> cropped() const {return cropped_;}

  /** \brief Returns the 2D skeleton keypoints.
   *
   * Points coordinates are in the image space of the source image, and
   * normalised between 0.0 and 1.0.
   *
   * The skeleton joints indices follow those defined in
   * http://docs.ros.org/en/api/hri_msgs/html/msg/Skeleton2D.html
   */
  std::optional<SkeletalKeypoints> skeleton() const {return skeleton_;}

private:
  void onRoI(hri_msgs::msg::NormalizedRegionOfInterest2D::ConstSharedPtr msg);
  void onCropped(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void onSkeleton(hri_msgs::msg::Skeleton2D::ConstSharedPtr msg);

  void invalidate();

  std::optional<cv::Rect2f> roi_;
  std::optional<cv::Mat> cropped_;
  std::optional<SkeletalKeypoints> skeleton_;

  rclcpp::Subscription<hri_msgs::msg::NormalizedRegionOfInterest2D>::SharedPtr roi_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cropped_subscriber_;
  rclcpp::Subscription<hri_msgs::msg::Skeleton2D>::SharedPtr skeleton_subscriber_;
};

typedef std::shared_ptr<Body> BodyPtr;

}  // namespace hri

#endif  // HRI__BODY_HPP_
