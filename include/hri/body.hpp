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
#include <vector>
#include <string>

#include <optional>

#include <opencv2/core.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <hri_msgs/msg/skeleton2_d.hpp>
#include <hri_msgs/msg/normalized_point_of_interest2_d.hpp>
#include <hri_msgs/msg/normalized_region_of_interest2_d.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "feature_tracker.hpp"


namespace hri
{
// the tf prefix follows REP-155
static const char BODY_TF_PREFIX[] = "body_";
static const rclcpp::Duration BODY_TF_TIMEOUT(rclcpp::Duration::from_seconds(0.01));

class Body : public FeatureTracker
{
public:
  typedef std::array<hri_msgs::msg::NormalizedPointOfInterest2D, 18> SkeletonPoints;
  typedef hri_msgs::msg::NormalizedRegionOfInterest2D RegionOfInterest;

  Body(
    ID id,
    rclcpp::Node::SharedPtr node,
    tf2::BufferCore & tf_buffer,
    const std::string & reference_frame);

  virtual ~Body();

  /** \brief the name of the tf frame that correspond to this body
   */
  std::string frame() const
  {
    return BODY_TF_PREFIX + id_;
  }

  /** \brief If available, returns the normalized 2D region of interest (RoI) of the body.
   *
   * Use example:
   *
   * ```cpp
   * HRIListener hri_listener;
   *
   * auto bodies = hri_listener.getBodies();
   *
   * for (auto const& body : bodies)
   * {
   *   auto roi = body.second.lock()->roi();
   *   cout << "Normalized size of body_" << body.first << ": ";
   *   cout << (roi.xmax - roi.xmin) << "x" << (roi.ymax - roi.ymin) << endl;
   * }
   * ```
   *
   * The coordinates are provided in the original camera's image coordinate
   * space.
   *
   * The header's timestamp is the same as a the timestamp of the original
   * image from which the body has been detected.
   */
  RegionOfInterest roi() const;

  /** \brief Returns the body image, cropped from the source image.
   */
  cv::Mat cropped() const;

  /** \brief Returns the 2D skeleton keypoints.
   *
   * Points coordinates are in the image space of the source image, and
   * normalised between 0.0 and 1.0.
   *
   * The skeleton joints indices follow those defined in
   * http://docs.ros.org/en/api/hri_msgs/html/msg/Skeleton2D.html
   */
  SkeletonPoints skeleton() const;

  /** \brief Returns the (stamped) 3D transform of the body (if available).
   */
  std::optional<geometry_msgs::msg::TransformStamped> transform() const;

  void init() override;

private:
  size_t nb_roi;

  std::unique_ptr<std::thread> dedicated_listener_thread_ {nullptr};
  rclcpp::Node::SharedPtr node_ {nullptr};
  rclcpp::Executor::SharedPtr executor_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};

  rclcpp::Subscription<RegionOfInterest>::SharedPtr roi_subscriber_ {nullptr};
  void onRoI(RegionOfInterest::ConstSharedPtr roi);
  RegionOfInterest roi_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cropped_subscriber_ {nullptr};
  void onCropped(sensor_msgs::msg::Image::ConstSharedPtr roi);
  cv::Mat cropped_;

  rclcpp::Subscription<hri_msgs::msg::Skeleton2D>::SharedPtr skeleton_subscriber_ {nullptr};
  void onSkeleton(hri_msgs::msg::Skeleton2D::ConstSharedPtr skeleton);
  SkeletonPoints skeleton_;

  std::string _reference_frame;
  tf2::BufferCore & tf_buffer_;
};

typedef std::shared_ptr<Body> BodyPtr;

}  // namespace hri
#endif  // HRI__BODY_HPP_
