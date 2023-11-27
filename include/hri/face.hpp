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

#ifndef HRI__FACE_HPP_
#define HRI__FACE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hri/feature_tracker.hpp"
#include "hri/types.hpp"
#include "hri_msgs/msg/facial_action_units.hpp"
#include "hri_msgs/msg/facial_landmarks.hpp"
#include "hri_msgs/msg/normalized_region_of_interest2_d.hpp"
#include "hri_msgs/msg/soft_biometrics.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"

namespace hri
{
// TODO(LJU): possibly subscribe also to the /frontalized and /expression sub-topics
class Face : public FeatureTracker
{
  friend class HRIListener;  // for invalidate()

public:
  Face(
    ID id,
    NodeInterfaces & node_interfaces,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    const tf2::BufferCore & tf_buffer,
    const std::string & reference_frame);

  virtual ~Face();

  /** \brief The name of the tf frame that correspond to the gaze direction and
   * orientation of the face.
   */
  std::string gazeFrame() const {return kGazeFrame_;}

  /** \brief Returns the normalized 2D region of interest (RoI) of the face, (if available).
   *
   * The pixel coordinates are provided in the original camera's image coordinate space.
   */
  std::optional<cv::Rect2f> roi() const {return roi_;}

  /** \brief Returns the face image, if necessary scaled, centered and 0-padded, (if available).
   */
  std::optional<cv::Mat> cropped() const {return cropped_;}

  /** \brief Returns the face image, if necessary scaled, centered and 0-padded, (if available).
   *
   * In addition, the face is rotated so that the eyes are horizontal.
   */
  std::optional<cv::Mat> aligned() const {return aligned_;}

  /** \brief The list of the 70 facial landmarks (2D points, expressed in normalized coordinates),
   * (if available).
   *
   * Constants defined in hri_msgs/msgs/facial_landmarks.hpp can be used to access
   * specific points on the face.
   */
  std::optional<FacialLandmarks> facialLandmarks() const {return landmarks_;}

  /** \brief the list of the facial action units detected on the face.
   *
   * Action units indices follow the Action Unit naming convention by Ekman.
   * List here: https://en.wikipedia.org/wiki/Facial_Action_Coding_System
   *
   * Note that the list is sparse (some indices do not exist in Ekman classification).
   * In addition, depending on the AU detector, some action units might not be
   * detected. In these cases, the confidence value will be 0.
   */
  std::optional<FacialActionUnits> facialActionUnits() const {return facial_action_units_;}

  /** \brief Estimated age of this face, if available (eg, the '/softbiometrics' is published).
   */
  std::optional<float> age() const {return age_;}

  /** \brief Estimated gender of this face, if available (eg, the '/softbiometrics' is published).
   */
  std::optional<Gender> gender() const {return gender_;}

  /** \brief Returns the (stamped) 3D transform of the gaze (if available).
   */
  std::optional<geometry_msgs::msg::TransformStamped> gazeTransform() const;

private:
  void onRoI(hri_msgs::msg::NormalizedRegionOfInterest2D::ConstSharedPtr msg);
  void onCropped(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void onAligned(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void onLandmarks(hri_msgs::msg::FacialLandmarks::ConstSharedPtr msg);
  void onSoftBiometrics(hri_msgs::msg::SoftBiometrics::ConstSharedPtr msg);
  void onFacs(hri_msgs::msg::FacialActionUnits::ConstSharedPtr msg);

  void invalidate();

  std::optional<cv::Rect2f> roi_;
  std::optional<cv::Mat> cropped_;
  std::optional<cv::Mat> aligned_;
  std::optional<FacialLandmarks> landmarks_;
  std::optional<float> age_;
  std::optional<Gender> gender_;
  std::optional<FacialActionUnits> facial_action_units_;

  rclcpp::Subscription<hri_msgs::msg::NormalizedRegionOfInterest2D>::SharedPtr roi_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cropped_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr aligned_subscriber_;
  rclcpp::Subscription<hri_msgs::msg::FacialLandmarks>::SharedPtr landmarks_subscriber_;
  rclcpp::Subscription<hri_msgs::msg::SoftBiometrics>::SharedPtr softbiometrics_subscriber_;
  rclcpp::Subscription<hri_msgs::msg::FacialActionUnits>::SharedPtr facial_action_units_subscriber_;

  const std::string kGazeFrame_;
};

typedef std::shared_ptr<Face> FacePtr;

}  // namespace hri

#endif  // HRI__FACE_HPP_
