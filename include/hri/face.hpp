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
#include <string>
#include <optional>

#include <opencv2/core.hpp>

#include <hri_msgs/msg/normalized_point_of_interest2_d.hpp>
#include <hri_msgs/msg/normalized_region_of_interest2_d.hpp>
#include <hri_msgs/msg/facial_landmarks.hpp>
#include <hri_msgs/msg/soft_biometrics.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "feature_tracker.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace hri
{
struct IntensityConfidence
{
  float intensity;
  float confidence;
};

enum Gender
{
  FEMALE = 1,
  MALE = 2,
  OTHER = 3
};

// the tf prefixes follow REP-155
static const char FACE_TF_PREFIX[] = "face_";
static const char GAZE_TF_PREFIX[] = "gaze_";
static const rclcpp::Duration FACE_TF_TIMEOUT(rclcpp::Duration::from_seconds(0.01));

class Face : public FeatureTracker
{
public:
  typedef hri_msgs::msg::NormalizedRegionOfInterest2D RegionOfInterest;

  Face(
    ID id,
    rclcpp::Node::SharedPtr node,
    tf2::BufferCore & tf_buffer,
    const std::string & reference_frame);

  virtual ~Face();

  /** \brief the name of the tf frame that correspond to this face
   */
  std::string frame() const
  {
    return FACE_TF_PREFIX + id_;
  }

  /** \brief the name of the tf frame that correspond to the gaze direction and
   * orientation of the face
   */
  std::string gazeFrame() const
  {
    return GAZE_TF_PREFIX + id_;
  }

  /** \brief Returns the normalized 2D region of interest (RoI) of the face.
   *
   * Use example:
   * ```
   * class ShowFaces : public rclcpp::Node
   * {
   * public:
   *   ShowFaces()
   *   : Node("show_faces")
   *   {
   *     hri_listener_ = std::make_shared<hri::HRIListener>();
   *     timer_ = create_wall_timer(
   *       500ms, std::bind(&ShowFaces::timer_callback, this));
   *   }
   *
   *
   *   void timer_callback()
   *   {
   *      auto faces = hri_listener_->getFaces();
   *     for (auto& f : faces)
   *     {
   *        auto roi = face.second.lock()->roi();
   *        cout << "Normalized size of face_" << face.first << ": ";
   *        cout << (roi.xmax - roi.xmin) << "x" << (roi.ymax - roi.ymin) << endl;
   *     }
   *   }
   *
   * private:
   *   std::shared_ptr<hri::HRIListener> hri_listener_{nullptr};
   *   rclcpp::TimerBase::SharedPtr timer_;
   * };
   * ```
   *
   * The pixel coordinates are provided in the original camera's image coordinate
   * space.
   */
  RegionOfInterest roi() const;


  /** \brief Returns the face image, if necessary scaled, centered and 0-padded
   * to match the /humans/faces/width and /humans/faces/height ROS parameters.
   */
  cv::Mat cropped() const;

  /** \brief Returns the face image, if necessary scaled, centered and 0-padded
   * to match the /humans/faces/width and /humans/faces/height ROS parameters.
   *
   * In addition, the face is rotated so that the eyes are horizontal.
   */
  cv::Mat aligned() const;

  /** \brief the list of the 66 facial landmarks (2D points, expressed in normalized coordinates).
   *
   * The location of the landmarks is defined here:
   * github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md#face-output-format
   *
   * Constants defined in hri_msgs/FacialLandmarks.h can be used to access
   * specific points on the face.
   */
  std::optional<std::array<hri_msgs::msg::NormalizedPointOfInterest2D, 70>>
  facialLandmarks() const
  {
    return facial_landmarks_;
  }

  /** \brief the list of the facial action units detected on the face.
   *
   * Action units indices follow the Action Unit naming convention by Ekman.
   * List here: https://en.wikipedia.org/wiki/Facial_Action_Coding_System
   *
   * Note that the list is sparse (some indices do not exist in Ekman classification).
   * In addition, depending on the AU detector, some action units might not be
   * detected. In these cases, the confidence value will be 0.
   *
   * Constants defined in hri_msgs/FacialActionUnits.h can be used to access
   * specific action units by name.
   */
  std::optional<std::array<IntensityConfidence, 99>> facialActionUnits() const
  {
    return facial_action_units_;
  }

  /* \brief estimated age of this face, if available (eg, the
   * '/softbiometrics' is published)
   */
  std::optional<float> age() const;

  /* \brief estimated gender of this face, if available (eg, the
   * '/softbiometrics' is published)
   */
  std::optional<Gender> gender() const;

  /** \brief Returns the (stamped) 3D transform of the face (if available).
   */
  std::optional<geometry_msgs::msg::TransformStamped> transform() const;

  /** \brief Returns the (stamped) 3D transform of the gaze (if available).
   */
  std::optional<geometry_msgs::msg::TransformStamped> gazeTransform() const;

  void init() override;

private:
  size_t nb_roi;

  std::unique_ptr<std::thread> dedicated_listener_thread_ {nullptr};
  rclcpp::Node::SharedPtr node_ {nullptr};


  rclcpp::Executor::SharedPtr executor_ {nullptr};

  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};

  rclcpp::Subscription<RegionOfInterest>::SharedPtr roi_subscriber_{nullptr};
  void onRoI(RegionOfInterest::ConstSharedPtr roi);
  RegionOfInterest roi_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cropped_subscriber_{nullptr};
  void onCropped(sensor_msgs::msg::Image::ConstSharedPtr roi);
  cv::Mat cropped_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr aligned_subscriber_{nullptr};
  void onAligned(sensor_msgs::msg::Image::ConstSharedPtr roi);
  cv::Mat aligned_;

  rclcpp::Subscription<hri_msgs::msg::FacialLandmarks>::SharedPtr landmarks_subscriber_{nullptr};
  void onLandmarks(hri_msgs::msg::FacialLandmarks::ConstSharedPtr landmarks);
  std::array<hri_msgs::msg::NormalizedPointOfInterest2D, 70> facial_landmarks_;

  rclcpp::Subscription<hri_msgs::msg::SoftBiometrics>::SharedPtr softbiometrics_subscriber_
  {nullptr};

  void onSoftBiometrics(hri_msgs::msg::SoftBiometrics::ConstSharedPtr biometrics);
  hri_msgs::msg::SoftBiometrics::ConstSharedPtr softbiometrics_;


  std::array<IntensityConfidence, 99> facial_action_units_;

  std::string _reference_frame;
  tf2::BufferCore & tf_buffer_;
};

typedef std::shared_ptr<Face> FacePtr;
typedef std::shared_ptr<const Face> FaceConstPtr;

}  // namespace hri
#endif  // HRI__FACE_HPP_
