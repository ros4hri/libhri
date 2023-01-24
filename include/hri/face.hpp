// Copyright 2021 PAL Robotics S.L.
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


#ifndef HRI__FACE_HPP_
#define HRI__FACE_HPP_

#include <hri_msgs/msg/normalized_point_of_interest2_d.hpp>
#include <hri_msgs/msg/facial_landmarks.hpp>
#include <hri_msgs/msg/soft_biometrics.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <memory>
#include <boost/optional.hpp>

#include "FeatureTracker.hpp"

#include <opencv2/core.hpp>

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
const static std::string FACE_TF_PREFIX("face_");
const static std::string GAZE_TF_PREFIX("gaze_");
const static rclcpp::Duration FACE_TF_TIMEOUT(rclcpp::Duration::from_seconds(0.01));

class Face : public FeatureTracker
{
public:
  Face(ID id, rclcpp::Node::SharedPtr node, tf2::BufferCore* tf_buffer_ptr,
       const std::string& reference_frame);



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

  /** \brief Returns the 2D region of interest (RoI) of the face.
   *
   * Use example:
   *
   * ```cpp
   * HRIListener hri_listener;
   *
   * auto faces = hri_listener.getFaces();
   *
   * for (auto const& face : faces)
   * {
   *   auto roi = face.second.lock()->roi();
   *   cout << "Size of face_" << face.first << ": ";
   *   cout << roi.width << "x" << roi.height << endl;
   * }
   * ```
   *
   * The pixel coordinates are provided in the original camera's image coordinate
   * space.
   */
  cv::Rect roi() const;


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

  /** \brief the list of the 66 facial landmarks (2D points, expressed in normalised coordinates).
   *
   * The location of the landmarks is defined here:
   * https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md#face-output-format
   *
   * Constants defined in hri_msgs/FacialLandmarks.h can be used to access
   * specific points on the face.
   */
  boost::optional<std::array<hri_msgs::msg::NormalizedPointOfInterest2D, 70>> facialLandmarks() const
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
  boost::optional<std::array<IntensityConfidence, 99>> facialActionUnits() const
  {
    return facial_action_units_;
  }

  /* \brief estimated age of this face, if available (eg, the
   * '/softbiometrics' is published)
   */
  boost::optional<float> age() const;

  /* \brief estimated gender of this face, if available (eg, the
   * '/softbiometrics' is published)
   */
  boost::optional<Gender> gender() const;

  /** \brief Returns the (stamped) 3D transform of the face (if available).
   */
  boost::optional<geometry_msgs::msg::TransformStamped> transform() const;

  /** \brief Returns the (stamped) 3D transform of the gaze (if available).
   */
  boost::optional<geometry_msgs::msg::TransformStamped> gazeTransform() const;

  void init() override;

private:
  size_t nb_roi;


  rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_subscriber_;
  void onRoI(sensor_msgs::msg::RegionOfInterest::SharedPtr roi);
  cv::Rect roi_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cropped_subscriber_;
  void onCropped(sensor_msgs::msg::Image::SharedPtr roi);
  cv::Mat cropped_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr aligned_subscriber_;
  void onAligned(sensor_msgs::msg::Image::SharedPtr roi);
  cv::Mat aligned_;

  rclcpp::Subscription<hri_msgs::msg::FacialLandmarks>::SharedPtr landmarks_subscriber_;
  void onLandmarks(hri_msgs::msg::FacialLandmarks::SharedPtr landmarks);
  std::array<hri_msgs::msg::NormalizedPointOfInterest2D, 70> facial_landmarks_;

  rclcpp::Subscription<hri_msgs::msg::SoftBiometrics>::SharedPtr softbiometrics_subscriber_;
  void onSoftBiometrics(hri_msgs::msg::SoftBiometrics::SharedPtr biometrics);
  hri_msgs::msg::SoftBiometrics::SharedPtr softbiometrics_;


  std::array<IntensityConfidence, 99> facial_action_units_;

  std::string _reference_frame;
  tf2_ros::Buffer* _tf_buffer_ptr;
};

typedef std::shared_ptr<Face> FacePtr;
typedef std::shared_ptr<const Face> FaceConstPtr;
typedef std::weak_ptr<Face> FaceWeakPtr;
typedef std::weak_ptr<const Face> FaceWeakConstPtr;

}  // namespace hri
#endif // HRI__FACE_HPP_
