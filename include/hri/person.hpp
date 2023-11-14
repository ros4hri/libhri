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

#ifndef HRI__PERSON_HPP_
#define HRI__PERSON_HPP_

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "hri/body.hpp"
#include "hri/face.hpp"
#include "hri/feature_tracker.hpp"
#include "hri/types.hpp"
#include "hri/voice.hpp"
#include "hri_msgs/msg/engagement_level.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"

namespace hri
{

class HRIListener;

class Person : public FeatureTracker
// TODO(LJU): possibly subscribe also to the /name and the /native_language sub-topics
{
public:
  Person(
    ID id,
    rclcpp::Node::SharedPtr node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    HRIListener * const listener,
    const tf2::BufferCore & tf_buffer,
    const std::string & reference_frame);

  virtual ~Person();

  /** \brief Returns a shared pointer to the face of this person, or
   * a nullptr if this person is currently not associated to any detected face.
   */
  FacePtr face() const;

  /** \brief Returns a shared pointer to the body of this person, or
   * a nullptr if this person is currently not associated to any detected body.
   */
  BodyPtr body() const;

  /** \brief Returns a shared pointer to the voice of this person, or
   * a nullptr if this person is currently not associated to any detected voice.
   */
  VoicePtr voice() const;

  std::optional<bool> anonymous() const {return anonymous_;}
  std::optional<EngagementLevel> engagement_status() const {return engagement_status_;}
  std::optional<float> location_confidence() const {return loc_confidence_;}
  std::optional<ID> alias() const {return alias_;}

  std::optional<Transform> transform() const override;

private:
  void onFaceId(std_msgs::msg::String::ConstSharedPtr msg);
  void onBodyId(std_msgs::msg::String::ConstSharedPtr msg);
  void onVoiceId(std_msgs::msg::String::ConstSharedPtr msg);
  void onAnonymous(std_msgs::msg::Bool::ConstSharedPtr msg);
  void onAlias(std_msgs::msg::String::ConstSharedPtr msg);
  void onEngagementStatus(hri_msgs::msg::EngagementLevel::ConstSharedPtr msg);
  void onLocationConfidence(std_msgs::msg::Float32::ConstSharedPtr msg);

  // we use a raw pointer here. `this` is owned by the pointed HRIListener, so
  // `this` would normally be destroyed before HRIListener (in reality, a
  // pointer to `this` *might* outlive `HRIListener` -- make sure HRIListener
  // is destroyed after all pointers to this person are released.
  HRIListener * const listener_;

  std::optional<ID> face_id_;
  std::optional<ID> body_id_;
  std::optional<ID> voice_id_;
  // if non-empty, this person 'does not exist' and is instead an alias to
  // another person.  hri::getPersons and hri::getTrackedPersons will returns
  // pointers to the aliased person.
  std::optional<ID> alias_;
  std::optional<bool> anonymous_;
  std::optional<EngagementLevel> engagement_status_;
  std::optional<float> loc_confidence_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr face_id_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr body_id_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_id_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr anonymous_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr alias_subscriber_;
  rclcpp::Subscription<hri_msgs::msg::EngagementLevel>::SharedPtr engagement_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr loc_confidence_subscriber_;
};

typedef std::shared_ptr<Person> PersonPtr;

}  // namespace hri

#endif  // HRI__PERSON_HPP_
