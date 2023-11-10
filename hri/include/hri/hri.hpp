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

#ifndef HRI__HRI_HPP_
#define HRI__HRI_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "hri/body.hpp"
#include "hri/face.hpp"
#include "hri/feature_tracker.hpp"
#include "hri/person.hpp"
#include "hri/types.hpp"
#include "hri/voice.hpp"
#include "hri_msgs/msg/ids_list.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace hri
{

/** \brief Main entry point to libhri. This is most likely what you want to use.
 *
 * See examples/node_show_faces.cpp for a minimal usage example
 */
class HRIListener : public std::enable_shared_from_this<HRIListener>
{
public:
  /** \brief Factory function for the libhri main class.
   *
   * The class can subscribe to topics and print logs, using the node interfaces arguments.
   * The class uses a factory design pattern to make sure a shared pointer to an instance of this
   * class is created, enabling the safe internal use of shared_from_this().
   * See https://en.cppreference.com/w/cpp/memory/enable_shared_from_this for more info.
   *
   * Internally the HRIListener uses node interfaces to adapt both to Node and LifecycleNode
   * (see tinyurl.com/ROSCon-2023-node-interfaces).
   * Since rclcpp::NodeInterface is not available in Humble, for convenience we use a variant for
   * this factory and internally pass a structure mocking rclcpp::NodeInterface.
   */
  [[nodiscard]] static std::shared_ptr<HRIListener> create(NodeLikeSharedPtr node_like)
  {
    auto node_interfaces = std::visit(
      [](auto && node) {
        return NodeInterfaces {
          node->get_node_base_interface(),
          node->get_node_logging_interface(),
          node->get_node_parameters_interface(),
          node->get_node_topics_interface()};
      }, node_like);
    return std::shared_ptr<HRIListener>(new HRIListener(node_interfaces));
  }

  ~HRIListener();

  /** \brief Returns the list of currently detected faces, mapped to their IDs
   */
  std::map<ID, ConstFacePtr> getFaces() const;

  /** \brief Registers a callback function, to be invoked everytime a new face
   * is detected.
   */
  void onFace(std::function<void(FacePtr)> callback)
  {
    face_callbacks_.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a
   * previously tracked face is lost (eg, not detected anymore)
   */
  void onFaceLost(std::function<void(ID)> callback)
  {
    face_lost_callbacks_.push_back(callback);
  }


  /** \brief Returns the list of currently detected bodies, mapped to their IDs
   */
  std::map<ID, ConstBodyPtr> getBodies() const;

  /** \brief Registers a callback function, to be invoked everytime a new body
   * is detected.
   */
  void onBody(std::function<void(BodyPtr)> callback)
  {
    body_callbacks_.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a
   * previously tracked body is lost (eg, not detected anymore)
   */
  void onBodyLost(std::function<void(ID)> callback)
  {
    body_lost_callbacks_.push_back(callback);
  }


  /** \brief Returns the list of currently detected voices, mapped to their IDs
   */
  std::map<ID, ConstVoicePtr> getVoices() const;

  /** \brief Registers a callback function, to be invoked everytime a new voice
   * is detected.
   */
  void onVoice(std::function<void(VoicePtr)> callback)
  {
    voice_callbacks_.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a
   * previously tracked voice is lost (eg, not detected anymore)
   */
  void onVoiceLost(std::function<void(ID)> callback)
  {
    voice_lost_callbacks_.push_back(callback);
  }


  /** \brief Returns the list of all known persons, whether or not they are
   * currently actively detected (eg, seen). The persons are mapped to their
   * IDs.
   */
  std::map<ID, ConstPersonPtr> getPersons() const;

  /** \brief Registers a callback function, to be invoked everytime a new person
   * is detected.
   */
  void onPerson(std::function<void(PersonPtr)> callback)
  {
    person_callbacks_.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a person
   * is lost. This can *only* happen for anonymous persons. Identified persons
   * will never be removed from the list of all known persons.
   */
  void onPersonLost(std::function<void(ID)> callback)
  {
    person_lost_callbacks_.push_back(callback);
  }

  /** \brief Returns the list of currently detected persons, mapped to their IDs
   */
  std::map<ID, ConstPersonPtr> getTrackedPersons() const;

  /** \brief Registers a callback function, to be invoked everytime a new person
   * is detected and actively tracked (eg, currently seen).
   */
  void onTrackedPerson(std::function<void(PersonPtr)> callback)
  {
    person_tracked_callbacks_.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a previously tracked
   * person is lost.
   */
  void onTrackedPersonLost(std::function<void(ID)> callback)
  {
    person_tracked_lost_callbacks_.push_back(callback);
  }

  /** \brief Sets the reference frame from which the TF transformations of the persons will
   * be returned (via `Person::transform()`).
   *
   * By default, `base_link`.
   */
  void setReferenceFrame(const std::string & frame)
  {
    reference_frame_ = frame;
  }

  /** \brief Get the mutually exclusive callback group used by HRIListener.
   *
   * This can be used to avoid race conditions between the internal subscribe callbacks and the
   * HRIListener getter functions (e.g., hri::HRIListener::getTrackedPersons(), hri::Face::roi()).
   * If a multithreaded executor is used to freely spin a node which interfaces are used by both
   * HRIListener and by timer/topic/... callbacks which call an HRIListener getter function,
   * then the latter should be added to this callback group.
   */
  rclcpp::CallbackGroup::SharedPtr getCallbackGroup() {return callback_group_;}

private:
  explicit HRIListener(NodeInterfaces & node_interfaces);
  void onTrackedFeature(FeatureType & feature, hri_msgs::msg::IdsList::ConstSharedPtr tracked);

  NodeInterfaces node_interfaces_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::map<FeatureType,
    rclcpp::Subscription<hri_msgs::msg::IdsList>::SharedPtr> feature_subscribers_;

  std::map<ID, FacePtr> faces_;
  std::vector<std::function<void(FacePtr)>> face_callbacks_;
  std::vector<std::function<void(ID)>> face_lost_callbacks_;

  std::map<ID, BodyPtr> bodies_;
  std::vector<std::function<void(BodyPtr)>> body_callbacks_;
  std::vector<std::function<void(ID)>> body_lost_callbacks_;

  std::map<ID, VoicePtr> voices_;
  std::vector<std::function<void(VoicePtr)>> voice_callbacks_;
  std::vector<std::function<void(ID)>> voice_lost_callbacks_;

  std::map<ID, PersonPtr> persons_;
  std::vector<std::function<void(PersonPtr)>> person_callbacks_;
  std::vector<std::function<void(ID)>> person_lost_callbacks_;
  std::map<ID, PersonPtr> tracked_persons_;
  std::vector<std::function<void(PersonPtr)>> person_tracked_callbacks_;
  std::vector<std::function<void(ID)>> person_tracked_lost_callbacks_;

  std::string reference_frame_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace hri

#endif  // HRI__HRI_HPP_
