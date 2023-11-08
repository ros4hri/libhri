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

#include "hri/hri.hpp"

#include <functional>
#include <memory>
#include <set>

#include "hri/body.hpp"
#include "hri/face.hpp"
#include "hri/person.hpp"
#include "hri/voice.hpp"
#include "hri/types.hpp"
#include "hri_msgs/msg/ids_list.hpp"

namespace hri
{

HRIListener::HRIListener(rclcpp::Node::SharedPtr node)
: node_(node), reference_frame_("base_link"), tf_listener_(tf_buffer_)
{
  init();
}

HRIListener::~HRIListener()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Closing the HRI Listener");
}

std::map<ID, FacePtr> HRIListener::getFaces() const
{
  std::map<ID, FacePtr> result;
  // creates a std::map of *shared* pointers from the internally managed list of
  // shared pointers
  for (auto const & f : faces_) {
    result[f.first] = f.second;
  }
  return result;
}

std::map<ID, BodyPtr> HRIListener::getBodies() const
{
  std::map<ID, BodyPtr> result;
  // creates a std::map of *shared* pointers from the internally managed list of
  // shared pointers
  for (auto const & f : bodies_) {
    result[f.first] = f.second;
  }
  return result;
}

std::map<ID, VoicePtr> HRIListener::getVoices() const
{
  std::map<ID, VoicePtr> result;
  // creates a std::map of *shared* pointers from the internally managed list of
  // shared pointers
  for (auto const & f : voices_) {
    result[f.first] = f.second;
  }
  return result;
}

std::map<ID, PersonPtr> HRIListener::getPersons() const
{
  std::map<ID, PersonPtr> result;
  std::vector<PersonPtr> aliased;

  // creates a map of *shared* pointers from the internally managed list of
  // shared pointers
  for (auto const & f : persons_) {
    if (!f.second->alias()) {
      result[f.first] = f.second;
    } else {
      aliased.push_back(f.second);
    }
  }

  for (auto const & p : aliased) {
    if (result.find(p->alias().value()) != result.end()) {
      result[p->id()] = result[p->alias().value()];
    } else {  // ouch! the person points to an inexistant alias! this should not happen
      RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Person " << p->id() << " has inexistant alias " << p->alias().value());
    }
  }

  return result;
}

std::map<ID, PersonPtr> HRIListener::getTrackedPersons() const
{
  std::map<ID, PersonPtr> result;
  std::vector<PersonPtr> aliased;

  // creates a map of *shared* pointers from the internally managed list of
  // shared pointers
  for (auto const & f : tracked_persons_) {
    if (!f.second->alias()) {
      result[f.first] = f.second;
    } else {
      aliased.push_back(f.second);
    }
  }

  for (auto const & p : aliased) {
    if (result.find(p->alias().value()) != result.end()) {
      result[p->id()] = result[p->alias().value()];
    } else {  // ouch! the person points to an inexistant alias! this should not happen.
      RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        "Person " << p->id() << " has inexistant alias " << p->alias().value());
    }
  }

  return result;
}

void HRIListener::init()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Initialising the HRI Listener");

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  auto default_qos = rclcpp::SystemDefaultsQoS();
  // There is a pending issue with binding functions with extra non-msg arguments, see
  // https://github.com/ros2/rclcpp/issues/766
  std::function<void(hri_msgs::msg::IdsList::ConstSharedPtr)> callback;

  callback = bind(&HRIListener::onTrackedFeature, this, FeatureType::kFace, std::placeholders::_1);
  feature_subscribers_[FeatureType::kFace] = node_->create_subscription<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", default_qos, callback, options);

  callback = bind(&HRIListener::onTrackedFeature, this, FeatureType::kBody, std::placeholders::_1);
  feature_subscribers_[FeatureType::kBody] = node_->create_subscription<hri_msgs::msg::IdsList>(
    "/humans/bodies/tracked", default_qos, callback, options);

  callback = bind(&HRIListener::onTrackedFeature, this, FeatureType::kVoice, std::placeholders::_1);
  feature_subscribers_[FeatureType::kVoice] = node_->create_subscription<hri_msgs::msg::IdsList>(
    "/humans/voices/tracked", default_qos, callback, options);

  callback = bind(
    &HRIListener::onTrackedFeature, this, FeatureType::kTrackedPerson, std::placeholders::_1);
  feature_subscribers_[FeatureType::kTrackedPerson] =
    node_->create_subscription<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", default_qos, callback, options);

  callback = bind(
    &HRIListener::onTrackedFeature, this, FeatureType::kPerson, std::placeholders::_1);
  feature_subscribers_[FeatureType::kPerson] = node_->create_subscription<hri_msgs::msg::IdsList>(
    "/humans/persons/known", default_qos, callback, options);
}

void HRIListener::onTrackedFeature(FeatureType feature, hri_msgs::msg::IdsList::ConstSharedPtr msg)
{
  // update the current list of tracked feature (face, body...) with
  // what has just been received on the respective /tracked topic.
  if (feature == FeatureType::kInvalid) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Received invalid tracked FeatureType");
    return;
  }

  std::set<ID> new_ids;
  for (auto const & id : msg->ids) {
    new_ids.insert(ID(id));
  }

  std::set<ID> current_ids;
  switch (feature) {
    case FeatureType::kFace:
      for (auto const & kv : faces_) {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::kBody:
      for (auto const & kv : bodies_) {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::kVoice:
      for (auto const & kv : voices_) {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::kPerson:
      for (auto const & kv : persons_) {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::kTrackedPerson:
      for (auto const & kv : tracked_persons_) {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::kInvalid:
      std::abort();  // unreachable
      break;
  }

  std::set<ID> to_add;
  for (auto id : new_ids) {
    if (current_ids.find(id) == current_ids.end()) {
      to_add.insert(id);
    }
  }

  std::set<ID> to_remove;
  for (auto id : current_ids) {
    if (new_ids.find(id) == new_ids.end()) {
      to_remove.insert(id);
    }
  }

  switch (feature) {
    case FeatureType::kFace:
      for (auto id : to_remove) {
        faces_.erase(id);

        // invoke all the callbacks
        for (auto & cb : face_lost_callbacks_) {
          cb(id);
        }
      }
      break;
    case FeatureType::kBody:
      for (auto id : to_remove) {
        bodies_.erase(id);

        // invoke all the callbacks
        for (auto & cb : body_lost_callbacks_) {
          cb(id);
        }
      }
      break;
    case FeatureType::kVoice:
      for (auto id : to_remove) {
        voices_.erase(id);

        // invoke all the callbacks
        for (auto & cb : voice_lost_callbacks_) {
          cb(id);
        }
      }
      break;
    case FeatureType::kTrackedPerson:
      for (auto id : to_remove) {
        tracked_persons_.erase(id);

        // also erase the *aliases* of this ID
        std::vector<ID> aliases;
        for (const auto & p : tracked_persons_) {
          if (p.second->alias() == id) {
            aliases.push_back(p.first);
          }
        }
        for (auto alias : aliases) {
          tracked_persons_.erase(alias);
        }

        // invoke all the callbacks
        for (auto & cb : person_tracked_lost_callbacks_) {
          cb(id);
        }
      }
      break;
    case FeatureType::kPerson:
      for (auto id : to_remove) {
        persons_.erase(id);

        // also erase the *aliases* of this ID
        std::vector<ID> aliases;
        for (const auto & p : persons_) {
          if (p.second->alias() == id) {
            aliases.push_back(p.first);
          }
        }
        for (auto alias : aliases) {
          persons_.erase(alias);
        }

        // invoke all the callbacks
        for (auto & cb : person_lost_callbacks_) {
          cb(id);
        }
      }
      break;
    case FeatureType::kInvalid:
      std::abort();  // unreachable
      break;
  }

  switch (feature) {
    case FeatureType::kFace:
      for (auto id  : to_add) {
        auto face =
          std::make_shared<Face>(id, node_, callback_group_, tf_buffer_, reference_frame_);
        face->init();
        faces_.insert({id, face});
        // invoke all the callbacks
        for (auto & cb : face_callbacks_) {
          cb(face);
        }
      }
      break;
    case FeatureType::kBody:
      for (auto id : to_add) {
        auto body =
          std::make_shared<Body>(id, node_, callback_group_, tf_buffer_, reference_frame_);
        body->init();
        bodies_.insert({id, body});

        // invoke all the callbacks
        for (auto & cb : body_callbacks_) {
          cb(body);
        }
      }
      break;
    case FeatureType::kVoice:
      for (auto id : to_add) {
        auto voice =
          std::make_shared<Voice>(id, node_, callback_group_, tf_buffer_, reference_frame_);
        voice->init();
        voices_.insert({id, voice});

        // invoke all the callbacks
        for (auto & cb : voice_callbacks_) {
          cb(voice);
        }
      }
      break;
    case FeatureType::kPerson:
      for (auto id : to_add) {
        auto person =
          std::make_shared<Person>(id, node_, callback_group_, this, tf_buffer_, reference_frame_);
        person->init();
        persons_.insert({id, person});

        // invoke all the callbacks
        for (auto & cb : person_callbacks_) {
          cb(person);
        }
      }
      break;
    case FeatureType::kTrackedPerson:
      for (auto id : to_add) {
        auto person =
          std::make_shared<Person>(id, node_, callback_group_, this, tf_buffer_, reference_frame_);
        person->init();
        tracked_persons_.insert({id, person});

        // invoke all the callbacks
        for (auto & cb : person_tracked_callbacks_) {
          cb(person);
        }
      }
      break;
    case FeatureType::kInvalid:
      std::abort();  // unreachable
      break;
  }
}

}  // namespace hri
