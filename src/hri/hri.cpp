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

#include <functional>
#include <memory>
#include <algorithm>
#include <iterator>
#include <tuple>
#include <utility>
#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "hri_msgs/msg/ids_list.hpp"

#include "hri/hri.hpp"
#include "hri/person.hpp"
#include "hri/face.hpp"
#include "hri/body.hpp"

namespace hri
{

HRIListener::HRIListener(rclcpp::Node::SharedPtr node)
: node_(node), _reference_frame("base_link"), _tf_listener(_tf_buffer)
{
  init();
}

HRIListener::~HRIListener()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Closing the HRI Listener");
  faces.clear();
  bodies.clear();
}


std::map<ID, FacePtr> HRIListener::getFaces() const
{
  std::map<ID, FacePtr> result;

  // creates a std::map of *shared* pointers from the internally managed list of
  // shared pointers
  for (auto const & f : faces) {
    result[f.first] = f.second;
  }


  return result;
}

std::map<ID, BodyPtr> HRIListener::getBodies() const
{
  std::map<ID, BodyPtr> result;

  // creates a std::map of *shared* pointers from the internally managed list of
  // shared pointers
  for (auto const & f : bodies) {
    result[f.first] = f.second;
  }

  return result;
}

std::map<ID, VoicePtr> HRIListener::getVoices() const
{
  std::map<ID, VoicePtr> result;

  // creates a std::map of *shared* pointers from the internally managed list of
  // shared pointers
  for (auto const & f : voices) {
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
  for (auto const & f : persons) {
    if (f.second->alias().empty()) {
      result[f.first] = f.second;
    } else {
      aliased.push_back(f.second);
    }
  }

  for (auto const & p : aliased) {
    if (result.count(p->alias()) != 0) {
      result[p->id()] = result[p->alias()];
    } else {  // ouch! the person points to an inexistant alias! this should not happen
      assert(false);
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
  for (auto const & f : tracked_persons) {
    if (f.second->alias().empty()) {
      result[f.first] = f.second;
    } else {
      aliased.push_back(f.second);
    }
  }

  for (auto const & p : aliased) {
    if (result.count(p->alias()) != 0) {
      result[p->id()] = result[p->alias()];
    } else {  // ouch! the person points to an inexistant alias! this should not happen.
      assert(false);
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
  auto qos = rclcpp::SystemDefaultsQoS();

  feature_subscribers_[FeatureType::face] = node_->create_subscription<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", qos,
    [this](const hri_msgs::msg::IdsList::SharedPtr tracked) {
      HRIListener::onTrackedFeature(FeatureType::face, tracked);
    }, options);

  feature_subscribers_[FeatureType::body] = node_->create_subscription<hri_msgs::msg::IdsList>(
    "/humans/bodies/tracked", qos,
    [this](const hri_msgs::msg::IdsList::SharedPtr tracked) {
      HRIListener::onTrackedFeature(FeatureType::body, tracked);
    }, options);

  feature_subscribers_[FeatureType::voice] = node_->create_subscription<hri_msgs::msg::IdsList>(
    "/humans/voices/tracked", qos,
    [this](const hri_msgs::msg::IdsList::SharedPtr tracked) {
      HRIListener::onTrackedFeature(FeatureType::voice, tracked);
    }, options);

  feature_subscribers_[FeatureType::tracked_person] =
    node_->create_subscription<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", qos,
    [this](const hri_msgs::msg::IdsList::SharedPtr tracked) {
      HRIListener::onTrackedFeature(FeatureType::tracked_person, tracked);
    }, options);

  feature_subscribers_[FeatureType::person] = node_->create_subscription<hri_msgs::msg::IdsList>(
    "/humans/persons/known", qos,
    [this](const hri_msgs::msg::IdsList::SharedPtr tracked) {
      HRIListener::onTrackedFeature(FeatureType::person, tracked);
    }, options);
}

void HRIListener::onTrackedFeature(FeatureType feature, hri_msgs::msg::IdsList::SharedPtr tracked)
{
  // update the current list of tracked feature (face, body...) with
  // what has just been received on the respective /tracked topic.
  if (feature == FeatureType::invalid) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Received invalid tracked FeatureType");
    return;
  }

  std::set<ID> new_ids;
  for (auto const & id : tracked->ids) {
    new_ids.insert(ID(id));
  }


  std::set<ID> to_remove;
  std::set<ID> to_add;

  std::set<ID> current_ids;

  switch (feature) {
    case FeatureType::face:
      for (auto const & kv : faces) {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::body:
      for (auto const & kv : bodies) {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::voice:
      for (auto const & kv : voices) {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::person:
      for (auto const & kv : persons) {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::tracked_person:
      for (auto const & kv : tracked_persons) {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::invalid:
      std::abort();  // unreachable
      break;
  }


  for (auto id : new_ids) {
    if (current_ids.find(id) == current_ids.end()) {
      to_add.insert(id);
    }
  }

  for (auto id : current_ids) {
    if (new_ids.find(id) == new_ids.end()) {
      to_remove.insert(id);
    }
  }

  switch (feature) {
    case FeatureType::face:
      for (auto id : to_remove) {
        faces.erase(id);

        // invoke all the callbacks
        for (auto & cb : face_lost_callbacks) {
          cb(id);
        }
      }
      break;
    case FeatureType::body:
      for (auto id : to_remove) {
        bodies.erase(id);

        // invoke all the callbacks
        for (auto & cb : body_lost_callbacks) {
          cb(id);
        }
      }
      break;
    case FeatureType::voice:
      for (auto id : to_remove) {
        voices.erase(id);

        // invoke all the callbacks
        for (auto & cb : voice_lost_callbacks) {
          cb(id);
        }
      }
      break;
    case FeatureType::tracked_person:
      for (auto id : to_remove) {
        tracked_persons.erase(id);

        // also erase the *aliases* of this ID
        std::vector<ID> aliases;
        for (const auto & p : tracked_persons) {
          if (p.second->alias() == id) {
            aliases.push_back(p.first);
          }
        }
        for (auto alias : aliases) {
          tracked_persons.erase(alias);
        }

        // invoke all the callbacks
        for (auto & cb : person_tracked_lost_callbacks) {
          cb(id);
        }
      }
      break;
    case FeatureType::person:
      for (auto id : to_remove) {
        persons.erase(id);

        // also erase the *aliases* of this ID
        std::vector<ID> aliases;
        for (const auto & p : persons) {
          if (p.second->alias() == id) {
            aliases.push_back(p.first);
          }
        }
        for (auto alias : aliases) {
          persons.erase(alias);
        }

        // invoke all the callbacks
        for (auto & cb : person_lost_callbacks) {
          cb(id);
        }
      }
      break;
    case FeatureType::invalid:
      std::abort();  // unreachable
      break;
  }

  switch (feature) {
    case FeatureType::face:
      for (auto id  : to_add) {
        auto face =
          std::make_shared<Face>(id, node_, callback_group_, _tf_buffer, _reference_frame);

        face->init();
        faces.insert({id, face});

        // invoke all the callbacks
        for (auto & cb : face_callbacks) {
          cb(face);
        }
      }
      break;
    case FeatureType::body:
      for (auto id : to_add) {
        auto body =
          std::make_shared<Body>(id, node_, callback_group_, _tf_buffer, _reference_frame);
        body->init();
        bodies.insert({id, body});

        // invoke all the callbacks
        for (auto & cb : body_callbacks) {
          cb(body);
        }
      }
      break;
    case FeatureType::voice:
      for (auto id : to_add) {
        auto voice =
          std::make_shared<Voice>(id, node_, callback_group_, _tf_buffer, _reference_frame);
        voice->init();
        voices.insert({id, voice});

        // invoke all the callbacks
        for (auto & cb : voice_callbacks) {
          cb(voice);
        }
      }
      break;
    case FeatureType::person:
      for (auto id : to_add) {
        auto person =
          std::make_shared<Person>(id, node_, callback_group_, this, _tf_buffer, _reference_frame);
        person->init();
        persons.insert({id, person});

        // invoke all the callbacks
        for (auto & cb : person_callbacks) {
          cb(person);
        }
      }
      break;
    case FeatureType::tracked_person:
      for (auto id : to_add) {
        auto person =
          std::make_shared<Person>(id, node_, callback_group_, this, _tf_buffer, _reference_frame);
        person->init();
        tracked_persons.insert({id, person});

        // invoke all the callbacks
        for (auto & cb : person_tracked_callbacks) {
          cb(person);
        }
      }
      break;
    case FeatureType::invalid:
      std::abort();  // unreachable
      break;
  }
}

}  // namespace hri
