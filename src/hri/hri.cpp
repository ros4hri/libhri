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

#include "hri/hri.hpp"
#include <functional>
#include <memory>
#include <algorithm>
#include <iterator>
#include <tuple>
#include <utility>
#include <string> 
#include "hri/person.hpp"
#include "hri/face.hpp"
#include "hri/body.hpp"
#include "hri_msgs/msg/ids_list.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>


namespace hri
{

HRIListener::HRIListener(): _reference_frame("base_link"),_tf_buffer(), _tf_listener(_tf_buffer) 
{
  init();
}

HRIListener::~HRIListener()
{
  std::cout << "[TEST] LISTENER DYING" << std::endl;
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Closing the HRI Listener");

  faces.clear();
  bodies.clear();
  executor_->cancel();
  dedicated_listener_thread_->join();
//TODO implement unsub when available
  // for (auto& sub : feature_subscribers_)
  // {
  //   sub.second.shutdown();
  // }
}


// std::map<ID, FacePtr> HRIListener::getFaces() const
std::map<ID, FaceWeakConstPtr> HRIListener::getFaces() const
{
  // std::map<ID, FacePtr> result;
  std::map<ID, FaceWeakConstPtr> result;

  // creates a std::map of *weak* pointers from the internally managed list of
  // shared pointers
  RCLCPP_INFO(node_->get_logger(),"got face");
  for (auto const& f : faces)
  {
    result[f.first] = f.second;
  }
  

  return result;
}

std::map<ID, BodyWeakConstPtr> HRIListener::getBodies() const
{
  std::map<ID, BodyWeakConstPtr> result;

  // creates a std::map of *weak* pointers from the internally managed list of
  // shared pointers
  for (auto const& f : bodies)
  {
    result[f.first] = f.second;
  }

  return result;
}

std::map<ID, VoiceWeakConstPtr> HRIListener::getVoices() const
{
  std::map<ID, VoiceWeakConstPtr> result;

  // creates a std::map of *weak* pointers from the internally managed list of
  // shared pointers
  for (auto const& f : voices)
  {
    result[f.first] = f.second;
  }

  return result;
}

std::map<ID, PersonWeakConstPtr> HRIListener::getPersons() const
{
  std::map<ID, PersonWeakConstPtr> result;

  std::vector<PersonConstPtr> aliased;

  // creates a map of *weak* pointers from the internally managed list of
  // shared pointers
  for (auto const& f : persons)
  {
    if (f.second->alias().empty())
    {
      result[f.first] = f.second;
    }
    else
    {
      aliased.push_back(f.second);
    }
  }
  for (auto& r : result)
  {
    std::cout << "[RESULTBEFORE]: " << r.first << std::endl;
  }
  for (auto const& p : aliased)
  {
    if (result.count(p->alias()) != 0)
    {
      result[p->id()] = result[p->alias()];
    }
    else  // ouch! the person points to an inexistant alias! this should not happen
    {
      assert(false);
    }
  }
  for (auto& r : result)
  {
    std::cout << "[RESULTAFTER]: " << r.first << std::endl;
  }
  return result;
}

std::map<ID, PersonWeakConstPtr> HRIListener::getTrackedPersons() const
{
  std::map<ID, PersonWeakConstPtr> result;

  std::vector<PersonConstPtr> aliased;

  // creates a map of *weak* pointers from the internally managed list of
  // shared pointers
  for (auto const& f : tracked_persons)
  {
    if (f.second->alias().empty())
    {
      result[f.first] = f.second;
    }
    else
    {
      aliased.push_back(f.second);
    }
  }

  for (auto const& p : aliased)
  {
    if (result.count(p->alias()) != 0)
    {
      result[p->id()] = result[p->alias()];
    }
    else  // ouch! the person points to an inexistant alias! this should not happen.
    {
      assert(false);
    }
  }

  return result;
}

void HRIListener::init()
{
  // create a unique name for the node
  // but specify its name in .arguments to override any __node passed on the command line.
  // avoiding sstream because it's behavior can be overridden by external libraries.
  // See this issue: https://github.com/ros2/geometry2/issues/540

  rclcpp::NodeOptions node_options;
  // node_options.arguments({"--ros-args", "-r", "__node:=", "HRIlistener"});
  node_options.start_parameter_event_publisher(false);
  node_options.start_parameter_services(false);
  node_ = rclcpp::Node::make_shared("HRIlistener", node_options);

  auto node_params = node_->get_node_parameters_interface();
  auto node_topics = node_->get_node_topics_interface();
  auto qos = rclcpp::SystemDefaultsQoS();

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, true);
  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
  options.callback_group = callback_group_;
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Initialising the HRI Listener");
  

 //https://github.com/ros2/rclcpp/issues/273
 //https://github.com/ros2/rclcpp/issues/583 
//  std::function<void(const FeatureType::face::SharedPtr msg)> bound_callback_func =
//   std::bind(&HRIListener::onTrackedFeature, std::placeholders::_1, topic_name);
//map
  feature_subscribers_[FeatureType::face] = rclcpp::create_subscription<hri_msgs::msg::IdsList>(
      node_params, node_topics, "/humans/faces/tracked", qos,[this](const hri_msgs::msg::IdsList::SharedPtr tracked) {
        HRIListener::onTrackedFeature(FeatureType::face,tracked);
      }, options); 

  feature_subscribers_[FeatureType::body] = rclcpp::create_subscription<hri_msgs::msg::IdsList>(
      node_params, node_topics, "/humans/bodies/tracked", qos,[this](const hri_msgs::msg::IdsList::SharedPtr tracked) {
        HRIListener::onTrackedFeature(FeatureType::body,tracked);
      }, options); 

  feature_subscribers_[FeatureType::voice] = rclcpp::create_subscription<hri_msgs::msg::IdsList>(
      node_params, node_topics, "/humans/voices/tracked", qos,[this](const hri_msgs::msg::IdsList::SharedPtr tracked) {
        HRIListener::onTrackedFeature(FeatureType::voice,tracked);
      }, options); 

  feature_subscribers_[FeatureType::tracked_person] = rclcpp::create_subscription<hri_msgs::msg::IdsList>(
      node_params, node_topics, "/humans/persons/tracked", qos,[this](const hri_msgs::msg::IdsList::SharedPtr tracked) {
        HRIListener::onTrackedFeature(FeatureType::tracked_person,tracked);
      }, options); 

  feature_subscribers_[FeatureType::person] = rclcpp::create_subscription<hri_msgs::msg::IdsList>(
      node_params, node_topics, "/humans/persons/known", qos,[this](const hri_msgs::msg::IdsList::SharedPtr tracked) {
        HRIListener::onTrackedFeature(FeatureType::person,tracked);
    }, options);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, node_->get_node_base_interface());  
  dedicated_listener_thread_ = std::make_unique<std::thread>([&]() {executor_->spin();}); 
}

void HRIListener::onTrackedFeature(FeatureType feature, hri_msgs::msg::IdsList::SharedPtr tracked)
{
  // update the current list of tracked feature (face, body...) with
  // what has just been received on the respective /tracked topic.
  

  std::set<ID> new_ids;
  for (auto const& id : tracked->ids)
  {
    // new_ids.insert(ID(id.c_str()));
    new_ids.insert(ID(id));
  }


  std::set<ID> to_remove;
  std::set<ID> to_add;

  std::set<ID> current_ids;

  switch (feature)
  {
    case FeatureType::face:
      RCLCPP_INFO(node_->get_logger(),"got to face callback");
      for (auto const& kv : faces)
      {
        current_ids.insert(kv.first);
      }      
      break;
    case FeatureType::body:
    RCLCPP_INFO(node_->get_logger(),"got to body callback");
      for (auto const& kv : bodies)
      {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::voice:
    RCLCPP_INFO(node_->get_logger(),"got to voice callback");
      for (auto const& kv : voices)
      {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::person:
    RCLCPP_INFO(node_->get_logger(),"got to person callback");
      for (auto const& kv : persons)
      {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::tracked_person:
    RCLCPP_INFO(node_->get_logger(),"got to tracked person callback");
      for (auto const& kv : tracked_persons)
      {
        current_ids.insert(kv.first);
      }
      break;
  }



  for (auto id : new_ids)
  {
    RCLCPP_INFO(node_->get_logger(),"to add new ids:%s",id.c_str());
    if (current_ids.find(id) == current_ids.end())
    {
      to_add.insert(id);
    }
  }

  for (auto id : current_ids)
  {
    RCLCPP_INFO(node_->get_logger(),"to currents ids:%s",id.c_str());
    if (new_ids.find(id) == new_ids.end())
    {
      RCLCPP_INFO(node_->get_logger(),"to remove id:%s",id.c_str());
      to_remove.insert(id);
    }
  }

  switch (feature)
  {
    case FeatureType::face:
      for (auto id : to_remove)
      {
        faces.erase(id);

        // invoke all the callbacks
        for (auto& cb : face_lost_callbacks)
        {
          cb(id);
          RCLCPP_INFO(node_->get_logger(),"lost callbacks invoked");
        }
      }
      break;
    case FeatureType::body:
      for (auto id : to_remove)
      {
        bodies.erase(id);

        // invoke all the callbacks
        for (auto& cb : body_lost_callbacks)
        {
          cb(id);
        }
      }
      break;
    case FeatureType::voice:
      for (auto id : to_remove)
      {
        voices.erase(id);

        // invoke all the callbacks
        for (auto& cb : voice_lost_callbacks)
        {
          cb(id);
        }
      }
      break;
    case FeatureType::tracked_person:
      for (auto id : to_remove)
      {
        tracked_persons.erase(id);

        // also erase the *aliases* of this ID
        std::vector<ID> aliases;
        for (const auto& p : tracked_persons)
        {
          if (p.second->alias() == id)
          {
            aliases.push_back(p.first);
          }
        }
        for (auto alias : aliases)
        {
          tracked_persons.erase(alias);
        }

        // invoke all the callbacks
        for (auto& cb : person_tracked_lost_callbacks)
        {
          cb(id);
        }
      }
      break;
    case FeatureType::person:
      for (auto id : to_remove)
      {
        persons.erase(id);

        // also erase the *aliases* of this ID
        std::vector<ID> aliases;
        for (const auto& p : persons)
        {
          if (p.second->alias() == id)
          {
            aliases.push_back(p.first);
          }
        }
        for (auto alias : aliases)
        {
          persons.erase(alias);
        }

        // invoke all the callbacks
        for (auto& cb : person_lost_callbacks)
        {
          cb(id);
        }
      }
      break;
  }

  switch (feature)
  {
    case FeatureType::face:
      for (auto id  : to_add)
      {      
        
        auto face = std::make_shared<Face>(id, node_, &_tf_buffer, _reference_frame);
      
        face->init();        
        RCLCPP_INFO(node_->get_logger(),"got to face adding switch, id:%s",id.c_str());
        faces.insert({ id, face });

         RCLCPP_INFO(node_->get_logger(),"callbacks len :%d",face_callbacks.size());
        // invoke all the callbacks
        for (auto& cb : face_callbacks)
        {          
          cb(face);
          RCLCPP_INFO(node_->get_logger(),"face callback invoked");
        }
      }
      break;
    case FeatureType::body:
      for (auto id : to_add)
      {
        auto body = std::make_shared<Body>(id, node_, &_tf_buffer, _reference_frame);
        body->init();
        bodies.insert({ id, body });

        // invoke all the callbacks
        for (auto& cb : body_callbacks)
        {
          cb(body);
        }
      }
      break;
    case FeatureType::voice:
      for (auto id : to_add)
      {
        auto voice = std::make_shared<Voice>(id, node_, &_tf_buffer, _reference_frame);
        voice->init();
        voices.insert({ id, voice });

        // invoke all the callbacks
        for (auto& cb : voice_callbacks)
        {
          cb(voice);
        }
      }
      break;
    case FeatureType::person:
      for (auto id : to_add)
      {
        auto person = std::make_shared<Person>(id, node_, this, _tf_buffer, _reference_frame);
        person->init();
        RCLCPP_INFO(node_->get_logger(),"adding id to persons, id:%s",id.c_str());
        persons.insert({ id, person });

        // invoke all the callbacks
        for (auto& cb : person_callbacks)
        {
          cb(person);
        }
      }
      break;
    case FeatureType::tracked_person:
      for (auto id : to_add)
      {
        auto person = std::make_shared<Person>(id, node_, this, _tf_buffer, _reference_frame);
        person->init();
        RCLCPP_INFO(node_->get_logger(),"adding id to tracked_persons, id:%s",id.c_str());
        tracked_persons.insert({ id, person });

        // invoke all the callbacks
        for (auto& cb : person_tracked_callbacks)
        {
          cb(person);
        }
      }
      break;
  }
}

}  // namespace hri