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

#include <thread>
#include <chrono>
#include <memory>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/transform_stamped.h"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/static_transform_broadcaster.h"

#include "hri/hri.hpp"
#include "hri/face.hpp"
#include "hri/person.hpp"
#include "hri/voice.hpp"
#include "hri_msgs/msg/engagement_level.hpp"
#include "hri_msgs/msg/ids_list.hpp"
#include "hri_msgs/msg/live_speech.hpp"
#include "hri_msgs/msg/normalized_region_of_interest2_d.hpp"
#include "hri_msgs/msg/soft_biometrics.hpp"


using namespace std::chrono_literals;

TEST(libhri_tests, GetFaces)
{
  auto node = rclcpp::Node::make_shared("test_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);


  auto publisher = node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);

  {
    auto hri_listener = std::make_shared<hri::HRIListener>();

    ASSERT_EQ(publisher->get_subscription_count(), 1U);
    ASSERT_EQ(hri_listener->getFaces().size(), 0U);

    auto ids = hri_msgs::msg::IdsList();

    RCLCPP_INFO(node->get_logger(), "[A]");
    ids.ids = {"A"};

    rclcpp::Rate rate(30);
    auto start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      publisher->publish(ids);
      executor.spin_some();
      rate.sleep();
    }

    auto faces = hri_listener->getFaces();

    EXPECT_EQ(faces.size(), 1U);
    ASSERT_TRUE(faces.find("A") != faces.end());
    EXPECT_EQ(faces["A"]->id(), "A");

    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      publisher->publish(ids);
      executor.spin_some();
      rate.sleep();
    }

    EXPECT_EQ(hri_listener->getFaces().size(), 1U);

    RCLCPP_INFO(node->get_logger(), "[A,B]");
    ids.ids = {"A", "B"};

    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      publisher->publish(ids);
      executor.spin_some();
      rate.sleep();
    }

    faces = hri_listener->getFaces();

    EXPECT_EQ(faces.size(), 2U);
    EXPECT_TRUE(faces.find("A") != faces.end());
    EXPECT_TRUE(faces.find("B") != faces.end());

    RCLCPP_INFO(node->get_logger(), "[A,B]");

    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      publisher->publish(ids);
      executor.spin_some();
      rate.sleep();
    }

    EXPECT_EQ(hri_listener->getFaces().size(), 2U);

    RCLCPP_INFO(node->get_logger(), "[B]");

    ids.ids = {"B"};

    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      publisher->publish(ids);
      executor.spin_some();
      rate.sleep();
    }

    faces = hri_listener->getFaces();

    EXPECT_EQ(faces.size(), 1U);
    EXPECT_TRUE(faces.find("A") == faces.end());
    ASSERT_TRUE(faces.find("B") != faces.end());

    std::shared_ptr<const hri::Face> face_b = faces["B"];
    int use_count_before_deletion = face_b.use_count();

    RCLCPP_INFO(node->get_logger(), "[]");

    ids.ids = {};
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      publisher->publish(ids);
      executor.spin_some();
      rate.sleep();
    }

    EXPECT_EQ(hri_listener->getFaces().size(), 0U);
    // check face B is not used anymore by hri_listener!
    EXPECT_EQ(face_b.use_count(), --use_count_before_deletion);

    hri_listener.reset();
  }

  EXPECT_EQ(publisher->get_subscription_count(), 0U);
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri_tests, GetFacesRoi)
{
  auto node = rclcpp::Node::make_shared("test_node");
  auto hri_listener = std::make_shared<hri::HRIListener>();

  rclcpp::Rate rate(30);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto pub = node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);

  // roi topic is transient local
  auto pub_r1 = node->create_publisher<hri::Face::RegionOfInterest>(
    "/humans/faces/A/roi", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  // roi topic is transient local
  auto pub_r2 = node->create_publisher<hri::Face::RegionOfInterest>(
    "/humans/faces/B/roi", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  auto ids = hri_msgs::msg::IdsList();
  ids.ids = {"A"};

  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    pub->publish(ids);
    executor.spin_some();
    rate.sleep();
  }

  EXPECT_EQ(pub_r1->get_subscription_count(), 1U);

  ids.ids = {"B"};

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    pub->publish(ids);
    executor.spin_some();
    rate.sleep();
  }

  EXPECT_EQ(pub_r1->get_subscription_count(), 0U);
  EXPECT_EQ(pub_r2->get_subscription_count(), 1U);

  auto faces = hri_listener->getFaces();

  ASSERT_TRUE(faces.find("B") != faces.end());

  auto roi = hri::Face::RegionOfInterest();

  {
    auto face = faces["B"];

    EXPECT_FALSE(face == nullptr);
    EXPECT_EQ(face->ns(), "/humans/faces/B");
    EXPECT_FLOAT_EQ((face->roi().xmax - face->roi().xmin), 0.f);

    roi.xmin = 0.1;

    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub_r2->publish(roi);
      executor.spin_some();
      rate.sleep();
    }

    EXPECT_FLOAT_EQ(face->roi().xmin, 0.1f);

    roi.xmin = 0.2;

    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub_r2->publish(roi);
      executor.spin_some();
      rate.sleep();
    }

    EXPECT_FLOAT_EQ(face->roi().xmin, 0.2f);
  }

  // RoI of face A published *before* face A is published in /faces/tracked,
  // but should still get its RoI, as /roi is latched.

  ids.ids = {"B", "A"};

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    pub->publish(ids);
    pub_r1->publish(roi);
    executor.spin_some();
    rate.sleep();
  }

  faces = hri_listener->getFaces();
  {
    auto face_a = faces["A"];
    auto face_b = faces["B"];

    ASSERT_FALSE(face_a == nullptr);
    ASSERT_FALSE(face_b == nullptr);

    EXPECT_EQ(face_a->ns(), "/humans/faces/A");
    EXPECT_FLOAT_EQ(face_a->roi().xmin, 0.2f);
    EXPECT_EQ(face_b->ns(), "/humans/faces/B");
    EXPECT_FLOAT_EQ(face_b->roi().xmin, 0.2f);
  }

  executor.remove_node(node);
  executor.cancel();
  hri_listener.reset();
}


TEST(libhri_tests, GetBodies)
{
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto pub = node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/bodies/tracked", 1);

  {
    auto hri_listener = std::make_shared<hri::HRIListener>();
    ASSERT_EQ(pub->get_subscription_count(), 1U);

    auto ids = hri_msgs::msg::IdsList();
    RCLCPP_INFO(node->get_logger(), "[A]");
    ids.ids = {"A"};

    auto start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    auto bodies = hri_listener->getBodies();

    EXPECT_EQ(bodies.size(), 1U);
    ASSERT_TRUE(bodies.find("A") != bodies.end());
    EXPECT_EQ(bodies["A"]->id(), "A");

    RCLCPP_INFO(node->get_logger(), "[A]");

    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    EXPECT_EQ(hri_listener->getBodies().size(), 1U);

    RCLCPP_INFO(node->get_logger(), "[A,B]");
    ids.ids = {"A", "B"};

    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    bodies = hri_listener->getBodies();

    EXPECT_EQ(bodies.size(), 2U);
    EXPECT_TRUE(bodies.find("A") != bodies.end());
    EXPECT_TRUE(bodies.find("B") != bodies.end());

    RCLCPP_INFO(node->get_logger(), "[A,B]");
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }

    EXPECT_EQ(hri_listener->getBodies().size(), 2U);

    RCLCPP_INFO(node->get_logger(), "[B]");
    ids.ids = {"B"};

    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }

    bodies = hri_listener->getBodies();

    EXPECT_EQ(bodies.size(), 1U);
    EXPECT_EQ(bodies.find("A"), bodies.end());
    ASSERT_TRUE(bodies.find("B") != bodies.end());

    std::shared_ptr<const hri::Body> body_b = bodies["B"];
    int use_count_before_deletion = body_b.use_count();

    RCLCPP_INFO(node->get_logger(), "[]");
    ids.ids = {};

    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }

    EXPECT_EQ(hri_listener->getBodies().size(), 0U);
    // check body B is not used anymore by hri_listener!
    EXPECT_EQ(body_b.use_count(), --use_count_before_deletion);

    hri_listener.reset();
  }

  EXPECT_EQ(pub->get_subscription_count(), 0U);

  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri_tests, GetVoices)
{
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);


  auto pub = node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/voices/tracked", 1);

  {
    auto hri_listener = std::make_shared<hri::HRIListener>();
    ASSERT_EQ(pub->get_subscription_count(), 1U);

    auto ids = hri_msgs::msg::IdsList();

    RCLCPP_INFO(node->get_logger(), "[A]");
    ids.ids = {"A"};
    auto start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    auto voices = hri_listener->getVoices();
    EXPECT_EQ(voices.size(), 1U);
    ASSERT_TRUE(voices.find("A") != voices.end());
    EXPECT_EQ(voices["A"]->id(), "A");

    RCLCPP_INFO(node->get_logger(), "[A]");
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    EXPECT_EQ(hri_listener->getVoices().size(), 1U);


    RCLCPP_INFO(node->get_logger(), "[A,B]");
    ids.ids = {"A", "B"};
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    voices = hri_listener->getVoices();
    EXPECT_EQ(voices.size(), 2U);
    EXPECT_TRUE(voices.find("A") != voices.end());
    EXPECT_TRUE(voices.find("B") != voices.end());

    RCLCPP_INFO(node->get_logger(), "[A,B]");
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    EXPECT_EQ(hri_listener->getVoices().size(), 2U);


    RCLCPP_INFO(node->get_logger(), "[B]");
    ids.ids = {"B"};
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    voices = hri_listener->getVoices();
    EXPECT_EQ(voices.size(), 1U);
    EXPECT_TRUE(voices.find("A") == voices.end());
    ASSERT_TRUE(voices.find("B") != voices.end());

    std::shared_ptr<const hri::Voice> voice_b = voices["B"];
    int use_count_before_deletion = voice_b.use_count();

    RCLCPP_INFO(node->get_logger(), "[]");
    ids.ids = {};
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    EXPECT_EQ(hri_listener->getVoices().size(), 0U);
    // check voice B is not used anymore by hri_listener!
    EXPECT_EQ(voice_b.use_count(), --use_count_before_deletion);

    hri_listener.reset();
  }

  EXPECT_EQ(pub->get_subscription_count(), 0U);
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri_tests, GetKnownPersons)
{
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);


  auto pub = node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/known", 1);

  {
    auto hri_listener = std::make_shared<hri::HRIListener>();
    ASSERT_EQ(pub->get_subscription_count(), 1U);

    auto ids = hri_msgs::msg::IdsList();

    RCLCPP_INFO(node->get_logger(), "[A]");
    ids.ids = {"A"};
    auto start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    auto persons = hri_listener->getPersons();
    EXPECT_EQ(persons.size(), 1U);
    ASSERT_TRUE(persons.find("A") != persons.end());
    EXPECT_EQ(persons["A"]->id(), "A");

    RCLCPP_INFO(node->get_logger(), "[A]");
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    EXPECT_EQ(hri_listener->getPersons().size(), 1U);

    RCLCPP_INFO(node->get_logger(), "[A,B]");
    ids.ids = {"A", "B"};
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    persons = hri_listener->getPersons();
    EXPECT_EQ(persons.size(), 2U);
    EXPECT_TRUE(persons.find("A") != persons.end());
    EXPECT_TRUE(persons.find("B") != persons.end());

    RCLCPP_INFO(node->get_logger(), "[A,B]");
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    EXPECT_EQ(hri_listener->getPersons().size(), 2U);

    RCLCPP_INFO(node->get_logger(), "[B]");
    ids.ids = {"B"};
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 300ms) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    persons = hri_listener->getPersons();
    EXPECT_EQ(persons.size(), 1U) << "known persons can go down in case of eg an anonymous person";
    EXPECT_TRUE(persons.find("A") == persons.end());
    ASSERT_TRUE(persons.find("B") != persons.end());

    std::shared_ptr<const hri::Person> person_b = persons["B"];
    EXPECT_TRUE(person_b != nullptr);  // person B exists!

    RCLCPP_INFO(node->get_logger(), "[]");
    ids.ids = {};
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    EXPECT_EQ(hri_listener->getPersons().size(), 0U);

    EXPECT_TRUE(person_b != nullptr);  // person B still exists!

    hri_listener.reset();
  }

  EXPECT_EQ(pub->get_subscription_count(), 0U);
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri_tests, GetTrackedPersons)
{
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);


  auto pub = node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);

  {
    auto hri_listener = std::make_shared<hri::HRIListener>();


    ASSERT_EQ(pub->get_subscription_count(), 1U);


    auto ids = hri_msgs::msg::IdsList();

    RCLCPP_INFO(node->get_logger(), "[A]");
    ids.ids = {"A"};
    auto start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    auto known_persons = hri_listener->getPersons();
    EXPECT_EQ(known_persons.size(), 0U);

    auto persons = hri_listener->getTrackedPersons();
    EXPECT_EQ(persons.size(), 1U);
    ASSERT_TRUE(persons.find("A") != persons.end());
    EXPECT_EQ(persons["A"]->id(), "A");

    RCLCPP_INFO(node->get_logger(), "[A]");
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    EXPECT_EQ(hri_listener->getTrackedPersons().size(), 1U);

    RCLCPP_INFO(node->get_logger(), "[A,B]");
    ids.ids = {"A", "B"};
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    persons = hri_listener->getTrackedPersons();
    EXPECT_EQ(persons.size(), 2U);
    EXPECT_TRUE(persons.find("A") != persons.end());
    EXPECT_TRUE(persons.find("B") != persons.end());

    RCLCPP_INFO(node->get_logger(), "[A,B]");
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    EXPECT_EQ(hri_listener->getTrackedPersons().size(), 2U);

    RCLCPP_INFO(node->get_logger(), "[B]");
    ids.ids = {"B"};
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    persons = hri_listener->getTrackedPersons();
    EXPECT_EQ(persons.size(), 1U);
    EXPECT_TRUE(persons.find("A") == persons.end());
    ASSERT_TRUE(persons.find("B") != persons.end());

    std::shared_ptr<const hri::Person> person_b = persons["B"];
    EXPECT_TRUE(person_b != nullptr);  // person B exists!

    RCLCPP_INFO(node->get_logger(), "[]");
    ids.ids = {};
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    EXPECT_EQ(hri_listener->getTrackedPersons().size(), 0U);

    EXPECT_TRUE(person_b != nullptr);  // person B still exists!

    hri_listener.reset();
  }

  EXPECT_EQ(pub->get_subscription_count(), 0U);
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri_tests, PersonAttributes)
{
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto hri_listener = std::make_shared<hri::HRIListener>();

  auto person_pub = node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);
  auto face_pub = node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto person_face_pub = node->create_publisher<std_msgs::msg::String>(
    "/humans/persons/p1/face_id", 1);

  auto person_ids = hri_msgs::msg::IdsList();
  person_ids.ids = {"p1"};
  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    person_pub->publish(person_ids);
    executor.spin_some();
    rate.sleep();
  }

  auto face_ids = hri_msgs::msg::IdsList();
  face_ids.ids = {"f1", "f2"};

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    face_pub->publish(face_ids);
    executor.spin_some();
    rate.sleep();
  }

  auto p1 = hri_listener->getTrackedPersons()["p1"];

  ASSERT_FALSE(p1->anonymous());

  auto face0 = p1->face();

  ASSERT_EQ(face0, nullptr);

  auto face_id = std_msgs::msg::String();
  face_id.data = "f1";

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    person_face_pub->publish(face_id);
    executor.spin_some();
    rate.sleep();
  }
  auto face1 = hri_listener->getTrackedPersons()["p1"]->face();

  ASSERT_NE(face1, nullptr);
  ASSERT_EQ(face1->id(), "f1");

  hri_listener.reset();
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri_tests, AnonymousPersonsAndAliases)
{
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto hri_listener = std::make_shared<hri::HRIListener>();

  auto person_pub = node->create_publisher<hri_msgs::msg::IdsList>("humans/persons/tracked", 1);
  auto p1_anon_pub = node->create_publisher<std_msgs::msg::Bool>("/humans/persons/p1/anonymous", 1);
  auto p2_anon_pub = node->create_publisher<std_msgs::msg::Bool>("/humans/persons/p2/anonymous", 1);

  auto face_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/faces/tracked", 1);
  auto p1_face_pub = node->create_publisher<std_msgs::msg::String>("/humans/persons/p1/face_id", 1);
  auto p2_face_pub = node->create_publisher<std_msgs::msg::String>("/humans/persons/p2/face_id", 1);

  auto p2_alias_pub = node->create_publisher<std_msgs::msg::String>("/humans/persons/p2/alias", 1);

  auto person_ids = hri_msgs::msg::IdsList();
  person_ids.ids = {"p1", "p2"};

  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    person_pub->publish(person_ids);
    executor.spin_some();
    rate.sleep();
  }

  auto face_ids = hri_msgs::msg::IdsList();
  face_ids.ids = {"f1", "f2"};

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    face_pub->publish(face_ids);
    executor.spin_some();
    rate.sleep();
  }

  // each person is associated to a face
  auto face_id = std_msgs::msg::String();
  face_id.data = "f1";

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    p1_face_pub->publish(face_id);
    executor.spin_some();
    rate.sleep();
  }

  face_id.data = "f2";

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    p2_face_pub->publish(face_id);
    executor.spin_some();
    rate.sleep();
  }

  std_msgs::msg::Bool msg;
  msg.data = false;

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    p1_anon_pub->publish(msg);
    executor.spin_some();
    rate.sleep();
  }

  msg.data = true;

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    p2_anon_pub->publish(msg);
    executor.spin_some();
    rate.sleep();
  }


  ASSERT_EQ(hri_listener->getTrackedPersons().size(), 2U);

  auto p1 = hri_listener->getTrackedPersons()["p1"];

  {
    auto p2 = hri_listener->getTrackedPersons()["p2"];

    ASSERT_TRUE(p1->anonymous());  // the anonymous optional flag should have been set
    ASSERT_TRUE(p2->anonymous());  // the anonymous optional flag should have been set
    ASSERT_FALSE(*(p1->anonymous()));
    ASSERT_TRUE(*(p2->anonymous()));

    // being anonymous or not should have no impact on face associations
    ASSERT_EQ(p1->face()->id(), "f1");
    ASSERT_EQ(p2->face()->id(), "f2");
  }

  ///////////// ALIASES ///////////////////////////

  // set p2 as an alias of p1
  auto alias_id = std_msgs::msg::String();
  alias_id.data = "p1";

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    p2_alias_pub->publish(alias_id);
    executor.spin_some();
    rate.sleep();
  }


  ASSERT_EQ(hri_listener->getTrackedPersons().size(), 2U);

  {
    auto p2 = hri_listener->getTrackedPersons()["p2"];

    ASSERT_EQ(p1, p2) << "p2 should now point to the same person as p1";

    ASSERT_EQ(p2->face()->id(), "f1") << "p2's face now points to f1";
  }
  // remove the alias
  alias_id.data = "";

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    p2_alias_pub->publish(alias_id);
    executor.spin_some();
    rate.sleep();
  }

  {
    auto p2 = hri_listener->getTrackedPersons()["p2"];

    ASSERT_NE(p1, p2) << "p2 is not anymore the same person as p1";

    ASSERT_EQ(p2->face()->id(), "f2")
      << "p2's face should still points to its former f2 face";
  }

  // republish the alias
  alias_id.data = "p1";

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    p2_alias_pub->publish(alias_id);
    executor.spin_some();
    rate.sleep();
  }

  auto p2 = hri_listener->getTrackedPersons()["p2"];

  ASSERT_EQ(p1, p2) << "p2 is again the same person as p1";

  // delete p1 -> p2 should be deleted as well

  person_ids.ids = {"p2"};
  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    person_pub->publish(person_ids);
    executor.spin_some();
    rate.sleep();
  }

  // ASSERT_EQ(hri_listener->getTrackedPersons().size(), 0U)
  //     << "the aliased person should have been deleted with its alias";

  hri_listener.reset();
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri_tests, SoftBiometrics)
{
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto hri_listener = std::make_shared<hri::HRIListener>();

  auto person_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/persons/tracked", 1);
  auto face_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/faces/tracked", 1);
  auto person_face_pub = node->create_publisher<std_msgs::msg::String>(
    "/humans/persons/p1/face_id",
    1);
  auto softbiometrics_pub = node->create_publisher<hri_msgs::msg::SoftBiometrics>(
    "/humans/faces/f1/softbiometrics", 1);

  auto person_ids = hri_msgs::msg::IdsList();
  person_ids.ids = {"p1"};

  auto face_ids = hri_msgs::msg::IdsList();
  face_ids.ids = {"f1"};

  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    person_pub->publish(person_ids);
    face_pub->publish(face_ids);
    executor.spin_some();
    rate.sleep();
  }

  auto softbiometrics_msg = hri_msgs::msg::SoftBiometrics();
  softbiometrics_msg.age = 45;
  softbiometrics_msg.age_confidence = 0.8;
  softbiometrics_msg.gender = hri_msgs::msg::SoftBiometrics::FEMALE;
  softbiometrics_msg.gender_confidence = 0.7;

  auto face_id = std_msgs::msg::String();
  face_id.data = "f1";

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    softbiometrics_pub->publish(softbiometrics_msg);
    person_face_pub->publish(face_id);
    executor.spin_some();
    rate.sleep();
  }

  auto face = hri_listener->getTrackedPersons()["p1"]->face();

  ASSERT_EQ(face->id(), "f1");

  ASSERT_TRUE(face->age());
  ASSERT_FLOAT_EQ(*(face->age()), 45.f);
  ASSERT_TRUE(face->gender());
  ASSERT_EQ(*(face->gender()), hri::FEMALE);

  softbiometrics_msg.gender = hri_msgs::msg::SoftBiometrics::OTHER;
  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    softbiometrics_pub->publish(softbiometrics_msg);
    executor.spin_some();
    rate.sleep();
  }


  ASSERT_EQ(*(face->gender()), hri::OTHER);

  softbiometrics_msg.gender = hri_msgs::msg::SoftBiometrics::UNDEFINED;

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    softbiometrics_pub->publish(softbiometrics_msg);
    executor.spin_some();
    rate.sleep();
  }

  ASSERT_FALSE(face->gender());


  hri_listener.reset();
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri_tests, EngagementLevel)
{
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto hri_listener = std::make_shared<hri::HRIListener>();


  auto person_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/persons/tracked", 1);

  auto engagement_pub =
    node->create_publisher<hri_msgs::msg::EngagementLevel>(
    "/humans/persons/p1/engagement_status",
    1);

  auto person_ids = hri_msgs::msg::IdsList();
  person_ids.ids = {"p1"};

  auto msg = hri_msgs::msg::EngagementLevel();
  msg.level = hri_msgs::msg::EngagementLevel::DISENGAGED;

  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    person_pub->publish(person_ids);
    engagement_pub->publish(msg);
    executor.spin_some();
    rate.sleep();
  }


  auto p = hri_listener->getTrackedPersons()["p1"];
  ASSERT_TRUE(p->engagement_status());
  ASSERT_EQ(*(p->engagement_status()), hri::DISENGAGED);

  msg.level = hri_msgs::msg::EngagementLevel::ENGAGED;

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 300ms) {
    engagement_pub->publish(msg);
    executor.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(*(p->engagement_status()), hri::ENGAGED);

  msg.level = hri_msgs::msg::EngagementLevel::UNKNOWN;

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 300ms) {
    engagement_pub->publish(msg);
    executor.spin_some();
    rate.sleep();
  }

  ASSERT_FALSE(p->engagement_status());

  hri_listener.reset();
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri_tests, Callback)
{
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto hri_listener = std::make_shared<hri::HRIListener>();

  int face_callbacks_invoked = 0;
  int lost_face_callbacks_invoked = 0;

  int body_callbacks_invoked = 0;
  int lost_body_callbacks_invoked = 0;

  int voice_callbacks_invoked = 0;
  int lost_voice_callbacks_invoked = 0;

  int person_callbacks_invoked = 0;

  int ontracked_person_callbacks_invoked = 0;
  int ontracked_lost_person_callbacks_invoked = 0;

  hri_listener->onFace(
    [&]([[maybe_unused]] hri::FacePtr face) {
      face_callbacks_invoked++;
    });

  hri_listener->onFaceLost(
    [&]([[maybe_unused]] hri::ID face_lost) {
      lost_face_callbacks_invoked++;
    });

  hri_listener->onBody(
    [&]([[maybe_unused]] hri::BodyPtr body) {
      body_callbacks_invoked++;
    });

  hri_listener->onBodyLost(
    [&]([[maybe_unused]] hri::ID body_lost) {
      lost_body_callbacks_invoked++;
    });

  hri_listener->onVoice(
    [&]([[maybe_unused]] hri::VoicePtr voice) {
      voice_callbacks_invoked++;
    });

  hri_listener->onVoiceLost(
    [&]([[maybe_unused]] hri::ID voice_lost) {
      lost_voice_callbacks_invoked++;
    });

  hri_listener->onPerson(
    [&]([[maybe_unused]] hri::PersonPtr person) {
      person_callbacks_invoked++;
    });

  hri_listener->onTrackedPerson(
    [&]([[maybe_unused]] hri::PersonPtr tracked_person) {
      ontracked_person_callbacks_invoked++;
    });

  hri_listener->onTrackedPersonLost(
    [&]([[maybe_unused]] hri::ID tracked_person_lost) {
      ontracked_lost_person_callbacks_invoked++;
    });

  auto ids = hri_msgs::msg::IdsList();

  auto face_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/faces/tracked", 1);
  auto body_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/bodies/tracked", 1);
  auto voice_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/voices/tracked", 1);
  auto person_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/persons/known", 1);
  auto person_tracked_pub = node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);

  EXPECT_EQ(face_callbacks_invoked, 0);
  EXPECT_EQ(lost_face_callbacks_invoked, 0);

  ids.ids = {"id1"};

  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 300ms) {
    face_pub->publish(ids);
    executor.spin_some();
    rate.sleep();
  }

  EXPECT_EQ(face_callbacks_invoked, 1);
  EXPECT_EQ(lost_face_callbacks_invoked, 0);

  ids.ids = {"id1", "id2"};

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 300ms) {
    face_pub->publish(ids);
    executor.spin_some();
    rate.sleep();
  }

  EXPECT_EQ(face_callbacks_invoked, 2);


  ids.ids = {"id3", "id4"};

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 300ms) {
    face_pub->publish(ids);
    executor.spin_some();
    rate.sleep();
  }

  EXPECT_EQ(lost_face_callbacks_invoked, 2);

  ids.ids = {"id1", "id2"};

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 300ms) {
    body_pub->publish(ids);
    executor.spin_some();
    rate.sleep();
  }

  EXPECT_EQ(body_callbacks_invoked, 2);
  EXPECT_EQ(lost_body_callbacks_invoked, 0);

  ids.ids = {"id1", "id2", "id3"};

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 300ms) {
    face_pub->publish(ids);
    body_pub->publish(ids);
    executor.spin_some();
    rate.sleep();
  }

  EXPECT_EQ(face_callbacks_invoked, 6);
  EXPECT_EQ(lost_face_callbacks_invoked, 3);
  EXPECT_EQ(body_callbacks_invoked, 3);
  EXPECT_EQ(lost_body_callbacks_invoked, 0);

  ids.ids = {"id5", "id6", "id7"};

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 300ms) {
    face_pub->publish(ids);
    body_pub->publish(ids);
    executor.spin_some();
    rate.sleep();
  }

  EXPECT_EQ(face_callbacks_invoked, 9);
  EXPECT_EQ(lost_face_callbacks_invoked, 6);
  EXPECT_EQ(body_callbacks_invoked, 6);
  EXPECT_EQ(lost_body_callbacks_invoked, 3);


  ids.ids = {"id1", "id2"};

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 300ms) {
    voice_pub->publish(ids);
    person_pub->publish(ids);
    person_tracked_pub->publish(ids);
    executor.spin_some();
    rate.sleep();
  }


  EXPECT_EQ(voice_callbacks_invoked, 2);
  EXPECT_EQ(person_callbacks_invoked, 2);
  EXPECT_EQ(ontracked_person_callbacks_invoked, 2);


  hri_listener.reset();
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri_tests, PeopleLocation)
{
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto now = node->now();
  geometry_msgs::msg::TransformStamped t;
  auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  auto hri_listener = std::make_shared<hri::HRIListener>();
  hri_listener->setReferenceFrame("base_link");


  geometry_msgs::msg::TransformStamped world_transform;
  world_transform.header.stamp = now;
  world_transform.header.frame_id = "world";
  world_transform.child_frame_id = "base_link";
  world_transform.transform.translation.x = -1.0;
  world_transform.transform.translation.y = 0.0;
  world_transform.transform.translation.z = 0.0;
  world_transform.transform.rotation.w = 1.0;

  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    static_broadcaster->sendTransform(world_transform);
    executor.spin_some();
    rate.sleep();
  }

  geometry_msgs::msg::TransformStamped p1_transform;
  p1_transform.header.stamp = now;
  p1_transform.header.frame_id = "world";
  p1_transform.child_frame_id = "person_p1";
  p1_transform.transform.translation.x = 1.0;
  p1_transform.transform.translation.y = 0.0;
  p1_transform.transform.translation.z = 0.0;
  p1_transform.transform.rotation.w = 1.0;

  auto person_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/persons/tracked", 1);
  auto loc_confidence_pub =
    node->create_publisher<std_msgs::msg::Float32>("/humans/persons/p1/location_confidence", 1);

  auto person_ids = hri_msgs::msg::IdsList();
  person_ids.ids = {"p1"};

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    person_pub->publish(person_ids);
    executor.spin_some();
    rate.sleep();
  }

  auto p = hri_listener->getTrackedPersons()["p1"];

  auto msg = std_msgs::msg::Float32();
  msg.data = 0.;

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    loc_confidence_pub->publish(msg);
    executor.spin_some();
    rate.sleep();
  }

  ASSERT_FLOAT_EQ(p->location_confidence(), 0.f);
  ASSERT_FALSE(p->transform()) << "location confidence at 0, no transform should be available";

  msg.data = 0.5;

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    loc_confidence_pub->publish(msg);
    executor.spin_some();
    rate.sleep();
  }


  ASSERT_FLOAT_EQ(p->location_confidence(), 0.5f);
  p->transform();
  ASSERT_FALSE(p->transform());

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    static_broadcaster->sendTransform(p1_transform);
    executor.spin_some();
    rate.sleep();
  }

  ASSERT_FLOAT_EQ(p->location_confidence(), 0.5f);
  ASSERT_TRUE(p->transform()) << "location confidence > 0 => a transform should be available";
  t = *(p->transform());
  std::cout << "[TEST] CHILD FROM TRANSFORM" << t.child_frame_id << std::endl;
  ASSERT_EQ(t.child_frame_id, "person_p1");
  ASSERT_EQ(t.header.frame_id, "base_link");
  ASSERT_FLOAT_EQ(t.transform.translation.x, 2.0f);

  msg.data = 1.0;
  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    loc_confidence_pub->publish(msg);
    executor.spin_some();
    rate.sleep();
  }

  ASSERT_FLOAT_EQ(p->location_confidence(), 1.f);
  ASSERT_TRUE(p->transform()) << "location confidence > 0 => a transform should be available";

  hri_listener.reset();
  executor.remove_node(node);
  executor.cancel();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  // ros::Time::init();  // needed for ros::Time::now()
  // ros::init(argc, argv, "test_hri");
  // ros::NodeHandle nh;
  // ROS_INFO("Starting HRI tests");
  return RUN_ALL_TESTS();
}
