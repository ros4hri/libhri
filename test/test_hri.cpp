// Copyright 2022 PAL Robotics
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the PAL Robotics S.L. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <thread>
#include <chrono>
#include <memory>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/transform_stamped.h"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "sensor_msgs/msg/region_of_interest.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/static_transform_broadcaster.h"

#include "hri/hri.hpp"
#include "hri/face.hpp"
#include "hri/person.hpp"
#include "hri_msgs/msg/engagement_level.hpp"
#include "hri_msgs/msg/ids_list.hpp"
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
    ASSERT_EQ(hri_listener->getFaces().size(), 0);

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
    EXPECT_EQ(faces["A"].lock()->id(), "A");

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

    std::weak_ptr<const hri::Face> face_b = faces["B"];
    EXPECT_FALSE(face_b.expired());  // face B exists!

    RCLCPP_INFO(node->get_logger(), "[]");

    ids.ids = {};
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      publisher->publish(ids);
      executor.spin_some();
      rate.sleep();
    }

    EXPECT_EQ(hri_listener->getFaces().size(), 0U);
    EXPECT_TRUE(face_b.expired());  // face B does not exist anymore!

    hri_listener.reset();
  }

  EXPECT_EQ(publisher->get_subscription_count(), 0);
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri_test, GetFacesRoi)
{
  auto node = rclcpp::Node::make_shared("test_node");
  auto hri_listener = std::make_shared<hri::HRIListener>();

  rclcpp::Rate rate(30);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto pub = node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);

  // roi topic is transient local
  auto pub_r1 = node->create_publisher<sensor_msgs::msg::RegionOfInterest>(
    "/humans/faces/A/roi", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  // roi topic is transient local
  auto pub_r2 = node->create_publisher<sensor_msgs::msg::RegionOfInterest>(
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

  ASSERT_FALSE(faces["B"].expired());

  auto roi = sensor_msgs::msg::RegionOfInterest();

  {
    auto face = faces["B"].lock();

    EXPECT_FALSE(face == nullptr);
    EXPECT_EQ(face->ns(), "/humans/faces/B");
    EXPECT_EQ(face->roi().width, 0);

    roi.width = 10;

    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub_r2->publish(roi);
      executor.spin_some();
      rate.sleep();
    }

    EXPECT_EQ(face->roi().width, 10);

    roi.width = 20;

    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub_r2->publish(roi);
      executor.spin_some();
      rate.sleep();
    }

    EXPECT_EQ(face->roi().width, 20);
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
    auto face_a = faces["A"].lock();
    auto face_b = faces["B"].lock();

    ASSERT_FALSE(face_a == nullptr);
    ASSERT_FALSE(face_b == nullptr);

    EXPECT_EQ(face_a->ns(), "/humans/faces/A");
    EXPECT_EQ(face_a->roi().width, 20);
    EXPECT_EQ(face_b->ns(), "/humans/faces/B");
    EXPECT_EQ(face_b->roi().width, 20);
  }

  executor.remove_node(node);
  executor.cancel();
  hri_listener.reset();
}


TEST(libhri_test, GetBodies)
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
    EXPECT_EQ(bodies["A"].lock()->id(), "A");

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

    std::weak_ptr<const hri::Body> body_b = bodies["B"];

    EXPECT_FALSE(body_b.expired());  // body B exists!

    RCLCPP_INFO(node->get_logger(), "[]");
    ids.ids = {};

    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }

    EXPECT_EQ(hri_listener->getBodies().size(), 0U);
    EXPECT_TRUE(body_b.expired());  // body B does not exist anymore!

    hri_listener.reset();
  }

  EXPECT_EQ(pub->get_subscription_count(), 0);

  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri, GetVoices)
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
    EXPECT_EQ(voices["A"].lock()->id(), "A");

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

    std::weak_ptr<const hri::Voice> voice_b = voices["B"];
    EXPECT_FALSE(voice_b.expired());  // voice B exists!

    RCLCPP_INFO(node->get_logger(), "[]");
    ids.ids = {};
    start = node->now();
    while (rclcpp::ok() && (node->now() - start) < 1s) {
      pub->publish(ids);
      executor.spin_some();
      rate.sleep();
    }
    EXPECT_EQ(hri_listener->getVoices().size(), 0U);

    EXPECT_TRUE(voice_b.expired());  // voice B does not exist anymore!

    hri_listener.reset();
  }

  EXPECT_EQ(pub->get_subscription_count(), 0);
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri, GetKnownPersons)
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
    EXPECT_EQ(persons["A"].lock()->id(), "A");

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

    std::shared_ptr<const hri::Person> person_b = persons["B"].lock();
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

  EXPECT_EQ(pub->get_subscription_count(), 0);
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri, GetTrackedPersons)
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
    EXPECT_EQ(persons["A"].lock()->id(), "A");

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

    std::shared_ptr<const hri::Person> person_b = persons["B"].lock();
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

  EXPECT_EQ(pub->get_subscription_count(), 0);
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri, PersonAttributes)
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

  auto p1 = hri_listener->getTrackedPersons()["p1"].lock();

  ASSERT_FALSE(p1->anonymous());

  auto face0 = p1->face();

  ASSERT_EQ(face0.lock(), nullptr);
  ASSERT_TRUE(face0.expired());

  auto face_id = std_msgs::msg::String();
  face_id.data = "f1";

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    person_face_pub->publish(face_id);
    executor.spin_some();
    rate.sleep();
  }
  auto face1 = hri_listener->getTrackedPersons()["p1"].lock()->face();

  ASSERT_NE(face1.lock(), nullptr);
  ASSERT_FALSE(face1.expired());
  ASSERT_EQ(face1.lock()->id(), "f1");

  hri_listener.reset();
  executor.remove_node(node);
  executor.cancel();
}

TEST(libhri, AnonymousPersonsAndAliases)
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


  ASSERT_EQ(hri_listener->getTrackedPersons().size(), 2);

  auto p1 = hri_listener->getTrackedPersons()["p1"].lock();

  {
    auto p2 = hri_listener->getTrackedPersons()["p2"].lock();

    ASSERT_FALSE(p1->anonymous());
    ASSERT_TRUE(p2->anonymous());

    // being anonymous or not should have no impact on face associations
    ASSERT_EQ(p1->face().lock()->id(), "f1");
    ASSERT_EQ(p2->face().lock()->id(), "f2");
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
    auto p2 = hri_listener->getTrackedPersons()["p2"].lock();

    ASSERT_EQ(p1, p2) << "p2 should now point to the same person as p1";

    ASSERT_EQ(p2->face().lock()->id(), "f1") << "p2's face now points to f1";
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
    auto p2 = hri_listener->getTrackedPersons()["p2"].lock();

    ASSERT_NE(p1, p2) << "p2 is not anymore the same person as p1";

    ASSERT_EQ(p2->face().lock()->id(), "f2")
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

  auto p2 = hri_listener->getTrackedPersons()["p2"].lock();

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

TEST(libhri, SoftBiometrics)
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

  auto face = hri_listener->getTrackedPersons()["p1"].lock()->face().lock();

  ASSERT_EQ(face->id(), "f1");

  ASSERT_TRUE(face->age());
  ASSERT_EQ(*(face->age()), 45);
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

TEST(libhri, EngagementLevel)
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


  auto p = hri_listener->getTrackedPersons()["p1"].lock();
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

TEST(libhri, Callback)
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
    [&](hri::FaceWeakConstPtr face) {
      face_callbacks_invoked++;
    });

  hri_listener->onFaceLost(
    [&](hri::ID face_lost) {
      lost_face_callbacks_invoked++;
    });

  hri_listener->onBody(
    [&](hri::BodyWeakConstPtr body) {
      body_callbacks_invoked++;
    });

  hri_listener->onBodyLost(
    [&](hri::ID body_lost) {
      lost_body_callbacks_invoked++;
    });

  hri_listener->onVoice(
    [&](hri::VoiceWeakConstPtr voice) {
      voice_callbacks_invoked++;
    });

  hri_listener->onVoiceLost(
    [&](hri::ID voice_lost) {
      lost_voice_callbacks_invoked++;
    });

  hri_listener->onPerson(
    [&](hri::PersonWeakConstPtr person) {
      person_callbacks_invoked++;
    });

  hri_listener->onTrackedPerson(
    [&](hri::PersonWeakConstPtr tracked_person) {
      ontracked_person_callbacks_invoked++;
    });

  hri_listener->onTrackedPersonLost(
    [&](hri::ID tracked_person_lost) {
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

  face_pub->publish(ids);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  EXPECT_EQ(face_callbacks_invoked, 1);
  EXPECT_EQ(lost_face_callbacks_invoked, 0);

  ids.ids = {"id1", "id2"};
  face_pub->publish(ids);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  EXPECT_EQ(face_callbacks_invoked, 2);


  ids.ids = {"id3", "id4"};
  face_pub->publish(ids);
  executor.spin_some();

  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  EXPECT_EQ(lost_face_callbacks_invoked, 2);

  ids.ids = {"id1", "id2"};

  body_pub->publish(ids);
  executor.spin_some();

  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  EXPECT_EQ(body_callbacks_invoked, 2);
  EXPECT_EQ(lost_body_callbacks_invoked, 0);

  ids.ids = {"id1", "id2", "id3"};

  face_pub->publish(ids);
  body_pub->publish(ids);
  executor.spin_some();

  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  EXPECT_EQ(face_callbacks_invoked, 6);
  EXPECT_EQ(lost_face_callbacks_invoked, 3);
  EXPECT_EQ(body_callbacks_invoked, 3);
  EXPECT_EQ(lost_body_callbacks_invoked, 0);

  ids.ids = {"id5", "id6", "id7"};

  face_pub->publish(ids);
  body_pub->publish(ids);
  executor.spin_some();

  std::this_thread::sleep_for(std::chrono::milliseconds(60));


  EXPECT_EQ(face_callbacks_invoked, 9);
  EXPECT_EQ(lost_face_callbacks_invoked, 6);
  EXPECT_EQ(body_callbacks_invoked, 6);
  EXPECT_EQ(lost_body_callbacks_invoked, 3);


  ids.ids = {"id1", "id2"};
  voice_pub->publish(ids);
  person_pub->publish(ids);
  person_tracked_pub->publish(ids);
  executor.spin_some();

  std::this_thread::sleep_for(std::chrono::milliseconds(30));


  EXPECT_EQ(voice_callbacks_invoked, 2);
  EXPECT_EQ(person_callbacks_invoked, 2);
  EXPECT_EQ(ontracked_person_callbacks_invoked, 2);


  hri_listener.reset();
  executor.remove_node(node);
  executor.cancel();
}
TEST(libhri, PeopleLocation)
{
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  rclcpp::Time now;
  geometry_msgs::msg::TransformStamped t;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;
  static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

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

  static_broadcaster->sendTransform(world_transform);
  executor.spin_some();

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

  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    person_pub->publish(person_ids);
    executor.spin_some();
    rate.sleep();
  }

  auto p = hri_listener->getTrackedPersons()["p1"].lock();

  auto msg = std_msgs::msg::Float32();
  msg.data = 0.;

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    loc_confidence_pub->publish(msg);
    executor.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(p->location_confidence(), 0.);
  ASSERT_FALSE(p->transform()) << "location confidence at 0, no transform should be available";

  msg.data = 0.5;

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    loc_confidence_pub->publish(msg);
    executor.spin_some();
    rate.sleep();
  }


  ASSERT_EQ(p->location_confidence(), 0.5);
  p->transform();
  ASSERT_FALSE(p->transform());

  static_broadcaster->sendTransform(p1_transform);
  executor.spin_some();

  ASSERT_EQ(p->location_confidence(), 0.5);
  ASSERT_TRUE(p->transform()) << "location confidence > 0 => a transform should be available";
  t = *(p->transform());
  std::cout << "[TEST] CHILD FROM TRANSFORM" << t.child_frame_id << std::endl;
  ASSERT_EQ(t.child_frame_id, "person_p1");
  ASSERT_EQ(t.header.frame_id, "base_link");
  ASSERT_EQ(t.transform.translation.x, 2.0);

  msg.data = 1.0;
  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    loc_confidence_pub->publish(msg);
    executor.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(p->location_confidence(), 1.);
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
