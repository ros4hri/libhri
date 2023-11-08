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
  rclcpp::executors::MultiThreadedExecutor executor;
  auto tester_node = rclcpp::Node::make_shared("tester_node");
  executor.add_node(tester_node);
  auto hri_node = rclcpp::Node::make_shared("hri_node");
  executor.add_node(hri_node);

  auto hri_listener = std::make_shared<hri::HRIListener>(hri_node);
  auto publisher = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);

  ASSERT_EQ(publisher->get_subscription_count(), 1U);
  EXPECT_EQ(hri_listener->getFaces().size(), 0U);

  RCLCPP_INFO(tester_node->get_logger(), "[A]");
  auto ids = hri_msgs::msg::IdsList();
  ids.ids = {"A"};
  publisher->publish(ids);
  executor.spin_all(1s);
  auto faces = hri_listener->getFaces();
  EXPECT_EQ(faces.size(), 1U);
  ASSERT_TRUE(faces.find("A") != faces.end());
  EXPECT_EQ(faces["A"]->id(), "A");

  RCLCPP_INFO(tester_node->get_logger(), "[A,B]");
  ids.ids = {"A", "B"};
  publisher->publish(ids);
  executor.spin_all(1s);
  faces = hri_listener->getFaces();
  EXPECT_EQ(faces.size(), 2U);
  EXPECT_TRUE(faces.find("A") != faces.end());
  EXPECT_TRUE(faces.find("B") != faces.end());

  RCLCPP_INFO(tester_node->get_logger(), "[B]");
  ids.ids = {"B"};
  publisher->publish(ids);
  executor.spin_all(1s);
  faces = hri_listener->getFaces();
  EXPECT_EQ(faces.size(), 1U);
  EXPECT_TRUE(faces.find("A") == faces.end());
  ASSERT_TRUE(faces.find("B") != faces.end());

  std::shared_ptr<const hri::Face> face_b = faces["B"];
  int use_count_before_deletion = face_b.use_count();

  RCLCPP_INFO(tester_node->get_logger(), "[]");
  ids.ids = {};
  publisher->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(hri_listener->getFaces().size(), 0U);
  // check face B is not used anymore by hri_listener!
  EXPECT_EQ(face_b.use_count(), --use_count_before_deletion);

  hri_listener.reset();
  EXPECT_EQ(publisher->get_subscription_count(), 0U);
}

TEST(libhri_tests, GetFacesRoi)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto tester_node = rclcpp::Node::make_shared("tester_node");
  executor.add_node(tester_node);
  auto hri_node = rclcpp::Node::make_shared("hri_node");
  executor.add_node(hri_node);

  auto hri_listener = std::make_shared<hri::HRIListener>(hri_node);
  auto pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  // roi topic is transient local
  auto pub_r1 = tester_node->create_publisher<hri_msgs::msg::NormalizedRegionOfInterest2D>(
    "/humans/faces/A/roi", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  // roi topic is transient local
  auto pub_r2 = tester_node->create_publisher<hri_msgs::msg::NormalizedRegionOfInterest2D>(
    "/humans/faces/B/roi", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  auto ids = hri_msgs::msg::IdsList();
  ids.ids = {"A"};
  pub->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(pub_r1->get_subscription_count(), 1U);

  ids.ids = {"B"};
  pub->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(pub_r1->get_subscription_count(), 0U);
  EXPECT_EQ(pub_r2->get_subscription_count(), 1U);
  auto faces = hri_listener->getFaces();
  ASSERT_TRUE(faces.find("B") != faces.end());
  auto face = faces["B"];
  EXPECT_FALSE(face == nullptr);
  EXPECT_EQ(face->ns(), "/humans/faces/B");
  EXPECT_FALSE(face->roi());

  auto roi = hri::RegionOfInterest();
  roi.xmin = 0.1;
  pub_r2->publish(roi);
  executor.spin_all(1s);
  ASSERT_TRUE(face->roi());
  EXPECT_FLOAT_EQ(face->roi().value().xmin, 0.1f);

  roi.xmin = 0.2;
  pub_r2->publish(roi);
  executor.spin_all(1s);
  EXPECT_FLOAT_EQ(face->roi().value().xmin, 0.2f);

  ids.ids = {"B", "A"};
  pub->publish(ids);
  executor.spin_all(1s);

  pub_r1->publish(roi);
  executor.spin_all(1s);
  faces = hri_listener->getFaces();
  auto face_a = faces["A"];
  auto face_b = faces["B"];
  ASSERT_FALSE(face_a == nullptr);
  ASSERT_FALSE(face_b == nullptr);
  EXPECT_EQ(face_a->ns(), "/humans/faces/A");
  ASSERT_TRUE(face_a->roi());
  EXPECT_FLOAT_EQ(face_a->roi().value().xmin, 0.2f);
  EXPECT_EQ(face_b->ns(), "/humans/faces/B");
  ASSERT_TRUE(face_b->roi());
  EXPECT_FLOAT_EQ(face_b->roi().value().xmin, 0.2f);
}


TEST(libhri_tests, GetBodies)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto tester_node = rclcpp::Node::make_shared("tester_node");
  executor.add_node(tester_node);
  auto hri_node = rclcpp::Node::make_shared("hri_node");
  executor.add_node(hri_node);

  auto hri_listener = std::make_shared<hri::HRIListener>(hri_node);
  auto pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/bodies/tracked", 1);

  ASSERT_EQ(pub->get_subscription_count(), 1U);

  RCLCPP_INFO(tester_node->get_logger(), "[A]");
  auto ids = hri_msgs::msg::IdsList();
  ids.ids = {"A"};
  pub->publish(ids);
  executor.spin_all(1s);
  auto bodies = hri_listener->getBodies();
  EXPECT_EQ(bodies.size(), 1U);
  ASSERT_TRUE(bodies.find("A") != bodies.end());
  EXPECT_EQ(bodies["A"]->id(), "A");

  RCLCPP_INFO(tester_node->get_logger(), "[A,B]");
  ids.ids = {"A", "B"};
  pub->publish(ids);
  executor.spin_all(1s);
  bodies = hri_listener->getBodies();
  EXPECT_EQ(bodies.size(), 2U);
  EXPECT_TRUE(bodies.find("A") != bodies.end());
  EXPECT_TRUE(bodies.find("B") != bodies.end());

  RCLCPP_INFO(tester_node->get_logger(), "[B]");
  ids.ids = {"B"};
  pub->publish(ids);
  executor.spin_all(1s);
  bodies = hri_listener->getBodies();
  EXPECT_EQ(bodies.size(), 1U);
  EXPECT_EQ(bodies.find("A"), bodies.end());
  ASSERT_TRUE(bodies.find("B") != bodies.end());

  std::shared_ptr<const hri::Body> body_b = bodies["B"];
  int use_count_before_deletion = body_b.use_count();

  RCLCPP_INFO(tester_node->get_logger(), "[]");
  ids.ids = {};
  pub->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(hri_listener->getBodies().size(), 0U);
  // check body B is not used anymore by hri_listener!
  EXPECT_EQ(body_b.use_count(), --use_count_before_deletion);

  hri_listener.reset();
  EXPECT_EQ(pub->get_subscription_count(), 0U);
}

TEST(libhri_tests, GetVoices)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto tester_node = rclcpp::Node::make_shared("tester_node");
  executor.add_node(tester_node);
  auto hri_node = rclcpp::Node::make_shared("hri_node");
  executor.add_node(hri_node);

  auto hri_listener = std::make_shared<hri::HRIListener>(hri_node);
  auto pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/voices/tracked", 1);

  ASSERT_EQ(pub->get_subscription_count(), 1U);

  RCLCPP_INFO(tester_node->get_logger(), "[A]");
  auto ids = hri_msgs::msg::IdsList();
  ids.ids = {"A"};
  pub->publish(ids);
  executor.spin_all(1s);
  auto voices = hri_listener->getVoices();
  EXPECT_EQ(voices.size(), 1U);
  ASSERT_TRUE(voices.find("A") != voices.end());
  EXPECT_EQ(voices["A"]->id(), "A");

  RCLCPP_INFO(tester_node->get_logger(), "[A]");
  pub->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(hri_listener->getVoices().size(), 1U);

  RCLCPP_INFO(tester_node->get_logger(), "[A,B]");
  ids.ids = {"A", "B"};
  pub->publish(ids);
  executor.spin_all(1s);
  voices = hri_listener->getVoices();
  EXPECT_EQ(voices.size(), 2U);
  EXPECT_TRUE(voices.find("A") != voices.end());
  EXPECT_TRUE(voices.find("B") != voices.end());

  RCLCPP_INFO(tester_node->get_logger(), "[B]");
  ids.ids = {"B"};
  pub->publish(ids);
  executor.spin_all(1s);
  voices = hri_listener->getVoices();
  EXPECT_EQ(voices.size(), 1U);
  EXPECT_TRUE(voices.find("A") == voices.end());
  ASSERT_TRUE(voices.find("B") != voices.end());

  std::shared_ptr<const hri::Voice> voice_b = voices["B"];
  int use_count_before_deletion = voice_b.use_count();

  RCLCPP_INFO(tester_node->get_logger(), "[]");
  ids.ids = {};
  pub->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(hri_listener->getVoices().size(), 0U);
  // check voice B is not used anymore by hri_listener!
  EXPECT_EQ(voice_b.use_count(), --use_count_before_deletion);

  hri_listener.reset();
  EXPECT_EQ(pub->get_subscription_count(), 0U);
}

TEST(libhri_tests, GetKnownPersons)
{
  rclcpp::executors::SingleThreadedExecutor tester_executor;
  auto tester_node = rclcpp::Node::make_shared("tester_node");
  tester_executor.add_node(tester_node);
  rclcpp::executors::SingleThreadedExecutor hri_executor;
  auto hri_node = rclcpp::Node::make_shared("hri_node");
  hri_executor.add_node(hri_node);

  auto hri_listener = std::make_shared<hri::HRIListener>(hri_node);
  auto pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/known", 1);

  ASSERT_EQ(pub->get_subscription_count(), 1U);

  RCLCPP_INFO(tester_node->get_logger(), "[A]");
  auto ids = hri_msgs::msg::IdsList();
  ids.ids = {"A"};
  pub->publish(ids);
  hri_executor.spin_all(1s);
  auto persons = hri_listener->getPersons();
  EXPECT_EQ(persons.size(), 1U);
  ASSERT_TRUE(persons.find("A") != persons.end());
  EXPECT_EQ(persons["A"]->id(), "A");

  RCLCPP_INFO(tester_node->get_logger(), "[A]");
  pub->publish(ids);
  hri_executor.spin_all(1s);
  EXPECT_EQ(hri_listener->getPersons().size(), 1U);

  RCLCPP_INFO(tester_node->get_logger(), "[A,B]");
  ids.ids = {"A", "B"};
  pub->publish(ids);
  hri_executor.spin_all(1s);
  persons = hri_listener->getPersons();
  EXPECT_EQ(persons.size(), 2U);
  EXPECT_TRUE(persons.find("A") != persons.end());
  EXPECT_TRUE(persons.find("B") != persons.end());

  RCLCPP_INFO(tester_node->get_logger(), "[B]");
  ids.ids = {"B"};
  pub->publish(ids);
  hri_executor.spin_all(1s);
  persons = hri_listener->getPersons();
  EXPECT_EQ(persons.size(), 1U) << "known persons can go down in case of eg an anonymous person";
  EXPECT_TRUE(persons.find("A") == persons.end());
  ASSERT_TRUE(persons.find("B") != persons.end());

  std::shared_ptr<const hri::Person> person_b = persons["B"];
  int use_count_before_deletion = person_b.use_count();

  RCLCPP_INFO(tester_node->get_logger(), "[]");
  ids.ids = {};
  pub->publish(ids);
  hri_executor.spin_all(1s);
  EXPECT_EQ(hri_listener->getPersons().size(), 0U);
  // check person B is not used anymore by hri_listener!
  EXPECT_EQ(person_b.use_count(), --use_count_before_deletion);

  hri_listener.reset();
  EXPECT_EQ(pub->get_subscription_count(), 0U);
}

TEST(libhri_tests, GetTrackedPersons)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto tester_node = rclcpp::Node::make_shared("tester_node");
  executor.add_node(tester_node);
  auto hri_node = rclcpp::Node::make_shared("hri_node");
  executor.add_node(hri_node);

  auto hri_listener = std::make_shared<hri::HRIListener>(hri_node);
  auto pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);

  ASSERT_EQ(pub->get_subscription_count(), 1U);

  RCLCPP_INFO(tester_node->get_logger(), "[A]");
  auto ids = hri_msgs::msg::IdsList();
  ids.ids = {"A"};
  pub->publish(ids);
  executor.spin_all(1s);
  auto known_persons = hri_listener->getPersons();
  EXPECT_EQ(known_persons.size(), 0U);
  auto persons = hri_listener->getTrackedPersons();
  EXPECT_EQ(persons.size(), 1U);
  ASSERT_TRUE(persons.find("A") != persons.end());
  EXPECT_EQ(persons["A"]->id(), "A");

  RCLCPP_INFO(tester_node->get_logger(), "[A,B]");
  ids.ids = {"A", "B"};
  pub->publish(ids);
  executor.spin_all(1s);
  persons = hri_listener->getTrackedPersons();
  EXPECT_EQ(persons.size(), 2U);
  EXPECT_TRUE(persons.find("A") != persons.end());
  EXPECT_TRUE(persons.find("B") != persons.end());

  RCLCPP_INFO(tester_node->get_logger(), "[B]");
  ids.ids = {"B"};
  pub->publish(ids);
  executor.spin_all(1s);
  persons = hri_listener->getTrackedPersons();
  EXPECT_EQ(persons.size(), 1U);
  EXPECT_TRUE(persons.find("A") == persons.end());
  ASSERT_TRUE(persons.find("B") != persons.end());

  std::shared_ptr<const hri::Person> person_b = persons["B"];
  int use_count_before_deletion = person_b.use_count();

  RCLCPP_INFO(tester_node->get_logger(), "[]");
  ids.ids = {};
  pub->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(hri_listener->getTrackedPersons().size(), 0U);
  // check person B is not used anymore by hri_listener!
  EXPECT_EQ(person_b.use_count(), --use_count_before_deletion);

  hri_listener.reset();
  EXPECT_EQ(pub->get_subscription_count(), 0U);
}

TEST(libhri_tests, PersonAttributes)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto tester_node = rclcpp::Node::make_shared("tester_node");
  executor.add_node(tester_node);
  auto hri_node = rclcpp::Node::make_shared("hri_node");
  executor.add_node(hri_node);

  auto hri_listener = std::make_shared<hri::HRIListener>(hri_node);
  auto person_pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);
  auto face_pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto person_face_pub = tester_node->create_publisher<std_msgs::msg::String>(
    "/humans/persons/p1/face_id", 1);

  auto person_ids = hri_msgs::msg::IdsList();
  person_ids.ids = {"p1"};
  person_pub->publish(person_ids);
  auto face_ids = hri_msgs::msg::IdsList();
  face_ids.ids = {"f1", "f2"};
  face_pub->publish(face_ids);
  executor.spin_all(1s);
  auto p1 = hri_listener->getTrackedPersons()["p1"];
  EXPECT_FALSE(p1->anonymous());
  auto face0 = p1->face();
  EXPECT_EQ(face0, nullptr);

  auto face_id = std_msgs::msg::String();
  face_id.data = "f1";
  person_face_pub->publish(face_id);
  executor.spin_all(1s);
  auto face1 = hri_listener->getTrackedPersons()["p1"]->face();
  ASSERT_NE(face1, nullptr);
  EXPECT_EQ(face1->id(), "f1");
}

TEST(libhri_tests, AnonymousPersonsAndAliases)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto tester_node = rclcpp::Node::make_shared("tester_node");
  executor.add_node(tester_node);
  auto hri_node = rclcpp::Node::make_shared("hri_node");
  executor.add_node(hri_node);

  auto hri_listener = std::make_shared<hri::HRIListener>(hri_node);
  auto person_pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "humans/persons/tracked", 1);
  auto p1_anon_pub = tester_node->create_publisher<std_msgs::msg::Bool>(
    "/humans/persons/p1/anonymous", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  auto p2_anon_pub = tester_node->create_publisher<std_msgs::msg::Bool>(
    "/humans/persons/p2/anonymous", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  auto face_pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto p1_face_pub = tester_node->create_publisher<std_msgs::msg::String>(
    "/humans/persons/p1/face_id", 1);
  auto p2_face_pub = tester_node->create_publisher<std_msgs::msg::String>(
    "/humans/persons/p2/face_id", 1);
  auto p2_alias_pub = tester_node->create_publisher<std_msgs::msg::String>(
    "/humans/persons/p2/alias", 1);

  auto person_ids = hri_msgs::msg::IdsList();
  person_ids.ids = {"p1", "p2"};
  person_pub->publish(person_ids);
  auto face_ids = hri_msgs::msg::IdsList();
  face_ids.ids = {"f1", "f2"};
  face_pub->publish(face_ids);
  executor.spin_all(1s);
  ASSERT_EQ(hri_listener->getTrackedPersons().size(), 2U);
  ASSERT_EQ(hri_listener->getFaces().size(), 2U);

  // each person is associated to a face
  auto face_id = std_msgs::msg::String();
  face_id.data = "f1";
  p1_face_pub->publish(face_id);
  face_id.data = "f2";
  p2_face_pub->publish(face_id);
  std_msgs::msg::Bool msg;
  msg.data = false;
  p1_anon_pub->publish(msg);
  msg.data = true;
  p2_anon_pub->publish(msg);
  executor.spin_all(1s);
  auto p1 = hri_listener->getTrackedPersons()["p1"];
  auto p2 = hri_listener->getTrackedPersons()["p2"];
  ASSERT_TRUE(p1->anonymous());  // the anonymous optional flag should have been set
  ASSERT_TRUE(p2->anonymous());  // the anonymous optional flag should have been set
  ASSERT_FALSE(p1->anonymous().value());
  ASSERT_TRUE(p2->anonymous().value());
  // being anonymous or not should have no impact on face associations
  ASSERT_EQ(p1->face()->id(), "f1");
  ASSERT_EQ(p2->face()->id(), "f2");

  ///////////// ALIASES ///////////////////////////

  // set p2 as an alias of p1
  auto alias_id = std_msgs::msg::String();
  alias_id.data = "p1";
  p2_alias_pub->publish(alias_id);
  executor.spin_all(1s);
  EXPECT_EQ(hri_listener->getTrackedPersons().size(), 2U);
  p2 = hri_listener->getTrackedPersons()["p2"];
  EXPECT_EQ(p1, p2) << "p2 should now point to the same person as p1";
  EXPECT_EQ(p2->face()->id(), "f1") << "p2's face now points to f1";

  // remove the alias
  alias_id.data = "";
  p2_alias_pub->publish(alias_id);
  executor.spin_all(1s);
  p2 = hri_listener->getTrackedPersons()["p2"];
  EXPECT_NE(p1, p2) << "p2 is not anymore the same person as p1";
  EXPECT_EQ(p2->face()->id(), "f2") << "p2's face should still points to its former f2 face";

  // republish the alias
  alias_id.data = "p1";
  p2_alias_pub->publish(alias_id);
  executor.spin_all(1s);
  p2 = hri_listener->getTrackedPersons()["p2"];
  EXPECT_EQ(p1, p2) << "p2 is again the same person as p1";

  // delete p1 -> p2 should be deleted as well
  person_ids.ids = {"p2"};
  person_pub->publish(person_ids);
  executor.spin_all(1s);

  ASSERT_EQ(hri_listener->getTrackedPersons().size(), 0U)
    << "the aliased person should have been deleted with its alias";
}

TEST(libhri_tests, SoftBiometrics)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto tester_node = rclcpp::Node::make_shared("tester_node");
  executor.add_node(tester_node);
  auto hri_node = rclcpp::Node::make_shared("hri_node");
  executor.add_node(hri_node);

  auto hri_listener = std::make_shared<hri::HRIListener>(hri_node);
  auto person_pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);
  auto face_pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto person_face_pub = tester_node->create_publisher<std_msgs::msg::String>(
    "/humans/persons/p1/face_id", 1);
  auto softbiometrics_pub = tester_node->create_publisher<hri_msgs::msg::SoftBiometrics>(
    "/humans/faces/f1/softbiometrics", 1);

  auto person_ids = hri_msgs::msg::IdsList();
  person_ids.ids = {"p1"};
  person_pub->publish(person_ids);
  auto face_ids = hri_msgs::msg::IdsList();
  face_ids.ids = {"f1"};
  face_pub->publish(face_ids);
  executor.spin_all(1s);
  ASSERT_EQ(hri_listener->getTrackedPersons().size(), 1U);
  ASSERT_EQ(hri_listener->getFaces().size(), 1U);

  auto softbiometrics_msg = hri_msgs::msg::SoftBiometrics();
  softbiometrics_msg.age = 45;
  softbiometrics_msg.age_confidence = 0.8;
  softbiometrics_msg.gender = hri_msgs::msg::SoftBiometrics::FEMALE;
  softbiometrics_msg.gender_confidence = 0.7;
  auto face_id = std_msgs::msg::String();
  face_id.data = "f1";
  softbiometrics_pub->publish(softbiometrics_msg);
  person_face_pub->publish(face_id);
  executor.spin_all(1s);
  auto face = hri_listener->getTrackedPersons()["p1"]->face();
  EXPECT_EQ(face->id(), "f1");
  ASSERT_TRUE(face->age());
  EXPECT_FLOAT_EQ(face->age().value(), 45.f);
  ASSERT_TRUE(face->gender());
  EXPECT_EQ(face->gender().value(), hri::Gender::kFemale);

  softbiometrics_msg.gender = hri_msgs::msg::SoftBiometrics::OTHER;
  softbiometrics_pub->publish(softbiometrics_msg);
  executor.spin_all(1s);
  EXPECT_EQ(face->gender().value(), hri::Gender::kOther);

  softbiometrics_msg.gender = hri_msgs::msg::SoftBiometrics::UNDEFINED;
  softbiometrics_pub->publish(softbiometrics_msg);
  executor.spin_all(1s);
  EXPECT_FALSE(face->gender());
}

TEST(libhri_tests, EngagementLevel)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto tester_node = rclcpp::Node::make_shared("tester_node");
  executor.add_node(tester_node);
  auto hri_node = rclcpp::Node::make_shared("hri_node");
  executor.add_node(hri_node);

  auto hri_listener = std::make_shared<hri::HRIListener>(hri_node);
  auto person_pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);
  auto engagement_pub = tester_node->create_publisher<hri_msgs::msg::EngagementLevel>(
    "/humans/persons/p1/engagement_status", 1);

  auto person_ids = hri_msgs::msg::IdsList();
  person_ids.ids = {"p1"};
  person_pub->publish(person_ids);
  executor.spin_all(1s);

  auto msg = hri_msgs::msg::EngagementLevel();
  msg.level = hri_msgs::msg::EngagementLevel::DISENGAGED;
  engagement_pub->publish(msg);
  executor.spin_all(1s);
  auto p = hri_listener->getTrackedPersons()["p1"];
  ASSERT_TRUE(p->engagement_status());
  EXPECT_EQ(p->engagement_status().value(), hri::EngagementLevel::kDisengaged);

  msg.level = hri_msgs::msg::EngagementLevel::ENGAGED;
  engagement_pub->publish(msg);
  executor.spin_all(1s);
  EXPECT_EQ(p->engagement_status().value(), hri::EngagementLevel::kEngaged);

  msg.level = hri_msgs::msg::EngagementLevel::UNKNOWN;
  engagement_pub->publish(msg);
  executor.spin_all(1s);
  EXPECT_FALSE(p->engagement_status());
}

TEST(libhri_tests, Callback)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto tester_node = rclcpp::Node::make_shared("tester_node");
  executor.add_node(tester_node);
  auto hri_node = rclcpp::Node::make_shared("hri_node");
  executor.add_node(hri_node);

  auto hri_listener = std::make_shared<hri::HRIListener>(hri_node);

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

  auto face_pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto body_pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/bodies/tracked", 1);
  auto voice_pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/voices/tracked", 1);
  auto person_pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/known", 1);
  auto person_tracked_pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);

  EXPECT_EQ(face_callbacks_invoked, 0);
  EXPECT_EQ(lost_face_callbacks_invoked, 0);

  auto ids = hri_msgs::msg::IdsList();
  ids.ids = {"id1"};
  face_pub->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(face_callbacks_invoked, 1);
  EXPECT_EQ(lost_face_callbacks_invoked, 0);

  ids.ids = {"id1", "id2"};
  face_pub->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(face_callbacks_invoked, 2);

  ids.ids = {"id3", "id4"};
  face_pub->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(lost_face_callbacks_invoked, 2);

  ids.ids = {"id1", "id2"};
  body_pub->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(body_callbacks_invoked, 2);
  EXPECT_EQ(lost_body_callbacks_invoked, 0);

  ids.ids = {"id1", "id2", "id3"};
  face_pub->publish(ids);
  body_pub->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(face_callbacks_invoked, 6);
  EXPECT_EQ(lost_face_callbacks_invoked, 3);
  EXPECT_EQ(body_callbacks_invoked, 3);
  EXPECT_EQ(lost_body_callbacks_invoked, 0);

  ids.ids = {"id5", "id6", "id7"};
  face_pub->publish(ids);
  body_pub->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(face_callbacks_invoked, 9);
  EXPECT_EQ(lost_face_callbacks_invoked, 6);
  EXPECT_EQ(body_callbacks_invoked, 6);
  EXPECT_EQ(lost_body_callbacks_invoked, 3);

  ids.ids = {"id1", "id2"};
  voice_pub->publish(ids);
  person_pub->publish(ids);
  person_tracked_pub->publish(ids);
  executor.spin_all(1s);
  EXPECT_EQ(voice_callbacks_invoked, 2);
  EXPECT_EQ(person_callbacks_invoked, 2);
  EXPECT_EQ(ontracked_person_callbacks_invoked, 2);
}

TEST(libhri_tests, PeopleLocation)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto tester_node = rclcpp::Node::make_shared("tester_node");
  executor.add_node(tester_node);
  auto hri_node = rclcpp::Node::make_shared("hri_node");
  executor.add_node(hri_node);

  auto hri_listener = std::make_shared<hri::HRIListener>(hri_node);
  auto person_pub = tester_node->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);
  auto loc_confidence_pub = tester_node->create_publisher<std_msgs::msg::Float32>(
    "/humans/persons/p1/location_confidence", 1);
  auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(tester_node);

  hri_listener->setReferenceFrame("base_link");
  auto now = tester_node->now();
  geometry_msgs::msg::TransformStamped world_transform;
  world_transform.header.stamp = now;
  world_transform.header.frame_id = "world";
  world_transform.child_frame_id = "base_link";
  world_transform.transform.translation.x = -1.0;
  world_transform.transform.translation.y = 0.0;
  world_transform.transform.translation.z = 0.0;
  world_transform.transform.rotation.w = 1.0;

  static_broadcaster->sendTransform(world_transform);
  executor.spin_all(1s);

  auto person_ids = hri_msgs::msg::IdsList();
  person_ids.ids = {"p1"};
  person_pub->publish(person_ids);
  executor.spin_all(1s);
  auto p = hri_listener->getTrackedPersons()["p1"];

  auto msg = std_msgs::msg::Float32();
  msg.data = 0.;
  loc_confidence_pub->publish(msg);
  executor.spin_all(1s);
  ASSERT_TRUE(p->location_confidence());
  EXPECT_FLOAT_EQ(p->location_confidence().value(), 0.f);
  EXPECT_FALSE(p->transform()) << "location confidence at 0, no transform should be available";

  msg.data = 0.5;
  loc_confidence_pub->publish(msg);
  executor.spin_all(1s);
  EXPECT_FLOAT_EQ(p->location_confidence().value(), 0.5f);
  p->transform();
  EXPECT_FALSE(p->transform())
    << "location confidence > 0 but no transform published yet -> no transform should be returned";

  geometry_msgs::msg::TransformStamped p1_transform;
  p1_transform.header.stamp = now;
  p1_transform.header.frame_id = "world";
  p1_transform.child_frame_id = "person_p1";
  p1_transform.transform.translation.x = 1.0;
  p1_transform.transform.translation.y = 0.0;
  p1_transform.transform.translation.z = 0.0;
  p1_transform.transform.rotation.w = 1.0;
  static_broadcaster->sendTransform(p1_transform);
  executor.spin_all(1s);
  EXPECT_FLOAT_EQ(p->location_confidence().value(), 0.5f);
  ASSERT_TRUE(p->transform()) << "location confidence > 0 => a transform should be available";
  auto t = p->transform().value();
  EXPECT_EQ(t.child_frame_id, "person_p1");
  EXPECT_EQ(t.header.frame_id, "base_link");
  EXPECT_FLOAT_EQ(t.transform.translation.x, 2.0f);

  hri_listener->setReferenceFrame("person_p1");
  ASSERT_TRUE(p->transform());
  t = p->transform().value();
  EXPECT_EQ(t.child_frame_id, "person_p1");
  EXPECT_EQ(t.header.frame_id, "person_p1");
  EXPECT_FLOAT_EQ(t.transform.translation.x, 0.f);

  msg.data = 1.0;
  loc_confidence_pub->publish(msg);
  executor.spin_all(1s);
  EXPECT_FLOAT_EQ(p->location_confidence().value(), 1.f);
  EXPECT_TRUE(p->transform()) << "location confidence > 0 => a transform should be available";
}

// TODO(LJU): missing quite a few tests, should have at least a basic one for each topic subscribed

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
