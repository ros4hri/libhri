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

#ifndef HRI__TYPES_HPP_
#define HRI__TYPES_HPP_

#include <string>
#include <map>
#include <variant>

#include "hri_msgs/msg/engagement_level.hpp"
#include "hri_msgs/msg/facial_action_units.hpp"
#include "hri_msgs/msg/facial_landmarks.hpp"
#include "hri_msgs/msg/normalized_point_of_interest2_d.hpp"
#include "hri_msgs/msg/normalized_region_of_interest2_d.hpp"
#include "hri_msgs/msg/skeleton2_d.hpp"
#include "hri_msgs/msg/soft_biometrics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace hri
{

enum class EngagementLevel
{
  // disengaged: the human has not looked in the direction of the robot
  kDisengaged = hri_msgs::msg::EngagementLevel::DISENGAGED,
  // engaging: the human has started to look in the direction of the robot
  kEngaging = hri_msgs::msg::EngagementLevel::ENGAGING,
  // engaged: the human is fully engaged with the robot
  kEngaged = hri_msgs::msg::EngagementLevel::ENGAGED,
  // disengaging: the human has started to look away from the robot
  kDisengaging = hri_msgs::msg::EngagementLevel::DISENGAGING
};

enum class FacialActionUnit
{
  kNeutralFace = hri_msgs::msg::FacialActionUnits::NEUTRAL_FACE,
  kInnerBrowRaiser = hri_msgs::msg::FacialActionUnits::INNER_BROW_RAISER,
  kOuterBrowRaiser = hri_msgs::msg::FacialActionUnits::OUTER_BROW_RAISER,
  kBrowLowerer = hri_msgs::msg::FacialActionUnits::BROW_LOWERER,
  kUpperLidRaiser = hri_msgs::msg::FacialActionUnits::UPPER_LID_RAISER,
  kCheeckRaiser = hri_msgs::msg::FacialActionUnits::CHEEK_RAISER,
  kLidTightener = hri_msgs::msg::FacialActionUnits::LID_TIGHTENER,
  kLipsTowardEachOther = hri_msgs::msg::FacialActionUnits::LIPS_TOWARD_EACH_OTHER,
  kNoseWrinkler = hri_msgs::msg::FacialActionUnits::NOSE_WRINKLER,
  kUpperLipRaiser = hri_msgs::msg::FacialActionUnits::UPPER_LIP_RAISER,
  kNasolabialDeepener = hri_msgs::msg::FacialActionUnits::NASOLABIAL_DEEPENER,
  kLipCornerPuller = hri_msgs::msg::FacialActionUnits::LIP_CORNER_PULLER,
  kSharpLipPuller = hri_msgs::msg::FacialActionUnits::SHARP_LIP_PULLER,
  kDimpler = hri_msgs::msg::FacialActionUnits::DIMPLER,
  kLipCornerDepressor = hri_msgs::msg::FacialActionUnits::LIP_CORNER_DEPRESSOR,
  kLowerLipDepressor = hri_msgs::msg::FacialActionUnits::LOWER_LIP_DEPRESSOR,
  kChinRaiser = hri_msgs::msg::FacialActionUnits::CHIN_RAISER,
  kLipPucker = hri_msgs::msg::FacialActionUnits::LIP_PUCKER,
  kTongueShow = hri_msgs::msg::FacialActionUnits::TONGUE_SHOW,
  kLipStretcher = hri_msgs::msg::FacialActionUnits::LIP_STRETCHER,
  kNeckTightener = hri_msgs::msg::FacialActionUnits::NECK_TIGHTENER,
  kLipFunneler = hri_msgs::msg::FacialActionUnits::LIP_FUNNELER,
  kLipTightener = hri_msgs::msg::FacialActionUnits::LIP_TIGHTENER,
  kLipPressor = hri_msgs::msg::FacialActionUnits::LIP_PRESSOR,
  kLipsPart = hri_msgs::msg::FacialActionUnits::LIPS_PART,
  kJawDrop = hri_msgs::msg::FacialActionUnits::JAW_DROP,
  kMouthStretch = hri_msgs::msg::FacialActionUnits::MOUTH_STRETCH,
  kLipSuck = hri_msgs::msg::FacialActionUnits::LIP_SUCK,
  kHeadTurnLeft = hri_msgs::msg::FacialActionUnits::HEAD_TURN_LEFT,
  kHeadTurnRight = hri_msgs::msg::FacialActionUnits::HEAD_TURN_RIGHT,
  kHeadUp = hri_msgs::msg::FacialActionUnits::HEAD_UP,
  kHeadDown = hri_msgs::msg::FacialActionUnits::HEAD_DOWN,
  kHeadTiltLeft = hri_msgs::msg::FacialActionUnits::HEAD_TILT_LEFT,
  kHeadTiltRight = hri_msgs::msg::FacialActionUnits::HEAD_TILT_RIGHT,
  kHeadForward = hri_msgs::msg::FacialActionUnits::HEAD_FORWARD,
  kHeadBack = hri_msgs::msg::FacialActionUnits::HEAD_BACK,
  kEyesTurnLeft = hri_msgs::msg::FacialActionUnits::EYES_TURN_LEFT,
  kEyesTurnRight = hri_msgs::msg::FacialActionUnits::EYES_TURN_RIGHT,
  kEyesUp = hri_msgs::msg::FacialActionUnits::EYES_UP,
  kEyesDown = hri_msgs::msg::FacialActionUnits::EYES_DOWN,
  kWalleye = hri_msgs::msg::FacialActionUnits::WALLEYE,
  kCrossEye = hri_msgs::msg::FacialActionUnits::CROSS_EYE,
  kEyesPositionedToLookAtOtherPerson =
    hri_msgs::msg::FacialActionUnits::EYES_POSITIONED_TO_LOOK_AT_OTHER_PERSON,
  kBrownsAndForeheadNotVisible = hri_msgs::msg::FacialActionUnits::BROWS_AND_FOREHEAD_NOT_VISIBLE,
  kEyesNotVisible = hri_msgs::msg::FacialActionUnits::EYES_NOT_VISIBLE,
  kLowerFaceNotVisible = hri_msgs::msg::FacialActionUnits::LOWER_FACE_NOT_VISIBLE,
  kEntireFaceNotVisible = hri_msgs::msg::FacialActionUnits::ENTIRE_FACE_NOT_VISIBLE,
  kUnsociable = hri_msgs::msg::FacialActionUnits::UNSOCIABLE,
  kJawThrust = hri_msgs::msg::FacialActionUnits::JAW_THRUST,
  kJawSideways = hri_msgs::msg::FacialActionUnits::JAW_SIDEWAYS,
  kJawClencher = hri_msgs::msg::FacialActionUnits::JAW_CLENCHER,
  kLipBite = hri_msgs::msg::FacialActionUnits::LIP_BITE,
  kCheekBlow = hri_msgs::msg::FacialActionUnits::CHEEK_BLOW,
  kCheekPuff = hri_msgs::msg::FacialActionUnits::CHEEK_PUFF,
  kCheekSuck = hri_msgs::msg::FacialActionUnits::CHEEK_SUCK,
  kTongueBulge = hri_msgs::msg::FacialActionUnits::TONGUE_BULGE,
  kLipWipe = hri_msgs::msg::FacialActionUnits::LIP_WIPE,
  kNostrilDilator = hri_msgs::msg::FacialActionUnits::NOSTRIL_DILATOR,
  kNostrilCompressor = hri_msgs::msg::FacialActionUnits::NOSTRIL_COMPRESSOR,
  kSniff = hri_msgs::msg::FacialActionUnits::SNIFF,
  kLidDroop = hri_msgs::msg::FacialActionUnits::LID_DROOP,
  kSlit = hri_msgs::msg::FacialActionUnits::SLIT,
  kEyesClosed = hri_msgs::msg::FacialActionUnits::EYES_CLOSED,
  kSquint = hri_msgs::msg::FacialActionUnits::SQUINT,
  kBlink = hri_msgs::msg::FacialActionUnits::BLINK,
  kWink = hri_msgs::msg::FacialActionUnits::WINK,
  kSpeech = hri_msgs::msg::FacialActionUnits::SPEECH,
  kSwallow = hri_msgs::msg::FacialActionUnits::SWALLOW,
  kChewing = hri_msgs::msg::FacialActionUnits::CHEWING,
  kShoulderShrug = hri_msgs::msg::FacialActionUnits::SHOULDER_SHRUG,
  kHeadShakeBackAndForth = hri_msgs::msg::FacialActionUnits::HEAD_SHAKE_BACK_AND_FORTH,
  kHeadNodUpAndDown = hri_msgs::msg::FacialActionUnits::HEAD_NOD_UP_AND_DOWN,
  kFlash = hri_msgs::msg::FacialActionUnits::FLASH,
  kPartialFlash = hri_msgs::msg::FacialActionUnits::PARTIAL_FLASH,
  kShiverTremble = hri_msgs::msg::FacialActionUnits::SHIVER_TREMBLE,
  kFastUpDownLook = hri_msgs::msg::FacialActionUnits::FAST_UP_DOWN_LOOK
};

enum class FacialLandmark
{
  kRightEar = hri_msgs::msg::FacialLandmarks::RIGHT_EAR,
  kRightProfile1 = hri_msgs::msg::FacialLandmarks::RIGHT_PROFILE_1,
  kRightProfile2 = hri_msgs::msg::FacialLandmarks::RIGHT_PROFILE_2,
  kRightProfile3 = hri_msgs::msg::FacialLandmarks::RIGHT_PROFILE_3,
  kRightProfile4 = hri_msgs::msg::FacialLandmarks::RIGHT_PROFILE_4,
  kRightProfile5 = hri_msgs::msg::FacialLandmarks::RIGHT_PROFILE_5,
  kRightProfile6 = hri_msgs::msg::FacialLandmarks::RIGHT_PROFILE_6,
  kRightProfile7 = hri_msgs::msg::FacialLandmarks::RIGHT_PROFILE_7,
  kMenton = hri_msgs::msg::FacialLandmarks::MENTON,
  kLeftEar = hri_msgs::msg::FacialLandmarks::LEFT_EAR,
  kLeftProfile1 = hri_msgs::msg::FacialLandmarks::LEFT_PROFILE_1,
  kLeftProfile2 = hri_msgs::msg::FacialLandmarks::LEFT_PROFILE_2,
  kLeftProfile3 = hri_msgs::msg::FacialLandmarks::LEFT_PROFILE_3,
  kLeftProfile4 = hri_msgs::msg::FacialLandmarks::LEFT_PROFILE_4,
  kLeftProfile5 = hri_msgs::msg::FacialLandmarks::LEFT_PROFILE_5,
  kLeftProfile6 = hri_msgs::msg::FacialLandmarks::LEFT_PROFILE_6,
  kLeftProfile7 = hri_msgs::msg::FacialLandmarks::LEFT_PROFILE_7,
  kRightEyebrowOutside = hri_msgs::msg::FacialLandmarks::RIGHT_EYEBROW_OUTSIDE,
  kRightEyebrow1 = hri_msgs::msg::FacialLandmarks::RIGHT_EYEBROW_1,
  kRightEyebrow2 = hri_msgs::msg::FacialLandmarks::RIGHT_EYEBROW_2,
  kRightEyebrow3 = hri_msgs::msg::FacialLandmarks::RIGHT_EYEBROW_3,
  kRightEyebrowInside = hri_msgs::msg::FacialLandmarks::RIGHT_EYEBROW_INSIDE,
  kRightEyeOutside = hri_msgs::msg::FacialLandmarks::RIGHT_EYE_OUTSIDE,
  kRightEyeTop1 = hri_msgs::msg::FacialLandmarks::RIGHT_EYE_TOP_1,
  kRightEyeTop2 = hri_msgs::msg::FacialLandmarks::RIGHT_EYE_TOP_2,
  kRightEyeInside = hri_msgs::msg::FacialLandmarks::RIGHT_EYE_INSIDE,
  kRightEyeBottom1 = hri_msgs::msg::FacialLandmarks::RIGHT_EYE_BOTTOM_1,
  kRightEyeBottom2 = hri_msgs::msg::FacialLandmarks::RIGHT_EYE_BOTTOM_2,
  kRightPupil = hri_msgs::msg::FacialLandmarks::RIGHT_PUPIL,
  kLeftEyebrowOutside = hri_msgs::msg::FacialLandmarks::LEFT_EYEBROW_OUTSIDE,
  kLeftEyebrow1 = hri_msgs::msg::FacialLandmarks::LEFT_EYEBROW_1,
  kLeftEyebrow2 = hri_msgs::msg::FacialLandmarks::LEFT_EYEBROW_2,
  kLeftEyebrow3 = hri_msgs::msg::FacialLandmarks::LEFT_EYEBROW_3,
  kLeftEyebrowInside = hri_msgs::msg::FacialLandmarks::LEFT_EYEBROW_INSIDE,
  kLeftEyeOutside = hri_msgs::msg::FacialLandmarks::LEFT_EYE_OUTSIDE,
  kLeftEyeTop1 = hri_msgs::msg::FacialLandmarks::LEFT_EYE_TOP_1,
  kLeftEyeTop2 = hri_msgs::msg::FacialLandmarks::LEFT_EYE_TOP_2,
  kLeftEyeInside = hri_msgs::msg::FacialLandmarks::LEFT_EYE_INSIDE,
  kLeftEyeBottom1 = hri_msgs::msg::FacialLandmarks::LEFT_EYE_BOTTOM_1,
  kLeftEyeBottom2 = hri_msgs::msg::FacialLandmarks::LEFT_EYE_BOTTOM_2,
  kLeftPupil = hri_msgs::msg::FacialLandmarks::LEFT_PUPIL,
  kSellion = hri_msgs::msg::FacialLandmarks::SELLION,
  kNose1 = hri_msgs::msg::FacialLandmarks::NOSE_1,
  kNose2 = hri_msgs::msg::FacialLandmarks::NOSE_2,
  kNose = hri_msgs::msg::FacialLandmarks::NOSE,
  kNostril1 = hri_msgs::msg::FacialLandmarks::NOSTRIL_1,
  kNostril2 = hri_msgs::msg::FacialLandmarks::NOSTRIL_2,
  kNostril3 = hri_msgs::msg::FacialLandmarks::NOSTRIL_3,
  kNostril4 = hri_msgs::msg::FacialLandmarks::NOSTRIL_4,
  kNostril5 = hri_msgs::msg::FacialLandmarks::NOSTRIL_5,
  kMouthOuterRight = hri_msgs::msg::FacialLandmarks::MOUTH_OUTER_RIGHT,
  kMouthOuterTop1 = hri_msgs::msg::FacialLandmarks::MOUTH_OUTER_TOP_1,
  kMouthOuterTop2 = hri_msgs::msg::FacialLandmarks::MOUTH_OUTER_TOP_2,
  kMouthOuterTop3 = hri_msgs::msg::FacialLandmarks::MOUTH_OUTER_TOP_3,
  kMouthOuterTop4 = hri_msgs::msg::FacialLandmarks::MOUTH_OUTER_TOP_4,
  kMouthOuterTop5 = hri_msgs::msg::FacialLandmarks::MOUTH_OUTER_TOP_5,
  kMouthOuterLeft = hri_msgs::msg::FacialLandmarks::MOUTH_OUTER_LEFT,
  kMouthOuterBottom1 = hri_msgs::msg::FacialLandmarks::MOUTH_OUTER_BOTTOM_1,
  kMouthOuterBottom2 = hri_msgs::msg::FacialLandmarks::MOUTH_OUTER_BOTTOM_2,
  kMouthOuterBottom3 = hri_msgs::msg::FacialLandmarks::MOUTH_OUTER_BOTTOM_3,
  kMouthOuterBottom4 = hri_msgs::msg::FacialLandmarks::MOUTH_OUTER_BOTTOM_4,
  kMouthOuterBottom5 = hri_msgs::msg::FacialLandmarks::MOUTH_OUTER_BOTTOM_5,
  kMouthInnerRight = hri_msgs::msg::FacialLandmarks::MOUTH_INNER_RIGHT,
  kMouthInnerTop1 = hri_msgs::msg::FacialLandmarks::MOUTH_INNER_TOP_1,
  kMouthInnerTop2 = hri_msgs::msg::FacialLandmarks::MOUTH_INNER_TOP_2,
  kMouthInnerTop3 = hri_msgs::msg::FacialLandmarks::MOUTH_INNER_TOP_3,
  kMouthInnerLeft = hri_msgs::msg::FacialLandmarks::MOUTH_INNER_LEFT,
  kMouthInnerBottom1 = hri_msgs::msg::FacialLandmarks::MOUTH_INNER_BOTTOM_1,
  kMouthInnerBottom2 = hri_msgs::msg::FacialLandmarks::MOUTH_INNER_BOTTOM_2,
  kMouthInnerBottom3 = hri_msgs::msg::FacialLandmarks::MOUTH_INNER_BOTTOM_3
};

enum class FeatureType
{
  kInvalid = 0,
  kPerson = (1u << 0),  // all known persons, whether or not they are currently seen
  kTrackedPerson = (1u << 1),  // only the actively tracked persons
  kFace = (1u << 2),
  kBody = (1u << 3),
  kVoice = (1u << 4)
};  // note that FeatureType values can also be used as bitmasks

enum class Gender
{
  kFemale = hri_msgs::msg::SoftBiometrics::FEMALE,
  kMale = hri_msgs::msg::SoftBiometrics::MALE,
  kOther = hri_msgs::msg::SoftBiometrics::OTHER
};

struct IntensityConfidence
{
  float intensity;
  float confidence;
};

typedef std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr>
  NodeLikeSharedPtr;

// structure mocking the rclcpp::NodeInterfaces, not yet available in Humble
struct NodeInterfaces
{
  explicit NodeInterfaces(NodeLikeSharedPtr node_like)
  {
    std::visit(
      [&](auto && node) {
        base = node->get_node_base_interface();
        clock = node->get_node_clock_interface();
        graph = node->get_node_graph_interface();
        logging = node->get_node_logging_interface();
        parameters = node->get_node_parameters_interface();
        services = node->get_node_services_interface();
        time_source = node->get_node_time_source_interface();
        timers = node->get_node_timers_interface();
        topics = node->get_node_topics_interface();
        waitables = node->get_node_waitables_interface();
      }, node_like);
  }

  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &
  get_node_base_interface() const {return base;}
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &
  get_node_base_interface() {return base;}

  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr &
  get_node_clock_interface() const {return clock;}
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr &
  get_node_clock_interface() {return clock;}

  const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr &
  get_node_graph_interface() const {return graph;}
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr &
  get_node_graph_interface() {return graph;}

  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr &
  get_node_logging_interface() const {return logging;}
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr &
  get_node_logging_interface() {return logging;}

  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr &
  get_node_parameters_interface() const {return parameters;}
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr &
  get_node_parameters_interface() {return parameters;}

  const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr &
  get_node_services_interface() const {return services;}
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr &
  get_node_services_interface() {return services;}

  const rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr &
  get_node_time_source_interface() const {return time_source;}
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr &
  get_node_time_source_interface() {return time_source;}

  const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr &
  get_node_timers_interface() const {return timers;}
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr &
  get_node_timers_interface() {return timers;}

  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr &
  get_node_topics_interface() const {return topics;}
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr &
  get_node_topics_interface() {return topics;}

  const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr &
  get_node_waitables_interface() const {return waitables;}
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr &
  get_node_waitables_interface() {return waitables;}

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services;
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr time_source;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables;
};

struct PointOfInterest
{
  float x;
  float y;
  float c;
};

enum class SkeletalKeypoint
{
  kNose = hri_msgs::msg::Skeleton2D::NOSE,
  kNeck = hri_msgs::msg::Skeleton2D::NECK,
  kRightShoulder = hri_msgs::msg::Skeleton2D::RIGHT_SHOULDER,
  kRightElbow = hri_msgs::msg::Skeleton2D::RIGHT_ELBOW,
  kRightWrist = hri_msgs::msg::Skeleton2D::RIGHT_WRIST,
  kLeftShoulder = hri_msgs::msg::Skeleton2D::LEFT_SHOULDER,
  kLeftElbow = hri_msgs::msg::Skeleton2D::LEFT_ELBOW,
  kLeftWrist = hri_msgs::msg::Skeleton2D::LEFT_WRIST,
  kRightHip = hri_msgs::msg::Skeleton2D::RIGHT_HIP,
  kRightKnee = hri_msgs::msg::Skeleton2D::RIGHT_KNEE,
  kRightAnkle = hri_msgs::msg::Skeleton2D::RIGHT_ANKLE,
  kLeftHip = hri_msgs::msg::Skeleton2D::LEFT_HIP,
  kLeftKnee = hri_msgs::msg::Skeleton2D::LEFT_KNEE,
  kLeftAnkle = hri_msgs::msg::Skeleton2D::LEFT_ANKLE,
  kLeftEye = hri_msgs::msg::Skeleton2D::LEFT_EYE,
  kRightEye = hri_msgs::msg::Skeleton2D::RIGHT_EYE,
  kLeftEar = hri_msgs::msg::Skeleton2D::LEFT_EAR,
  kRightEar = hri_msgs::msg::Skeleton2D::RIGHT_EAR
};

typedef std::map<FacialActionUnit, IntensityConfidence> FacialActionUnits;
typedef std::map<FacialLandmark, PointOfInterest> FacialLandmarks;
typedef std::string ID;
typedef std::map<SkeletalKeypoint, PointOfInterest> SkeletalKeypoints;

}  // namespace hri

#endif  // HRI__TYPES_HPP_
