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

#include <chrono>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "hri/body.hpp"
#include "hri/face.hpp"
#include "hri/feature_tracker.hpp"
#include "hri/hri.hpp"
#include "hri/person.hpp"
#include "hri/types.hpp"
#include "hri/voice.hpp"
#include "pybind11/chrono.h"
#include "pybind11/functional.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"

#include "pyhri/ndarray_converter.h"
#include "pyhri/converters.hpp"

namespace py = pybind11;

namespace pyhri
{

class PyFeatureTracker : public hri::FeatureTracker
{
public:
  using hri::FeatureTracker::FeatureTracker;
  std::optional<geometry_msgs::msg::TransformStamped> transform() const override
  {
    PYBIND11_OVERRIDE(
      // cppcheck-suppress[syntaxError] trailing comma is required for 0 arg function overrides
      std::optional<geometry_msgs::msg::TransformStamped>, hri::FeatureTracker, transform, );
  }
};

class PyPubFeatureTracker : public hri::FeatureTracker
{
public:
  using hri::FeatureTracker::transform;
  using hri::FeatureTracker::transformFromReference;
};

class PyHRIListener : public hri::HRIListener
{
public:
  [[nodiscard]] static std::shared_ptr<PyHRIListener> create(
    std::string node_name, bool auto_spin)
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, NULL);
    }
    return std::shared_ptr<PyHRIListener>(
      new PyHRIListener(rclcpp::Node::make_shared(node_name), auto_spin));
  }

  void spin_some(std::chrono::nanoseconds timeout) {executor_->spin_some(timeout);}

protected:
  explicit PyHRIListener(rclcpp::Node::SharedPtr node, bool auto_spin)
  : hri::HRIListener(node), node_(node)
  {
    executor_ = rclcpp::executors::SingleThreadedExecutor::make_unique();
    executor_->add_node(node_);
    if (auto_spin) {
      thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::UniquePtr executor_;
  std::unique_ptr<std::thread> thread_;
};

constexpr char pybind_enum_doc[]{
  R"(
  This is an enumeration class but different from enum.Enum.

  For a basic usage, see
  https://pybind11.readthedocs.io/en/stable/classes.html#enumerations-and-internal-types.
  For some insights on the differences with enum.Enum, see
  https://github.com/pybind/pybind11/issues/2332.

  Notably, the object is not iterable, you can use use `__members__` method instead.
  )"};

PYBIND11_MODULE(hri, m) {
  m.doc() =
    R"(
    Python wrapper library to the ROS4HRI (https://www.ros.org/reps/rep-0155.html) framework.

    Each exported object is documented, to view it use `print(<class>.__doc__)` and/or
    `help(<class)`.
    The main entry point for its usage is `HRIListener` class.
    )";

  NDArrayConverter::init_numpy();

  py::enum_<hri::EngagementLevel> engagement_level(m, "EngagementLevel", pybind_enum_doc);
  engagement_level.value("DISENGAGED", hri::EngagementLevel::kDisengaged);
  engagement_level.value("ENGAGING", hri::EngagementLevel::kEngaging);
  engagement_level.value("ENGAGED", hri::EngagementLevel::kEngaged);
  engagement_level.value("DISENGAGING", hri::EngagementLevel::kDisengaging);

  py::enum_<hri::Gender> gender(m, "Gender", pybind_enum_doc);
  gender.value("FEMALE", hri::Gender::kFemale);
  gender.value("MALE", hri::Gender::kMale);
  gender.value("OTHER", hri::Gender::kOther);

  py::enum_<hri::FacialActionUnit> fau(m, "FacialActionUnit", pybind_enum_doc);
  fau.value("NEUTRAL_FACE", hri::FacialActionUnit::kNeutralFace);
  fau.value("INNER_BROW_RAISER", hri::FacialActionUnit::kInnerBrowRaiser);
  fau.value("OUTER_BROW_RAISER", hri::FacialActionUnit::kOuterBrowRaiser);
  fau.value("BROW_LOWERER", hri::FacialActionUnit::kBrowLowerer);
  fau.value("UPPER_LID_RAISER", hri::FacialActionUnit::kUpperLidRaiser);
  fau.value("CHEEK_RAISER", hri::FacialActionUnit::kCheeckRaiser);
  fau.value("LID_TIGHTENER", hri::FacialActionUnit::kLidTightener);
  fau.value("LIPS_TOWARD_EACH_OTHER", hri::FacialActionUnit::kLipsTowardEachOther);
  fau.value("NOSE_WRINKLER", hri::FacialActionUnit::kNoseWrinkler);
  fau.value("UPPER_LIP_RAISER", hri::FacialActionUnit::kUpperLipRaiser);
  fau.value("NASOLABIAL_DEEPENER", hri::FacialActionUnit::kNasolabialDeepener);
  fau.value("LIP_CORNER_PULLER", hri::FacialActionUnit::kLipCornerPuller);
  fau.value("SHARP_LIP_PULLER", hri::FacialActionUnit::kSharpLipPuller);
  fau.value("DIMPLER", hri::FacialActionUnit::kDimpler);
  fau.value("LIP_CORNER_DEPRESSOR", hri::FacialActionUnit::kLipCornerDepressor);
  fau.value("LOWER_LIP_DEPRESSOR", hri::FacialActionUnit::kLowerLipDepressor);
  fau.value("CHIN_RAISER", hri::FacialActionUnit::kChinRaiser);
  fau.value("LIP_PUCKER", hri::FacialActionUnit::kLipPucker);
  fau.value("TONGUE_SHOW", hri::FacialActionUnit::kTongueShow);
  fau.value("LIP_STRETCHER", hri::FacialActionUnit::kLipStretcher);
  fau.value("NECK_TIGHTENER", hri::FacialActionUnit::kNeckTightener);
  fau.value("LIP_FUNNELER", hri::FacialActionUnit::kLipFunneler);
  fau.value("LIP_TIGHTENER", hri::FacialActionUnit::kLipTightener);
  fau.value("LIP_PRESSOR", hri::FacialActionUnit::kLipPressor);
  fau.value("LIPS_PART", hri::FacialActionUnit::kLipsPart);
  fau.value("JAW_DROP", hri::FacialActionUnit::kJawDrop);
  fau.value("MOUTH_STRETCH", hri::FacialActionUnit::kMouthStretch);
  fau.value("LIP_SUCK", hri::FacialActionUnit::kLipSuck);
  fau.value("HEAD_TURN_LEFT", hri::FacialActionUnit::kHeadTurnLeft);
  fau.value("HEAD_TURN_RIGHT", hri::FacialActionUnit::kHeadTurnRight);
  fau.value("HEAD_UP", hri::FacialActionUnit::kHeadUp);
  fau.value("HEAD_DOWN", hri::FacialActionUnit::kHeadDown);
  fau.value("HEAD_TILT_LEFT", hri::FacialActionUnit::kHeadTiltLeft);
  fau.value("HEAD_TILT_RIGHT", hri::FacialActionUnit::kHeadTiltRight);
  fau.value("HEAD_FORWARD", hri::FacialActionUnit::kHeadForward);
  fau.value("HEAD_BACK", hri::FacialActionUnit::kHeadBack);
  fau.value("EYES_TURN_LEFT", hri::FacialActionUnit::kEyesTurnLeft);
  fau.value("EYES_TURN_RIGHT", hri::FacialActionUnit::kEyesTurnRight);
  fau.value("EYES_UP", hri::FacialActionUnit::kEyesUp);
  fau.value("EYES_DOWN", hri::FacialActionUnit::kEyesDown);
  fau.value("WALLEYE", hri::FacialActionUnit::kWalleye);
  fau.value("CROSS_EYE", hri::FacialActionUnit::kCrossEye);
  fau.value(
    "EYES_POSITIONED_TO_LOOK_AT_OTHER_PERSON",
    hri::FacialActionUnit::kEyesPositionedToLookAtOtherPerson);
  fau.value("BROWS_AND_FOREHEAD_NOT_VISIBLE", hri::FacialActionUnit::kBrownsAndForeheadNotVisible);
  fau.value("EYES_NOT_VISIBLE", hri::FacialActionUnit::kEyesNotVisible);
  fau.value("LOWER_FACE_NOT_VISIBLE", hri::FacialActionUnit::kLowerFaceNotVisible);
  fau.value("ENTIRE_FACE_NOT_VISIBLE", hri::FacialActionUnit::kEntireFaceNotVisible);
  fau.value("UNSOCIABLE", hri::FacialActionUnit::kUnsociable);
  fau.value("JAW_THRUST", hri::FacialActionUnit::kJawThrust);
  fau.value("JAW_SIDEWAYS", hri::FacialActionUnit::kJawSideways);
  fau.value("JAW_CLENCHER", hri::FacialActionUnit::kJawClencher);
  fau.value("LIP_BITE", hri::FacialActionUnit::kLipBite);
  fau.value("CHEEK_BLOW", hri::FacialActionUnit::kCheekBlow);
  fau.value("CHEEK_PUFF", hri::FacialActionUnit::kCheekPuff);
  fau.value("CHEEK_SUCK", hri::FacialActionUnit::kCheekSuck);
  fau.value("TONGUE_BULGE", hri::FacialActionUnit::kTongueBulge);
  fau.value("LIP_WIPE", hri::FacialActionUnit::kLipWipe);
  fau.value("NOSTRIL_DILATOR", hri::FacialActionUnit::kNostrilDilator);
  fau.value("NOSTRIL_COMPRESSOR", hri::FacialActionUnit::kNostrilCompressor);
  fau.value("SNIFF", hri::FacialActionUnit::kSniff);
  fau.value("LID_DROOP", hri::FacialActionUnit::kLidDroop);
  fau.value("SLIT", hri::FacialActionUnit::kSlit);
  fau.value("EYES_CLOSED", hri::FacialActionUnit::kEyesClosed);
  fau.value("SQUINT", hri::FacialActionUnit::kSquint);
  fau.value("BLINK", hri::FacialActionUnit::kBlink);
  fau.value("WINK", hri::FacialActionUnit::kWink);
  fau.value("SPEECH", hri::FacialActionUnit::kSpeech);
  fau.value("SWALLOW", hri::FacialActionUnit::kSwallow);
  fau.value("CHEWING", hri::FacialActionUnit::kChewing);
  fau.value("SHOULDER_SHRUG", hri::FacialActionUnit::kShoulderShrug);
  fau.value("HEAD_SHAKE_BACK_AND_FORTH", hri::FacialActionUnit::kHeadShakeBackAndForth);
  fau.value("HEAD_NOD_UP_AND_DOWN", hri::FacialActionUnit::kHeadNodUpAndDown);
  fau.value("FLASH", hri::FacialActionUnit::kFlash);
  fau.value("PARTIAL_FLASH", hri::FacialActionUnit::kPartialFlash);
  fau.value("SHIVER_TREMBLE", hri::FacialActionUnit::kShiverTremble);
  fau.value("FAST_UP_DOWN_LOOK", hri::FacialActionUnit::kFastUpDownLook);

  py::enum_<hri::FacialLandmark> facial_landmark(m, "FacialLandmark", pybind_enum_doc);
  facial_landmark.value("RIGHT_EAR", hri::FacialLandmark::kRightEar);
  facial_landmark.value("RIGHT_PROFILE_1", hri::FacialLandmark::kRightProfile1);
  facial_landmark.value("RIGHT_PROFILE_2", hri::FacialLandmark::kRightProfile2);
  facial_landmark.value("RIGHT_PROFILE_3", hri::FacialLandmark::kRightProfile3);
  facial_landmark.value("RIGHT_PROFILE_4", hri::FacialLandmark::kRightProfile4);
  facial_landmark.value("RIGHT_PROFILE_5", hri::FacialLandmark::kRightProfile5);
  facial_landmark.value("RIGHT_PROFILE_6", hri::FacialLandmark::kRightProfile6);
  facial_landmark.value("RIGHT_PROFILE_7", hri::FacialLandmark::kRightProfile7);
  facial_landmark.value("MENTON", hri::FacialLandmark::kMenton);
  facial_landmark.value("LEFT_EAR", hri::FacialLandmark::kLeftEar);
  facial_landmark.value("LEFT_PROFILE_1", hri::FacialLandmark::kLeftProfile1);
  facial_landmark.value("LEFT_PROFILE_2", hri::FacialLandmark::kLeftProfile2);
  facial_landmark.value("LEFT_PROFILE_3", hri::FacialLandmark::kLeftProfile3);
  facial_landmark.value("LEFT_PROFILE_4", hri::FacialLandmark::kLeftProfile4);
  facial_landmark.value("LEFT_PROFILE_5", hri::FacialLandmark::kLeftProfile5);
  facial_landmark.value("LEFT_PROFILE_6", hri::FacialLandmark::kLeftProfile6);
  facial_landmark.value("LEFT_PROFILE_7", hri::FacialLandmark::kLeftProfile7);
  facial_landmark.value("RIGHT_EYEBROW_OUTSIDE", hri::FacialLandmark::kRightEyebrowOutside);
  facial_landmark.value("RIGHT_EYEBROW_1", hri::FacialLandmark::kRightEyebrow1);
  facial_landmark.value("RIGHT_EYEBROW_2", hri::FacialLandmark::kRightEyebrow2);
  facial_landmark.value("RIGHT_EYEBROW_3", hri::FacialLandmark::kRightEyebrow3);
  facial_landmark.value("RIGHT_EYEBROW_INSIDE", hri::FacialLandmark::kRightEyebrowInside);
  facial_landmark.value("RIGHT_EYE_OUTSIDE", hri::FacialLandmark::kRightEyeOutside);
  facial_landmark.value("RIGHT_EYE_TOP_1", hri::FacialLandmark::kRightEyeTop1);
  facial_landmark.value("RIGHT_EYE_TOP_2", hri::FacialLandmark::kRightEyeTop2);
  facial_landmark.value("RIGHT_EYE_INSIDE", hri::FacialLandmark::kRightEyeInside);
  facial_landmark.value("RIGHT_EYE_BOTTOM_1", hri::FacialLandmark::kRightEyeBottom1);
  facial_landmark.value("RIGHT_EYE_BOTTOM_2", hri::FacialLandmark::kRightEyeBottom2);
  facial_landmark.value("RIGHT_PUPIL", hri::FacialLandmark::kRightPupil);
  facial_landmark.value("LEFT_EYEBROW_OUTSIDE", hri::FacialLandmark::kLeftEyebrowOutside);
  facial_landmark.value("LEFT_EYEBROW_1", hri::FacialLandmark::kLeftEyebrow1);
  facial_landmark.value("LEFT_EYEBROW_2", hri::FacialLandmark::kLeftEyebrow2);
  facial_landmark.value("LEFT_EYEBROW_3", hri::FacialLandmark::kLeftEyebrow3);
  facial_landmark.value("LEFT_EYEBROW_INSIDE", hri::FacialLandmark::kLeftEyebrowInside);
  facial_landmark.value("LEFT_EYE_OUTSIDE", hri::FacialLandmark::kLeftEyeOutside);
  facial_landmark.value("LEFT_EYE_TOP_1", hri::FacialLandmark::kLeftEyeTop1);
  facial_landmark.value("LEFT_EYE_TOP_2", hri::FacialLandmark::kLeftEyeTop2);
  facial_landmark.value("LEFT_EYE_INSIDE", hri::FacialLandmark::kLeftEyeInside);
  facial_landmark.value("LEFT_EYE_BOTTOM_1", hri::FacialLandmark::kLeftEyeBottom1);
  facial_landmark.value("LEFT_EYE_BOTTOM_2", hri::FacialLandmark::kLeftEyeBottom2);
  facial_landmark.value("LEFT_PUPIL", hri::FacialLandmark::kLeftPupil);
  facial_landmark.value("SELLION", hri::FacialLandmark::kSellion);
  facial_landmark.value("NOSE_1", hri::FacialLandmark::kNose1);
  facial_landmark.value("NOSE_2", hri::FacialLandmark::kNose2);
  facial_landmark.value("NOSE", hri::FacialLandmark::kNose);
  facial_landmark.value("NOSTRIL_1", hri::FacialLandmark::kNostril1);
  facial_landmark.value("NOSTRIL_2", hri::FacialLandmark::kNostril2);
  facial_landmark.value("NOSTRIL_3", hri::FacialLandmark::kNostril3);
  facial_landmark.value("NOSTRIL_4", hri::FacialLandmark::kNostril4);
  facial_landmark.value("NOSTRIL_5", hri::FacialLandmark::kNostril5);
  facial_landmark.value("MOUTH_OUTER_RIGHT", hri::FacialLandmark::kMouthOuterRight);
  facial_landmark.value("MOUTH_OUTER_TOP_1", hri::FacialLandmark::kMouthOuterTop1);
  facial_landmark.value("MOUTH_OUTER_TOP_2", hri::FacialLandmark::kMouthOuterTop2);
  facial_landmark.value("MOUTH_OUTER_TOP_3", hri::FacialLandmark::kMouthOuterTop3);
  facial_landmark.value("MOUTH_OUTER_TOP_4", hri::FacialLandmark::kMouthOuterTop4);
  facial_landmark.value("MOUTH_OUTER_TOP_5", hri::FacialLandmark::kMouthOuterTop5);
  facial_landmark.value("MOUTH_OUTER_LEFT", hri::FacialLandmark::kMouthOuterLeft);
  facial_landmark.value("MOUTH_OUTER_BOTTOM_1", hri::FacialLandmark::kMouthOuterBottom1);
  facial_landmark.value("MOUTH_OUTER_BOTTOM_2", hri::FacialLandmark::kMouthOuterBottom2);
  facial_landmark.value("MOUTH_OUTER_BOTTOM_3", hri::FacialLandmark::kMouthOuterBottom3);
  facial_landmark.value("MOUTH_OUTER_BOTTOM_4", hri::FacialLandmark::kMouthOuterBottom4);
  facial_landmark.value("MOUTH_OUTER_BOTTOM_5", hri::FacialLandmark::kMouthOuterBottom5);
  facial_landmark.value("MOUTH_INNER_RIGHT", hri::FacialLandmark::kMouthInnerRight);
  facial_landmark.value("MOUTH_INNER_TOP_1", hri::FacialLandmark::kMouthInnerTop1);
  facial_landmark.value("MOUTH_INNER_TOP_2", hri::FacialLandmark::kMouthInnerTop2);
  facial_landmark.value("MOUTH_INNER_TOP_3", hri::FacialLandmark::kMouthInnerTop3);
  facial_landmark.value("MOUTH_INNER_LEFT", hri::FacialLandmark::kMouthInnerLeft);
  facial_landmark.value("MOUTH_INNER_BOTTOM_1", hri::FacialLandmark::kMouthInnerBottom1);
  facial_landmark.value("MOUTH_INNER_BOTTOM_2", hri::FacialLandmark::kMouthInnerBottom2);
  facial_landmark.value("MOUTH_INNER_BOTTOM_3", hri::FacialLandmark::kMouthInnerBottom3);

  py::enum_<hri::SkeletalKeypoint> skeletal_keypoint(m, "SkeletalKeypoint", pybind_enum_doc);
  skeletal_keypoint.value("NOSE", hri::SkeletalKeypoint::kNose);
  skeletal_keypoint.value("NECK", hri::SkeletalKeypoint::kNeck);
  skeletal_keypoint.value("RIGHT_SHOULDER", hri::SkeletalKeypoint::kRightShoulder);
  skeletal_keypoint.value("RIGHT_ELBOW", hri::SkeletalKeypoint::kRightElbow);
  skeletal_keypoint.value("RIGHT_WRIST", hri::SkeletalKeypoint::kRightWrist);
  skeletal_keypoint.value("LEFT_SHOULDER", hri::SkeletalKeypoint::kLeftShoulder);
  skeletal_keypoint.value("LEFT_ELBOW", hri::SkeletalKeypoint::kLeftElbow);
  skeletal_keypoint.value("LEFT_WRIST", hri::SkeletalKeypoint::kLeftWrist);
  skeletal_keypoint.value("RIGHT_HIP", hri::SkeletalKeypoint::kRightHip);
  skeletal_keypoint.value("RIGHT_KNEE", hri::SkeletalKeypoint::kRightKnee);
  skeletal_keypoint.value("RIGHT_ANKLE", hri::SkeletalKeypoint::kRightAnkle);
  skeletal_keypoint.value("LEFT_HIP", hri::SkeletalKeypoint::kLeftHip);
  skeletal_keypoint.value("LEFT_KNEE", hri::SkeletalKeypoint::kLeftKnee);
  skeletal_keypoint.value("LEFT_ANKLE", hri::SkeletalKeypoint::kLeftAnkle);
  skeletal_keypoint.value("LEFT_EYE", hri::SkeletalKeypoint::kLeftEye);
  skeletal_keypoint.value("RIGHT_EYE", hri::SkeletalKeypoint::kRightEye);
  skeletal_keypoint.value("LEFT_EAR", hri::SkeletalKeypoint::kLeftEar);
  skeletal_keypoint.value("RIGHT_EAR", hri::SkeletalKeypoint::kRightEar);

  py::class_<hri::IntensityConfidence> intensity_confidence(m, "IntensityConfidence");
  intensity_confidence.doc() =
    R"(
    A trait evaluation result.

    Attributes:
    intensity -- strenght of the trait (float [0., 1.])
    confidence -- confidence of the evaluation itself (float [0., 1.])
    )";
  intensity_confidence.def(
    py::init<float, float>(), py::arg("intensity") = 0., py::arg("confidence") = 0.);
  intensity_confidence.def_readwrite("intensity", &hri::IntensityConfidence::intensity);
  intensity_confidence.def_readwrite("confidence", &hri::IntensityConfidence::confidence);
  intensity_confidence.def(
    "__repr__", [](const hri::IntensityConfidence & obj) {
      std::ostringstream ss;
      ss << "hri.IntensityConfidence {intensity: " << obj.intensity << ", confidence: " <<
        obj.confidence << "}";
      return ss.str();
    });

  py::class_<hri::PointOfInterest> point_of_interest(m, "PointOfInterest");
  point_of_interest.doc() =
    R"(
    The evaluation of the location in an image of a point of interest.

    Attributes:
    x -- relative horizontal position, starting from the left (float [0., 1.])
    y -- relative vertical position, starting from the top (float [0., 1.])
    c -- confidence of the evaluation itself (float [0., 1.])
    )";
  point_of_interest.def(
    py::init<float, float, float>(), py::arg("x") = 0., py::arg("y") = 0., py::arg("c") = 0.);
  point_of_interest.def_readwrite("x", &hri::PointOfInterest::x);
  point_of_interest.def_readwrite("y", &hri::PointOfInterest::y);
  point_of_interest.def_readwrite("c", &hri::PointOfInterest::c);
  point_of_interest.def(
    "__repr__", [](const hri::PointOfInterest & obj) {
      std::ostringstream ss;
      ss << "hri.PointOfInterest {x: " << obj.x << ", y: " << obj.y << ", c: " << obj.c << "}";
      return ss.str();
    });

  py::class_<hri::FeatureTracker, std::shared_ptr<hri::FeatureTracker>, PyFeatureTracker>
  feature_tracker(m, "FeatureTracker");
  feature_tracker.doc() =
    R"(
    The generic feature instance being tracked.

    This class should be created and managed only by HRIListener, it is exposed only for read access
    purposes.
    All its properties may return None if not available.

    Properties:
    id -- unique ID of this feature (str)
    ns -- fully-qualified topic namespace under which this feature is published (str)
    frame -- name of the tf frame that correspond to this feature (str)
    transform -- feature stamped 3D transform (geometry_msgs.msg.TransformStamped)
    valid -- whether the feature is still 'valid', i.e., existing (bool)
    )";
  feature_tracker.def_property_readonly(
    "id", &hri::FeatureTracker::id, "Unique ID of this feature");
  feature_tracker.def_property_readonly(
    "ns", &hri::FeatureTracker::ns,
    "Fully-qualified topic namespace under which this feature is published");
  feature_tracker.def_property_readonly(
    "frame", &hri::FeatureTracker::frame, "Name of the tf frame that correspond to this feature");
  feature_tracker.def_property_readonly(
    "transform", &PyPubFeatureTracker::transform,
    "Feature stamped 3D transform (geometry_msgs.msg.TransformStamped)");
  feature_tracker.def_property_readonly(
    "valid", &PyPubFeatureTracker::valid, "Whether the feature is still 'valid', i.e., existing");
  feature_tracker.def(py::self < py::self);

  py::class_<hri::Body, std::shared_ptr<hri::Body>> body(m, "Body", feature_tracker);
  body.doc() =
    R"(
    The body feature instance being tracked.

    This class should be created and managed only by HRIListener, it is exposed only for read access
    purposes.
    It inherits from FeatureTracker, check its documentation for additional properties.
    All its properties may return None if not available.

    Properties:
    roi -- normalized 2D region of interest (RoI) of the body (Tuple (x,y,width,height))
    cropped -- body image, cropped from the source image (numpy.ndarray)
    skeleton -- 2D skeleton keypoints (Dict[SkeletalKeypoint, PointOfInterest])
    )";
  body.def_property_readonly(
    "roi", &hri::Body::roi,
    "Normalized 2D region of interest (RoI) of the body (Tuple (x,y,width,height))");
  body.def_property_readonly(
    "cropped", &hri::Body::cropped, "Body image, cropped from the source image (numpy.ndarray)");
  body.def_property_readonly(
    "skeleton", &hri::Body::skeleton,
    "2D skeleton keypoints (Dict[SkeletalKeypoint, PointOfInterest])");

  py::class_<hri::Face, std::shared_ptr<hri::Face>> face(m, "Face", feature_tracker);
  face.doc() =
    R"(
    The face feature instance being tracked.

    This class should be created and managed only by HRIListener, it is exposed only for read access
    purposes.
    It inherits from FeatureTracker, check its documentation for additional properties.
    All its properties may return None if not available.

    Properties:
    roi -- normalized 2D region of interest (RoI) of the face (Tuple (x,y,width,height))
    cropped -- face image, cropped from the source image (numpy.ndarray)
    aligned -- face image, cropped and aligned from the source image (numpy.ndarray)
    facial_landmarks -- facial landmarks (Dict[FacialLandmark, PointOfInterest])
    facial_action_units -- facial action units (Dict[FacialActionUnit, IntensityConfidence])
    age -- person's age in years (float)
    gender -- person's gender (Gender)
    gaze_transform -- gaze's stamped 3D transform (geometry_msgs.msg.TransformStamped)
    )";
  face.def_property_readonly(
    "roi", &hri::Face::roi,
    "Normalized 2D region of interest (RoI) of the face (Tuple (x,y,width,height))");
  face.def_property_readonly(
    "cropped", &hri::Face::cropped, "Face image, cropped from the source image (numpy.ndarray)");
  face.def_property_readonly(
    "aligned", &hri::Face::aligned,
    "Face image, cropped and aligned from the source image (numpy.ndarray)");
  face.def_property_readonly(
    "facial_landmarks", &hri::Face::facialLandmarks,
    "Facial landmarks (Dict[FacialLandmark, PointOfInterest])");
  face.def_property_readonly(
    "facial_action_units", &hri::Face::facialActionUnits,
    "Facial action units (Dict[FacialActionUnit, IntensityConfidence])");
  face.def_property_readonly(
    "age", &hri::Face::age, "Person's age in years (float)");
  face.def_property_readonly(
    "gender", &hri::Face::gender, "Person's gender (Gender)");
  face.def_property_readonly(
    "gaze_transform", &hri::Face::gazeTransform,
    "Gaze's stamped 3D transform (geometry_msgs.msg.TransformStamped)");

  py::class_<hri::Voice, std::shared_ptr<hri::Voice>> voice(m, "Voice", feature_tracker);
  voice.doc() =
    R"(
    The voice feature instance being tracked.

    This class should be created and managed only by HRIListener, it is exposed only for read access
    purposes.
    It inherits from FeatureTracker, check its documentation for additional properties.
    All its properties may return None if not available.

    Properties:
    is_speaking -- whether speech is currently detected in this voice (bool)
    speech -- last recognised final sentence (str)
    incremental_speech -- last recognised incremental sentence (str)

    Methods (use `help(Voice)` to see the signatures):
    on_speaking -- registers a callback function, to be invoked everytime speech is detected
    on_speech -- registers a callback function, to be invoked everytime a final sentence is detected
    on_incremental_speech -- registers a callback function, to be invoked everytime an incremental
                             sentence is detected
    )";
  voice.def_property_readonly(
    "is_speaking", &hri::Voice::isSpeaking,
    "Whether speech is currently detected in this voice (bool)");
  voice.def_property_readonly(
    "speech", &hri::Voice::speech, "Last recognised final sentence (str)");
  voice.def_property_readonly(
    "incremental_speech", &hri::Voice::incrementalSpeech,
    "Last recognised incremental sentence (str)");
  voice.def(
    "on_speaking", &hri::Voice::onSpeaking, py::arg("callback"),
    "Registers a callback function, to be invoked everytime speech is detected");
  voice.def(
    "on_speech", &hri::Voice::onSpeech, py::arg("callback"),
    "Registers a callback function, to be invoked everytime a final sentence is detected");
  voice.def(
    "on_incremental_speech", &hri::Voice::onIncrementalSpeech, py::arg("callback"),
    "Registers a callback function, to be invoked everytime san incremental sentence is detected");

  py::class_<hri::Person, std::shared_ptr<hri::Person>> person(m, "Person", feature_tracker);
  person.doc() =
    R"(
    The person feature instance being tracked or known.

    This class should be created and managed only by HRIListener, it is exposed only for read access
    purposes.
    It inherits from FeatureTracker, check its documentation for additional properties.
    All its properties may return None if not available.

    Properties:
    face -- face associated with the person (Face)
    body -- body associated with the person (Body)
    voice -- voice associated with the person (Voice)
    anonymous -- whether the person has not been identified yet (bool)
    engagement_status -- current engagement status with the robot (EngagementLevel)
    location_confidence -- confidence of the person transform estimate (float [0., 1.])
    alias -- ID of another Person object associated with the same person (str)
    )";
  person.def_property_readonly(
    "face", &hri::Person::face, "Face associated with the person (Face)");
  person.def_property_readonly(
    "body", &hri::Person::body, "Body associated with the person (Body)");
  person.def_property_readonly(
    "voice", &hri::Person::voice, "Voice associated with the person (Voice)");
  person.def_property_readonly(
    "anonymous", &hri::Person::anonymous, "Whether the person has not been identified yet (bool)");
  person.def_property_readonly(
    "engagement_status", &hri::Person::engagementStatus,
    "Current engagement status with the robot (EngagementLevel)");
  person.def_property_readonly(
    "location_confidence", &hri::Person::locationConfidence,
    "confidence of the person transform estimate (float [0., 1.])");
  person.def_property_readonly(
    "alias", &hri::Person::alias,
    "ID of another Person object associated with the same person (str)");
  // overrides the TrackedFeature one
  person.def_property_readonly("transform", &hri::Person::transform);

  py::class_<PyHRIListener, std::shared_ptr<PyHRIListener>> hri_listener(m, "HRIListener");
  hri_listener.doc() =
    R"(
    Main entry point to the library.

    The class must be instantiated through the factory function `create`.
    I will spawn a ROS node and use it to subscribe to all the ROS4HRI topics.
    The tracked features information can be accessed in Python native objects throught this object
    properties.

    Properties:
    faces -- currently tracked faces (Dict[str, Face])
    bodies -- currently tracked bodies (Dict[str, Body])
    voices -- currently tracked voices (Dict[str, Voice])
    persons -- currently known persons (Dict[str, Person])
    tracked_persons -- currently tracked persons (Dict[str, Person])

    Methods (use `help(HRIListener)` to see the signatures):
    create -- generate the class, selecting the spawned node name and whether it spins automatically
    on_face -- registers a callback function, to be invoked everytime a new face is tracked
    on_body -- registers a callback function, to be invoked everytime a new body is tracked
    on_voice -- registers a callback function, to be invoked everytime a new voice is tracked
    on_person -- registers a callback function, to be invoked everytime a new person is known
    on_tracked_person -- registers a callback function, to be invoked everytime a new person is
                         tracked
    on_face_lost -- registers a callback function, to be invoked everytime a tracked face is lost
    on_body_lost -- registers a callback function, to be invoked everytime a tracked body is lost
    on_voice_lost -- registers a callback function, to be invoked everytime a tracked voice is lost
    on_person_lost -- registers a callback function, to be invoked everytime a known person is
                      forgotten
    on_tracked_person_lost -- registers a callback function, to be invoked everytime a tracked
                              person is lost
    set_reference_frame -- selects the reference frame for all the `transform` properties
    spin_some -- if the class node does not spin automatically, this function must be called
                 regularly to manually spin it
    )";
  hri_listener.def(
    py::init(&PyHRIListener::create), py::arg("node_name"), py::arg("auto_spin") = true,
    "Generate the class, selecting the spawned node name and whether it spins automatically");
  hri_listener.def_property_readonly(
    "faces", &hri::HRIListener::getFaces, "Currently tracked faces (Dict[str, Face])");
  hri_listener.def_property_readonly(
    "bodies", &hri::HRIListener::getBodies, "Currently tracked bodies (Dict[str, Body])");
  hri_listener.def_property_readonly(
    "voices", &hri::HRIListener::getVoices, "Currently tracked voices (Dict[str, Voice])");
  hri_listener.def_property_readonly(
    "persons", &hri::HRIListener::getPersons, "Currently known persons (Dict[str, Person])");
  hri_listener.def_property_readonly(
    "tracked_persons", &hri::HRIListener::getTrackedPersons,
    "Currently tracked persons (Dict[str, Person])");
  hri_listener.def(
    "on_face", &hri::HRIListener::onFace, py::arg("callback"),
    "Registers a callback function, to be invoked everytime a new face is tracked");
  hri_listener.def(
    "on_body", &hri::HRIListener::onBody, py::arg("callback"),
    "Registers a callback function, to be invoked everytime a new body is tracked");
  hri_listener.def(
    "on_voice", &hri::HRIListener::onVoice, py::arg("callback"),
    "Registers a callback function, to be invoked everytime a new voice is tracked");
  hri_listener.def(
    "on_person", &hri::HRIListener::onPerson, py::arg("callback"),
    "Registers a callback function, to be invoked everytime a new person is known");
  hri_listener.def(
    "on_tracked_person", &hri::HRIListener::onTrackedPerson, py::arg("callback"),
    "Registers a callback function, to be invoked everytime a new person is tracked");
  hri_listener.def(
    "on_face_lost", &hri::HRIListener::onFaceLost, py::arg("callback"),
    "Registers a callback function, to be invoked everytime a tracked face is lost");
  hri_listener.def(
    "on_body_lost", &hri::HRIListener::onBodyLost, py::arg("callback"),
    "Registers a callback function, to be invoked everytime a tracked body is lost");
  hri_listener.def(
    "on_voice_lost", &hri::HRIListener::onVoiceLost, py::arg("callback"),
    "Registers a callback function, to be invoked everytime a tracked voice is lost");
  hri_listener.def(
    "on_person_lost", &hri::HRIListener::onPersonLost, py::arg("callback"),
    "Registers a callback function, to be invoked everytime a known person is forgotten");
  hri_listener.def(
    "on_tracked_person_lost", &hri::HRIListener::onTrackedPersonLost, py::arg("callback"),
    "Registers a callback function, to be invoked everytime a tracked person is lost");
  hri_listener.def(
    "set_reference_frame", &hri::HRIListener::setReferenceFrame, py::arg("frame"),
    "Selects the reference frame for all the `transform` properties");
  hri_listener.def(
    "spin_some", &PyHRIListener::spin_some, py::arg("timeout"),
    "If the class node does not spin automatically, this function must be called regularly to "
    "manually spin it");
}

}  // namespace pyhri
