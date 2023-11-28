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


PYBIND11_MODULE(pyhri, m) {
  m.doc() = "Python wrapper of the hri API";

  NDArrayConverter::init_numpy();

  py::enum_<hri::EngagementLevel> engagement_level(m, "EngagementLevel");
  engagement_level.value("DISENGAGED", hri::EngagementLevel::kDisengaged);
  engagement_level.value("ENGAGING", hri::EngagementLevel::kEngaging);
  engagement_level.value("ENGAGED", hri::EngagementLevel::kEngaged);
  engagement_level.value("DISENGAGING", hri::EngagementLevel::kDisengaging);
  engagement_level.export_values();

  py::enum_<hri::Gender> gender(m, "Gender");
  gender.value("FEMALE", hri::Gender::kFemale);
  gender.value("MALE", hri::Gender::kMale);
  gender.value("OTHER", hri::Gender::kOther);
  gender.export_values();

  py::enum_<hri::FacialActionUnit> fau(m, "FacialActionUnit");
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
  fau.export_values();

  py::enum_<hri::FacialLandmark> facial_landmark(m, "FacialLandmark");
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
  facial_landmark.export_values();

  py::enum_<hri::SkeletalKeypoint> skeletal_keypoint(m, "SkeletalKeypoint");
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
  skeletal_keypoint.export_values();

  py::class_<hri::IntensityConfidence> intensity_confidence(m, "IntensityConfidence");
  intensity_confidence.def_readwrite("intensity", &hri::IntensityConfidence::intensity);
  intensity_confidence.def_readwrite("confidence", &hri::IntensityConfidence::confidence);

  py::class_<hri::PointOfInterest> point_of_interest(m, "PointOfInterest");
  point_of_interest.def_readwrite("x", &hri::PointOfInterest::x);
  point_of_interest.def_readwrite("y", &hri::PointOfInterest::y);
  point_of_interest.def_readwrite("c", &hri::PointOfInterest::c);

  py::class_<hri::FeatureTracker, std::shared_ptr<hri::FeatureTracker>, PyFeatureTracker>
  feature_tracker(m, "FeatureTracker");
  feature_tracker.def_property_readonly("id", &hri::FeatureTracker::id);
  feature_tracker.def_property_readonly("ns", &hri::FeatureTracker::ns);
  feature_tracker.def_property_readonly("frame", &hri::FeatureTracker::frame);
  feature_tracker.def_property_readonly("transform", &PyPubFeatureTracker::transform);
  feature_tracker.def(py::self < py::self);

  py::class_<hri::Body, std::shared_ptr<hri::Body>> body(m, "Body", feature_tracker);
  body.def_property_readonly("roi", &hri::Body::roi);
  body.def_property_readonly("cropped", &hri::Body::cropped);
  body.def_property_readonly("skeleton", &hri::Body::skeleton);

  py::class_<hri::Face, std::shared_ptr<hri::Face>> face(m, "Face", feature_tracker);
  face.def_property_readonly("roi", &hri::Face::roi);
  face.def_property_readonly("cropped", &hri::Face::cropped);
  face.def_property_readonly("aligned", &hri::Face::aligned);
  face.def_property_readonly("facial_landmarks", &hri::Face::facialLandmarks);
  face.def_property_readonly("facial_action_units", &hri::Face::facialActionUnits);
  face.def_property_readonly("age", &hri::Face::age);
  face.def_property_readonly("gender", &hri::Face::gender);
  face.def_property_readonly("gaze_transform", &hri::Face::gazeTransform);

  py::class_<hri::Voice, std::shared_ptr<hri::Voice>> voice(m, "Voice", feature_tracker);
  voice.def_property_readonly("is_speaking", &hri::Voice::isSpeaking);
  voice.def_property_readonly("speech", &hri::Voice::speech);
  voice.def_property_readonly("incremental_speech", &hri::Voice::incrementalSpeech);
  voice.def("on_speaking", &hri::Voice::onSpeaking, py::arg("callback"));
  voice.def("on_speech", &hri::Voice::onSpeech, py::arg("callback"));
  voice.def("on_incremental_speech", &hri::Voice::onIncrementalSpeech, py::arg("callback"));

  py::class_<hri::Person, std::shared_ptr<hri::Person>> person(m, "Person", feature_tracker);
  person.def_property_readonly("face", &hri::Person::face);
  person.def_property_readonly("body", &hri::Person::body);
  person.def_property_readonly("voice", &hri::Person::voice);
  person.def_property_readonly("anonymous", &hri::Person::anonymous);
  person.def_property_readonly("engagement_status", &hri::Person::engagementStatus);
  person.def_property_readonly("location_confidence", &hri::Person::locationConfidence);
  person.def_property_readonly("alias", &hri::Person::alias);
  person.def_property_readonly("transform", &hri::Person::transform);

  py::class_<PyHRIListener, std::shared_ptr<PyHRIListener>> hri_listener(m, "HRIListener");
  hri_listener.def(
    py::init(&PyHRIListener::create), py::arg("node_name"), py::arg("auto_spin") = true);
  hri_listener.def_property_readonly("faces", &hri::HRIListener::getFaces);
  hri_listener.def_property_readonly("bodies", &hri::HRIListener::getBodies);
  hri_listener.def_property_readonly("voices", &hri::HRIListener::getVoices);
  hri_listener.def_property_readonly("persons", &hri::HRIListener::getPersons);
  hri_listener.def_property_readonly("tracked_persons", &hri::HRIListener::getTrackedPersons);
  hri_listener.def("on_face", &hri::HRIListener::onFace, py::arg("callback"));
  hri_listener.def("on_body", &hri::HRIListener::onBody, py::arg("callback"));
  hri_listener.def("on_voice", &hri::HRIListener::onVoice, py::arg("callback"));
  hri_listener.def("on_person", &hri::HRIListener::onPerson, py::arg("callback"));
  hri_listener.def("on_tracked_person", &hri::HRIListener::onTrackedPerson, py::arg("callback"));
  hri_listener.def("on_face_lost", &hri::HRIListener::onFaceLost, py::arg("callback"));
  hri_listener.def("on_body_lost", &hri::HRIListener::onBodyLost, py::arg("callback"));
  hri_listener.def("on_voice_lost", &hri::HRIListener::onVoiceLost, py::arg("callback"));
  hri_listener.def("on_person_lost", &hri::HRIListener::onPersonLost, py::arg("callback"));
  hri_listener.def(
    "on_tracked_person_lost", &hri::HRIListener::onTrackedPersonLost, py::arg("callback"));
  hri_listener.def("set_reference_frame", &hri::HRIListener::setReferenceFrame, py::arg("frame"));
  hri_listener.def("spin_some", &PyHRIListener::spin_some, py::arg("timeout"));
}

}  // namespace pyhri
