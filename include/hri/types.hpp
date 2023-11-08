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

#include <array>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hri_msgs/msg/engagement_level.hpp"
#include "hri_msgs/msg/normalized_point_of_interest2_d.hpp"
#include "hri_msgs/msg/normalized_region_of_interest2_d.hpp"
#include "hri_msgs/msg/soft_biometrics.hpp"
#include "opencv2/core.hpp"

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

typedef std::array<IntensityConfidence, 99> FacialActionUnits;
typedef std::array<hri_msgs::msg::NormalizedPointOfInterest2D, 70> FacialLandmarks;
typedef std::string ID;
typedef cv::Mat Image;
typedef hri_msgs::msg::NormalizedRegionOfInterest2D RegionOfInterest;
typedef std::array<hri_msgs::msg::NormalizedPointOfInterest2D, 18> SkeletonPoints;
typedef geometry_msgs::msg::TransformStamped Transform;

}  // namespace hri

#endif  // HRI__TYPES_HPP_
