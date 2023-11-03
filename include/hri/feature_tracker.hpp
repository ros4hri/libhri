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

#ifndef HRI__FEATURE_TRACKER_HPP_
#define HRI__FEATURE_TRACKER_HPP_

#include <memory>
#include <optional>
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace hri
{
typedef std::string ID;

enum FeatureType
{
  invalid = 0,
  person = (1u << 0),  // all known persons, whether or not they are currently seen
  tracked_person = (1u << 1),  // only the actively tracked persons
  face = (1u << 2),
  body = (1u << 3),
  voice = (1u << 4)
};  // note that FeatureType values can also be used as bitmasks

class FeatureTracker
{
public:
  /* creates a new feature tracker (eg, a face, body or voice tracker).
   *
   * This constructor should not be called directly. Instead, use one of the
   * specialization: hri::Face, hri::Body, hri::Voice.
   *
   * Note however that instances would normally be automatically created, and accessed via
   * the methods exposed by hri::HRIListener.
   *
   * Note that the resulting instance is non-copyable, as it includes
   * non-trivial, and typically non-reentrant, logic to subscribe/unsubscribe
   * HRI-related topics.
   */
  explicit FeatureTracker(ID id)
  : id_(id), ns_("")
  {
  }

  virtual ~FeatureTracker()
  {
  }

  // forbids copies of our 'feature trackers', as we need to internally manage
  // if/when they disappear. Instead, access them via shared pointers (cf HRIListener API).

  // TODO(SLE): ask a C++ expert how to enable that while avoid compilation errors when
  // building/moving a FeatureTracker into a container (in HRIListener)
  FeatureTracker(const FeatureTracker &) = delete;


  /* returns the unique ID of this feature.
   *
   * :see: FeatureTracker::getNamespace, to access the fully-qualified topic
   * namespace under which this feature is published.
   */
  ID id() const
  {
    return id_;
  }

  /* returns the topic namespace under which this feature is advertised.
   */
  std::string getNamespace() const
  {
    return ns_;
  }

  /* alias for FeatureTracker::getNamespace
   */
  std::string ns() const
  {
    return getNamespace();
  }

  bool operator<(const FeatureTracker & other) const
  {
    return id_ < other.id();
  }


  virtual void init() = 0;

protected:
  ID id_;
  // topic namespace under which this feature is advertised
  std::string ns_;
};

}  // namespace hri

#endif  // HRI__FEATURE_TRACKER_HPP_
