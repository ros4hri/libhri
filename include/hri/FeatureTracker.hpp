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


#ifndef HRI__FEATURETRACKER_HPP_
#define HRI__FEATURETRACKER_HPP_

#include <memory>
#include <optional>
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace hri
{
typedef std::string ID;

enum FeatureType
{
  person,          // all known persons, whether or not they are currently seen
  tracked_person,  // only the actively tracked persons
  face,
  body,
  voice
};

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
  // if/when they disappear. Instead, access them via weak pointers (cf HRIListener API).

  // TODO(todo): ask a C++ expert how to enable that while avoid compilation errors when
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

#endif  // HRI__FEATURETRACKER_HPP_
