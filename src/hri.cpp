// Copyright 2021 PAL Robotics S.L.
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

#include "hri/hri.h"
#include <algorithm>
#include <functional>
#include <iterator>
#include <memory>
#include <tuple>
#include <utility>
#include "hri_msgs/IdsList.h"

using namespace std;
using namespace hri;

HRIListener::HRIListener()
{
  init();
}

HRIListener::~HRIListener()
{
  ROS_DEBUG("Closing the HRI Listener");

  faces.clear();
}

void HRIListener::init()
{
  ROS_DEBUG("Initialising the HRI Listener");
  feature_subscribers_[FeatureType::face] = node_.subscribe<hri_msgs::IdsList>(
      "/humans/faces/tracked", 1,
      bind(&HRIListener::onTrackedFeature, this, FeatureType::face, _1));
}

void HRIListener::onTrackedFeature(FeatureType feature, hri_msgs::IdsListConstPtr tracked)
{
  if (feature == FeatureType::face)
  {
    // update the current list of tracked feature (face, body...) with
    // what has just been received on the respective /tracked topic.
    set<ID> current_ids;
    for (auto const& kv : faces)
    {
      current_ids.insert(kv.first);
    }

    set<ID> new_ids;
    for (auto const& id : tracked->ids)
    {
      new_ids.insert(ID(id));
    }

    set<ID> to_remove;
    set<ID> to_add;

    for (auto id : new_ids)
    {
      if (current_ids.find(id) == current_ids.end())
      {
        to_add.insert(id);
      }
    }

    for (auto id : current_ids)
    {
      if (new_ids.find(id) == new_ids.end())
      {
        to_remove.insert(id);
      }
    }

    for (auto id : to_remove)
    {
      faces.erase(id);
    }

    for (auto id : to_add)
    {
      auto face = make_shared<Face>(id, node_);
      face->init();
      faces.insert({ id, face });
    }
  }
}

