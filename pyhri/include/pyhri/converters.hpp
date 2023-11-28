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

// Copyright (c) 2015-2019, Carnegie Mellon University. All rights reserved.
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
//    * Neither the name of the {copyright_holder} nor the names of its
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

// Copied and adapted from https://github.com/dimatura/pyrosmsg

#ifndef PYHRI__CONVERTERS_HPP_
#define PYHRI__CONVERTERS_HPP_

#include <set>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "opencv2/core.hpp"
#include "pybind11/pybind11.h"

namespace pyhri
{

static bool has_all_attribute_fields(pybind11::handle handle, const std::set<std::string> fields)
{
  for (const auto & field : fields) {
    if (!pybind11::hasattr(handle, field.c_str())) {
      return false;
    }
  }
  return true;
}

}  // namespace pyhri


namespace PYBIND11_NAMESPACE
{
namespace detail
{

template<>
struct type_caster<builtin_interfaces::msg::Time>
{
public:
  PYBIND11_TYPE_CASTER(
    builtin_interfaces::msg::Time, const_name("builtin_interfaces::msg::Time"));

  bool load(handle py_handle, bool)
  {
    if (!pyhri::has_all_attribute_fields(py_handle, {"sec", "nanosec"})) {
      return false;
    }

    value.sec = (py_handle.attr("sec")).cast<int32_t>();
    value.nanosec = (py_handle.attr("nanosec")).cast<uint32_t>();
    return true;
  }

  static handle cast(builtin_interfaces::msg::Time msg, return_value_policy, handle)
  {
    object py_obj = module::import("builtin_interfaces.msg").attr("Time")();
    py_obj.attr("sec") = pybind11::cast(msg.sec);
    py_obj.attr("nanosec") = pybind11::cast(msg.nanosec);
    py_obj.inc_ref();
    return py_obj;
  }
};

template<>
struct type_caster<std_msgs::msg::Header>
{
public:
  PYBIND11_TYPE_CASTER(std_msgs::msg::Header, const_name("std_msgs::msg::Header"));

  bool load(handle py_handle, bool)
  {
    if (!pyhri::has_all_attribute_fields(py_handle, {"stamp", "frame_id"})) {
      return false;
    }

    value.stamp = (py_handle.attr("stamp")).cast<builtin_interfaces::msg::Time>();
    value.frame_id = (py_handle.attr("frame_id")).cast<std::string>();
    return true;
  }

  static handle cast(std_msgs::msg::Header msg, return_value_policy, handle)
  {
    object py_obj = module::import("std_msgs.msg").attr("Header")();
    py_obj.attr("stamp") = pybind11::cast(msg.stamp);
    // ROS 2 uses UTF-8 for 'string' in messages and can directly convert to str(), see
    // https://design.ros2.org/articles/wide_strings.html
    // https://pybind11.readthedocs.io/en/stable/advanced/cast/strings.html
    py_obj.attr("frame_id") = pybind11::cast(msg.frame_id);
    py_obj.inc_ref();
    return py_obj;
  }
};

template<>
struct type_caster<geometry_msgs::msg::Vector3>
{
public:
  PYBIND11_TYPE_CASTER(geometry_msgs::msg::Vector3, const_name("geometry_msgs::msg::Vector3"));

  bool load(handle py_handle, bool)
  {
    if (!pyhri::has_all_attribute_fields(py_handle, {"x", "y", "z"})) {
      return false;
    }

    value.x = (py_handle.attr("x")).cast<double>();
    value.y = (py_handle.attr("y")).cast<double>();
    value.z = (py_handle.attr("z")).cast<double>();
    return true;
  }

  static handle cast(geometry_msgs::msg::Vector3 msg, return_value_policy, handle)
  {
    object py_obj = module::import("geometry_msgs.msg").attr("Vector3")();
    py_obj.attr("x") = pybind11::cast(msg.x);
    py_obj.attr("y") = pybind11::cast(msg.y);
    py_obj.attr("z") = pybind11::cast(msg.z);
    py_obj.inc_ref();
    return py_obj;
  }
};

template<>
struct type_caster<geometry_msgs::msg::Quaternion>
{
public:
  PYBIND11_TYPE_CASTER(
    geometry_msgs::msg::Quaternion, const_name("geometry_msgs::msg::Quaternion"));

  bool load(handle py_handle, bool)
  {
    if (!pyhri::has_all_attribute_fields(py_handle, {"x", "y", "z", "w"})) {
      return false;
    }

    value.x = (py_handle.attr("x")).cast<double>();
    value.y = (py_handle.attr("y")).cast<double>();
    value.z = (py_handle.attr("z")).cast<double>();
    value.z = (py_handle.attr("w")).cast<double>();
    return true;
  }

  static handle cast(geometry_msgs::msg::Quaternion msg, return_value_policy, handle)
  {
    object py_obj = module::import("geometry_msgs.msg").attr("Quaternion")();
    py_obj.attr("x") = pybind11::cast(msg.x);
    py_obj.attr("y") = pybind11::cast(msg.y);
    py_obj.attr("z") = pybind11::cast(msg.z);
    py_obj.attr("w") = pybind11::cast(msg.w);
    py_obj.inc_ref();
    return py_obj;
  }
};

template<>
struct type_caster<geometry_msgs::msg::Transform>
{
public:
  PYBIND11_TYPE_CASTER(geometry_msgs::msg::Transform, const_name("geometry_msgs::msg::Transform"));

  bool load(handle py_handle, bool)
  {
    if (!pyhri::has_all_attribute_fields(py_handle, {"translation", "rotation"})) {
      return false;
    }

    value.translation = (py_handle.attr("translation")).cast<geometry_msgs::msg::Vector3>();
    value.rotation = (py_handle.attr("rotation")).cast<geometry_msgs::msg::Quaternion>();
    return true;
  }

  static handle cast(geometry_msgs::msg::Transform msg, return_value_policy, handle)
  {
    object py_obj = module::import("geometry_msgs.msg").attr("Transform")();
    py_obj.attr("translation") = pybind11::cast(msg.translation);
    py_obj.attr("rotation") = pybind11::cast(msg.rotation);
    py_obj.inc_ref();
    return py_obj;
  }
};

template<>
struct type_caster<geometry_msgs::msg::TransformStamped>
{
public:
  PYBIND11_TYPE_CASTER(
    geometry_msgs::msg::TransformStamped, const_name("geometry_msgs::msg::TransformStamped"));

  bool load(handle py_handle, bool)
  {
    if (!pyhri::has_all_attribute_fields(py_handle, {"x", "y", "width", "height"})) {
      return false;
    }

    value.header = (py_handle.attr("header")).cast<std_msgs::msg::Header>();
    value.child_frame_id = (py_handle.attr("child_frame_id")).cast<std::string>();
    value.transform = (py_handle.attr("header")).cast<geometry_msgs::msg::Transform>();
    return true;
  }

  static handle cast(geometry_msgs::msg::TransformStamped msg, return_value_policy, handle)
  {
    object py_obj = module::import("geometry_msgs.msg").attr("TransformStamped")();
    py_obj.attr("header") = pybind11::cast(msg.header);
    // ROS 2 uses UTF-8 for 'string' in messages and can directly convert to str(), see
    // https://design.ros2.org/articles/wide_strings.html
    // https://pybind11.readthedocs.io/en/stable/advanced/cast/strings.html
    py_obj.attr("child_frame_id") = pybind11::cast(msg.child_frame_id);
    py_obj.attr("transform") = pybind11::cast(msg.transform);
    py_obj.inc_ref();
    return py_obj;
  }
};

template<>
struct type_caster<cv::Rect2f>
{
public:
  PYBIND11_TYPE_CASTER(
    cv::Rect2f, const_name("cv::Rect2f"));

  bool load(handle py_handle, bool)
  {
    if (!isinstance<tuple>(py_handle)) {
      return false;
    }
    const auto py_as_tuple = reinterpret_borrow<tuple>(py_handle);
    if (py_as_tuple.size() != 4) {
      return false;
    }
    for (const auto & element : py_as_tuple) {
      if (!isinstance<float>(element)) {
        return false;
      }
    }

    value.x = py_as_tuple[0].cast<float>();
    value.y = py_as_tuple[1].cast<float>();
    value.width = py_as_tuple[2].cast<float>();
    value.height = py_as_tuple[3].cast<float>();
    return true;
  }

  static handle cast(cv::Rect2f rect, return_value_policy, handle)
  {
    auto py_obj = tuple(4);
    py_obj[0] = pybind11::cast(rect.x);
    py_obj[1] = pybind11::cast(rect.y);
    py_obj[2] = pybind11::cast(rect.width);
    py_obj[3] = pybind11::cast(rect.height);
    py_obj.inc_ref();
    return py_obj;
  }
};

}  // namespace detail
}  // namespace PYBIND11_NAMESPACE

#endif  // PYHRI__CONVERTERS_HPP_
