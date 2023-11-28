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

// Copyright 2016, 2017, 2018, 2019, 2020, 2021 Erwan BERNARD.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// Copied and adapted from https://github.com/edmBernard/pybind11_opencv_numpy

# ifndef PYHRI__NDARRAY_CONVERTER_H__
# define PYHRI__NDARRAY_CONVERTER_H__

#include <Python.h>
#include <opencv2/core/core.hpp>


class NDArrayConverter {
public:
    // must call this first, or the other routines don't work!
    static bool init_numpy();

    static bool toMat(PyObject* o, cv::Mat &m);
    static PyObject* toNDArray(const cv::Mat& mat);
};

//
// Define the type converter
//

#include <pybind11/pybind11.h>

namespace pybind11 { namespace detail {

template <> struct type_caster<cv::Mat> {
public:

    PYBIND11_TYPE_CASTER(cv::Mat, _("numpy.ndarray"));

    bool load(handle src, bool /* convert */) {
        return NDArrayConverter::toMat(src.ptr(), value);
    }

    static handle cast(const cv::Mat &m, return_value_policy, handle) {
        return handle(NDArrayConverter::toNDArray(m));
    }
};


}} // namespace pybind11::detail

# endif  // PYHRI__NDARRAY_CONVERTER_H__
