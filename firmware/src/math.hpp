/****************************************************************************
*
*   Copyright (C) 2016  Zubax Robotics  <info@zubax.com>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <utility>
#include <cassert>


namespace math
{

using Scalar = float;

template <int Rows, int Cols>
using Matrix = Eigen::Matrix<Scalar, Rows, Cols>;

template <int Size>
using Vector = Matrix<Size, 1>;

/**
 * Inclusive range of the form [min, max].
 */
template <typename T = Scalar>
struct Range
{
    const T min;
    const T max;

    constexpr Range(const T min, const T max) :
        min(min),
        max(max)
    {
        assert(min < max);
    }

    constexpr T constrain(const T value) const
    {
        return std::min(std::max(value, min), max);
    }

    constexpr bool contains(const T value) const
    {
        return (value >= min) && (value <= max);
    }
};

/**
 * First-order IIR low pass filter.
 */
template <typename ValueType = Scalar,
          typename ScalarType = Scalar>
class LowPassIIRFilter
{
    ValueType y_;
    const ScalarType f_cutoff_;
    bool needs_initialization_;

public:
    LowPassIIRFilter(const ScalarType& cutoff_frequency, const ValueType& init_value) :
        y_(init_value),
        f_cutoff_(cutoff_frequency),
        needs_initialization_(false)
    { }

    LowPassIIRFilter(const ScalarType& cutoff_frequency) :
        f_cutoff_(cutoff_frequency),
        needs_initialization_(true)
    { }

    const ValueType& update(const ValueType& x, const ScalarType& dt)
    {
        assert(dt > 0);

        if (needs_initialization_)
        {
            needs_initialization_ = false;
            y_ = x;
        }
        else
        {
            constexpr ScalarType Pi2 = ScalarType(M_PI * 2);

            const ScalarType pi2_dt_cutoff = Pi2 * dt * f_cutoff_;
            const ScalarType alpha = pi2_dt_cutoff / (pi2_dt_cutoff + 1);

            y_ += alpha * (x - y_);
        }

        return y_;
    }

    const ValueType& get() const { return y_; }
};

}
