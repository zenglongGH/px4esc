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
#include <cmath>


namespace math
{

using Scalar = float;
using Const = const Scalar;

template <int Rows, int Cols>
using Matrix = Eigen::Matrix<Scalar, Rows, Cols>;

template <int Size>
using Vector = Matrix<Size, 1>;


constexpr inline float convertKelvinToCelsius(float kelvin)
{
    return kelvin - 273.15F;
}

/**
 * Constants
 */
constexpr auto Pi = Scalar(3.141592653589793);

/**
 * Inclusive range of the form [min, max].
 */
template <typename T = Scalar>
struct Range
{
    T min = T(0);
    T max = T(0);

    constexpr Range() { }

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
 * Wrappers over the CMSIS DSP library.
 * Note that the CMSIS math headers MUST NOT be included, because they dump a pile of garbage into the global scope.
 * @{
 */
namespace impl_
{
extern "C"
{

float arm_sin_f32(float);
float arm_cos_f32(float);

}
}

inline Scalar sin(Scalar x)
{
    return impl_::arm_sin_f32(x);
}

inline Scalar cos(Scalar x)
{
    return impl_::arm_cos_f32(x);
}
/**
 * @}
 */

}
