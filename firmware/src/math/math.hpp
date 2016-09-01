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

template <int Rows>
using DiagonalMatrix = Eigen::DiagonalMatrix<Scalar, Rows>;

template <int Size>
using Vector = Matrix<Size, 1>;


constexpr inline Scalar convertKelvinToCelsius(Const kelvin)
{
    return kelvin - 273.15F;
}

/**
 * Constants
 */
constexpr auto Pi = Scalar(3.141592653589793);

/**
 * Substracts right angle from left angle, result will be in the range (-180, 180].
 * Apply std::abs() to the result to get the absolute angular difference.
 */
constexpr inline Scalar subtractAngles(Const left, Const right)
{
    Const diff = left - right;
    if (diff > Pi)
    {
        return diff - Pi * 2.0F;
    }
    if (diff < -Pi)
    {
        return diff + Pi * 2.0F;
    }
    return diff;
}

constexpr inline Scalar convertDegreesToRadian(Const deg)
{
    return deg / (180.0F / Pi);
}

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

/**
 * Implementation details, do not use directly.
 */
namespace impl_
{

template <typename Scalar, int Size>
using RowVector = Eigen::Matrix<Scalar, 1, Size, Eigen::RowMajor>;

template <typename Scalar, int Size, typename Head>
inline void fillVector(RowVector<Scalar, Size>& vector,
                       int next_index,
                       Head head)
{
    vector[next_index] = static_cast<Scalar>(head);
}

template <typename Scalar, int Size, typename Head, typename... Tail>
inline void fillVector(RowVector<Scalar, Size>& vector,
                       int next_index,
                       Head head,
                       Tail... tail)
{
    vector[next_index] = static_cast<Scalar>(head);
    fillVector(vector, next_index + 1, tail...);
}

template <typename Scalar, int Rows, int Columns>
inline void fillMatrix(Eigen::Matrix<Scalar, Rows, Columns>& matrix,
                       int next_row,
                       const impl_::RowVector<Scalar, Columns>& head)
{
    matrix.row(next_row) = head;
}

template <typename Scalar, int Rows, int Columns, typename... Tail>
inline void fillMatrix(Eigen::Matrix<Scalar, Rows, Columns>& matrix,
                       int next_row,
                       const impl_::RowVector<Scalar, Columns>& head,
                       Tail... tail)
{
    matrix.row(next_row) = head;
    fillMatrix(matrix, next_row + 1, tail...);
}

template <typename Scalar, int DiagonalSize>
inline void fillDiagonalMatrix(Eigen::DiagonalMatrix<Scalar, DiagonalSize>& matrix,
                               int next_position,
                               Scalar head)
{
    matrix.diagonal()[next_position] = static_cast<Scalar>(head);
}

template <typename Scalar, int DiagonalSize, typename... Tail>
inline void fillDiagonalMatrix(Eigen::DiagonalMatrix<Scalar, DiagonalSize>& matrix,
                               int next_position,
                               Scalar head,
                               Tail... tail)
{
    matrix.diagonal()[next_position] = static_cast<Scalar>(head);
    fillDiagonalMatrix<Scalar, DiagonalSize>(matrix, next_position + 1, tail...);
}

} // namespace impl_

/**
 * Creates a row vector of arbitrary length, represented as a static row vector Eigen::Matrix<>.
 * Scalar type can be overriden.
 * See also @ref makeMatrix().
 */
template <typename Scalar = Scalar, typename... Tail>
inline impl_::RowVector<Scalar, sizeof...(Tail)>
makeRow(Tail... tail)
{
    impl_::RowVector<Scalar, sizeof...(Tail)> vector;
    impl_::fillVector(vector, 0, tail...);
    return vector;
}

/**
 * Creates a static matrix (Eigen::Matrix<>) of arbitrary size from a set of row vectors.
 * The matrix will use default layout, which is columnn-major for Eigen.
 * Note that row vectors of unequal size trigger a compile-time error.
 * Scalar type will be deduced automatically from the first row.
 * This function is totally type safe.
 * Usage:
 *      makeMatrix(makeRow(1, 2),
 *                 makeRow(3, 4))  // Produces a 2x2 matrix
 */
template <typename Scalar, int Columns, typename... Tail>
inline Eigen::Matrix<Scalar, sizeof...(Tail) + 1, Columns>
makeMatrix(const impl_::RowVector<Scalar, Columns>& head,
           Tail... tail)
{
    Eigen::Matrix<Scalar, sizeof...(Tail) + 1, Columns> matrix;
    matrix.setZero();
    impl_::fillMatrix(matrix, 0, head, tail...);
    return matrix;
}

/**
 * A helper like @ref makeMatrix() that creates diagonal matrix.
 * Size is dereved from the argument list.
 * Scalar type can be overriden.
 */
template <typename Scalar = Scalar, typename... Diagonal>
inline Eigen::DiagonalMatrix<Scalar, sizeof...(Diagonal)>
makeDiagonalMatrix(Diagonal... diag)
{
    Eigen::DiagonalMatrix<Scalar, sizeof...(Diagonal)> matrix;
    matrix.setZero();
    impl_::fillDiagonalMatrix<Scalar, sizeof...(Diagonal)>(matrix, 0, diag...);
    return matrix;
}

}
