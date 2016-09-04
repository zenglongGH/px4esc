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

#include <zubax_chibios/util/heapless.hpp>
#include <math/math.hpp>
#include <cstdint>


namespace foc
{
/**
 * This structure entails all information about the connected load.
 * Some of these parameters can be automatically identified.
 */
struct MotorParameters
{
    math::Scalar min_current = 0;
    math::Scalar max_current = 0;

    math::Scalar field_flux = 0;        ///< Phi, Weber
    math::Scalar r_ab = 0;              ///< Phase-to-phase resistance, Ohm
    math::Scalar l_ab = 0;              ///< Phase-to-phase inductance, Henry

    std::uint_fast8_t num_poles = 0;

    bool isValid() const
    {
        return min_current      > 0 &&
               max_current      > 0 &&
               field_flux       > 0 &&
               r_ab             > 0 &&
               l_ab             > 0 &&
               num_poles       >= 2 &&
               (num_poles % 2 == 0);
    }

    auto toString() const
    {
        return os::heapless::format(
            "Imin=%.1fA, Imax=%.1fA, Phi=%.6fWb, Rab=%.6fOhm, Lab=%.6fH, Npoles=%u [%s]",
            double(min_current),
            double(max_current),
            double(field_flux),
            double(r_ab), double(l_ab),
            unsigned(num_poles),
            isValid() ? "VALID" : "INVALID");
    }
};

}
