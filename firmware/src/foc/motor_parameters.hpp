/**
 * Copyright (c) 2016  Zubax Robotics OU  <info@zubax.com>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "common.hpp"
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
    Scalar min_current = 0;             ///< Min phase current for stable observer operation, Ampere
    Scalar max_current = 0;             ///< Max phase current, Ampere

    Scalar spinup_current_slope = 0;    ///< Current increase rate during spinup, Ampere/second

    Scalar min_electrical_ang_vel = 0;  ///< Min electric angular velocity for stable operation, radian/second

    Scalar field_flux = 0;              ///< Phi, Weber
    Scalar r_ab = 0;                    ///< Phase-to-phase resistance, Ohm
    Scalar l_ab = 0;                    ///< Phase-to-phase inductance, Henry

    std::uint_fast8_t num_poles = 0;

    bool isValid() const
    {
        static auto is_positive = [](Const x) { return (x > 0) && std::isfinite(x); };

        return is_positive(min_current)                 &&
               is_positive(max_current)                 &&
               is_positive(spinup_current_slope)        &&
               is_positive(min_electrical_ang_vel)      &&
               is_positive(field_flux)                  &&
               is_positive(r_ab)                        &&
               is_positive(l_ab)                        &&
               (num_poles >= 2)                         &&
               (num_poles % 2 == 0)                     &&
               (max_current > min_current);
    }

    auto toString() const
    {
        return os::heapless::format("Imin  : %-7.1f A\n"
                                    "Imax  : %-7.1f A\n"
                                    "SCS   : %-7.1f A/s\n"
                                    "Wmin  : %-7.1f Rad/s, %.1f Hz\n"
                                    "Phi   : %-7.3f mWb\n"
                                    "Rab   : %-7.3f Ohm\n"
                                    "Lab   : %-7.3f uH\n"
                                    "Npoles: %u\n"
                                    "Valid : %s\n",
                                    double(min_current),
                                    double(max_current),
                                    double(spinup_current_slope),
                                    double(min_electrical_ang_vel), double(min_electrical_ang_vel / (math::Pi * 2.0F)),
                                    double(field_flux) * 1e3,
                                    double(r_ab),
                                    double(l_ab) * 1e6,
                                    unsigned(num_poles),
                                    isValid() ? "YES" : "NO");
    }
};

}
