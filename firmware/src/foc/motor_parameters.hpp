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
    Scalar nominal_spinup_duration = 0; ///< Preferred duration of spinup, real duration may slightly differ, seconds
    Scalar min_electrical_ang_vel = 0;  ///< Min electric angular velocity for stable operation, radian/second

    Scalar min_current = 0;             ///< Min phase current for stable observer operation, Ampere
    Scalar max_current = 0;             ///< Max phase current, Ampere
    Scalar spinup_current = 0;          ///< Initial current applied at spinup, Ampere

    Scalar current_ramp_amp_per_s = 0;  ///< Current setpoint slope, Ampere/second.

    Scalar phi = 0;                     ///< Magnetic field flux linkage, Weber
    Scalar r_ab = 0;                    ///< Phase-to-phase resistance, Ohm
    Scalar l_ab = 0;                    ///< Phase-to-phase inductance, Henry

    std::uint_fast8_t num_poles = 0;    ///< Number of magnetic poles (not pairs!); must be a positive even number

    std::uint32_t num_stalls_to_latch = 0; ///< If the rotor stalled this many times in a row, latch into FAULT state


    static math::Range<> getPhiLimits()
    {
        return {   0.02e-3F,
                 100.00e-3F };
    }

    static math::Range<> getRabLimits()
    {
        return { 0.01F,
                 2.00F };
    }

    static math::Range<> getLabLimits()
    {
        return {    5e-6F,
                 1000e-6F };
    }


    void deduceMissingParameters()
    {
        if (!os::float_eq::positive(spinup_current) &&
            os::float_eq::positive(max_current))
        {
            spinup_current = max_current * 0.5F;
        }
    }

    bool isValid() const
    {
        static const auto is_positive = [](Const x) { return (x > 0) && std::isfinite(x); };

        return is_positive(nominal_spinup_duration)     &&
               is_positive(min_electrical_ang_vel)      &&
               is_positive(min_current)                 &&
               is_positive(max_current)                 &&
               is_positive(spinup_current)              &&
               is_positive(current_ramp_amp_per_s)      &&
               getPhiLimits().contains(phi)             &&
               getRabLimits().contains(r_ab)            &&
               getLabLimits().contains(l_ab)            &&
               (num_poles >= 2)                         &&
               (num_poles % 2 == 0)                     &&
               (num_stalls_to_latch >= 1)               &&
               (max_current > min_current)              &&
               (max_current >= spinup_current)          &&
               (spinup_current > min_current);
    }

    auto toString() const
    {
        const bool num_poles_known = (num_poles >= 2) && (num_poles % 2 == 0);

        Scalar kv = 0;
        if ((phi > 0) && num_poles_known)
        {
            kv = convertFluxLinkageToKV(phi, num_poles);
        }

        Scalar min_mrpm = 0;
        if ((min_electrical_ang_vel > 0) && num_poles_known)
        {
            min_mrpm = convertRotationRateElectricalToMechanical(convertAngularVelocityToRPM(min_electrical_ang_vel),
                                                                 num_poles);
        }

        return os::heapless::String<220>(
            "Tspup: %-7.1f s\n"
            "Wmin : %-7.1f Rad/s, %.1f MRPM\n"
            "Imin : %-7.1f A\n"
            "Imax : %-7.1f A\n"
            "Ispup: %-7.1f A\n"
            "Iramp: %-7.1f A/s\n"
            "Phi  : %-7.3f mWb\n"
            "Rab  : %-7.3f Ohm\n"
            "Lab  : %-7.3f uH\n"
            "Npols: %u, %.1f MRPM/V\n"
            "Nstlt: %u\n"
            "Valid: %s").format(
            double(nominal_spinup_duration),
            double(min_electrical_ang_vel),
            double(min_mrpm),
            double(min_current),
            double(max_current),
            double(spinup_current),
            double(current_ramp_amp_per_s),
            double(phi) * 1e3,
            double(r_ab),
            double(l_ab) * 1e6,
            unsigned(num_poles),
            double(kv),
            unsigned(num_stalls_to_latch),
            isValid() ? "YES" : "NO");
    }
};

}
