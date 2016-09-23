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


namespace foc
{
/**
 * Various control modes.
 * See the function definitions below for usages.
 */
enum class ControlMode
{
    RatiometricCurrent,         ///< Value in [-1, 1] that defines the current setpoint relative to the maximum
    RatiometricMRPM,            ///< Value in [-1, 1] that defines the RPM setpoint relative to the maximum
    Current,                    ///< Ampere
    MRPM,                       ///< Mechanical RPM
};

/**
 * This class encapsulates the transfer function from the input setpoint value in different units
 * (where units are encoded using @ref ControlMode) to the Iq reference current setpoint.
 * At the time of writing this I am not yet sure about whether it will fit the design well, so we may need
 * to refactor it later.
 */
class CurrentSetpointController
{
    Const max_current_;
    Const min_current_;
    Const current_ramp_amp_s_;

public:
    CurrentSetpointController(Const max_current,
                              Const min_current,
                              Const current_ramp_amp_s) :
        max_current_(max_current),
        min_current_(min_current),
        current_ramp_amp_s_(current_ramp_amp_s)
    { }

    /**
     * Discrete transfer function from input setpoint to current setpoint.
     *
     * @param period                            Update interval in seconds
     * @param target_setpoint                   Target setpoint, units defined by @ref control_mode
     * @param control_mode                      The actual transfer function to use, this defines the units
     * @param reference_current                 Iq reference current
     * @param electrical_angular_velocity       Electrical angular velocity of the rotor in radian/second
     * @return                                  New Iq reference current
     */
    Scalar update(Const period,
                  Const target_setpoint,
                  const ControlMode control_mode,
                  Const reference_current,
                  Const electrical_angular_velocity)
    {
        switch (control_mode)
        {
        case ControlMode::RatiometricCurrent:
        case ControlMode::Current:
        {
            // Computing the new setpoint
            Scalar new_current = target_setpoint;
            if (control_mode == ControlMode::RatiometricCurrent)
            {
                new_current *= max_current_;
            }
            new_current = math::Range<>(-max_current_, max_current_).constrain(new_current);

            // Applying the ramp
            if (new_current > reference_current)
            {
                new_current = reference_current + current_ramp_amp_s_ * period;
            }
            else
            {
                new_current = reference_current - current_ramp_amp_s_ * period;
            }

            // Constraining the minimums, only if the new setpoint and the reference are of the same sign
            if ((new_current > 0) == (target_setpoint > 0))
            {
                new_current = std::copysign(std::max(min_current_, std::abs(new_current)),
                                             new_current);
            }

            return new_current;
        }

        case ControlMode::RatiometricMRPM:
        case ControlMode::MRPM:
        {
            (void) electrical_angular_velocity;
            assert(false);              // TODO: Implement
            return 0;
            break;
        }

        default:
        {
            assert(false);
            return 0;
            break;
        }
        }
    }
};

}
