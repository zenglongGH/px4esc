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


namespace board
{
namespace motor
{
namespace driver
{
/**
 * Initializes the driver IC GPIO interface and applies default configuration, which is as follows:
 * - Current limiting disabled
 * - DC_CAL disabled (it's broken anyway, so this function is not exposed via the module API)
 * - EN_GATE disabled (driver disabled)
 * - Current amplification set at x40
 */
void init();

/**
 * Writes the EN_GATE pin of the driver IC.
 */
void setGateDriverEnabled(bool enabled);

/**
 * @ref setCurrentAmplifierGain().
 */
enum class CurrentAmplifierGain
{
    X10,
    X40
};

/**
 * Writes the GAIN pin of the driver IC. See @ref CurrentAmplifierGain.
 */
void setCurrentAmplifierGain(CurrentAmplifierGain gain);

/**
 * @ref readFailureIndicators().
 */
struct FailureIndicators
{
    bool bad_power = false;     ///< PWRGD
    bool overload = false;      ///< OCTW
    bool fault = false;         ///< FAULT

    bool allGood() const
    {
        return !(bad_power || overload || fault);
    }

    auto toString() const
    {
        return os::heapless::concatenate<40>("BadPower=", bad_power,
                                             " Overload=", overload,
                                             " Fault=", fault);
    }
};

/**
 * Read failure information from the driver IC's pins.
 */
FailureIndicators readFailureIndicators();

}
}
}
