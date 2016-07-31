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

#include <cstdint>
#include <math.hpp>


namespace board
{
namespace motor
{
namespace adc
{
/**
 * Must be invoked first.
 *
 * @ref setTotalCurrentGain() must be invoked after initialization to provide a meaningful gain value.
 * Deafult gain is zero, which renders current measurements useless.
 */
void init();

/**
 * Informs the driver about the total gain of the current measurement curcuits.
 * The actual hardware configuration is commanded elsewhere.
 * The current computation is as follows:
 *      current = adv_voltage * G
 * The G value is set by this function, it is defined by the shunt resistance (if shunts are used) and the
 * amplifier's gain.
 */
void setTotalCurrentGain(float gain);

/**
 * This function is invoked from the IRQ context when new data arrives.
 * It must be defined elsewhere.
 * All units are SI units (Volt, Ampere, Kelvin).
 */
extern void handleSampleIRQ(const math::Vector<3>& phase_voltages_abc,
                            const math::Vector<2>& phase_currents_ab,
                            float inverter_voltage,
                            float inverter_temperature);

}
}
}
