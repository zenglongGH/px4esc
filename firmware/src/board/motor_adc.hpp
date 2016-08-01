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
#include <functional>


namespace board
{
namespace motor
{
namespace adc
{
/**
 * Limits imposed by the hardware.
 */
constexpr math::Range<> VoltageGainRange(1, 1000);

/**
 * Must be invoked first.
 *
 * All voltages and currents measured by this driver are assumed to be zero-offset linear functions of ADC input
 * voltages. In other words:
 *      Parameter = ADCVoltage * Gain
 *
 * The voltage gain is assumed to depend only on the hardware configuration and assumed to be fixed,
 * therefore it can be set only once during initialization.
 *
 * The current gain is assumed to be switchable at run time, so there is a dedicated function for its
 * configuration - @ref setTotalCurrentGain(). It must be invoked after initialization at least one in order
 * to supply the driver with a valid gain value - the default value is zero which renders current measurements
 * useless.
 *
 * Unlike voltage and current, outputs of analog temperature sensors are typically not zero-offset linear functions
 * of temperature, sometimes they are not linear functions at all. Therefore the driver acccepts a function that
 * performs the arbitrary conversion from ADC voltage to tempeature depending on the hardware being used.
 */
void init(const float voltage_gain,
          const std::function<float (float)>& temperature_transfer_function);

/**
 * Informs the driver about the total gain of the current measurement curcuits.
 * The actual hardware configuration is commanded elsewhere.
 */
void setCurrentGain(const float current_gain);

/**
 * Returns the current temperature in Kelvin (remember all units are SI units).
 */
float getTemperature();

/**
 * This function is invoked from the IRQ context when new data arrives. It must be defined elsewhere.
 * All units are SI units.
 */
extern void handleSampleIRQ(const math::Vector<2>& phase_currents_ab,
                            const float inverter_voltage);

}
}
}
