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
namespace pwm
{
/**
 * Limits imposed by the hardware.
 */
constexpr math::Range<> FrequencyRange(50000, 80000);
constexpr math::Range<> DeadTimeRange(0.0F, 700e-9F);

/**
 * @param frequency     Preferred PWM frequency in Hertz
 * @param dead_time     Preferred PWM dead time in seconds
 */
void init(float frequency, float dead_time);

/**
 * Meaningful results guaranteed only after initialization.
 * @return Carrier frequency in Hertz.
 */
float getFrequency();

/**
 * Meaningful results guaranteed only after initialization.
 * @return Dead time in seconds.
 */
float getDeadTime();

/**
 * Activates the outputs and initializes PWM channels to safe values.
 */
void activate();

/**
 * Deactivates the PWM outputs (shuts down the carrier).
 */
void deactivate();

/**
 * This function should only be called after @ref activate() and before @ref deactivate().
 * @param abc           PWM values per channel in the range [0, 1].
 */
void set(const math::Vector<3>& abc);

/**
 * Immediately deactivates the PWM outputs (shuts down the carrier).
 * Further use of the driver may not be possible.
 * This function can be called from ANY context, e.g. from Hard Fault handler.
 */
void emergency();

}
}
}
