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

#include <hal.h>
#include "board.hpp"


namespace board
{
namespace motor
{
namespace driver
{

void init()
{
    // Disable over current protection by default
    palWritePad(GPIOA, GPIOA_OC_ADJ, true);

    // Disable DC offset calibration mode (it's broken anyway and should never be activated)
    // (it's a hardware bug in the driver IC)
    palWritePad(GPIOA, GPIOA_DC_CAL, false);

    // Initializing other defaults that can be changed at run time
    setGateDriverEnabled(false);
    setCurrentAmplifierGain(CurrentAmplifierGain::X40);

    // TODO: Set up an interrupt to trigger when PWRGD goes down. Call an external handler on it, or just halt the OS.
}

void setGateDriverEnabled(bool enabled)
{
    palWritePad(GPIOA, GPIOA_EN_GATE, enabled);
}

void setCurrentAmplifierGain(CurrentAmplifierGain gain)
{
    palWritePad(GPIOB, GPIOB_GAIN, bool(gain));
}

FailureIndicators readFailureIndicators()
{
    FailureIndicators fi;

    fi.bad_power = !palReadPad(GPIOC, GPIOC_POWER_GOOD);
    fi.overload  = !palReadPad(GPIOC, GPIOC_OVER_TEMP_WARNING_INVERSE);
    fi.fault     = !palReadPad(GPIOC, GPIOC_DRIVER_FAULT_INVERSE);

    return fi;
}

}
}
}
