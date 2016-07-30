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
namespace pwm
{
namespace
{

constexpr float TIM1ClockFrequency = float(STM32_TIMCLK2);

constexpr unsigned IRQPriority = 1;     // We don't need the highest priority here

} // namespace

void init(float frequency, float dead_time)
{
    {
        os::CriticalSectionLocker locker;

        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
    }

    TIM1->CR1 = TIM_CR1_CMS_0 | TIM_CR1_CMS_0;

    TIM1->CR2 = TIM_CR2_MMS_0 | TIM_CR2_MMS_1 | TIM_CR2_MMS_2 |
                TIM_CR2_CCDS | TIM_CR2_CCUS | TIM_CR2_CCPC;

    // Channels 1, 2, 3 are used for PWM phases A, B, C, respectively
    // Channel 4 is used to trigger DMA events
    TIM1->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

    TIM1->CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 |
                  TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1;

    TIM1->CCER = TIM_CCER_CC4E;

    // Configuring the carrier frequency
    frequency = FrequencyRange.constrain(frequency);
    const auto pwm_cycle_ticks = std::uint16_t((TIM1ClockFrequency / frequency) / 2.0F + 0.5F);
    assert(pwm_cycle_ticks > 100);                      // The lower limit is an arbitrarily selected lowest sane value

    TIM1->ARR = pwm_cycle_ticks - 1U;

    // Configuring dead time
    dead_time = DeadTimeRange.constrain(dead_time);
    auto dead_time_ticks = std::uint16_t(dead_time * TIM1ClockFrequency + 0.5F);
    assert(dead_time_ticks < (pwm_cycle_ticks / 2));

    // DTS clock divider set 0, hence fDTS = input clock.
    // DTG bit 7 must be 0, otherwise it will change multiplier which is not supported yet.
    // At 180 MHz one tick ~5.(5) nsec, max 127 * 5.(5) = 705.(5) nsec, which is large enough.
    dead_time_ticks &= 0x7F;

    TIM1->BDTR = TIM_BDTR_AOE | TIM_BDTR_MOE | dead_time_ticks;

    // Configuring interrupts
    TIM1->DIER = TIM_DIER_CC4IE;

    NVIC_SetPriority(TIM1_CC_IRQn, IRQPriority);
    NVIC_EnableIRQ(TIM1_CC_IRQn);

    // Launching the timer
    os::lowsyslog("PWM: Frequency %.6f kHz, %u ticks; Dead Time %.1f ns, %u ticks\n",
                  double(getFrequency() * 1e-3F), pwm_cycle_ticks, double(getDeadTime() * 1e9F), dead_time_ticks);

    TIM1->CR1 |= TIM_CR1_CEN;
}

float getFrequency()
{
    return TIM1ClockFrequency / float((TIM1->ARR + 1U) * 2U);
}

float getDeadTime()
{
    auto dtg = std::uint8_t(TIM1->BDTR & 0xFFU);
    assert(dtg <= 127);         // Other modes are not supported yet, see initialization for details
    return float(dtg) / TIM1ClockFrequency;
}

void activate()
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE |
                  TIM_CCER_CC2E | TIM_CCER_CC2NE |
                  TIM_CCER_CC3E | TIM_CCER_CC3NE;

    TIM1->EGR = TIM_EGR_COMG | TIM_EGR_UG;
}

void deactivate()
{
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    TIM1->EGR = TIM_EGR_COMG | TIM_EGR_UG;
}

void set(math::Vector<3> abc)
{
    assert((abc.array() >= 0).all() && (abc.array() <= 1).all());

    const auto arr = float(TIM1->ARR);

    TIM1->CCR1 = std::uint16_t(abc[0] * arr + 0.4F);
    TIM1->CCR2 = std::uint16_t(abc[1] * arr + 0.4F);
    TIM1->CCR3 = std::uint16_t(abc[2] * arr + 0.4F);

    assert(TIM1->CCR1 <= TIM1->ARR);
    assert(TIM1->CCR2 <= TIM1->ARR);
    assert(TIM1->CCR3 <= TIM1->ARR);
}

void emergency()
{
    // This completely wreaks the driver, further use will be impossible until it's reinitialized again
    TIM1->CCER = 0;

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    TIM1->EGR = TIM_EGR_COMG | TIM_EGR_UG;

    TIM1->CR1 = 0;
}

}
}
}
