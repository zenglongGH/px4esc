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

#include <board/board.hpp>
#include <unistd.h>


namespace board
{
namespace motor
{
namespace
{

constexpr unsigned TIM1ClockFrequency = STM32_TIMCLK2;

constexpr unsigned PhaseACurrentChannelIndex    = 13;
constexpr unsigned PhaseBCurrentChannelIndex    = 12;
constexpr unsigned InverterVoltageChannelIndex  = 11;
constexpr unsigned TemperatureChannelIndex      = 10;

constexpr unsigned NumADC = 3;

constexpr unsigned SamplesPerADCPerIRQ = 2;

//std::uint16_t g_dma_buffer[SamplesPerADCPerIRQ * NumADC];

float g_current_gain;           ///< Unset (zero) by default


void initPWM(float pwm_frequency, float pwm_dead_time)
{
    {
        os::CriticalSectionLocker locker;

        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
    }

    TIM1->CR1 = TIM_CR1_CMS_0 | TIM_CR1_CMS_0;

    // MMS - output event on CCR4 match
    TIM1->CR2 = TIM_CR2_MMS_2 | TIM_CR2_MMS_1 | TIM_CR2_MMS_0 | TIM_CR2_CCUS | TIM_CR2_CCPC;

    // Channels 1, 2, 3 are used for PWM phases A, B, C, respectively
    // Channel 4 is used for synchronization with TIM8, so CCR4 must be always 0
    TIM1->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

    TIM1->CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 |
                  TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1;        // CC4 toggle mode!

    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                 TIM_CCER_CC2E | TIM_CCER_CC2NE |
                 TIM_CCER_CC3E | TIM_CCER_CC3NE |
                 TIM_CCER_CC4E;

    TIM1->CCR4 = 0;     // Always zero!

    // Configuring the carrier frequency
    assert(PWMFrequencyRange.contains(pwm_frequency));
    pwm_frequency = PWMFrequencyRange.constrain(pwm_frequency);
    const auto pwm_cycle_ticks = std::uint16_t((double(TIM1ClockFrequency) / double(pwm_frequency)) / 2.0 + 0.5);
    assert(pwm_cycle_ticks > 100);                      // The lower limit is an arbitrarily selected lowest sane value

    TIM1->ARR = pwm_cycle_ticks - 1U;

    // Configuring dead time
    assert(PWMDeadTimeRange.contains(pwm_dead_time));
    pwm_dead_time = PWMDeadTimeRange.constrain(pwm_dead_time);
    auto dead_time_ticks = std::uint16_t(double(pwm_dead_time) * double(TIM1ClockFrequency) + 0.5);
    assert(dead_time_ticks < (pwm_cycle_ticks / 2));

    // DTS clock divider set 0, hence fDTS = input clock.
    // DTG bit 7 must be 0, otherwise it will change multiplier which is not supported yet.
    // At 180 MHz one tick ~5.(5) nsec, max 127 * 5.(5) = 705.(5) nsec, which is large enough.
    dead_time_ticks &= 0x7F;

    // Enabling break input; if the clock fails, PWM outputs will be disabled automatically
    TIM1->BDTR = TIM_BDTR_MOE | TIM_BDTR_BKP | TIM_BDTR_BKE | dead_time_ticks;

    // Launching the timer
    os::lowsyslog("Motor HW Driver: PWM Frequency %.6f kHz, %u ticks; PWM Dead Time %.1f ns, %u ticks\n",
                  double(getPWMFrequency() * 1e-3F), pwm_cycle_ticks, double(getPWMDeadTime() * 1e9F), dead_time_ticks);

    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->EGR = TIM_EGR_COMG | TIM_EGR_UG;

    // Freezing configuration
    TIM1->BDTR |= TIM_BDTR_LOCK_0 | TIM_BDTR_LOCK_1;
    // From now on we'll be changing ONLY CCR[1-3] REGISTERS

    assert((TIM1->SR & TIM_SR_BIF) == 0);       // Making sure there was no break
}

void initPWMADCSync()
{
    {
        os::CriticalSectionLocker locker;

        RCC->APB2ENR  |=  RCC_APB2ENR_TIM8EN;
        RCC->APB2RSTR |=  RCC_APB2RSTR_TIM8RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM8RST;
    }

    /*
     * We use TIM8 to trigger measurements and TIM1 (PWM timer) to reset TIM8 every period, thus achieving
     * synchronization. We can't trigger the ADC directly from TIM1 because of the limitations of the ADC
     * external trigger logic.
     *
     * The measurements are performed always at the top of the PWM timer counter, when all phases are shorted to
     * the ground. This is the only point where we can sample the currents, that matches the following conditions:
     *  1. All phases are shorted; by Kirchhoff's rule, the sum of the phase currents will be zero.
     *  2. Phases which currents we measure are shorted to the ground, guaranteeing that the currents will flow
     *     through the current sensors.
     */
    TIM8->CR1 = TIM_CR1_CMS_0 | TIM_CR1_CMS_0;

    TIM8->CR2 = TIM_CR2_CCUS | TIM_CR2_CCPC;

    // Only channel 1 is used here - it triggers ADC conversions
    // Mode is exactly the same as that of TIM1, except that it's inverted - we need positive-going pulse
    // at the middle of the PWM period to trigger the ADC; without inversion the pulse would be negative-going.
    TIM8->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

    TIM8->CCER = TIM_CCER_CC1E;

    // Same ARR
    TIM8->ARR = TIM1->ARR;
    assert((TIM8->ARR > 0) && (TIM8->ARR < 0xFFFF));    // Making sure it's initialized indeed

    // Configuring the ADC trigger point - at the middle of the PWM period, when all phases are at the LOW level
    TIM8->CCR1 = TIM8->ARR - 1U;
    assert(TIM8->CCR1 < TIM8->ARR);

    // We don't care about dead time or breaks here
    TIM8->BDTR = TIM_BDTR_MOE;

    // Configuring slave mode - this is where synchronization is defined
    // First we're starting the timer synchronously with TIM1 by entering the Trigger Mode here:
    TIM8->SMCR = TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;

    // Now, waiting for the timer to start - it will happen when TIM1 reaches zero:
    os::lowsyslog("Motor HW Driver: Waiting for timer sync...\n");
    while ((TIM8->CNT == 0) || ((TIM8->CR1 & TIM_CR1_CEN) == 0))
    {
        ::usleep(1000);
    }

    // Now timers are running, setting the reset-on-trigger slave mode:
    TIM8->SMCR = TIM_SMCR_SMS_2;

    // Freezing configuration
    TIM8->BDTR |= TIM_BDTR_LOCK_0 | TIM_BDTR_LOCK_1;

#if 0
    // Using RPM output to test timer sycnhronization
    // CCR1 should be decreased because very short pulses are not visible
    // This thing should only be used for hardcore driver debugging sessions!
    TIM8->CCR1 = TIM8->ARR - 5U;
    palSetPadMode(GPIOC, GPIOC_RPM_PULSE_FEEDBACK, PAL_STM32_OTYPE_PUSHPULL | PAL_MODE_ALTERNATE(3));
#endif
}

void initADC()
{
    {
        os::CriticalSectionLocker locker;

        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;
        RCC->APB2RSTR |=  RCC_APB2RSTR_ADCRST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;
    }

    // Enabling all three ADC in simultaneous mode with DMA
    ADC->CCR = ADC_CCR_ADCPRE_0 | ADC_CCR_DMA_0 | 0b10001;

    ADC1->CR1 = ADC_CR1_SCAN |  ADC_CR1_EOCIE;
    ADC2->CR1 = ADC_CR1_SCAN;
    ADC3->CR1 = ADC2->CR1;

    constexpr unsigned ExtSel = 0b1101;
    ADC1->CR2 = ADC_CR2_EXTEN_0 | ADC_CR2_EXTEN_1 | (ExtSel << 24) | ADC_CR2_DDS | ADC_CR2_DMA;
    ADC2->CR2 = ADC1->CR2;
    ADC3->CR2 = ADC1->CR2;

    /*
     * We're using 3 ticks per sample for all channels.
     * The number of samples is actually dependent on a lot of things such as input impedance and capacity.
     * The original experimental firmware contained the following code to compute the number of samples needed:
     *
     *  const uint64_t adc_clk_hz = bsp_pclk2_hz / 4;
     *  const uint64_t adc_r_ohm = bsp_adc_r_internal_ohm + bsp_adc_r_external_ohm;
     *  const uint64_t adc_c_pf = bsp_adc_c_internal_pf + bsp_adc_c_external_pf;
     *  const uint64_t adc_logn_13bit = 9010913;
     *  adc_t_smpl_tcks =
     *          ((((adc_clk_hz * adc_logn_13bit) / 1000000) * adc_r_ohm * adc_c_pf) / 1000000000 + 500) / 1000;
     *
     * adc_t_smpl_tcks used to always evaluate to about 3 samples per tick, which is the minimum supported by the
     * hardware. Increasing the number of ticks per sample will break other things such as string timing
     * requirements etc, so we probably should consider the 3 ticks per sample a hard requirement of this design and
     * mandate that the hardware adheres to it. So, you see, it's not a to-do comment, rather it's just an explanation.
     */
    ADC1->SMPR1 = 0;
    ADC1->SMPR2 = 0;

    ADC2->SMPR1 = 0;
    ADC2->SMPR2 = 0;

    ADC3->SMPR1 = 0;
    ADC3->SMPR2 = 0;

    /*
     * Initializing the sequences. ADC assignment is as follows:
     *  ADC1 - inverter voltage
     *  ADC2 - phase current A
     *  ADC3 - phase current B
     *
     * TODO: Temperature is currently not yet being sampled. Probably it should be sampled as injected channel
     *       automatically immediately after the main channels (set the bit JAUTO in CR1).
     */
    static_assert(SamplesPerADCPerIRQ <= 6, "SQR2 and SQR1 initialization must be updated");

    ADC1->SQR1 = (SamplesPerADCPerIRQ - 1) << 20;       // Sequence length is identical for all ADC
    ADC2->SQR1 = ADC1->SQR1;
    ADC3->SQR1 = ADC1->SQR1;

    static constexpr auto fill_sqr = [](unsigned ch)
        {
            return (ch << 0) | (ch << 5) | (ch << 10) | (ch << 15) | (ch << 20) | (ch << 25);
        };

    ADC1->SQR3 = fill_sqr(InverterVoltageChannelIndex);
    ADC2->SQR3 = fill_sqr(PhaseACurrentChannelIndex);
    ADC3->SQR3 = fill_sqr(PhaseBCurrentChannelIndex);

    // TODO: Configure triggering
    // TODO: Configure DMA
}

float convertTemperatureSensorVoltageToKelvin(float voltage)
{
    return voltage;     // TODO
}

} // namespace

void init(const float pwm_frequency,
          const float pwm_dead_time)
{
    // Disable over current protection by default
    palWritePad(GPIOA, GPIOA_OC_ADJ, true);

    // Disable DC offset calibration mode (it's broken anyway and should never be activated)
    // (it's a hardware bug in the driver IC)
    palWritePad(GPIOA, GPIOA_DC_CAL, false);

    // Disable the driver outputs by default (they should be disabled anyway though...)
    palWritePad(GPIOA, GPIOA_EN_GATE, false);

    // Initializing other defaults that can be changed at run time
    setCurrentAmplifierGain(CurrentAmplifierGain::X40);

    // TODO: Set up an interrupt to trigger when PWRGD goes down. Call an external handler on it, or just halt the OS.

    initPWM(pwm_frequency, pwm_dead_time);

    initPWMADCSync();

    initADC();
}

void setActive(bool active)
{
    setPWM({0, 0, 0});

    palWritePad(GPIOA, GPIOA_EN_GATE, active);

    setPWM({0, 0, 0});
}

float getPWMFrequency()
{
    return float(TIM1ClockFrequency) / float((TIM1->ARR + 1U) * 2U);
}

float getPWMDeadTime()
{
    auto dtg = std::uint8_t(TIM1->BDTR & 0xFFU);
    assert(dtg <= 127);         // Other modes are not supported yet, see initialization for details
    return float(dtg) / float(TIM1ClockFrequency);
}

void setPWM(const math::Vector<3>& abc)
{
    assert((abc.array() >= 0).all() && (abc.array() <= 1).all());

    const auto arr = float(TIM1->ARR);

    const auto c1 = std::uint16_t(abc[0] * arr + 0.4F);
    const auto c2 = std::uint16_t(abc[1] * arr + 0.4F);
    const auto c3 = std::uint16_t(abc[2] * arr + 0.4F);

    assert((c1 <= arr) &&
           (c2 <= arr) &&
           (c3 <= arr));

    /*
     * If CNT reaches zero between writes to CCR1 and CCR3, PWM will break, because the PWM signals will not
     * be in agreement with each other (CCR1 and possibly CCR2 will be using the new values, CCR3 and possibly
     * CCR2 will keep old values until the next update event).
     * Therefore we need to ensure that this function is NOT invoked when CNT is close to zero. Luckily, during
     * normal operation this requirement should be met automatically, because this function will be invoked
     * (very indirectly) from the ADC interrupt handler, which in turn is synchronized with timer update event.
     */
    TIM1->CCR1 = c1;
    TIM1->CCR2 = c2;
    TIM1->CCR3 = c3;
}

void setCurrentAmplifierGain(CurrentAmplifierGain gain)
{
    switch (gain)
    {
    case CurrentAmplifierGain::X10:
    {
        palWritePad(GPIOB, GPIOB_GAIN, false);
        g_current_gain = 10.0F;                         // TODO: Set full gain value
        break;
    }
    case CurrentAmplifierGain::X40:
    {
        palWritePad(GPIOB, GPIOB_GAIN, true);
        g_current_gain = 40.0F;                         // TODO: Set full gain value
        break;
    }
    default:
    {
        assert(false);
    }
    }
}

void emergency()
{
    // Generating software break, this will reset the PWM outputs to zero immediately
    TIM1->EGR = TIM_EGR_BG;

    // This completely wreaks the driver, further use will be impossible until it's reinitialized again
    TIM1->CR1 = 0;
    TIM1->CR2 = 0;
}

Status getStatus()
{
    Status s;

    s.power_stage_temperature = convertTemperatureSensorVoltageToKelvin(0.0F);  // TODO Temperature

    s.bad_power = !palReadPad(GPIOC, GPIOC_POWER_GOOD);
    s.overload  = !palReadPad(GPIOC, GPIOC_OVER_TEMP_WARNING_INVERSE);
    s.fault     = !palReadPad(GPIOC, GPIOC_DRIVER_FAULT_INVERSE);

    return s;
}

}
}

/*
 * Interrupt handlers
 */
extern "C"
{

CH_FAST_IRQ_HANDLER(STM32_ADC_HANDLER)
{
    board::RAIIToggler<board::setTestPointA> tp_toggler;

    ADC1->SR = 0;         // Reset the IRQ flags
}

}
