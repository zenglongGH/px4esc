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


namespace board
{
namespace motor
{
namespace adc
{
namespace
{

constexpr unsigned PhaseACurrentChannelIndex    = 13;
constexpr unsigned PhaseBCurrentChannelIndex    = 12;
constexpr unsigned InverterVoltageChannelIndex  = 11;
constexpr unsigned TemperatureChannelIndex      = 10;

constexpr unsigned NumADC = 3;

constexpr unsigned SamplesPerADCPerIRQ = 2;

//std::uint16_t g_dma_buffer[SamplesPerADCPerIRQ * NumADC];

float g_voltage_gain;
float g_current_gain;           ///< Unset (zero) by default

std::function<float (float)> g_temperature_transfer_function = [](float) { return 0; };


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

void initTimers()
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
     */
}

}

void init(const float voltage_gain,
          const std::function<float (float)>& temperature_transfer_function)
{
    assert(VoltageGainRange.contains(voltage_gain));
    g_voltage_gain = VoltageGainRange.constrain(voltage_gain);

    g_temperature_transfer_function = temperature_transfer_function;
    assert(g_temperature_transfer_function);

    initADC();
    initTimers();
}

void setCurrentGain(const float current_gain)
{
    g_current_gain = current_gain;
}

float getTemperature()
{
    // TODO
    return g_temperature_transfer_function(0);
}

}
}
}

extern "C"
{

CH_FAST_IRQ_HANDLER(STM32_ADC_HANDLER)
{
    board::RAIIToggler<board::setTestPointA> tp_toggler;

    ADC1->SR = 0;         // Reset the IRQ flags
}

}
