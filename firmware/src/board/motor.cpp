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
#include <zubax_chibios/config/config.hpp>
#include <unistd.h>
#include <algorithm>
#include <functional>
#include <numeric>


namespace board
{
namespace motor
{
namespace
{
/*
 * Driver configuration
 */
constexpr unsigned SamplesPerADCPerIRQ = 2;

/**
 * If this constant is less than (g_fast_irq_to_main_irq_period_ratio * SamplesPerADCPerIRQ),
 * we'll be overwriting some values in the buffer before having a chance to use them.
 * If this constant is greater than (g_fast_irq_to_main_irq_period_ratio * SamplesPerADCPerIRQ),
 * we'll be using some samples from the previous period of the main IRQ.
 * Both of the above can be avoided by using the runtime computed value of (g_fast_irq_to_main_irq_period_ratio * 2)
 * instead of this constant; however, that increases complexity and harms performance. Neither of the above described
 * scenarios are errors, so I chose to use a fixed constant for simplicity and performance reasons.
 */
constexpr unsigned InverterVoltageSampleBufferLength = 6 * SamplesPerADCPerIRQ;

constexpr unsigned FastIRQPriority = 0;
constexpr unsigned MainIRQPriority = 1;

constexpr float MainIRQPreferredPeriod = 150e-6F;

constexpr float InverterVoltageInnovationWeight = 0.1F;         ///< Has to account for possible aliasing effect
constexpr float TemperatureInnovationWeight     = 0.001F;       ///< The input is noisy, high damping is necessary

constexpr float CalibrationDuration = 1.0F;

static_assert(FastIRQPriority < MainIRQPriority, "Fast IRQ must be able to preempt the main IRQ");
static_assert(MainIRQPriority < CORTEX_PRIORITY_SVCALL, "Main IRQ must be able to preempt the RTOS");

/*
 * Hardware-defined parameters
 */
constexpr unsigned TIM1ClockFrequency = STM32_TIMCLK2;

constexpr unsigned PhaseACurrentChannelIndex    = 13;
constexpr unsigned PhaseBCurrentChannelIndex    = 12;
constexpr unsigned InverterVoltageChannelIndex  = 11;
constexpr unsigned TemperatureChannelIndex      = 10;

constexpr float ADCReferenceVoltage = 3.3F;
constexpr unsigned ADCResolutionBits = 12;

/*
 * ADC DMA buffers. We're using the canaries to ensure that no data corruption is happening.
 */
constexpr std::uint32_t CanaryValue = 0xA55AA55A;

volatile std::uint32_t g_canary_a = CanaryValue;
std::uint16_t g_dma_buffer_inverter_voltage[InverterVoltageSampleBufferLength];
volatile std::uint32_t g_canary_b = CanaryValue;
std::uint16_t g_dma_buffer_phase_a_current[SamplesPerADCPerIRQ];
volatile std::uint32_t g_canary_c = CanaryValue;
std::uint16_t g_dma_buffer_phase_b_current[SamplesPerADCPerIRQ];
volatile std::uint32_t g_canary_d = CanaryValue;

/*
 * Configuration parameters
 */
os::config::Param<float> g_config_pwm_frequency_khz             ("drv.pwm_frq_khz",     60.0F,     50.0F,     80.0F);
os::config::Param<float> g_config_pwm_dead_time_nsec            ("drv.pwm_dead_ns",    400.0F,     50.0F,    900.0F);

/*
 * Current state variables
 */
float g_pwm_period;

float g_dead_time;

float g_main_irq_period;

unsigned g_fast_irq_to_main_irq_period_ratio;

/// Sometimes referred to as VBAT (ideally it should be volatile)
float g_inverter_voltage;

/// Raw output voltage of the temperature sensor (not converted to Kelvin) (ideally it should be volatile)
float g_inverter_temperature_sensor_voltage;

/// True ADC zero offset on the current channels.
math::Vector<2> g_phase_current_zero_offset = math::Vector<2>::Ones() * (ADCReferenceVoltage / 2.0F);

/// True if PWM outputs are active and the driver outputs are enabled
volatile bool g_is_active = false;

/**
 * This is used for current zero offset calibration.
 * It is instantiated from @ref beginCalibration(), which may be invoked from IRQ context,
 * and destroyed in the local IRQ handler.
 */
class Calibrator
{
    math::Vector<2> phase_current_adc_voltages_accumulator_ = {0.0F, 0.0F};
    unsigned num_samples_collected_ = 0;

    const unsigned num_samples_needed_;

    const bool original_driver_state_ = isActive();

public:
    Calibrator(unsigned num_samples_required) :
        num_samples_needed_(num_samples_required)
    {
        setActive(false);
        palWritePad(GPIOA, GPIOA_EN_GATE, true);
    }

    ~Calibrator()
    {
        setActive(original_driver_state_);
    }

    void addSample(const math::Vector<2>& phase_current_adc_voltages)
    {
        phase_current_adc_voltages_accumulator_ += phase_current_adc_voltages;
        num_samples_collected_++;
    }

    bool isComplete() const { return num_samples_collected_ >= num_samples_needed_; }

    math::Vector<2> getPhaseCurrentZeroOffset() const
    {
        if (num_samples_collected_ > 0)
        {
            return phase_current_adc_voltages_accumulator_ / float(num_samples_collected_);
        }
        else
        {
            return math::Vector<2>::Zero();
        }
    }
};

Calibrator* volatile g_calibrator = nullptr;

/**
 * This class holds parameters specific to the board we're running on.
 * At some point we'll need to add support for other hardware revisions with different voltage dividers,
 * different current shunt measurement circuits, etc. In that case this class will need to be modified accordingly.
 *
 * This class is not a good architectural solution, but we chose this way for performance reasons.
 *
 * TODO: REVISIT LATER, NEEDS REFACTORING.
 */
static class BoardFeatures final
{
    float inverter_voltage_gain_ = 0.0F;

    float current_shunt_resistance_ = 0.0F;
    float current_amplifier_gain_ = 0.0F;

    std::function<float (float)> temperature_transfer_function_ = [](float x) { return x; };

    /// Transfer function [Voltage] --> [Kelvin] for temperature sensors MCP9700/MCP9700A
    static float temperatureTransferFunctionMCP9700(float voltage)
    {
        return (100.0F * (voltage - 0.5F)) + 273.15F;
    }

    static constexpr float computeResistorDividerGain(float upper, float lower)
    {
        return lower / (lower + upper);
    }

public:
    void init()
    {
        const auto hwver = board::detectHardwareVersion();

        if (hwver.major == 1 && hwver.minor == 0)
        {
            os::lowsyslog("Motor HW Driver: Detected Pixhawk ESC v1.6 compatible board\n");

            inverter_voltage_gain_ = 1.0F / computeResistorDividerGain(5100 * 2, 330 * 2);
            current_shunt_resistance_ = 1 * 1e-3F;

            palWritePad(GPIOB, GPIOB_GAIN, true);
            current_amplifier_gain_ = 1.0F / (40.0F * current_shunt_resistance_);

            temperature_transfer_function_ = &BoardFeatures::temperatureTransferFunctionMCP9700;
        }
        else
        {
            assert(false);
            os::lowsyslog("Motor HW Driver: UNSUPPORTED HARDWARE VERSION %u.%u\n", hwver.major, hwver.minor);
        }
    }

    void adjustCurrentGain(const math::Vector<2>& v)
    {
        const float max_current = v.maxCoeff();
        (void)max_current;
        // TODO: Automatic Gain Control
    }

    float convertADCVoltageToInverterVoltage(float voltage) const
    {
        return voltage * inverter_voltage_gain_;
    }

    math::Vector<2> convertADCVoltagesToPhaseCurrents(const math::Vector<2>& voltages_without_zero_offset) const
    {
        return voltages_without_zero_offset * current_amplifier_gain_;
    }

    float convertADCVoltageToInverterTemperature(float voltage) const
    {
        return temperature_transfer_function_(voltage);
    }
} g_board_features;


void initPWM(const double pwm_frequency,
             const double pwm_dead_time)
{
    {
        AbsoluteCriticalSectionLocker locker;

        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
    }

    TIM1->CR1 = TIM_CR1_CMS_0;

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

    TIM1->CCR4 = 0;                                     // Always zero!

    // Configuring the carrier frequency
    const auto pwm_cycle_ticks = std::uint16_t((double(TIM1ClockFrequency) / pwm_frequency) / 2.0 + 0.5);
    assert(pwm_cycle_ticks > 100);                      // The lower limit is an arbitrarily selected lowest sane value

    TIM1->ARR = pwm_cycle_ticks - 1U;

    // Configuring dead time
    auto dead_time_ticks = std::uint16_t(pwm_dead_time * double(TIM1ClockFrequency) + 0.5);
    assert(dead_time_ticks < (pwm_cycle_ticks / 2));

    // DTS clock divider set 0, hence fDTS = input clock.
    // DTG bit 7 must be 0, otherwise it will change multiplier which is not supported yet.
    // At 180 MHz one tick ~5.(5) nsec, max 127 * 5.(5) = 705.(5) nsec, which is large enough.
    dead_time_ticks &= 0x7F;

    // Enabling break input; if the clock fails, PWM outputs will be disabled automatically
    TIM1->BDTR = TIM_BDTR_MOE | TIM_BDTR_BKP | TIM_BDTR_BKE | dead_time_ticks;

    // Launching the timer
    os::lowsyslog("Motor HW Driver: PWM cycle %u ticks, dead time %u ticks\n", pwm_cycle_ticks, dead_time_ticks);

    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->EGR = TIM_EGR_COMG | TIM_EGR_UG;

    // Freezing configuration
    TIM1->BDTR |= TIM_BDTR_LOCK_0 | TIM_BDTR_LOCK_1;
    // From now on we'll be changing ONLY CCR[1-3] REGISTERS

    assert((TIM1->SR & TIM_SR_BIF) == 0);       // Making sure there was no break
}


void initSynchronizationTimer()
{
    {
        AbsoluteCriticalSectionLocker locker;

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
    TIM8->CR1 = TIM_CR1_CMS_0;

    TIM8->CR2 = TIM_CR2_CCUS | TIM_CR2_CCPC;

    // Channel 1 triggers ADC conversions.
    // Channel 2 triggers the Fast IRQ.
    // Mode is exactly the same as that of TIM1, except that it's inverted - we need positive-going pulse
    // at the middle of the PWM period to trigger the ADC; without inversion the pulse would be negative-going.
    TIM8->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

    // The output MUST BE enabled in order for synchronization to work!
    TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;

    // Same ARR
    TIM8->ARR = TIM1->ARR;
    assert((TIM8->ARR > 0) && (TIM8->ARR < 0xFFFF));    // Making sure it's initialized indeed

    // Configuring the ADC trigger point - at the middle of the PWM period, when all phases are at the LOW level
    TIM8->CCR1 = TIM8->ARR - 1U;
    assert(TIM8->CCR1 < TIM8->ARR);

    // The Fast IRQ should be triggered at the point that matches the following requirements:
    //
    //  - IRQ handler should be finished well before the next PWM update event (this ensures minimal latency of
    //    PWM update). It should be also guaranteed that the PWM update will not happen between writes to CCR1..3,
    //    because that would skew the PWM waveforms.
    //
    //  - It must be executed well before or after the ADC IRQ is triggered (explained in the comments in the handler).
    //
    // Since we're in CMS mode 1, it is easier to generate IRQ on the up counting stage.
    //   /\    /
    //  /  \  /
    // /    \/
    // ^ ^  ^  ^
    // | |  PWM reload points (counter overflow and underflow)
    // | ADC trigger point
    // Fast IRQ trigger point
    TIM8->CCR2 = 1U;
    assert(TIM8->CCR2 < TIM8->ARR);

    // Configuring IRQ sources
    TIM8->DIER = TIM_DIER_CC2IE;

    // We don't care about dead time or breaks here
    TIM8->BDTR = TIM_BDTR_MOE;

    // Configuring slave mode - this is where synchronization is defined
    // First we're starting the timer synchronously with TIM1 by entering the Trigger Mode here:
    TIM8->SMCR = TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;

    // Now, waiting for the timer to start - it will happen when TIM1 reaches zero:
    DEBUG_LOG("Waiting for timer sync...\n");
    while ((TIM8->CNT == 0) || ((TIM8->CR1 & TIM_CR1_CEN) == 0))
    {
        ::usleep(1000);
    }
    DEBUG_LOG("Timers synced\n");

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

    // Configuring IRQ
    {
        AbsoluteCriticalSectionLocker locker;
        nvicClearPending(TIM8_CC_IRQn);
        nvicEnableVector(TIM8_CC_IRQn, FastIRQPriority);
    }
}


void initADC()
{
    {
        AbsoluteCriticalSectionLocker locker;

        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;
        RCC->APB2RSTR |=  RCC_APB2RSTR_ADCRST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;
    }

    /*
     * Enabling all three ADC in independent mode with DMA.
     * We could use multi-ADC simulatenous mode as well, but it brings no benefit and only increases complexity.
     * Besides, the documentation provided for multi-ADC mode is scarce at best.
     *
     * ADCs are running independently, but they all are triggered by the same source and perform identical
     * number of conversions, so despite the independent mode they still operate quasi synchronously. This
     * allows us to use only one IRQ - that of ADC1 - to handle all conversions.
     *
     * If there is need, it is also possible to configure different sampling modes per ADC - the advantage of
     * independent mode.
     */
    ADC->CCR = ADC_CCR_ADCPRE_0;                        // Prescaler

    ADC1->CR1 = ADC_CR1_SCAN |  ADC_CR1_EOCIE;          // Only ADC1 can generate interrupts
    ADC2->CR1 = ADC_CR1_SCAN;
    ADC3->CR1 = ADC_CR1_SCAN;

    // ADC triggering: RISING EDGE on TIM8 CC1
    constexpr unsigned ExtSel = 0b1101;
    constexpr unsigned CR2 = ADC_CR2_EXTEN_0 | (ExtSel << 24) | ADC_CR2_DDS | ADC_CR2_DMA;
    ADC1->CR2 = CR2;
    ADC2->CR2 = CR2;
    ADC3->CR2 = CR2;

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
     * hardware. Increasing the number of ticks per sample will break other things such as strict timing
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
     *  ADC     Regular Mode            Injected Mode
     *  -------------------------------------------------------------
     *  ADC1    inverter voltage        inverter temperature
     *  ADC2    phase current A         nothing
     *  ADC3    phase current B         nothing
     */
    static_assert(SamplesPerADCPerIRQ <= 6, "SQR2 and SQR1 initialization must be updated");

    constexpr unsigned SQR1 = (SamplesPerADCPerIRQ - 1) << 20;          // Sequence length is identical for all ADC
    ADC1->SQR1 = SQR1;
    ADC2->SQR1 = SQR1;
    ADC3->SQR1 = SQR1;

    static constexpr auto fill_sqr = [](unsigned ch)
        {
            return (ch << 0) | (ch << 5) | (ch << 10) | (ch << 15) | (ch << 20) | (ch << 25);
        };

    ADC1->SQR3 = fill_sqr(InverterVoltageChannelIndex);
    ADC2->SQR3 = fill_sqr(PhaseACurrentChannelIndex);           // ADC2 phase A
    ADC3->SQR3 = fill_sqr(PhaseBCurrentChannelIndex);           // ADC3 phase B

    // Configuring injected channels
    ADC1->JSQR = TemperatureChannelIndex << 15;                 // Temperature
    ADC1->CR1 |= ADC_CR1_JAUTO;                                 // Perform automatic injected conversions after regular

    // Configuring IRQ
    {
        AbsoluteCriticalSectionLocker locker;
        nvicClearPending(ADC_IRQn);
        nvicEnableVector(ADC_IRQn, MainIRQPriority);
        // Once the IRQ is configured, disable it back, because not everything is ready to handle the IRQ yet.
        // The IRQ will be normally enabled from the Fast IRQ handler.
        nvicDisableVector(ADC_IRQn);
    }

    /*
     * Configuring DMA - three channels.
     * Refer to 9.3.17 - Stream configuration procedure.
     */
    {
        AbsoluteCriticalSectionLocker locker;
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // No reset because it could be shared with other peripherals.
    }

    static const auto configure_dma = [](DMA_Stream_TypeDef* const stream,
                                         volatile void* const source,
                                         void* const destination,
                                         const unsigned buffer_length,
                                         const unsigned channel)
        {
            // Resetting as per 9.3.17
            stream->CR = 0;
            DEBUG_LOG("Resetting DMA stream @%08x\n", unsigned(stream));
            while (stream->CR != 0)
            {
                ::usleep(1000);
            }

            stream->PAR = reinterpret_cast<std::uint32_t>(source);
            stream->M0AR = reinterpret_cast<std::uint32_t>(destination);

            stream->NDTR = buffer_length;
            stream->FCR = 0;

            stream->CR = (channel << 25) |
                         DMA_SxCR_PL_0 | DMA_SxCR_PL_1 |        // Maximum priority
                         DMA_SxCR_MSIZE_0 |                     // 16-bit memory
                         DMA_SxCR_PSIZE_0 |                     // 16-bit peripheral
                         DMA_SxCR_MINC |                        // Memory increment enabled
                         DMA_SxCR_CIRC |                        // Circular mode
                         DMA_SxCR_EN;
        };

    // DMA2 Stream 0 - ADC1
    configure_dma(DMA2_Stream0,
                  &ADC1->DR,
                  &g_dma_buffer_inverter_voltage[0],
                  InverterVoltageSampleBufferLength,
                  0);

    // DMA2 Stream 2 - ADC2
    configure_dma(DMA2_Stream2,
                  &ADC2->DR,
                  &g_dma_buffer_phase_a_current[0],
                  SamplesPerADCPerIRQ,
                  1);

    // DMA2 Stream 1 - ADC3
    configure_dma(DMA2_Stream1,
                  &ADC3->DR,
                  &g_dma_buffer_phase_b_current[0],
                  SamplesPerADCPerIRQ,
                  2);

    // Everything is configured, enabling ADC
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC2->CR2 |= ADC_CR2_ADON;
    ADC3->CR2 |= ADC_CR2_ADON;
}


inline void setRawPWM(const std::uint16_t a, const std::uint16_t b, const std::uint16_t c)
{
    assert((a <= TIM1->ARR) &&
           (b <= TIM1->ARR) &&
           (c <= TIM1->ARR));
    /*
     * If CNT reaches zero between writes to CCR1 and CCR3, PWM will break, because the PWM signals will not
     * be in agreement with each other (CCR1 and possibly CCR2 will be using the new values, CCR3 and possibly
     * CCR2 will keep old values until the next update event).
     * Therefore we need to ensure that this function is NOT invoked when CNT is close to zero. Luckily, during
     * normal operation this requirement should be met automatically, because this function will be invoked
     * (very indirectly) from the ADC interrupt handler, which in turn is synchronized with timer update event.
     */
    TIM1->CCR1 = a;
    TIM1->CCR2 = b;
    TIM1->CCR3 = c;
}


template <unsigned NumSamples>
inline float convertADCSamplesToVoltage(const std::uint16_t (&x)[NumSamples])
{
    constexpr double VoltsPerLSB = double(ADCReferenceVoltage) / double((1 << ADCResolutionBits) - 1);

    constexpr float ConversionMultiplier = float(VoltsPerLSB / double(NumSamples));

    return float(std::accumulate(std::begin(x), std::end(x), 0U)) * ConversionMultiplier;
}


inline void checkInvariants()
{
    assert((ADC->CSR & (ADC_CSR_DOVR1 | ADC_CSR_DOVR2 | ADC_CSR_DOVR3)) == 0);                  // ADC overrun

    constexpr unsigned DMAErrorMask = DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0 |
                                      DMA_LISR_TEIF1 | DMA_LISR_DMEIF1 | DMA_LISR_FEIF1 |
                                      DMA_LISR_TEIF2 | DMA_LISR_DMEIF2 | DMA_LISR_FEIF2;
    (void) DMAErrorMask;
    assert((DMA2->LISR & DMAErrorMask) == 0);                                                   // DMA errors

    (void) g_canary_a;
    (void) g_canary_b;
    (void) g_canary_c;
    (void) g_canary_d;                                                                           // Bad DMA writes
    assert((g_canary_a == CanaryValue) &&
           (g_canary_b == CanaryValue) &&
           (g_canary_c == CanaryValue) &&
           (g_canary_d == CanaryValue));
}

} // namespace


void init()
{
    /*
     * Initializing GPIO
     */
    // Disable over current protection by default
    palWritePad(GPIOA, GPIOA_OC_ADJ, true);

    // Disable DC offset calibration mode (it's broken anyway and should never be activated)
    // (it's a hardware bug in the driver IC)
    palWritePad(GPIOA, GPIOA_DC_CAL, false);

    // Disable the driver outputs by default (they should be disabled anyway though...)
    palWritePad(GPIOA, GPIOA_EN_GATE, false);

    // TODO: Set up an interrupt to trigger when PWRGD goes down. Call an external handler on it, or just halt the OS.

    g_board_features.init();

    /*
     * Initializing the MCU peripherals.
     * The variables must be initialized BEFORE the first IRQ is triggered.
     */
    initPWM(double(g_config_pwm_frequency_khz.get()) * 1e3,
            double(g_config_pwm_dead_time_nsec.get()) * 1e-9);

    g_pwm_period = float(double((TIM1->ARR + 1U) * 2U) / double(TIM1ClockFrequency));

    g_dead_time = float(double(TIM1->BDTR & 0xFFU) / double(TIM1ClockFrequency));

    g_fast_irq_to_main_irq_period_ratio = unsigned(MainIRQPreferredPeriod / g_pwm_period + 0.5F);

    g_main_irq_period = float(double(g_pwm_period) * double(g_fast_irq_to_main_irq_period_ratio));

    initADC();

    initSynchronizationTimer();

    os::lowsyslog("Motor HW Driver: Fast IRQ period: %.1f us, Main IRQ period: %.1f us, ratio %u\n",
                  double(g_pwm_period) * 1e6,
                  double(g_main_irq_period) * 1e6,
                  g_fast_irq_to_main_irq_period_ratio);
}

void setActive(bool active)
{
    AbsoluteCriticalSectionLocker locker;

    setRawPWM(0, 0, 0);

    palWritePad(GPIOA, GPIOA_EN_GATE, active);

    setRawPWM(0, 0, 0);

    g_is_active = active;
}

bool isActive()
{
    return g_is_active;
}

void beginCalibration()
{
    /*
     * There is no race condition here. The pointer to the calibrator instance can be only cleared to nullptr
     * from the IRQ, so if it was already zeroed at the moment of this check, it is guaranteed to stay this way
     * until we assigned it again later in this function.
     */
    if (g_calibrator != nullptr)
    {
        return;         // Already in progress, or just finished - exiting
    }

    __DMB();            // Preventing memory access reordering relative to the previous check
    __DSB();

    const unsigned num_samples_needed = std::max(1000U, unsigned(CalibrationDuration / g_main_irq_period + 0.5F));

    alignas(16) static std::uint8_t storage[sizeof(Calibrator)];

    g_calibrator = new (&storage[0]) Calibrator(num_samples_needed);
}

bool isCalibrationInProgress()
{
    return g_calibrator != nullptr;
}

float getPWMPeriod()
{
    return g_pwm_period;
}

float getPWMDeadTime()
{
    return g_dead_time;
}

float getInverterVoltage()
{
    return g_inverter_voltage;          // Atomic read, no need to lock
}

void setPWM(const math::Vector<3>& abc)
{
    assert(g_is_active);
    assert((abc.array() >= 0).all() && (abc.array() <= 1).all());

    const auto arr = float(TIM1->ARR);

    setRawPWM(std::uint16_t(abc[0] * arr + 0.4F),
              std::uint16_t(abc[1] * arr + 0.4F),
              std::uint16_t(abc[2] * arr + 0.4F));
}

void emergency()
{
    AbsoluteCriticalSectionLocker mighty_locker;

    // Generating software break, this will reset the PWM outputs to zero immediately
    TIM1->EGR = TIM_EGR_BG;

    // This completely wreaks the driver, further use will be impossible until it's reinitialized again
    TIM1->CR1 = 0;
    TIM1->CR2 = 0;

    TIM8->CR1 = 0;
    TIM8->CR2 = 0;
}

Status getStatus()
{
    Status s;

    s.inverter_temperature =
        g_board_features.convertADCVoltageToInverterTemperature(g_inverter_temperature_sensor_voltage);
    s.inverter_voltage = g_inverter_voltage;

    s.phase_current_zero_offset = g_phase_current_zero_offset;

    s.power_ok  =  palReadPad(GPIOC, GPIOC_POWER_GOOD);
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

#ifndef NDEBUG
/// This is used to check DMA logic in debug builds.
/// Note that we're not checking the inverter voltage DMA channel, because it works asynchronously (see constants).
constexpr unsigned DMATransferCompleteMask = DMA_LISR_TCIF1 | DMA_LISR_TCIF2;
#endif


/// MAIN IRQ (every N-th PWM period, ASAP after the ADC samples are ready)
CH_FAST_IRQ_HANDLER(STM32_ADC_HANDLER)
{
    using namespace board::motor;

    board::RAIIToggler<board::setTestPointA> tp_toggler;

    /*
     * By the time we get here, the DMA controller should have completed all transfers.
     * Making sure this assumption is true.
     */
    assert((DMA2->LISR & DMATransferCompleteMask) == DMATransferCompleteMask);

    /*
     * Processing the samples and invoking the application handler.
     * These tasks need to be completed ASAP in order to minimize latency.
     */
    const math::Vector<2> phase_currents_adc_voltages
    {
        convertADCSamplesToVoltage(g_dma_buffer_phase_a_current),
        convertADCSamplesToVoltage(g_dma_buffer_phase_b_current)
    };

    // While EN_GATE is low, the current amplifiers are shut down, so we're measuring garbage
    const auto phase_currents =
        g_is_active ?
        g_board_features.convertADCVoltagesToPhaseCurrents(phase_currents_adc_voltages - g_phase_current_zero_offset) :
        math::Vector<2>::Zero();

    {
        const float new_inverter_voltage = g_board_features.convertADCVoltageToInverterVoltage(
            convertADCSamplesToVoltage(g_dma_buffer_inverter_voltage));

        g_inverter_voltage += InverterVoltageInnovationWeight * (new_inverter_voltage - g_inverter_voltage);
    }

    handleMainIRQ(g_main_irq_period, phase_currents, g_inverter_voltage);

    /*
     * Performing less time-critical tasks after the application's handler has been executed.
     */
    g_board_features.adjustCurrentGain(phase_currents);

    if (!g_is_active)
    {
        if (g_calibrator != nullptr)
        {
            g_calibrator->addSample(phase_currents_adc_voltages);

            if (g_calibrator->isComplete())
            {
                g_phase_current_zero_offset = g_calibrator->getPhaseCurrentZeroOffset();

                g_calibrator->~Calibrator();
                g_calibrator = nullptr;
            }
        }
    }

    // Temperature processing (injected conversion should be finished by this time; even if not, it's also fine)
    {
        const std::uint16_t temperature_sample[1] = { std::uint16_t(ADC1->JDR1) };
        const float new_temperature = convertADCSamplesToVoltage(temperature_sample);
        g_inverter_temperature_sensor_voltage +=
            TemperatureInnovationWeight * (new_temperature - g_inverter_temperature_sensor_voltage);
    }

    ADC1->SR = 0;               // Reset the IRQ flags
    NVIC_DisableIRQ(ADC_IRQn);  // Disabling until the next Main IRQ

    checkInvariants();
}

/// FAST IRQ (every PWM period)
CH_FAST_IRQ_HANDLER(STM32_TIM8_CC_HANDLER)
{
    using namespace board::motor;

    board::RAIIToggler<board::setTestPointB> tp_toggler;

    /*
     * Triggering the main IRQ when necessary.
     *
     * WARNING: It is crucial that this block is executed either ALWAYS BEFORE or ALWAYS AFTER the
     * moment when the ADC IRQ is triggered. If it's alternating, we may get an off-by-one ratio error.
     * Consider this (not to scale) (F - fast IRQ, M - main IRQ):
     *
     *  F       F       F       F       F       F       F...
     *  MMMMMMMMMMMMMMMMMMMMMMMMMM              MMMMMMMMM...
     *
     *                                          ^ BUG HERE
     *
     * If the synchronization point is a bit off, the fast IRQ may end up undecidedly either before or after
     * the ADC IRQ is triggered, which will randomly add or subtract one fast period between main IRQs.
     * This is a correct solution:
     *
     *  F       F       F       F       F       F       F...
     *      MMMMMMMMMMMMMMMMMMMMMMMMMM              MMMMM...
     *
     *                                          ^^^^ Correct, no race condition here
     *
     * In this case, we have plenty of time around the fast IRQ to ensure that the ADC IRQ will not be triggered
     * while we're updating the main trigger counter.
     *
     * One may have an idea to implement an alternative solution in hopes to make things simpler.
     * The alternative solutions I could think of are the following:
     *
     *  - Trigger both Main and Fast IRQ using a hardware timer.
     *    This is not a solid idea, because we'll need to guarantee that the ADC IRQ will never try to access
     *    the DMA buffers while the conversions are in progress. Essentially, same issue as here.
     *    Besides, this will increase the latency of the ADC measurements.
     *
     *  - Instead of enabling the ADC IRQ, just trigger it manually via NVIC.
     *    This approach has the same drawbacks as the previous one.
     *
     * In order to further limit the chances of a race condition breaking things, we need to process the triggering
     * logic in the first order, before dispatching to the application handler.
     */
    static unsigned main_irq_trigger_counter = 0;
    main_irq_trigger_counter++;
    if (main_irq_trigger_counter >= g_fast_irq_to_main_irq_period_ratio)
    {
        main_irq_trigger_counter = 0;

        // When the next conversion is finished, the IRQ will be triggered, then its handler will disable it again.
        ADC1->SR = 0;
        NVIC_ClearPendingIRQ(ADC_IRQn);
        NVIC_EnableIRQ(ADC_IRQn);

        // Runtime checks. See the main handler for the counterparts.
#ifndef NDEBUG
        DMA2->LIFCR = DMATransferCompleteMask;  // Complete flags must be set by the time we get into the ADC handler
#endif
    }

    /*
     * Processing the application handler.
     */
    handleFastIRQ(g_pwm_period);

    TIM8->SR = 0;               // Reset IRQ flags
}

}
