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

#include "motor.hpp"


namespace board
{
namespace motor
{
/**
 * This class holds parameters specific to the board we're running on.
 * At some point we'll need to add support for other hardware revisions with different voltage dividers,
 * different current shunt measurement circuits, etc. In that case this class will need to be modified accordingly.
 */
class BoardFeatures final
{
    static constexpr float ADCReferenceVoltage = 3.3F;
    static constexpr unsigned ADCResolutionBits = 12;

    static constexpr float CurrentOffsetCalibrationDuration = 1.0F;

    static constexpr float MaxUnipolarVoltageAtCurrentSensorOutput  = (ADCReferenceVoltage / 2.0F) * 0.9F;  ///< Volt
    static constexpr float MinCurrentGainSwitchInterval             = 0.01F;                                ///< Second
    static constexpr float CurrentGainAdjustmentHysteresisCoeff     = 0.9F;

    /**
     * If both current sensors output voltages lower than this, we assume that the current amplifiers are
     * not yet activated.
     * This heuristic helps to avoid unnecessary gain switching shortly after power stage activation.
     * The condition when both channels report that low voltages should never appear during normal operation.
     */
    static constexpr float MinCurrentSensorsOutputVoltage  = 0.008F;


    struct CurrentZeroOffsetCalibrator
    {
        PWMHandle pwm_handle;
        float duration = 0;
        math::CumulativeAverageComputer<math::Vector<2>> averager;
        bool in_progress = false;

        void reset()
        {
            pwm_handle.release();
            in_progress = false;
            resetDurationAndAverage();
        }

        void resetDurationAndAverage()
        {
            duration = 0;
            averager = math::CumulativeAverageComputer<math::Vector<2>>(math::Vector<2>::Zero());
        }

        CurrentZeroOffsetCalibrator() { reset(); }
    };

    struct BoardConfig
    {
        const char* name = "<UNKNOWN>";

        float inverter_voltage_gain = 0.0F;

        float current_shunt_resistance = 0.0F;
        std::array<float, 2> current_amplifier_low_high_gains{};

        std::function<float (float)> temperature_transfer_function = [](float) { return 0; };

        Limits limits;

        struct DefaultSettings
        {
            float pwm_frequency = 0;
            float pwm_dead_time = 0;
        } default_settings;
    };

    // Board-dependent constants
    const BoardConfig board_config_;

    // State variables
    std::array<math::Vector<2>, 2> current_zero_offsets_low_high_{};             ///< Per gain level
    bool current_amplifier_high_gain_selected_ = true;
    float time_since_current_was_above_high_gain_threshold_ = 0.0F;

    CurrentZeroOffsetCalibrator current_zero_offset_calibrator_;


    const math::Vector<2>& getCurrentZeroOffsets() const
    {
        return current_zero_offsets_low_high_[int(current_amplifier_high_gain_selected_)];
    }

    void setCurrentAmplifierGain(bool high)
    {
        palWritePad(GPIOB, GPIOB_GAIN, high);
        current_amplifier_high_gain_selected_ = high;
    }


    static BoardConfig detectBoardConfig()
    {
        static const auto compute_resistor_divider_gain = [](float up, float low) { return (low + up) / low; };

        /// Transfer function [Voltage] --> [Kelvin] for temperature sensors MCP9700/MCP9700A
        static const auto temp_tf_MCP9700 = [](float v) { return (100.0F * (v - 0.5F)) + 273.15F; };

        const auto hwver = board::detectHardwareVersion();
        DEBUG_LOG("HW version %s\n", hwver.toString().c_str());

        if (hwver.major == 1 &&
            hwver.minor == 0)
        {
            Limits lim;
            lim.measurement_range.inverter_temperature = { math::convertCelsiusToKelvin(-40.0F),
                                                           math::convertCelsiusToKelvin(125.0F)};
            lim.measurement_range.inverter_voltage = { 5.0F, 54.3F };
            lim.safe_operating_area = lim.measurement_range;
            lim.safe_operating_area.inverter_temperature.max = math::convertCelsiusToKelvin(85.0F);
            lim.safe_operating_area.inverter_voltage = { 8.0F, 51.0F };

            return {
                "Pixhawk ESC v1.6",
                compute_resistor_divider_gain(5100 * 2, 330 * 2),
                1 * 1e-3F,
                { 10.0F, 40.0F },
                temp_tf_MCP9700,
                lim,
                {
                    50e3F,
                    200e-9F
                }
            };
        }

        chSysHalt("UNSUPPORTED HARDWARE");
        return BoardConfig();
    }

public:
    BoardFeatures() :
        board_config_(detectBoardConfig())
    {
        std::fill(current_zero_offsets_low_high_.begin(), current_zero_offsets_low_high_.end(),
                  math::Vector<2>::Ones() * (ADCReferenceVoltage * 0.5F));

        palWritePad(GPIOB, GPIOB_GAIN, true);
        current_amplifier_high_gain_selected_ = true;
    }

    const char* getBoardName() const { return board_config_.name; }

    const Limits& getLimits() const { return board_config_.limits; }

    const BoardConfig::DefaultSettings& getDefaultSettings() const { return board_config_.default_settings; }

    float convertADCVoltageToInverterVoltage(float voltage) const
    {
        return voltage * board_config_.inverter_voltage_gain;
    }

    void adjustCurrentGain(const float period,
                           const math::Vector<2>& currents)
    {
        assert(!isCalibrationInProgress());

        const float upper_threshold = MaxUnipolarVoltageAtCurrentSensorOutput /
                                      board_config_.current_amplifier_low_high_gains[1] /
                                      board_config_.current_shunt_resistance;

        const float lower_threshold = upper_threshold * CurrentGainAdjustmentHysteresisCoeff;

        const float peak = currents.lpNorm<Eigen::Infinity>();

        if (peak > (current_amplifier_high_gain_selected_ ? upper_threshold : lower_threshold))
        {
            // Current above threshold, selecting low gain
            setCurrentAmplifierGain(false);

            time_since_current_was_above_high_gain_threshold_ = 0.0F;
        }
        else
        {
            // Current below threshold, checking if we're allowed to switch
            if (time_since_current_was_above_high_gain_threshold_ > MinCurrentGainSwitchInterval)
            {
                setCurrentAmplifierGain(true);
            }
            else
            {
                time_since_current_was_above_high_gain_threshold_ += period;
            }
        }
    }

    float getCurrentGain() const
    {
        return board_config_.current_amplifier_low_high_gains[int(current_amplifier_high_gain_selected_)];
    }

    auto getCurrentSensorsZeroOffsets() const { return current_zero_offsets_low_high_; }

    math::Vector<2> convertADCVoltagesToPhaseCurrents(const math::Vector<2>& raw_voltages) const
    {
        return (raw_voltages - getCurrentZeroOffsets()) / (board_config_.current_shunt_resistance * getCurrentGain());
    }

    float convertADCVoltageToInverterTemperature(float voltage) const
    {
        return board_config_.temperature_transfer_function(voltage);
    }

    void beginCalibration()
    {
        AbsoluteCriticalSectionLocker locker;

        if (!current_zero_offset_calibrator_.in_progress)
        {
            setCurrentAmplifierGain(false);

            current_zero_offset_calibrator_.reset();
            current_zero_offset_calibrator_.in_progress = true;
            current_zero_offset_calibrator_.pwm_handle.setPWM(math::Vector<3>::Zero());

            assert(current_zero_offset_calibrator_.pwm_handle.isUnique());
        }
    }

    bool isCalibrationInProgress() const
    {
        return current_zero_offset_calibrator_.in_progress;
    }

    void processCalibration(const float period,
                            const math::Vector<2>& current_sensors_output_voltages)
    {
        assert(isCalibrationInProgress());

        current_zero_offset_calibrator_.pwm_handle.setPWM(math::Vector<3>::Zero());     // Paranoia

        current_zero_offset_calibrator_.averager.addSample(current_sensors_output_voltages);
        current_zero_offset_calibrator_.duration += period;

        if (current_zero_offset_calibrator_.duration > CurrentOffsetCalibrationDuration)
        {
            if (current_amplifier_high_gain_selected_)
            {
                current_zero_offsets_low_high_[1] = current_zero_offset_calibrator_.averager.getAverage();
                current_zero_offset_calibrator_.reset();
            }
            else
            {
                current_zero_offsets_low_high_[0] = current_zero_offset_calibrator_.averager.getAverage();
                setCurrentAmplifierGain(true);
                current_zero_offset_calibrator_.resetDurationAndAverage();
            }
        }
    }

    /// Could be static, but we keep it non-static for future proofness.
    template <unsigned NumSamples>
    float convertADCSamplesToVoltage(const std::uint16_t (&x)[NumSamples]) const
    {
        constexpr double VoltsPerLSB = double(ADCReferenceVoltage) / double((1U << ADCResolutionBits) - 1);

        constexpr float ConversionMultiplier = float(VoltsPerLSB / double(NumSamples));

        return float(std::accumulate(std::begin(x), std::end(x), 0U)) * ConversionMultiplier;
    }

    /**
     * When current amplifiers are disabled, their outputs are assumed to remain in a certain state.
     * Here we're checking if the outputs are in the said state.
     */
    bool areCurrentSensorOutputsValid(const math::Vector<2>& current_sensors_output_voltages) const
    {
        return current_sensors_output_voltages.mean() > MinCurrentSensorsOutputVoltage;
    }
};

}
}
