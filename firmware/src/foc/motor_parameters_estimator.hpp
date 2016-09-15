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

#include "common.hpp"
#include "voltage_modulator.hpp"
#include "motor_parameters.hpp"
#include <math/math.hpp>
#include <cstdint>


namespace foc
{
/**
 * Greater modes (listed later) allow to identify more parameters,
 * but impose more restrictions on the connected load.
 * Read the comments for details.
 */
enum class MotorIdentificationMode
{
    /**
     * In this mode, the motor will not rotate, therefore it doesn't matter what load it is connected to.
     * Estimated parameters: Rab, Lab.
     */
    Static,

    /**
     * In this mode, the motor WILL SPIN.
     * In order to achieve correct results, the motor MUST NOT BE CONNECTED TO ANY MECHANICAL LOAD.
     * Estimated parameters: Rab, Lab, Phi.
     */
    RotationWithoutMechanicalLoad
};

/**
 * Magical and hard-to-use class that estimates parameters of a given motor, such as magnetic flux and inductance.
 * The business logic of this class outstretches its tentacles of dependency to a lot of other stuff,
 * so it's really hard to encapsulate it cleanly. Or maybe I'm just a shitty architect, you never know.
 */
class MotorParametersEstimator
{
    static constexpr Scalar WarmingUpDuration           = 5.0F;
    static constexpr Scalar RsPreMeasurementTimeout     = 5.0F;
    static constexpr Scalar RsMeasurementDuration       = 10.0F;
    static constexpr Scalar RoverLMeasurementDuration   = 5.0F;

    static constexpr unsigned IdqMovingAverageLength = 5;

    class Averager
    {
        using Accumulator = double;

        std::uint32_t num_samples_ = 0;
        Accumulator accumulator_ = 0;

    public:
        void addSample(Accumulator x)
        {
            num_samples_++;
            accumulator_ += x;
        }

        Scalar getAverage() const
        {
            if (num_samples_ > 0)
            {
                return Scalar(accumulator_ / Accumulator(num_samples_));
            }
            else
            {
                assert(false);
                return 0;
            }
        }

        auto getNumSamples() const { return num_samples_; }
    };

    class VoltageModulatorWrapper       // This is such a massive reinvented wheel. Do something about it.
    {
        using Modulator = ThreePhaseVoltageModulator<IdqMovingAverageLength>;

        // Poor man's aligned storage
        alignas(Modulator) std::uint8_t storage_[sizeof(Modulator)];
        Modulator* modulator_ = nullptr;

    public:
        ~VoltageModulatorWrapper()
        {
            if (modulator_ != nullptr)
            {
                modulator_->~ThreePhaseVoltageModulator();
                modulator_ = nullptr;
            }
        }

        template <typename... Args>
        void init(Args... args)
        {
            if (modulator_ != nullptr)
            {
                assert(false);
                modulator_->~ThreePhaseVoltageModulator();
            }
            modulator_ = new (storage_) Modulator(args...);
        }

        Modulator& access()
        {
            assert(modulator_ != nullptr);
            return *modulator_;
        }
    } voltage_modulator_wrapper_;

    const MotorIdentificationMode mode_;
    Const estimation_current_;
    Const Ls_current_frequency_;
    Const pwm_period_;
    Const pwm_dead_time_;

    MotorParameters result_;

    enum class State
    {
        Initialization,
        PreRsMeasurement,
        RsMeasurement,
        PreLsMeasurement,
        LsMeasurement,
        PhiMeasurement,
        Finalization,
        Finished
    } state_ = State::Initialization;

    Scalar time_ = 0;
    Scalar state_switched_at_ = 0;

    Averager averager_;
    Averager averager_2_;

    Scalar angular_position_ = 0;

    math::SimpleMovingAverageFilter<100, Vector<2>> phase_currents_filter_;

    void switchState(State new_state)
    {
        state_ = new_state;
        state_switched_at_ = time_;
        averager_ = Averager();
        averager_2_ = Averager();
    }

    Scalar getTimeSinceStateSwitch() const
    {
        assert(time_ > state_switched_at_);
        return time_ - state_switched_at_;
    }

    Scalar computeRelativePhaseVoltage(Const desired_voltage,
                                       Const inverter_voltage) const
    {
        assert(desired_voltage > 0);
        assert(inverter_voltage > 0);

        Const voltage_drop_due_to_dead_time = (pwm_dead_time_ / pwm_period_) * inverter_voltage;

        return (desired_voltage + voltage_drop_due_to_dead_time) / inverter_voltage;
    }

    static Scalar computeLineVoltageForResistanceMeasurement(Const desired_current,
                                                             Const phase_to_phase_resistance)
    {
        return (desired_current * phase_to_phase_resistance / 2.0F) * Scalar(3.0 / 2.0);
    }

public:
    MotorParametersEstimator(MotorIdentificationMode mode,
                             const MotorParameters& initial_parameters,
                             Const estimation_current,
                             Const Ls_current_frequency,
                             Const pwm_period,
                             Const pwm_dead_time) :
        mode_(mode),
        estimation_current_(estimation_current),
        Ls_current_frequency_(Ls_current_frequency),
        pwm_period_(pwm_period),
        pwm_dead_time_(pwm_dead_time),
        result_(initial_parameters),
        phase_currents_filter_(Vector<2>::Zero())
    { }

    /**
     * Must be invoked on every PWM period with appropriate measurements.
     * Returns the desired PWM setpoints, possibly zero.
     */
    Vector<3> onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                              Const inverter_voltage)
    {
        Vector<3> pwm_vector = Vector<3>::Ones() * 0.5F;

        time_ += pwm_period_;

        phase_currents_filter_.update(phase_currents_ab);

        switch (state_)
        {
        case State::Initialization:
        {
            // Invalidating before measurement
            result_.r_ab = 0;
            result_.l_ab = 0;

            switchState(State::PreRsMeasurement);
            break;
        }

        case State::PreRsMeasurement:
        {
            if (getTimeSinceStateSwitch() > RsPreMeasurementTimeout)
            {
                result_.r_ab = 0;                       // Failed - timeout
                switchState(State::Finalization);
            }
            else
            {
                /*
                 * Slowly increasing the voltage until we've reached the required current.
                 * Note that we're using phase B instead of A in order to heat up the motor more uniformly
                 * and also to check correct operation of both phases.
                 */
                if (phase_currents_filter_.getValue()[1] < estimation_current_)
                {
                    constexpr Scalar OhmPerSec = 0.1F;
                    result_.r_ab += OhmPerSec * pwm_period_;    // Very rough initial estimation
                }
                else
                {
                    switchState(State::RsMeasurement);
                }

                Const voltage = computeLineVoltageForResistanceMeasurement(estimation_current_, result_.r_ab);
                pwm_vector[1] += computeRelativePhaseVoltage(voltage, inverter_voltage);
            }
            break;
        }

        case State::RsMeasurement:
        {
            /*
             * Precise resistance measurement.
             * We're using the rough measurement in order to maintain the requested current, more or less.
             */
            Const voltage = computeLineVoltageForResistanceMeasurement(estimation_current_, result_.r_ab);
            pwm_vector[0] += computeRelativePhaseVoltage(voltage, inverter_voltage);

            if (phase_currents_ab[0] > 1e-3F)
            {
                averager_.addSample(double(voltage / phase_currents_ab[0]) * (2.0 / 3.0));
            }

            if (getTimeSinceStateSwitch() > RsMeasurementDuration)
            {
                if (averager_.getNumSamples() > 100)
                {
                    result_.r_ab = averager_.getAverage() * 2.0F;
                }
                else
                {
                    result_.r_ab = 0;        // Failed
                }

                if (result_.r_ab > 0)
                {
                    switchState(State::PreLsMeasurement);
                }
                else
                {
                    switchState(State::Finalization);
                }
            }
            break;
        }

        case State::PreLsMeasurement:
        {
            constexpr Scalar OneSizeFitsAllLab = 100.0e-6F;

            voltage_modulator_wrapper_.init(OneSizeFitsAllLab / 2.0F,
                                            result_.r_ab / 2.0F,
                                            estimation_current_,
                                            pwm_period_);
            // Ourowrapos - a wrapper that wraps itself.
            switchState(State::LsMeasurement);
            break;
        }

        case State::LsMeasurement:
        {
            Const w = Ls_current_frequency_ * (math::Pi * 2.0F);

            const auto output = voltage_modulator_wrapper_.access().update(phase_currents_ab,
                                                                           inverter_voltage,
                                                                           w,
                                                                           angular_position_,
                                                                           estimation_current_);

            angular_position_ = output.extrapolated_angular_position;
            pwm_vector = output.pwm_setpoint;

            Const Ud = output.reference_Udq[0];
            Const Iq = output.estimated_Idq[1];

            averager_.addSample(Ud * Ud);
            averager_2_.addSample(Iq * Iq);

            if (getTimeSinceStateSwitch() > RoverLMeasurementDuration)
            {
                Const Ud_squared = averager_.getAverage();
                Const Iq_squared = averager_2_.getAverage();
                Const w_squared = w * w;

                Const Ls = std::sqrt(Ud_squared / (w_squared * Iq_squared));

                result_.l_ab = Ls * 2.0F;

                if (mode_ == MotorIdentificationMode::Static)
                {
                    switchState(State::Finalization);
                }
                else if (mode_ == MotorIdentificationMode::RotationWithoutMechanicalLoad)
                {
                    switchState(State::PhiMeasurement);
                }
                else
                {
                    assert(false);
                    switchState(State::Finalization);
                }
            }
            break;
        }

        case State::PhiMeasurement:
        {
            switchState(State::Finalization);
            break;
        }

        case State::Finalization:
        {
            switchState(State::Finished);
            break;
        }

        case State::Finished:
        {
            break;
        }

        default:
        {
            assert(false);
        }
        }

        return pwm_vector;
    }

    bool isFinished() const { return state_ == State::Finished; }

    const MotorParameters& getEstimatedMotorParameters() const { return result_; }
};

}
