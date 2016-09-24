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
#include "irq_debug_output.hpp"
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
 *
 * Refer to the Dmitry's doc for derivations and explanation of what's going on here.
 */
class MotorParametersEstimator
{
    static constexpr Scalar RsMeasurementDuration       = 15.0F;
    static constexpr Scalar LsMeasurementDuration       = 15.0F;

    // Voltage reduction during the measurement phase shold be very slow in order to
    // reduce phase delay of the current filter.
    static constexpr Scalar PhiMeasurementVoltageSlopeLengthSec = 40.0F;

    static constexpr unsigned IdqMovingAverageLength = 5;

    /*
     * This constant limits the maximum PWM value.
     * Exceeding this value may cause the ADC samples to occur at the moment when FET are switching,
     * which leads to incorrect measurements.
     * TODO: This parameter is heavily hardware-dependent, so it should be provided by the board driver.
     */
    static constexpr Scalar PWMLimit = 0.8F;

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

        Accumulator getAverage() const
        {
            if (num_samples_ > 0)
            {
                return accumulator_ / Accumulator(num_samples_);
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
    public:
        using Modulator = ThreePhaseVoltageModulator<IdqMovingAverageLength>;

    private:
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

        const Modulator& access() const
        {
            assert(modulator_ != nullptr);
            return *modulator_;
        }
    } voltage_modulator_wrapper_;

    const MotorIdentificationMode mode_;
    Const estimation_current_;
    Const Ls_current_frequency_;
    Const Phi_angular_velocity_;
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
        PhiMeasurementInitialization,
        PhiMeasurementAcceleration,
        PhiMeasurement,
        Finalization,
        Finished
    } state_ = State::Initialization;

    Scalar time_ = 0;
    Scalar state_switched_at_ = 0;

    std::array<Averager, 2> averagers_;
    std::array<Scalar, 4> state_variables_{};

    math::SimpleMovingAverageFilter<500, Vector<2>> currents_filter_;

    void switchState(State new_state, bool retain_states = false)
    {
        state_ = new_state;
        state_switched_at_ = time_;

        if (!retain_states)
        {
            std::fill(std::begin(averagers_), std::end(averagers_), Averager());
            std::fill(std::begin(state_variables_), std::end(state_variables_), 0.0F);
            currents_filter_.reset(Vector<2>::Zero());
        }
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
                             Const Ls_current_frequency,
                             Const Phi_angular_velocity,
                             Const pwm_period,
                             Const pwm_dead_time) :
        mode_(mode),
        estimation_current_(initial_parameters.max_current * 0.5F),
        Ls_current_frequency_(Ls_current_frequency),
        Phi_angular_velocity_(Phi_angular_velocity),
        pwm_period_(pwm_period),
        pwm_dead_time_(pwm_dead_time),
        result_(initial_parameters),
        currents_filter_(Vector<2>::Zero())
    {
        assert(estimation_current_ > 0);
    }

    /**
     * Must be invoked on every PWM period with appropriate measurements.
     * Returns the desired PWM setpoints, possibly zero.
     */
    Vector<3> onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                              Const inverter_voltage)
    {
        static constexpr int PhaseIndexC = 2;

        Vector<3> pwm_vector = Vector<3>::Ones() * 0.5F;

        time_ += pwm_period_;

        switch (state_)
        {
        case State::Initialization:
        {
            // Invalidating before measurement
            result_.r_ab = 0;
            result_.l_ab = 0;

            if (estimation_current_ > 0.1F)
            {
                switchState(State::PreRsMeasurement);
            }
            else
            {
                switchState(State::Finalization);       // Invalid settings
            }
            break;
        }

        case State::PreRsMeasurement:
        {
            currents_filter_.update(phase_currents_ab);

            // Slowly increasing the voltage until we've reached the required current.
            if (currents_filter_.getValue().sum() < estimation_current_)
            {
                constexpr Scalar OhmPerSec = 0.1F;

                result_.r_ab = std::max(MotorParameters::getRabLimits().min,
                                        result_.r_ab + OhmPerSec * pwm_period_);    // Very coarse initial estimation
            }
            else
            {
                switchState(State::RsMeasurement);
            }

            Const voltage = computeLineVoltageForResistanceMeasurement(estimation_current_, result_.r_ab);
            Const relative_voltage = computeRelativePhaseVoltage(voltage, inverter_voltage);

            if ((relative_voltage < (PWMLimit - 0.5F)) &&
                MotorParameters::getRabLimits().contains(result_.r_ab))
            {
                pwm_vector[PhaseIndexC] -= relative_voltage;
            }
            else
            {
                // Voltage or resistance is too high, aborting
                result_.r_ab = 0;
                switchState(State::Finalization);
            }
            break;
        }

        case State::RsMeasurement:
        {
            static constexpr Scalar ValidCurrentThreshold = 1e-3F;

            /*
             * Precise resistance measurement.
             * We're using the rough measurement in order to maintain the requested current, more or less.
             */
            Const voltage = computeLineVoltageForResistanceMeasurement(estimation_current_, result_.r_ab);
            pwm_vector[PhaseIndexC] -= computeRelativePhaseVoltage(voltage, inverter_voltage);

            Const phase_c_current = phase_currents_ab.sum();

            if (phase_c_current > ValidCurrentThreshold)
            {
                averagers_[0].addSample(double(voltage / phase_c_current) * (2.0 / 3.0));
            }

            if (getTimeSinceStateSwitch() > RsMeasurementDuration)
            {
                if (averagers_[0].getNumSamples() > 100)
                {
                    result_.r_ab = Scalar(averagers_[0].getAverage() * 2.0);
                }
                else
                {
                    result_.r_ab = 0;        // Failed
                }

                IRQDebugOutputBuffer::getInstance().setVariableFromIRQ<0>(result_.r_ab);

                if (MotorParameters::getRabLimits().contains(result_.r_ab))
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
                                            pwm_period_,
                                            pwm_dead_time_,
                                            VoltageModulatorWrapper::Modulator::DeadTimeCompensationPolicy::Disabled);
            // Ourowrapos - a wrapper that wraps itself.
            switchState(State::LsMeasurement);
            break;
        }

        case State::LsMeasurement:
        {
            Const w = Ls_current_frequency_ * (math::Pi * 2.0F);

            const auto output = voltage_modulator_wrapper_.access().onNextPWMPeriod(phase_currents_ab,
                                                                                    inverter_voltage,
                                                                                    w,
                                                                                    state_variables_[0],
                                                                                    estimation_current_);

            const bool Udq_was_constrained =
                voltage_modulator_wrapper_.access().getUdqNormalizationCounter().get() > 0;

            state_variables_[0] = output.extrapolated_angular_position;
            pwm_vector = output.pwm_setpoint;

            averagers_[0].addSample(output.reference_Udq[0]);
            averagers_[1].addSample(output.reference_Udq[1]);

            // Saving internal states into the state variables to make them observable from outside, for debugging.
            state_variables_[1] = output.reference_Udq[0];
            state_variables_[2] = output.reference_Udq[1];
            state_variables_[3] = output.estimated_Idq[1];

            if ((getTimeSinceStateSwitch() > LsMeasurementDuration) && !Udq_was_constrained)
            {
                // We're crunching very large numbers here, so we use double to avoid degradation of precision
                static const auto square_double = [](double x) { return double(x) * double(x); };

                const double Ud_square = square_double(averagers_[0].getAverage());
                const double Uq_square = square_double(averagers_[1].getAverage());

                Const RoverL = Scalar(std::sqrt((Uq_square * double(w) * double(w)) / Ud_square));

                result_.l_ab = result_.r_ab / RoverL;

                if ((mode_ == MotorIdentificationMode::Static) ||
                    !MotorParameters::getLabLimits().contains(result_.l_ab))
                {
                    switchState(State::Finalization);
                }
                else if (mode_ == MotorIdentificationMode::RotationWithoutMechanicalLoad)
                {
                    switchState(State::PhiMeasurementInitialization);
                }
                else
                {
                    assert(false);
                    switchState(State::Finalization);
                }
            }

            if (Udq_was_constrained)
            {
                // Udq was constrained, therefore the measurements cannot be trusted
                result_.l_ab = 0;
                switchState(State::Finalization);
            }
            break;
        }

        case State::PhiMeasurementInitialization:
        case State::PhiMeasurementAcceleration:
        case State::PhiMeasurement:
        {
            constexpr int IdxVoltage = 0;
            constexpr int IdxAngVel  = 1;
            constexpr int IdxAngPos  = 2;
            constexpr int IdxI       = 3;

            // This is the maximum voltage we start from.
            Const initial_voltage = estimation_current_ * (result_.r_ab / 2.0F) * (3.0F / 2.0F);

            // Continuously maintaining the smoothed out current estimates throughout the whole process.
            const auto Idq = performParkTransform(performClarkeTransform(phase_currents_ab),
                                                  math::sin(state_variables_[IdxAngPos]),
                                                  math::cos(state_variables_[IdxAngPos]));
            currents_filter_.update(Idq);

            // Additional Iq filtering
            Const prev_I = state_variables_[IdxI];
            state_variables_[IdxI] += pwm_period_ * 10.0F *
                (currents_filter_.getValue().norm() - state_variables_[IdxI]);

            if (state_ == State::PhiMeasurementInitialization)
            {
                result_.phi = 0;

                state_variables_[IdxVoltage] = initial_voltage;

                switchState(State::PhiMeasurementAcceleration, true);
            }
            else if (state_ == State::PhiMeasurementAcceleration)
            {
                state_variables_[IdxAngVel] +=
                    (Phi_angular_velocity_ / (PhiMeasurementVoltageSlopeLengthSec / 2.0F)) * pwm_period_;

                if (state_variables_[IdxAngVel] >= Phi_angular_velocity_)
                {
                    switchState(State::PhiMeasurement, true);
                }
            }
            else if (state_ == State::PhiMeasurement)
            {
                Const Uq = state_variables_[IdxVoltage];
                Const I  = state_variables_[IdxI];
                Const w  = state_variables_[IdxAngVel];
                Const Rs = result_.r_ab / 2.0F;

                Const new_phi = (Uq - I * Rs) / w;

                if (result_.phi > 0)
                {
                    result_.phi = std::min(result_.phi, new_phi);
                }
                else
                {
                    result_.phi = new_phi;
                }

                Const dIdt = (state_variables_[IdxI] - prev_I) / pwm_period_;

                // TODO: we could automatically learn the worst case di/dt after the acceleration phase?
                // 2 - triggers false positive
                // 3 - works fine
                // 6 - works fine
                Const dIdt_threshold = prev_I * 4.0F;

                if (dIdt > dIdt_threshold)
                {
                    switchState(State::Finalization);
                }
                else
                {
                    // Minimum is not reached yet, continuing to reduce voltage
                    state_variables_[IdxVoltage] -=
                        (initial_voltage / PhiMeasurementVoltageSlopeLengthSec) * pwm_period_;
                }
            }
            else
            {
                assert(false);
            }

            // Voltage modulation
            if (state_ == State::PhiMeasurementInitialization ||
                state_ == State::PhiMeasurementAcceleration ||
                state_ == State::PhiMeasurement)
            {
                Const max_voltage = inverter_voltage * 0.8F;

                const math::Range<> voltage_range(0.1F, max_voltage);

                if (voltage_range.contains(state_variables_[IdxVoltage]))
                {
                    state_variables_[IdxAngPos] = constrainAngularPosition(state_variables_[IdxAngPos] +
                                                                           state_variables_[IdxAngVel] * pwm_period_);

                    const auto Uab = performInverseParkTransform({0.0F, state_variables_[IdxVoltage]},
                                                                 math::sin(state_variables_[IdxAngPos]),
                                                                 math::cos(state_variables_[IdxAngPos]));

                    const auto uncompensated = performSpaceVectorTransform(Uab, inverter_voltage).first;

                    // Compensation is not used because it underperforms at low currents
                    pwm_vector = uncompensated;
                }
                else
                {
                    // Voltage is not in the valid range, aborting
                    result_.phi = 0;
                    switchState(State::Finalization);
                }
            }
            break;
        }

        case State::Finalization:
        {
            pwm_vector.setZero();
            switchState(State::Finished, true); // Retaining last variables for debugging purposes
            break;
        }

        case State::Finished:
        {
            pwm_vector.setZero();
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

    /**
     * State accessor for debugging purposes.
     */
    auto getDebugValues() const
    {
        std::array<Scalar, 7> out
        {
            state_variables_[0],
            state_variables_[1],
            state_variables_[2],
            state_variables_[3]
        };

        if (state_ == State::LsMeasurement)
        {
            out[4] = Scalar(voltage_modulator_wrapper_.access().getUdqNormalizationCounter().get());
        }

        return out;
    }
};

}
