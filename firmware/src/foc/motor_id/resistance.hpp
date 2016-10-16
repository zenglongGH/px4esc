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


namespace foc
{
namespace motor_id
{
/**
 * Phase resistance estimation task.
 * This one should be always executed first, since it doesn't depend on anything.
 */
class ResistanceTask : public ITask
{
    static constexpr Scalar RotorStabilizationDuration  =  1.0F;
    static constexpr Scalar PhaseMeasurementDuration    = 10.0F;
    static constexpr Scalar OhmPerSec                   = 0.1F;
    static constexpr Scalar ValidCurrentThreshold       = 1e-3F;
    static constexpr unsigned MinSamples                = 100000;

    enum class State
    {
        CoarseMeasurement,
        PhaseA,
        PhaseB,
        PhaseC,
        Computation,
        FinishedSuccessfully,
        Failed
    } state_ = State::CoarseMeasurement;

    Scalar state_switched_at_ = 0;

    ContextReference context_;
    MotorParameters result_;

    Const estimation_current_;

    std::array<math::CumulativeAverageComputer<>, 3> averagers_;

    math::SimpleMovingAverageFilter<500, Vector<2>> currents_filter_;


    void switchState(State new_state)
    {
        state_ = new_state;
        state_switched_at_ = context_.getTime();
    }

    Scalar getTimeSinceStateSwitch() const
    {
        return context_.getTime() - state_switched_at_;
    }

    Scalar computeRelativePhaseVoltage(Const desired_voltage,
                                       Const inverter_voltage) const
    {
        assert(desired_voltage > 0);
        assert(inverter_voltage > 0);

        Const voltage_drop_due_to_dead_time = (context_.pwm_dead_time / context_.pwm_period) * inverter_voltage;

        return (desired_voltage + voltage_drop_due_to_dead_time) / inverter_voltage;
    }

    static Scalar computeLineVoltageForResistanceMeasurement(Const desired_current,
                                                             Const phase_resistance)
    {
        return (desired_current * phase_resistance) * Scalar(3.0 / 2.0);
    }

    bool processOneMeasurement(Const current,
                               Const voltage,
                               math::CumulativeAverageComputer<>& averager)
    {
        Const state_duration = getTimeSinceStateSwitch();

        if ((state_duration > RotorStabilizationDuration) &&
            (current > ValidCurrentThreshold))
        {
            averager.addSample(voltage * (2.0F / 3.0F) / current);
        }

        return state_duration > (PhaseMeasurementDuration + RotorStabilizationDuration);
    }

public:
    ResistanceTask(ContextReference context,
                   const MotorParameters& initial_parameters) :
        context_(context),
        result_(initial_parameters),
        estimation_current_(initial_parameters.max_current * context.params.fraction_of_max_current),
        currents_filter_(Vector<2>::Zero())
    {
        result_.rs = 0;

        if (!context_.params.isValid() ||
            !os::float_eq::positive(result_.max_current))
        {
            state_ = State::Failed;
        }
    }

    void onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                         Const inverter_voltage) override
    {
        currents_filter_.update(phase_currents_ab);

        const auto filtered_currents = currents_filter_.getValue();

        for (unsigned i = 0; i < 2; i++)
        {
            context_.setDebugVariable(i, filtered_currents[i]);
        }

        switch (state_)
        {
        case State::CoarseMeasurement:
        {
            // Slowly increasing the voltage until we've reached the required current.
            // We're supplying phase C in order to be able to use both current sensors with maximum resolution.
            if ((-filtered_currents.sum()) < estimation_current_)
            {
                result_.rs = std::max(MotorParameters::getRsLimits().min,
                                      result_.rs + OhmPerSec * context_.pwm_period); // Very coarse initial estimation
            }
            else
            {
                IRQDebugOutputBuffer::setVariableFromIRQ<0>(result_.rs);
                switchState(State::PhaseA);
            }

            Const voltage = computeLineVoltageForResistanceMeasurement(estimation_current_, result_.rs);
            Const relative_voltage = computeRelativePhaseVoltage(voltage, inverter_voltage);

            if ((relative_voltage < (context_.pwm_upper_limit - 0.5F)) &&
                MotorParameters::getRsLimits().contains(result_.rs))
            {
                context_.setPWM({
                    0.0F,
                    0.0F,
                    relative_voltage
                });
            }
            else
            {
                // Voltage or resistance is too high, aborting
                result_.rs = 0;
                switchState(State::Failed);
            }
            break;
        }

        case State::PhaseA:
        case State::PhaseB:
        case State::PhaseC:
        {
            /*
             * Precise resistance measurement.
             * We're using the rough measurement in order to maintain the requested current, more or less.
             * We also need to measure all three phases INDEPENDENTLY, because many motors have
             * very different phase resistances. It is not possible to find the statistically optimal
             * resistance without individual measurements per phase.
             */
            Const voltage = computeLineVoltageForResistanceMeasurement(estimation_current_, result_.rs);
            Const pwm_channel_setpoint = computeRelativePhaseVoltage(voltage, inverter_voltage);

            if (state_ == State::PhaseA)
            {
                context_.setPWM({
                    pwm_channel_setpoint,
                    0.0F,
                    0.0F
                });
                Const current = phase_currents_ab[0];
                if (processOneMeasurement(current, voltage, averagers_[0]))
                {
                    switchState(State::PhaseB);
                }
            }
            else if (state_ == State::PhaseB)
            {
                context_.setPWM({
                    0.0F,
                    pwm_channel_setpoint,
                    0.0F
                });
                Const current = phase_currents_ab[1];
                if (processOneMeasurement(current, voltage, averagers_[1]))
                {
                    switchState(State::PhaseC);
                }
            }
            else if (state_ == State::PhaseC)
            {
                context_.setPWM({
                    0.0F,
                    0.0F,
                    pwm_channel_setpoint
                });
                Const current = -phase_currents_ab.sum();
                if (processOneMeasurement(current, voltage, averagers_[2]))
                {
                    switchState(State::Computation);
                }
            }
            else
            {
                assert(false);
                switchState(State::Failed);
                break;
            }
            break;
        }

        case State::Computation:
        {
            context_.setPWM(Vector<3>::Zero());

            Scalar r_samples[3]{};
            std::transform(averagers_.begin(), averagers_.begin() + 3, std::begin(r_samples),
                           [](math::CumulativeAverageComputer<>& a) {
                return (a.getNumSamples() > MinSamples) ? Scalar(a.getAverage()) : Scalar(0);
            });
            std::sort(std::begin(r_samples), std::end(r_samples));

            for (unsigned i = 0; i < 3; i++)
            {
                IRQDebugOutputBuffer::setVariableFromIRQ(i, r_samples[i]);
            }

            result_.rs = r_samples[1];  // Taking the median

            // If at least one sample is invalid, throw out all measurements!
            if (std::all_of(std::begin(r_samples), std::end(r_samples),
                            [](Const x) { return MotorParameters::getRsLimits().contains(x); }) &&
                MotorParameters::getRsLimits().contains(result_.rs))    // Megaparanoia!
            {
                switchState(State::FinishedSuccessfully);
            }
            else
            {
                switchState(State::Failed);
            }
            break;
        }

        case State::FinishedSuccessfully:
        {
            context_.setPWM(Vector<3>::Zero());
            break;
        }

        case State::Failed:
        {
            context_.setPWM(Vector<3>::Zero());
            result_.rs = 0;
            break;
        }
        }
    }

    Status getStatus() const override
    {
        if (state_ == State::FinishedSuccessfully)
        {
            return Status::Succeeded;
        }
        else if (state_ == State::Failed)
        {
            return Status::Failed;
        }
        else
        {
            return Status::InProgress;
        }
    }

    MotorParameters getEstimatedMotorParameters() const override { return result_; }
};

}
}
