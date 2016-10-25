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

#include <foc/common.hpp>
#include <foc/task.hpp>
#include "report.hpp"


namespace foc
{
namespace hw_test
{
/**
 * This class ensures that the power stage hardware, sensors, and the connected motor are functioning correctly.
 */
class HardwareTestingTask : public ITask
{
    static constexpr Scalar TestingVoltage      = 0.7F;
    static constexpr Scalar ThresholdCurrent    = 0.15F;
    static constexpr Scalar StabilizationTime   = 0.3F;

    using Range = math::Range<>;

    enum class State : unsigned
    {
        Initialization,
        ZeroCheck,

        PreTestA,
        TestA,

        PreTestB,
        TestB,

        PreTestC,
        TestC,

        Finished
    };

    const TaskContext context_;

    State state_ = State::Initialization;
    Scalar time_ = 0;
    Scalar state_switched_at_ = 0;

    math::SimpleMovingAverageFilter<200, Vector<2>> currents_filter_;

    Report test_report_;

    board::motor::Status hardware_status_;

    Vector<3> pwm_output_ = Vector<3>::Zero();


    Scalar getTimeSinceStateSwitch() const
    {
        assert(time_ > state_switched_at_);
        return time_ - state_switched_at_;
    }

    void switchState(State new_state)
    {
        state_ = new_state;
        state_switched_at_ = time_;
    }

    void switchToNextState()
    {
        switchState(State(unsigned(state_) + 1U));
    }

    void switchToNextStateIfStabilizationTimeExpired()
    {
        if (getTimeSinceStateSwitch() >= StabilizationTime)
        {
            switchToNextState();
        }
    }

    void registerError(const Report::ErrorFlag f)
    {
        test_report_.mask_ |= Report::flag2mask(f);

        // If all three phases are misbehaving, the motor is probably not connected.
        constexpr auto PhaseFlags = Report::flag2mask(Report::ErrorFlag::PhaseAError) |
                                    Report::flag2mask(Report::ErrorFlag::PhaseBError) |
                                    Report::flag2mask(Report::ErrorFlag::PhaseCError);

        if ((test_report_.mask_ & PhaseFlags) == PhaseFlags)
        {
            test_report_.mask_ &= ~PhaseFlags;
            test_report_.mask_ |= Report::flag2mask(Report::ErrorFlag::MotorNotConnected);
        }
    }

public:
    HardwareTestingTask(const TaskContext& context) :
        context_(context),
        currents_filter_(Vector<2>::Zero())
    {
        assert(context_.board.limits.measurement_range.inverter_temperature.contains(
            math::convertCelsiusToKelvin(25.0F)));
    }

    const char* getName() const override { return "hw_test"; }

    void onMainIRQ(Const period,
                   const board::motor::Status& hw_status) override
    {
        hardware_status_ = hw_status;

        time_ += period;

        pwm_output_.setZero();

        const auto currents = currents_filter_.getValue();

        // No dead time compensation here
        Const relative_testing_voltage = TestingVoltage / hw_status.inverter_voltage;

        if (!context_.board.limits.measurement_range.inverter_voltage.contains(hw_status.inverter_voltage))
        {
            registerError(Report::ErrorFlag::InverterVoltageSensorError);
        }

        if (!context_.board.limits.measurement_range.inverter_temperature.contains(hw_status.inverter_temperature))
        {
            registerError(Report::ErrorFlag::InverterTemperatureSensorError);
        }

        if (hw_status.overload)
        {
            registerError(Report::ErrorFlag::InverterOverloadSignal);
        }

        switch (state_)
        {
        case State::Initialization:
        {
            switchToNextStateIfStabilizationTimeExpired();
            break;
        }

        case State::ZeroCheck:
        {
            const bool ok = (std::abs(currents[0]) < ThresholdCurrent) &&
                            (std::abs(currents[1]) < ThresholdCurrent);
            if (!ok)
            {
                registerError(Report::ErrorFlag::CurrentSensorsZeroOffsetError);
            }
            switchToNextState();
            break;
        }

        case State::PreTestA:
        {
            pwm_output_[0] += relative_testing_voltage;
            switchToNextStateIfStabilizationTimeExpired();
            break;
        }

        case State::TestA:
        {
            const bool ok = (currents[0] > ThresholdCurrent) &&
                            (currents[1] < -ThresholdCurrent);
            if (!ok)
            {
                registerError(Report::ErrorFlag::PhaseAError);
            }
            switchToNextState();
            break;
        }

        case State::PreTestB:
        {
            pwm_output_[1] += relative_testing_voltage;
            switchToNextStateIfStabilizationTimeExpired();
            break;
        }

        case State::TestB:
        {
            const bool ok = (currents[0] < -ThresholdCurrent) &&
                            (currents[1] > ThresholdCurrent);
            if (!ok)
            {
                registerError(Report::ErrorFlag::PhaseBError);
            }
            switchToNextState();
            break;
        }

        case State::PreTestC:
        {
            pwm_output_[2] += relative_testing_voltage;
            switchToNextStateIfStabilizationTimeExpired();
            break;
        }

        case State::TestC:
        {
            const bool ok = (currents[0] < -ThresholdCurrent) &&
                            (currents[1] < -ThresholdCurrent);
            if (!ok)
            {
                registerError(Report::ErrorFlag::PhaseCError);
            }
            switchToNextState();
            break;
        }

        case State::Finished:
        {
            /*
             * Normally we should be checking this in all states, but there's something wrong with the hardware
             * which causes intermittent FAULT reports sometimes, so we moved the check here temporarily.
             */
            if (hw_status.fault)
            {
                registerError(Report::ErrorFlag::InverterFaultSignal);
            }
            break;
        }

        default:
        {
            assert(false);
            break;
        }
        }
    }

    std::pair<Vector<3>, bool> onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                                               Const inverter_voltage) override
    {
        (void) inverter_voltage;
        currents_filter_.update(phase_currents_ab);
        return {pwm_output_, true};
    }

    Status getStatus() const override
    {
        if (state_ == State::Finished)
        {
            return test_report_.isSuccessful() ? Status::Finished : Status::Failed;
        }
        else
        {
            return Status::Running;
        }
    }

    std::array<Scalar, NumDebugVariables> getDebugVariables() const override
    {
        const auto currents = currents_filter_.getValue();

        return {
            hardware_status_.inverter_voltage,
            math::convertKelvinToCelsius(hardware_status_.inverter_temperature),
            hardware_status_.current_sensor_gain,
            currents[0],
            currents[1]
        };
    }

    void applyResultToGlobalContext(TaskContext& inout_context) const override
    {
        inout_context.hw_test_report = test_report_;
    }
};

}
}
