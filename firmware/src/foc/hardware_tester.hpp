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
#include <math/math.hpp>
#include <zubax_chibios/util/heapless.hpp>
#include <cassert>


namespace foc
{
/**
 * This class ensures that the power stage hardware, sensors, and the connected motor are functioning correctly.
 */
class HardwareTester
{
public:
    class TestReport
    {
        friend class HardwareTester;

        std::uint32_t mask_ = 0;

    public:
        enum class ErrorFlag
        {
            InverterVoltageSensorError,
            InverterTemperatureSensorError,
            CurrentSensorsZeroOffsetError,
            PhaseAError,
            PhaseBError,
            PhaseCError,
            MotorNotConnected,
            InverterOverloadSignal,
            InverterFaultSignal
        };

        static constexpr std::uint32_t flag2mask(const ErrorFlag f) { return 1U << unsigned(f); }

        std::uint32_t getErrorMask() const { return mask_; }

        unsigned getNumberOfErrors() const
        {
            unsigned out = 0;
            for (unsigned i = 0; i < 32; i++)
            {
                if ((mask_ & (1U << i)) != 0)
                {
                    out++;
                }
            }
            return out;
        }

        bool isSuccessful() const { return getErrorMask() == 0U; }

        auto toString() const
        {
            return os::heapless::format("NumErrors: %u, Code: 0x%04x 0b%s",
                                        getNumberOfErrors(),
                                        unsigned(mask_),
                                        os::heapless::intToString<2>(mask_).c_str());
        }
    };

private:
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

    Const pwm_period_;
    Range inverter_voltage_range_;
    Range inverter_temperature_range_;

    State state_ = State::Initialization;
    Scalar time_ = 0;
    Scalar state_switched_at_ = 0;

    math::SimpleMovingAverageFilter<200, Vector<2>> currents_filter_;

    TestReport test_report_;


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

    void registerError(const TestReport::ErrorFlag f)
    {
        test_report_.mask_ |= TestReport::flag2mask(f);

        // If all three phases are misbehaving, the motor is probably not connected.
        constexpr auto PhaseFlags = TestReport::flag2mask(TestReport::ErrorFlag::PhaseAError) |
                                    TestReport::flag2mask(TestReport::ErrorFlag::PhaseBError) |
                                    TestReport::flag2mask(TestReport::ErrorFlag::PhaseCError);

        if ((test_report_.mask_ & PhaseFlags) == PhaseFlags)
        {
            test_report_.mask_ &= ~PhaseFlags;
            test_report_.mask_ |= TestReport::flag2mask(TestReport::ErrorFlag::MotorNotConnected);
        }
    }

public:
    HardwareTester(Const pwm_period,
                   const Range inverter_voltage_range,
                   const Range inverter_temperature_range) :
        pwm_period_(pwm_period),
        inverter_voltage_range_(inverter_voltage_range),
        inverter_temperature_range_(inverter_temperature_range),
        currents_filter_(Vector<2>::Zero())
    {
        assert(pwm_period_ > 0);
        assert(inverter_temperature_range_.contains(math::convertCelsiusToKelvin(25.0F)));
    }

    /**
     * Must be invoked on every PWM period with appropriate measurements.
     * Returns the desired PWM setpoints, possibly zero.
     */
    Vector<3> onNextPWMPeriod(const Vector<2>& raw_phase_currents_ab,
                              Const inverter_voltage,
                              Const inverter_temperature,
                              const bool inverter_overload,
                              const bool inverter_fault)
    {
        time_ += pwm_period_;

        Vector<3> pwm_vector = Vector<3>::Zero();

        currents_filter_.update(raw_phase_currents_ab);
        const auto currents = currents_filter_.getValue();

        // No dead time compensation here
        Const relative_testing_voltage = TestingVoltage / inverter_voltage;

        if (!inverter_voltage_range_.contains(inverter_voltage))
        {
            registerError(TestReport::ErrorFlag::InverterVoltageSensorError);
        }

        if (!inverter_temperature_range_.contains(inverter_temperature))
        {
            registerError(TestReport::ErrorFlag::InverterTemperatureSensorError);
        }

        if (inverter_overload)
        {
            registerError(TestReport::ErrorFlag::InverterOverloadSignal);
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
                registerError(TestReport::ErrorFlag::CurrentSensorsZeroOffsetError);
            }
            switchToNextState();
            break;
        }

        case State::PreTestA:
        {
            pwm_vector[0] += relative_testing_voltage;
            switchToNextStateIfStabilizationTimeExpired();
            break;
        }

        case State::TestA:
        {
            const bool ok = (currents[0] > ThresholdCurrent) &&
                            (currents[1] < -ThresholdCurrent);
            if (!ok)
            {
                registerError(TestReport::ErrorFlag::PhaseAError);
            }
            switchToNextState();
            break;
        }

        case State::PreTestB:
        {
            pwm_vector[1] += relative_testing_voltage;
            switchToNextStateIfStabilizationTimeExpired();
            break;
        }

        case State::TestB:
        {
            const bool ok = (currents[0] < -ThresholdCurrent) &&
                            (currents[1] > ThresholdCurrent);
            if (!ok)
            {
                registerError(TestReport::ErrorFlag::PhaseBError);
            }
            switchToNextState();
            break;
        }

        case State::PreTestC:
        {
            pwm_vector[2] += relative_testing_voltage;
            switchToNextStateIfStabilizationTimeExpired();
            break;
        }

        case State::TestC:
        {
            const bool ok = (currents[0] < -ThresholdCurrent) &&
                            (currents[1] < -ThresholdCurrent);
            if (!ok)
            {
                registerError(TestReport::ErrorFlag::PhaseCError);
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
            if (inverter_fault)
            {
                registerError(TestReport::ErrorFlag::InverterFaultSignal);
            }
            break;
        }

        default:
        {
            assert(false);
            break;
        }
        }

        return pwm_vector;
    }

    bool isFinished() const { return state_ == State::Finished; }

    const TestReport& getTestReport() const { return test_report_; }

    /**
     * State accessors for debugging purposes.
     */
    const auto& getCurrentsFilter() const { return currents_filter_; }
};

}
