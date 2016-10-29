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

#include "task.hpp"
#include "motor_runner.hpp"
#include <zubax_chibios/util/helpers.hpp>


namespace foc
{
/**
 * Various control modes.
 * See the function definitions below for usages.
 */
enum class ControlMode
{
    RatiometricCurrent,         ///< Value in [-1, 1] that defines the current setpoint relative to the maximum
    RatiometricMRPM,            ///< Ditto, RPM setpoint relative to the maximum
    RatiometricVoltage,         ///< Ditto, voltage setpoint relative to the inverter supply voltage (Vbus)

    Current,                    ///< Ampere
    MRPM,                       ///< Mechanical RPM
    Voltage                     ///< Voltage applied in the quadrature axis
};


constexpr unsigned FirstRatiometricControlMode = unsigned(ControlMode::RatiometricCurrent);
constexpr unsigned LastRatiometricControlMode  = unsigned(ControlMode::RatiometricVoltage);

static_assert(FirstRatiometricControlMode < LastRatiometricControlMode,
              "Ford, you're turning into a penguin. Stop it.");


/**
 * This class encapsulates the transfer function from the input setpoint value in different units
 * (where units are encoded using @ref ControlMode) to the Iq reference current setpoint.
 * At the time of writing this I am not yet sure about whether it will fit the design well, so we may need
 * to refactor it later.
 */
class SetpointController
{
    Const max_current_;
    Const min_current_;
    Const min_voltage_;
    Const current_ramp_amp_s_;
    Const voltage_ramp_volt_s_;

public:
    SetpointController(Const max_current,
                       Const min_current,
                       Const min_voltage,
                       Const current_ramp_amp_s,
                       Const voltage_ramp_volt_s) :
        max_current_(max_current),
        min_current_(min_current),
        min_voltage_(min_voltage),
        current_ramp_amp_s_(current_ramp_amp_s),
        voltage_ramp_volt_s_(voltage_ramp_volt_s)
    { }

    /**
     * Discrete transfer function from input setpoint to current/voltage setpoint.
     *
     * @param period                            Update interval in seconds
     * @param target_setpoint                   Target setpoint, units defined by @ref control_mode
     * @param control_mode                      The actual transfer function to use, this defines the units
     * @param reference                         Iq reference current or Uq reference voltage, depending on the mode
     * @param max_voltage                       Maximum achievable axis voltage
     * @param electrical_angular_velocity       Electrical angular velocity of the rotor in radian/second
     * @return                                  New Iq/Uq reference, depending on the mode
     */
    Scalar update(Const period,
                  Const target_setpoint,
                  const ControlMode control_mode,
                  Const reference,
                  Const max_voltage,
                  Const electrical_angular_velocity) const
    {
        const bool zero_setpoint = os::float_eq::closeToZero(target_setpoint);

        switch (control_mode)
        {
        case ControlMode::RatiometricCurrent:
        case ControlMode::Current:
        {
            // Computing the new setpoint
            Scalar new_current = target_setpoint;
            if (control_mode == ControlMode::RatiometricCurrent)
            {
                new_current *= max_current_;
            }
            new_current = math::Range<>(-max_current_, max_current_).constrain(new_current);

            // Applying the ramp
            if (new_current > reference)
            {
                new_current = reference + current_ramp_amp_s_ * period;
            }
            else
            {
                new_current = reference - current_ramp_amp_s_ * period;
            }

            // Constraining the minimums, only if the sign is the same and the setpoint is non-zero
            if (((new_current > 0) == (target_setpoint > 0)) && !zero_setpoint)
            {
                new_current = std::copysign(std::max(min_current_, std::abs(new_current)),
                                            new_current);
            }

            return new_current;
        }

        case ControlMode::RatiometricMRPM:
        case ControlMode::MRPM:
        {
            (void) electrical_angular_velocity;
            assert(false);              // TODO: Implement
            return 0;
            break;
        }

        case ControlMode::RatiometricVoltage:
        case ControlMode::Voltage:
        {
            // Computing the new setpoint
            Scalar new_voltage = target_setpoint;
            if (control_mode == ControlMode::RatiometricVoltage)
            {
                new_voltage *= max_voltage;
            }
            new_voltage = math::Range<>(-max_voltage, max_voltage).constrain(new_voltage);

            // Applying the ramp
            if (new_voltage > reference)
            {
                new_voltage = reference + voltage_ramp_volt_s_ * period;
            }
            else
            {
                new_voltage = reference - voltage_ramp_volt_s_ * period;
            }

            // Constraining the minimums, only if the sign is the same and the setpoint is non-zero
            if (((new_voltage > 0) == (target_setpoint > 0)) && !zero_setpoint)
            {
                new_voltage = std::copysign(std::max(min_voltage_, std::abs(new_voltage)),
                                            new_voltage);
            }

            return new_voltage;
        }

        default:
        {
            assert(false);
            return 0;
            break;
        }
        }
    }
};

/**
 * Main motor control logic.
 */
class RunningTask : public ITask
{
    static constexpr Result::ExitCode ExitCodeTooManyStalls = 1;

    const TaskContext context_;

    const SetpointController setpoint_controller_;
    os::helpers::LazyConstructor<MotorRunner, os::helpers::MemoryInitializationPolicy::NoInit> runner_;

    std::uint32_t num_successive_stalls_ = 0;

    ControlMode requested_control_mode_ = ControlMode(0);
    Scalar raw_setpoint_ = 0;
    Scalar remaining_setpoint_timeout_ = 0;

    struct LowPassFilteredValues
    {
        static constexpr Scalar InnovationWeight = 0.05F;

        static void update(Scalar& value, const Scalar& new_value)
        {
            value += InnovationWeight * (new_value - value);
        }

        Scalar inverter_power = 0;
        Scalar demand_factor = 0;
    } low_pass_filtered_values_;


    MotorRunner::Setpoint computeSetpoint(Const period, const board::motor::Status& hw_status) const
    {
        MotorRunner::Setpoint new_sp;

        if (requested_control_mode_ == ControlMode::RatiometricVoltage ||
            requested_control_mode_ == ControlMode::Voltage)
        {
            new_sp.mode = MotorRunner::Setpoint::Mode::Uq;
        }
        else
        {
            new_sp.mode = MotorRunner::Setpoint::Mode::Iq;
        }

        MotorRunner::Setpoint old_sp = runner_->getSetpoint();

        if (old_sp.mode != new_sp.mode)
        {
            // Watch your step! We're transitioning between voltage-controlled mode and current-controlled mode!
            if (new_sp.mode == MotorRunner::Setpoint::Mode::Uq)
            {
                old_sp.value = runner_->getUdq()[1];    // Synthesizing the old setpoint from the real value
            }
            else
            {
                old_sp.value = runner_->getIdq()[1];
            }
        }

        const auto max_voltage = computeLineVoltageLimit(hw_status.inverter_voltage, context_.board.pwm.upper_limit);

        new_sp.value = setpoint_controller_.update(period,
                                                   raw_setpoint_,
                                                   requested_control_mode_,
                                                   old_sp.value,
                                                   max_voltage,
                                                   runner_->getElectricalAngularVelocity());
        return new_sp;
    }

public:
    RunningTask(const TaskContext& context,
                ControlMode control_mode,
                Const initial_setpoint,
                Const initial_setpoint_ttl) :
        context_(context),
        setpoint_controller_(context_.params.motor.max_current,
                             context_.params.motor.min_current,
                             context_.params.motor.computeMinVoltage(),
                             context_.params.motor.current_ramp_amp_per_s,
                             context_.params.motor.voltage_ramp_volt_per_s)
    {
        assert(context_.params.isValid());

        setSetpoint(control_mode, initial_setpoint, initial_setpoint_ttl);
    }

    const char* getName() const override { return "running"; }

    void setSetpoint(ControlMode control_mode,
                     Const value,
                     Const request_ttl)
    {
        AbsoluteCriticalSectionLocker locker;
        requested_control_mode_     = control_mode;
        raw_setpoint_               = value;
        remaining_setpoint_timeout_ = request_ttl;
    }

    Result onMainIRQ(Const period, const board::motor::Status& hw_status) override
    {
        if (!runner_.isConstructed())
        {
            AbsoluteCriticalSectionLocker locker;
            runner_.construct(context_.params.controller,
                              context_.params.motor,
                              context_.params.observer,
                              context_.board.pwm,
                              (raw_setpoint_ > 0) ? MotorRunner::Direction::Forward : MotorRunner::Direction::Reverse);
        }

        AbsoluteCriticalSectionLocker::assertNotLocked();
        runner_->updateStateEstimation(period, hw_status);

        {
            AbsoluteCriticalSectionLocker locker;

            switch (runner_->getState())
            {
            case MotorRunner::State::Spinup:
            {
                /*
                 * Nothing to do, just rolling.
                 * If change of direction was requested, we'll ignore it until spinup has been finished.
                 * Maybe this decision should be revisited later.
                 */
                break;
            }

            case MotorRunner::State::Running:
            {
                runner_->setSetpoint(computeSetpoint(period, hw_status));
                break;
            }

            case MotorRunner::State::Stopped:
            {
                runner_.destroy();
                num_successive_stalls_ = 0;
                if (os::float_eq::closeToZero(raw_setpoint_))
                {
                    return Result::success();
                }
                break;
            }

            case MotorRunner::State::Stalled:
            {
                const auto direction = runner_->getDirection();

                runner_.destroy();

                if (((direction == MotorRunner::Direction::Forward) && (raw_setpoint_ < 0)) ||
                    ((direction == MotorRunner::Direction::Reverse) && (raw_setpoint_ > 0)))
                {
                    num_successive_stalls_ = 0;
                }
                else
                {
                    num_successive_stalls_++;
                }

                if (num_successive_stalls_ > context_.params.controller.num_stalls_to_latch)
                {
                    return Result::failure(ExitCodeTooManyStalls);
                }
                else if (os::float_eq::closeToZero(raw_setpoint_))
                {
                    return Result::success();
                }
                else
                {
                    ;   // No change, keep running
                }
                break;
            }
            }
        }

        {
            AbsoluteCriticalSectionLocker locker;

            remaining_setpoint_timeout_ -= period;
            if (remaining_setpoint_timeout_ < 0)
            {
                remaining_setpoint_timeout_ = 0;
                raw_setpoint_ = 0;
            }
        }

        AbsoluteCriticalSectionLocker::assertNotLocked();

        if (runner_.isConstructed())
        {
            LowPassFilteredValues::update(low_pass_filtered_values_.inverter_power,
                                          runner_->computeInverterPower());

            LowPassFilteredValues::update(low_pass_filtered_values_.demand_factor,
                                          std::abs(runner_->getIdq()[1]) / context_.params.motor.max_current);
        }

        return Result::inProgress();
    }

    std::pair<Vector<3>, bool> onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                                               Const inverter_voltage) override
    {
        if (runner_.isConstructed())
        {
            return {
                runner_->updatePWMOutputsFromIRQ(phase_currents_ab, inverter_voltage),
                true
            };
        }
        else
        {
            return { Vector<3>::Zero(), false };
        }
    }

    std::array<Scalar, NumDebugVariables> getDebugVariables() const override
    {
        std::array<Scalar, NumDebugVariables> out{};

        {
            AbsoluteCriticalSectionLocker locker;
            if (runner_.isConstructed())
            {
                const auto vals = runner_->getDebugVariables();
                std::copy(vals.begin(), vals.end(), out.begin());
            }
        }

        // TODO: Return Dmitry's formula back
        return out;
    }

    bool isSpinupInProgress() const
    {
        AbsoluteCriticalSectionLocker locker;
        return runner_.isConstructed() ? (runner_->getState() == MotorRunner::State::Spinup) : false;
    }

    std::uint32_t getNumSuccessiveStalls() const
    {
        return num_successive_stalls_;  // Atomic read, no locking
    }

    Vector<2> getUdq() const
    {
        AbsoluteCriticalSectionLocker locker;
        return runner_.isConstructed() ? runner_->getUdq() : Vector<2>::Zero();
    }

    Vector<2> getIdq() const
    {
        AbsoluteCriticalSectionLocker locker;
        return runner_.isConstructed() ? runner_->getIdq() : Vector<2>::Zero();
    }

    Scalar getElectricalAngularVelocity() const
    {
        AbsoluteCriticalSectionLocker locker;
        return runner_.isConstructed() ? runner_->getElectricalAngularVelocity() : 0.0F;
    }

    LowPassFilteredValues getLowPassFilteredValues() const
    {
        AbsoluteCriticalSectionLocker locker;
        return low_pass_filtered_values_;
    }
};

}
