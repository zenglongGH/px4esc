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

#include "parameters.hpp"
#include "voltage_modulator.hpp"
#include <math/math.hpp>
#include <board/motor.hpp>
#include <cassert>


namespace foc
{

using math::Scalar;
using math::Const;
using math::Vector;
using board::motor::AbsoluteCriticalSectionLocker;

/**
 * Main motor control class used during normal operation (spinup/running).
 * Fundamentally, the rest of the application exists only for the sake of this class.
 * This is the main business logic.
 */
class MotorRunner final
{
    static constexpr unsigned IdqMovingAverageLength = 5;

    static constexpr Scalar MaximumSpinupDurationFraction          = 1.5F;
    static constexpr Scalar SpinupAngularVelocityHysteresis        = 3.0F;

    using Modulator = ThreePhaseVoltageModulator<IdqMovingAverageLength>;

public:
    enum class State
    {
        Spinup,
        Running,
        Stopped,    ///< Normal stop, e.g. setpoint assigned zero
        Stalled     ///< Abnormal stop, e.g. rotor was locked mechanically or control failure
    };

    enum class Direction
    {
        Forward,
        Reverse
    };

    using Setpoint = Modulator::Setpoint;

    using DebugVariables = std::array<Scalar, 6>;

private:
    const ControllerParameters controller_params_;
    const MotorParameters motor_params_;

    const Direction direction_;

    State state_ = State::Spinup;

    observer::Observer observer_;

    Setpoint regular_setpoint_;
    Setpoint spinup_setpoint_;

    Scalar angular_velocity_ = 0;

    Scalar remaining_time_before_stall_detection_enabled_ = 0;
    Scalar spinup_time_ = 0;

    // Mutable entities can be modified from the PWM modulation method
    mutable Modulator modulator_;
    mutable Scalar angular_position_ = 0;
    mutable Vector<2> Idq_ = Vector<2>::Zero();
    mutable Vector<2> reference_Udq_ = Vector<2>::Zero();


    bool isReversed() const { return direction_ == Direction::Reverse; }

public:
    MotorRunner(const ControllerParameters& controller_params,
                const MotorParameters& motor_params,
                const observer::Parameters& observer_params,
                const board::motor::PWMParameters& pwm_params,
                const Direction dir) :
        controller_params_(controller_params),
        motor_params_(motor_params),
        direction_(dir),

        observer_(observer_params,
                  motor_params.phi,
                  motor_params.lq,      // TODO: MOVE THE ASSUMPTION THAT Lq = Ld INTO THE OBSERVER
                  motor_params.lq,
                  motor_params.rs),

        modulator_(motor_params.lq,
                   motor_params.rs,
                   motor_params.max_current,
                   controller_params.voltage_modulator_bandwidth,
                   pwm_params,
                   modulator_.DeadTimeCompensationPolicy::Disabled,
                   controller_params.voltage_modulator_cross_coupling_inductance_compensation ?
                       modulator_.CrossCouplingCompensationPolicy::Enabled :
                       modulator_.CrossCouplingCompensationPolicy::Disabled)
    { }

    /**
     * This is the only method that can be preempted by a higher priority IRQ!
     */
    void updateStateEstimation(Const period,
                               const board::motor::Status& hw_status)
    {
        (void) hw_status;       // We don't need it, but keep it anyway for future proofness

        AbsoluteCriticalSectionLocker::assertNotLocked();

        /*
         * Making a local copy of state variables ASAP, before the next fast IRQ fires.
         */
        const auto Idq = Idq_;
        const auto Udq = reference_Udq_;

        if (state_ != State::Spinup &&
            state_ != State::Running)
        {
            return;     // Nothing to do really
        }

        /*
         * Running the observer, this takes forever.
         * By the time the observer has finished, the rotor has moved some angle forward, which we compensate.
         */
        observer_.update(period, Idq, Udq);     // A very long call

        /*
         * Once the observer has finished, a state mutation intensive part begins, so we acquire the expensive lock.
         */
        AbsoluteCriticalSectionLocker locker;

        angular_velocity_ = observer_.getAngularVelocity();

        // Correcting the angle estimation latency, assuming that the observer runs for about half period.
        angular_position_ = math::normalizeAngle(observer_.getAngularPosition() + angular_velocity_ * (period * 0.5F));

        if (state_ != State::Spinup)
        {
            observer_.setDirectionConstraint(observer::DirectionConstraint::None);

            // Rotor stall detection
            if (remaining_time_before_stall_detection_enabled_ > 0)
            {
                // We've just entered the running mode, stall detection is temporarily suppressed
                remaining_time_before_stall_detection_enabled_ -= period;
            }
            else
            {
                // Stopping if the angular velocity is too low
                if (std::abs(angular_velocity_) < motor_params_.min_electrical_ang_vel)
                {
                    const bool reverse = isReversed();
                    const bool forward = !reverse;

                    if ((forward && !os::float_eq::positive(regular_setpoint_.value)) ||
                        (reverse && !os::float_eq::negative(regular_setpoint_.value)))
                    {
                        state_ = State::Stopped;    // Setpoint zeroed or flipped, this is a deliberate stop
                    }
                    else
                    {
                        state_ = State::Stalled;
                    }
                }
            }
        }
        else
        {
            observer_.setDirectionConstraint(isReversed() ?
                                             observer::DirectionConstraint::Reverse :
                                             observer::DirectionConstraint::Forward);

            spinup_time_ += period;

            Const spinup_fraction = spinup_time_ / controller_params_.nominal_spinup_duration;

            // TODO: Try voltage setpoint?
            spinup_setpoint_.mode = Setpoint::Mode::Iq;
            spinup_setpoint_.value = (isReversed() ? -1.0F : 1.0F) * motor_params_.spinup_current *
                                     math::Range<>(0.0F, 1.0F).constrain(spinup_fraction);

            regular_setpoint_ = spinup_setpoint_;

            if (std::abs(spinup_setpoint_.value) > motor_params_.min_current)
            {
                Const ang_vel_threshold = motor_params_.min_electrical_ang_vel * SpinupAngularVelocityHysteresis;
                if (std::abs(angular_velocity_) > ang_vel_threshold)
                {
                    // Running fast enough, switching to normal mode
                    state_ = State::Running;
                    remaining_time_before_stall_detection_enabled_ = spinup_time_ * 2.0F;
                }
            }

            if (spinup_fraction > MaximumSpinupDurationFraction)
            {
                state_ = State::Stalled;
            }
        }
    }

    /**
     * This method may be invoked concurrently with the state estimation update method from an IRQ (possibly nested).
     * Critical section is not used here.
     */
    Vector<3> updatePWMOutputsFromIRQ(const Vector<2>& phase_currents_ab,
                                      Const inverter_voltage) const
    {
        if (state_ == State::Spinup ||
            state_ == State::Running)
        {
            const auto sp = (state_ == State::Spinup) ? spinup_setpoint_ : regular_setpoint_;

            const auto output = modulator_.onNextPWMPeriod(phase_currents_ab,
                                                           inverter_voltage,
                                                           angular_velocity_,
                                                           angular_position_,
                                                           sp);
            Idq_ = output.Idq;
            reference_Udq_ = output.reference_Udq;
            angular_position_ = output.extrapolated_angular_position;

            return output.pwm_setpoint;
        }
        else
        {
            return Vector<3>::Zero();
        }
    }

    /**
     * Updating setpoint during spinup is meaningless, because the inner logic will overwrite it anyway.
     * Calling this method only makes sense if the state is Running.
     */
    void setSetpoint(const Setpoint& sp)
    {
        AbsoluteCriticalSectionLocker locker;
        regular_setpoint_ = sp;
    }

    Setpoint getSetpoint() const
    {
        AbsoluteCriticalSectionLocker locker;
        return regular_setpoint_;
    }

    State getState() const { return state_; }

    Vector<2> getUdq() const
    {
        AbsoluteCriticalSectionLocker locker;
        return reference_Udq_;
    }

    Vector<2> getIdq() const
    {
        AbsoluteCriticalSectionLocker locker;
        return Idq_;
    }

    Scalar getElectricalAngularVelocity() const
    {
        return angular_velocity_;   // No locking needed
    }

    Scalar computeInverterPower() const
    {
        AbsoluteCriticalSectionLocker locker;
        return (reference_Udq_.transpose() * Idq_)[0] * 1.5F;
    }

    Direction getDirection() const { return direction_; }

    DebugVariables getDebugVariables() const
    {
        AbsoluteCriticalSectionLocker locker;
        return {
            reference_Udq_[0],
            reference_Udq_[1],
            Idq_[0],
            Idq_[1],
            regular_setpoint_.value,
            observer_.getAngularVelocity()
        };
    }

    /**
     * Returns a MUTABLE REFERENCE to the observer object.
     * This is PRONE TO RACE CONDITIONS, make sure you know what you're doing.
     */
    observer::Observer& getObserver()
    {
        return observer_;
    }
};

}
