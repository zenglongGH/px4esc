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
     * Discrete transfer function from input setpoint to current setpoint.
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
                  Const electrical_angular_velocity)
    {
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

            // Constraining the minimums, only if the new setpoint and the reference are of the same sign
            if ((new_current > 0) == (target_setpoint > 0))
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

            // Constraining the minimums, only if the new setpoint and the reference are of the same sign
            if ((new_voltage > 0) == (target_setpoint > 0))
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
    constexpr unsigned IdqMovingAverageLength = 5;

    constexpr Scalar MinimumSpinupDurationFraction          = 0.1F;
    constexpr Scalar MaximumSpinupDurationFraction          = 1.5F;
    constexpr Scalar SpinupAngularVelocityHysteresis        = 3.0F;

    const CompleteParameterSet params_;

    Status status_ = Status::Running;

    Observer observer_;

    ThreePhaseVoltageModulator<IdqMovingAverageLength> modulator_;

    bool spinup_completed_ = false;

    SetpointController setpoint_controller_;

    Scalar angular_velocity_ = 0;                               ///< Radian per second, read in the fast IRQ
    Scalar angular_position_ = 0;                               ///< Radians [0, Pi*2]; extrapolated in the fast IRQ

    Vector<2> estimated_Idq_ = Vector<2>::Zero();               ///< Ampere, updated from the fast IRQ
    Vector<2> reference_Udq_ = Vector<2>::Zero();               ///< Volt, updated from the fast IRQ

    Scalar inverter_power_ = 0;

    Scalar remaining_time_before_stall_detection_enabled_ = 0;
    Scalar spinup_time_ = 0;

    Scalar setpoint_Iq_ = 0;
    Scalar setpoint_Uq_ = 0;
    bool use_voltage_setpoint_ = false;

    std::uint32_t num_successive_stalls_ = 0;

    ControlMode requested_control_mode_ = ControlMode(0);
    Scalar raw_setpoint_ = 0;
    Scalar setpoint_remaining_ttl_ = 0;         ///< Seconds left before the setpoint will be zeroed, unless updated

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

public:
    RunningTask(const CompleteParameterSet& params) :
        params_(params),

        observer_(params.observer,
                  params.motor.phi,
                  params.motor.lq,      // TODO: MOVE THE ASSUMPTION THAT Lq = Ld INTO THE OBSERVER
                  params.motor.lq,
                  params.motor.rs),

        modulator_(params.motor.lq,
                   params.motor.rs,
                   params.motor.max_current,
                   params.pwm,
                   modulator_.DeadTimeCompensationPolicy::Disabled,
                   modulator_.CrossCouplingCompensationPolicy::Disabled),

        setpoint_controller_(params.motor.max_current,
                             params.motor.min_current,
                             params.motor.computeMinVoltage(),
                             params.motor.current_ramp_amp_per_s,
                             params.motor.voltage_ramp_volt_per_s)
    {
        assert(params_.isValid());

        // Initializing the default setpoint
        requested_control_mode_ = ControlMode::Current;
        raw_setpoint_           = params.motor.min_current;
        setpoint_remaining_ttl_ = 1.0F;                     // Arbitrary
    }

    void setSetpoint(ControlMode control_mode,
                     Const value,
                     Const request_ttl)
    {
        AbsoluteCriticalSectionLocker locker;

        const bool zero_setpoint = os::float_eq::closeToZero(value);

        const bool sign_flip = ((raw_setpoint_ > 0) && (value < 0)) ||
                               ((raw_setpoint_ < 0) && (value > 0));

        if (zero_setpoint || sign_flip)
        {
            num_successive_stalls_ = 0;
        }

        requested_control_mode_ = control_mode;
        raw_setpoint_ = value;
        setpoint_remaining_ttl_ = request_ttl;
    }

    void onMainIRQ(Const period,
                   const board::motor::Status& hw_status) override
    {
        /*
         * Making a local copy of state variables ASAP, before the next fast IRQ fires.
         */
        const auto Idq = estimated_Idq_;
        const auto Udq = reference_Udq_;

        /*
         * Running the observer, this takes forever.
         * By the time the observer has finished, the rotor has moved some angle forward, which we compensate.
         */
        if (spinup_completed_)
        {
            observer_.setDirectionConstraint(Observer::DirectionConstraint::None);
        }
        else
        {
            observer_.setDirectionConstraint((raw_setpoint_ > 0) ?
                                             Observer::DirectionConstraint::Forward :
                                             Observer::DirectionConstraint::Reverse);
        }

        observer_.update(period, Idq, Udq);

        /*
         * Updating the state estimate.
         * Critical section is required because at this point we're no longer synchronized with the fast IRQ.
         */
        {
            AbsoluteCriticalSectionLocker locker;

            if (spinup_completed_)
            {
                angular_velocity_ = observer_.getAngularVelocity();

                // Correcting the angle estimation latency, assuming that the observer runs for about half period.
                Const angle_slip = angular_velocity_ * (period * 0.5F);
                angular_position_ =
                    math::normalizeAngle(observer_.getAngularPosition() + angle_slip);

                // Computing new setpoint using the appropriate control mode (current, voltage, RPM)
                if (requested_control_mode_ == ControlMode::RatiometricVoltage ||
                    requested_control_mode_ == ControlMode::Voltage)
                {
                    const auto max_voltage = computeLineVoltageLimit(board::motor::getInverterVoltage(),
                                                                     params_.pwm.upper_limit);
                    use_voltage_setpoint_ = true;
                    setpoint_Iq_ = estimated_Idq_[1];
                    setpoint_Uq_ = setpoint_controller_.update(period,
                                                               raw_setpoint_,
                                                               requested_control_mode_,
                                                               setpoint_Uq_,
                                                               max_voltage,
                                                               angular_velocity_);
                }
                else
                {
                    use_voltage_setpoint_ = false;
                    setpoint_Uq_ = reference_Udq_[1];
                    setpoint_Iq_ = setpoint_controller_.update(period,
                                                               raw_setpoint_,
                                                               requested_control_mode_,
                                                               setpoint_Iq_,
                                                               0.0F,
                                                               angular_velocity_);
                }

                // Rotor stall detection
                if (remaining_time_before_stall_detection_enabled_ > 0)
                {
                    // We've just entered the running mode, stall detection is temporarily suppressed
                    remaining_time_before_stall_detection_enabled_ -= period;
                }
                else
                {
                    // Stopping if the angular velocity is too low
                    if (std::abs(angular_velocity_) < params_.motor.min_electrical_ang_vel)
                    {
                        g_state = State::Idle;      // We'll possibly switch back to spinup from idle later
                        num_successive_stalls_++;
                    }
                }
            }
            else
            {
                // Change of direction while starting
                if ((raw_setpoint_ > 0) != (setpoint_Iq_ > 0))
                {
                    setpoint_Iq_ = 0.0F;
                    spinup_time_ = 0;
                    use_voltage_setpoint_ = false;
                }

                // Slowly increasing current
                if (std::abs(setpoint_Iq_) < params_.motor.spinup_current)
                {
                    Const current_delta =
                        (params_.motor.spinup_current / params_.controller.nominal_spinup_duration) * period;

                    setpoint_Iq_ += std::copysign(current_delta, raw_setpoint_);
                }

                // State update
                angular_velocity_ = observer_.getAngularVelocity();
                angular_position_ = observer_.getAngularPosition();
                spinup_time_ += period;

                // Checking if spinup is finished, switching to Running if so
                const bool spinup_in_progress_long_enough = spinup_time_ >
                    (params_.controller.nominal_spinup_duration * MinimumSpinupDurationFraction);

                const bool min_current_exceeded = std::abs(setpoint_Iq_) > params_.motor.min_current;

                if (spinup_in_progress_long_enough && min_current_exceeded)
                {
                    Const ang_vel_threshold = params_.motor.min_electrical_ang_vel * SpinupAngularVelocityHysteresis;
                    if (std::abs(angular_velocity_) > ang_vel_threshold)
                    {
                        // Running fast enough, switching to normal mode
                        g_state = State::Running;
                        remaining_time_before_stall_detection_enabled_ = spinup_time_ * 2.0F;
                    }
                }

                // Checking failure conditions
                Const spinup_timeout = params_.controller.nominal_spinup_duration * MaximumSpinupDurationFraction;
                if (spinup_time_ > spinup_timeout)
                {
                    // Timed out
                    raw_setpoint_ = 0;
                    num_successive_stalls_++;
                }
            }
        }

        /*
         * Non-time-critical stuff
         */
        inverter_power_ = (Udq.transpose() * Idq)[0] * 1.5F;

        LowPassFilteredValues::update(low_pass_filtered_values_.inverter_power,
                                      inverter_power_);

        LowPassFilteredValues::update(low_pass_filtered_values_.demand_factor,
                                      std::abs(Idq[1]) / params_.motor.max_current);
    }

    std::pair<Vector<3>, bool> onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                                               Const inverter_voltage) override
    {

        decltype(modulator_)::Setpoint setpoint;
        if (use_voltage_setpoint_)
        {
            setpoint.mode  = setpoint.Mode::Uq;
            setpoint.value = setpoint_Uq_;
        }
        else
        {
            setpoint.mode  = setpoint.Mode::Iq;
            setpoint.value = setpoint_Iq_;
        }

        const auto output = modulator_.onNextPWMPeriod(phase_currents_ab,
                                                       inverter_voltage,
                                                       angular_velocity_,
                                                       angular_position_,
                                                       setpoint);
        estimated_Idq_ = output.estimated_Idq;
        reference_Udq_ = output.reference_Udq;
        angular_position_ = output.extrapolated_angular_position;

        return {output.pwm_setpoint, true};
    }

    Status getStatus() const override { return status_; }

    std::array<Scalar, NumDebugVariables> getDebugVariables() const override
    {
        AbsoluteCriticalSectionLocker locker;

        std::array<Scalar, NumDebugVariables> out{
            reference_Udq_[0],
            reference_Udq_[1],
            estimated_Idq_[0],
            estimated_Idq_[1],
        };

        // TODO: Return Dmitry's formula back
        out[4] = use_voltage_setpoint_ ? setpoint_Uq_ : setpoint_Iq_;

        out[5] = observer_.getAngularVelocity();
        out[6] = spinup_completed_ ? (inverter_power_ / board::motor::getInverterVoltage()) : angular_velocity_;

        return out;
    }
};

}
