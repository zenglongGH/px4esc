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

class MagneticFluxTask : public ISubTask
{
    // Voltage reduction during the measurement phase shold be very slow in order to
    // reduce phase delay of the current filter.
    static constexpr Scalar VoltageSlopeLengthSec       = 40.0F;
    static constexpr Scalar MinVoltage                  = 0.001F;
    static constexpr unsigned IdqMovingAverageLength    = 5;

    using Modulator = ThreePhaseVoltageModulator<IdqMovingAverageLength>;

    SubTaskContextReference context_;
    MotorParameters result_;

    Const initial_Uq_;

    Scalar started_at_ = -1.0F;
    Status status_ = Status::InProgress;

    std::array<math::CumulativeAverageComputer<>, 3> averagers_;

    Modulator modulator_;

    math::SimpleMovingAverageFilter<500, Vector<2>> currents_filter_;
    math::SimpleMovingAverageFilter<500, Vector<2>> voltage_filter_;

    // State variables
    Scalar angular_velocity_ = 0;
    Scalar angular_position_ = 0;
    Scalar Uq_;
    Scalar min_I_ = 0;
    Scalar I_ = 0;
    Scalar U_ = 0;
    Scalar phi_ = 0;

public:
    MagneticFluxTask(SubTaskContextReference context,
                     const MotorParameters& initial_parameters) :
        context_(context),
        result_(initial_parameters),
        initial_Uq_((initial_parameters.max_current * context.params.motor_id.fraction_of_max_current) *
                    result_.rs * 1.5F),
        modulator_(result_.lq,
                   result_.rs,
                   result_.max_current,
                   context.board.pwm,
                   Modulator::DeadTimeCompensationPolicy::Disabled,
                   Modulator::CrossCouplingCompensationPolicy::Disabled),
        currents_filter_(Vector<2>::Zero()),
        voltage_filter_(Vector<2>::Zero()),
        Uq_(initial_Uq_)
    {
        result_.phi = 0;

        if (!context_.params.motor_id.isValid() ||
            !result_.getRsLimits().contains(result_.rs) ||
            !result_.getLqLimits().contains(result_.lq) ||
            !os::float_eq::positive(result_.max_current))
        {
            status_ = Status::Failed;
        }
    }

    void onMainIRQ(Const period) override
    {
        (void) period;
        context_.reportDebugVariables({
            angular_velocity_,
            Uq_,
            I_,
            U_,
            phi_ * 1e3F
        });
    }

    void onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                         Const inverter_voltage) override
    {
        if (status_ != Status::InProgress)
        {
            return;
        }

        if (started_at_ < 0)
        {
            started_at_ = context_.getTime();
        }

        Const low_pass_filter_innovation = context_.board.pwm.period * 10.0F;

        Const prev_I = I_;

        /*
         * Voltage modulation and filter update
         */
        if (Uq_ > MinVoltage)
        {
            Modulator::Setpoint setpoint;
            setpoint.mode = Modulator::Setpoint::Mode::Uq;
            setpoint.value = Uq_;

            const auto out = modulator_.onNextPWMPeriod(phase_currents_ab,
                                                        inverter_voltage,
                                                        angular_velocity_,
                                                        angular_position_,
                                                        setpoint);
            context_.setPWM(out.pwm_setpoint);
            angular_position_ = out.extrapolated_angular_position;

            // Current filter update
            currents_filter_.update(out.estimated_Idq);
            I_ += low_pass_filter_innovation * (currents_filter_.getValue().norm() - I_);

            // Voltage filter update
            voltage_filter_.update(out.reference_Udq);
            U_ += low_pass_filter_innovation * (voltage_filter_.getValue().norm() - U_);
        }
        else
        {
            // Voltage is not in the valid range, aborting
            result_.phi = 0;
            status_ = Status::Failed;
            return;
        }

        /*
         * Acceleration/measurement
         */
        if (angular_velocity_ < context_.params.motor_id.phi_estimation_electrical_angular_velocity)
        {
            // Acceleration
            angular_velocity_ += (context_.params.motor_id.phi_estimation_electrical_angular_velocity /
                                  (VoltageSlopeLengthSec / 2.0F)) * context_.board.pwm.period;
        }
        else
        {
            {
                // TODO: Compensation disabled, since it yields lower values than expected
                Const dead_time_compensation_mult = 1.0F;
                //Const dead_time_compensation_mult = 1.0F - pwm_dead_time_ / pwm_period_;

                Const U = U_ * dead_time_compensation_mult;

                Const new_phi = (U - I_ * result_.rs) / angular_velocity_;

                if (phi_ > 0)
                {
                    phi_ += low_pass_filter_innovation * (new_phi - phi_);
                }
                else
                {
                    phi_ = new_phi;
                }
            }

            if ((I_ < min_I_) || (min_I_ <= 0))
            {
                min_I_ = I_;
                result_.phi = phi_;
            }

            if (result_.phi >= 0.0F)
            {
                Const dIdt = (I_ - prev_I) / context_.board.pwm.period;

                // TODO: we could automatically learn the worst case di/dt after the acceleration phase?
                // 2 - triggers false positive
                // 3 - works fine
                // 6 - works fine
                Const dIdt_threshold = prev_I * 3.0F;

                if (dIdt > dIdt_threshold)
                {
                    status_ = Status::Succeeded;
                }
                else
                {
                    // Minimum is not reached yet, continuing to reduce voltage
                    Uq_ -= (initial_Uq_ / VoltageSlopeLengthSec) * context_.board.pwm.period;
                }
            }
            else
            {
                // Negative Phi value encountered at any point indicates that the Rs value is likely too high
                status_ = Status::Failed;
            }
        }
    }

    Status getStatus() const override { return status_; }

    MotorParameters getEstimatedMotorParameters() const override { return result_; }
};

}
}
