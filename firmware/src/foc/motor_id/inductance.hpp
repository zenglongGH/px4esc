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
 * Inductance estimation task.
 *
 * The Lq identification process assumes that the rotor is aligned in the quadrature axis.
 * Such alignment can be achieved by allowing the rotor to rotate freely while keeping any one phase
 * powered (excited) and the two other grounded.
 * Normally we would want to ensure the proper alignment here, by powering the phases in the right way,
 * and providing the rotor with sufficient time to turn into the right position; however, this is not
 * really necessary because the rotor will be already aligned as a side effect of the Rs measurement
 * process performed in the previous step.
 *
 * Furthermore, the inductance measurement process needs to know the Rs parameter for the purpose of
 * controlling the currents.
 *
 * Therefore, keep in mind that this operation is dependent on the Rs measurement procedure, and they
 * should be viewed holistically rather than as independent operations.
 */
class InductanceTask : public ISubTask
{
    static constexpr Scalar MeasurementDuration         = 15.0F;
    static constexpr Scalar MinValidSampleRatio         = 0.9F;
    static constexpr Scalar OneSizeFitsAllLq            = 50.0e-6F;
    static constexpr unsigned IdqMovingAverageLength    = 5;

    using Modulator = ThreePhaseVoltageModulator<IdqMovingAverageLength>;

    SubTaskContextReference context_;
    MotorParameters result_;

    Const estimation_current_;
    Const angular_velocity_;

    Scalar started_at_ = -1.0F;
    Status status_ = Status::InProgress;

    std::array<math::CumulativeAverageComputer<>, 3> averagers_;

    Modulator modulator_;
    Modulator::Output last_modulator_output_;

    Scalar angular_position_ = 0;

public:
    InductanceTask(SubTaskContextReference context,
                   const MotorParameters& initial_parameters) :
       context_(context),
       result_(initial_parameters),
       estimation_current_(initial_parameters.max_current * context.params.motor_id.fraction_of_max_current),
       angular_velocity_(context.params.motor_id.current_injection_frequency * math::Pi2),
       modulator_(OneSizeFitsAllLq,
                  result_.rs,
                  result_.max_current,
                  context.params.controller.voltage_modulator_bandwidth,
                  context.board.pwm,
                  Modulator::DeadTimeCompensationPolicy::Disabled,
                  Modulator::CrossCouplingCompensationPolicy::Disabled)
    {
       result_.lq = 0;

       if (!context_.params.motor_id.isValid() ||
           !result_.getRsLimits().contains(result_.rs) ||
           !os::float_eq::positive(result_.max_current))
       {
           status_ = Status::Failed;
       }
    }

    void onMainIRQ(Const period) override
    {
        (void) period;
        AbsoluteCriticalSectionLocker locker;
        context_.reportDebugVariables({
            last_modulator_output_.reference_Udq[0],
            last_modulator_output_.reference_Udq[1],
            last_modulator_output_.estimated_Idq[0],
            last_modulator_output_.estimated_Idq[1],
            Scalar(modulator_.getUdqNormalizationCounter())
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

        Const duration = context_.getTime() - started_at_;
        assert(duration >= 0);

        if (duration < MeasurementDuration)
        {
            Modulator::Setpoint modulator_setpoint;
            modulator_setpoint.mode = Modulator::Setpoint::Mode::Iq;
            modulator_setpoint.value = estimation_current_;

            last_modulator_output_ =
                modulator_.onNextPWMPeriod(phase_currents_ab,
                                           inverter_voltage,
                                           angular_velocity_,
                                           angular_position_,
                                           modulator_setpoint);
            angular_position_ = last_modulator_output_.extrapolated_angular_position;
            context_.setPWM(last_modulator_output_.pwm_setpoint);

            // TODO: Compensation disabled, since it yields lower values than expected
            Const dead_time_compensation_mult = 1.0F;
            //Const dead_time_compensation_mult = 1.0F - pwm_dead_time_ / pwm_period_;

            if (!last_modulator_output_.Udq_was_limited)
            {
                averagers_[0].addSample(last_modulator_output_.reference_Udq[0] * dead_time_compensation_mult);
                averagers_[1].addSample(last_modulator_output_.reference_Udq[1] * dead_time_compensation_mult);
                averagers_[2].addSample(last_modulator_output_.estimated_Idq[1]);
            }
        }
        else
        {
            const auto min_samples_needed =
                unsigned((MeasurementDuration / context_.board.pwm.period) * MinValidSampleRatio);
            const auto num_samples_acquired = averagers_[0].getNumSamples();

            assert(std::all_of(averagers_.begin(), averagers_.begin() + 3, [=](math::CumulativeAverageComputer<>& x) {
                return x.getNumSamples() == num_samples_acquired;
            }));

            IRQDebugOutputBuffer::setVariableFromIRQ<4>(num_samples_acquired);

            if (num_samples_acquired >= min_samples_needed)
            {
                Const Ud = Scalar(averagers_[0].getAverage());
                Const Uq = Scalar(averagers_[1].getAverage());
                Const Iq = Scalar(averagers_[2].getAverage());

                Const LqHF = std::abs(Ud / (angular_velocity_ * Iq));    // Normally we'd need to use only this formula

                Const RoverL = std::abs((angular_velocity_ * Uq) / Ud);  // This is a backup solution
                Const LqRoverL = result_.rs / RoverL;

                IRQDebugOutputBuffer::setVariableFromIRQ<0>(LqHF);
                IRQDebugOutputBuffer::setVariableFromIRQ<1>(LqRoverL);
                IRQDebugOutputBuffer::setVariableFromIRQ<2>(RoverL);

                if (MotorParameters::getLqLimits().contains(LqHF) &&
                    MotorParameters::getLqLimits().contains(LqRoverL))
                {
                    /*
                     * Measured values are valid.
                     * The LqHF formula is robust and tends to provide reliable results,
                     * whereas RoverL is highly dependent on frequency and current.
                     */
                    result_.lq = LqHF;
                    status_ = Status::Succeeded;
                }
                else
                {
                    // Measured values are invalid
                    result_.lq = 0;
                    status_ = Status::Failed;
                }
            }
            else
            {
                // Not enough valid samples collected
                result_.lq = 0;
                status_ = Status::Failed;
            }
        }
    }

    Status getStatus() const override { return status_; }

    MotorParameters getEstimatedMotorParameters() const override { return result_; }
};

}
}
