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
#include "../motor_runner.hpp"
#include <zubax_chibios/util/helpers.hpp>


namespace foc
{
namespace motor_id
{

class FineTuningTask : public ISubTask
{
    static constexpr Scalar AccelerationDurationSec         = 10.0F;
    static constexpr Scalar InitialRelativeSetpoint         = 0.1F;     // Starting with 10% power
    static constexpr Scalar RelativeMinimumLowerPhi         = 0.6F;
    static constexpr Scalar PhiReductionPerAttempt          = 0.03F;
    static constexpr Scalar DelayBetweenAttempts            = 1.0F;

    SubTaskContextReference context_;
    MotorParameters result_;

    Status status_ = Status::InProgress;

    Scalar remaining_delay_ = DelayBetweenAttempts;

    volatile bool processing_enabled_ = false;

    os::helpers::LazyConstructor<MotorRunner, os::helpers::MemoryInitializationPolicy::NoInit> runner_;

    MotorRunner::Setpoint setpoint_;

    int remaining_attempts_ = 10;

    void constructRunner()
    {
        runner_.destroy();

        auto ctl_params = context_.params.controller;
        ctl_params.voltage_modulator_cross_coupling_inductance_compensation = false;    // Forcing compensation OFF

        runner_.construct(ctl_params,
                          result_,
                          context_.params.observer,
                          context_.board.pwm,
                          MotorRunner::Direction::Forward);
    }

public:
    FineTuningTask(SubTaskContextReference context,
                   const MotorParameters& initial_parameters) :
        context_(context),
        result_(initial_parameters)
    {
        if (!context_.params.motor_id.isValid() ||
            !result_.getRsLimits().contains(result_.rs) ||
            !result_.getLqLimits().contains(result_.lq) ||
            !result_.getPhiLimits().contains(result_.phi) ||
            !os::float_eq::positive(result_.max_current))
        {
            status_ = Status::Failed;
        }
    }

    void onMainIRQ(Const period, const board::motor::Status& hw_status) override
    {
        if (remaining_delay_ > 0)
        {
            context_.reportDebugVariables({});
            remaining_delay_ -= period;
            return;
        }

        if (!runner_.isConstructed())
        {
            processing_enabled_ = false;
            context_.reportDebugVariables({});

            setpoint_.mode = setpoint_.Mode::Iq;
            setpoint_.value = InitialRelativeSetpoint * result_.max_current;  // Back to the beginning

            constructRunner();

            return;     // Can't do much else on this cycle, we don't want the IRQ to stretch forever
        }

        {
            const Vector<2> Udq = runner_->getUdq();
            const Vector<2> Idq = runner_->getIdq();

            context_.reportDebugVariables({
                Udq[0],
                Udq[1],
                Idq[0],
                Idq[1],
                runner_->getElectricalAngularVelocity(),
                result_.phi * 1e3F
            });
        }

        setpoint_.value += (result_.max_current / AccelerationDurationSec) * period;
        if (setpoint_.value >= result_.max_current)
        {
            runner_.destroy();
            status_ = Status::Succeeded;
            return;
        }
        runner_->setSetpoint(setpoint_);

        processing_enabled_ = true;
        runner_->updateStateEstimation(period, hw_status);

        AbsoluteCriticalSectionLocker locker;

        switch (runner_->getState())
        {
        case MotorRunner::State::Spinup:
        case MotorRunner::State::Running:
        {
            break;
        }

        case MotorRunner::State::Stopped:
        {
            assert(false);
            status_ = Status::Failed;       // This is impossible, we don't command the runner to stop
            break;
        }

        case MotorRunner::State::Stalled:
        {
            runner_.destroy();
            remaining_delay_ = DelayBetweenAttempts;

            if (remaining_attempts_ == 0)
            {
                status_ = Status::Failed;
            }
            else
            {
                remaining_attempts_--;

                result_.phi -= result_.phi * PhiReductionPerAttempt;

                IRQDebugOutputBuffer::setStringPointerFromIRQ("Stalled; trying lower phi");
                IRQDebugOutputBuffer::setVariableFromIRQ<0>(result_.phi);
            }
            break;
        }
        }
    }

    void onNextPWMPeriod(const Vector<2>& phase_currents_ab, Const inverter_voltage) override
    {
        if (status_ != Status::InProgress ||
            !processing_enabled_)
        {
            return;
        }

        if (runner_.isConstructed())
        {
            const auto output = runner_->updatePWMOutputsFromIRQ(phase_currents_ab, inverter_voltage);
            context_.setPWM(output);
        }
    }

    Status getStatus() const override { return status_; }

    MotorParameters getEstimatedMotorParameters() const override { return result_; }
};

}
}
