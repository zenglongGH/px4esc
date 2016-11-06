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

class FluxLinkageVarianceTask : public ISubTask
{
    // Acceleration during the measurement phase shold be very slow in order to reduce phase delay of the filters.
    static constexpr Scalar AccelerationDurationSec         = 40.0F;
    static constexpr Scalar InitialRelativeVoltageSetpoint  = 0.1F;     // Starting with 10% power
    static constexpr Scalar RelativeMinimumLowerPhi         = 0.5F;
    static constexpr Scalar DelayBetweenAttempts            = 0.5F;

    SubTaskContextReference context_;
    MotorParameters result_;

    Status status_ = Status::InProgress;

    Vector<2> Udq_ = Vector<2>::Zero();

    Scalar lower_phi_ = 0;
    Scalar lower_phi_current_ = 0;

    Scalar remaining_delay_ = DelayBetweenAttempts;

    volatile bool processing_enabled_ = false;

    os::helpers::LazyConstructor<MotorRunner, os::helpers::MemoryInitializationPolicy::NoInit> runner_;

    void constructRunner(Const phi)
    {
        runner_.destroy();

        auto ctl_params = context_.params.controller;
        ctl_params.voltage_modulator_cross_coupling_inductance_compensation = false;    // Forcing compensation OFF

        auto model = result_;
        model.phi = phi;

        runner_.construct(ctl_params,
                          model,
                          context_.params.observer,
                          context_.board.pwm,
                          MotorRunner::Direction::Forward);
    }

public:
    FluxLinkageVarianceTask(SubTaskContextReference context,
                            const MotorParameters& initial_parameters) :
        context_(context),
        result_(initial_parameters),
        lower_phi_(initial_parameters.phi)
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

            Udq_[0] = 0.0F;
            Udq_[1] = InitialRelativeVoltageSetpoint * hw_status.inverter_voltage;  // Back to the beginning

            constructRunner(lower_phi_);

            return;     // Can't do much else on this cycle, we don't want the IRQ to stretch forever
        }

        const Vector<2> Idq = runner_->getIdq();

        context_.reportDebugVariables({
            Udq_[0],
            Udq_[1],
            Idq[0],
            Idq[1],
            runner_->getElectricalAngularVelocity(),
            lower_phi_
        });

        Udq_[1] += (hw_status.inverter_voltage / AccelerationDurationSec) * period;
        if (Udq_[1] >= hw_status.inverter_voltage)
        {
            runner_.destroy();

            // TODO: Phi variance computation

            status_ = Status::Succeeded;
            return;
        }

        runner_->setSetpoint({ Udq_[1], MotorRunner::Setpoint::Mode::Uq });

        processing_enabled_ = true;

        runner_->updateStateEstimation(period, hw_status);

        Udq_[0] += period * 10.0F * (runner_->getUdq()[0] - Udq_[0]);

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

            lower_phi_ *= 0.96F;

            IRQDebugOutputBuffer::setStringPointerFromIRQ("Stalled; trying lower phi");
            IRQDebugOutputBuffer::setVariableFromIRQ<0>(result_.phi);
            IRQDebugOutputBuffer::setVariableFromIRQ<1>(lower_phi_);

            if (lower_phi_ / result_.phi < RelativeMinimumLowerPhi)
            {
                status_ = Status::Failed;   // Lower phi is too low
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
