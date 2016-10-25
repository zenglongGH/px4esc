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


namespace foc
{
/**
 * Do-nothing task.
 */
class BeepingTask : public ITask
{
    static constexpr math::Range<> DurationLimits{0, 3.0F};
    static constexpr math::Range<> FrequencyLimits{100.0F, 15000.0F};

    static constexpr FailureCode FailureCodeBadHardwareStatus   = 1;

    Status status_ = Status::Running;
    FailureCode failure_code_ = 0;

    const TaskContext context_;

    Const excitation_period_ = 0;

    Scalar remaining_duration_ = 0;
    Scalar time_to_next_excitation_ = 0;
    unsigned next_phase_index_ = 0;

public:
    BeepingTask(const TaskContext& context,
                Const frequency,
                Const duration) :
        context_(context),
        excitation_period_(1.0F / FrequencyLimits.constrain(frequency)),
        remaining_duration_(DurationLimits.constrain(duration)),
        time_to_next_excitation_(excitation_period_)
    { }

    const char* getName() const override { return "beep"; }

    void onMainIRQ(Const period,
                   const board::motor::Status& hw_status) override
    {
        (void) period;

        if (!hw_status.power_ok)
        {
            status_ = Status::Failed;
            failure_code_ = FailureCodeBadHardwareStatus;
        }
    }

    std::pair<Vector<3>, bool> onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                                               Const inverter_voltage) override
    {
        (void) phase_currents_ab;
        (void) inverter_voltage;

        // Beeping
        if (remaining_duration_ > 0)
        {
            remaining_duration_ -= context_.board.pwm.period;
            time_to_next_excitation_ -= context_.board.pwm.period;
            if (time_to_next_excitation_ <= 0)
            {
                time_to_next_excitation_ += excitation_period_;
                Vector<3> output = Vector<3>::Zero();
                output[next_phase_index_++ % 3] = 1.0F;
                return {output, true};
            }
            else
            {
                return {Vector<3>::Zero(), true};
            }
        }
        else
        {
            status_ = Status::Finished;
            return {Vector<3>::Zero(), false};
        }
    }

    Status getStatus() const override { return status_; }

    FailureCode getFailureCode() const override { return failure_code_; }
};

}
