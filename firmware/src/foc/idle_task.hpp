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
 * Do-nothing task.
 */
class IdleTask : public ITask
{
    static constexpr math::Range<> BeepDurationLimits{0, 3.0F};
    static constexpr math::Range<> BeepFrequencyLimits{100.0F, 15000.0F};

    Status status_ = Status::Running;

    const CompleteParameterSet params_;

    Scalar remaining_beep_duration_ = 0;
    Scalar time_to_next_excitation_ = 0;
    Scalar excitation_period_ = 0;
    unsigned next_phase_index_ = 0;

public:
    IdleTask(const CompleteParameterSet& params) :
        params_(params)
    { }

    void onMainIRQ(Const period,
                   const board::motor::Status& hw_status) override
    {
        (void) period;

        if (!hw_status.isOkay())
        {
            status_ = Status::Failed;
        }
    }

    std::pair<Vector<3>, bool> onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                                               Const inverter_voltage) override
    {
        (void) phase_currents_ab;
        (void) inverter_voltage;

        // Beeping
        if (remaining_beep_duration_ > 0)
        {
            remaining_beep_duration_ -= params_.pwm.period;
            time_to_next_excitation_ -= params_.pwm.period;
            if (time_to_next_excitation_ <= 0)
            {
                time_to_next_excitation_ = excitation_period_;
                Vector<3> output = Vector<3>::Zero();
                output[next_phase_index_++ % 3] = 1.0F;
                return {output, true};
            }
        }

        return {Vector<3>::Zero(), false};
    }

    void beep(Const frequency,
              Const duration)
    {
        remaining_beep_duration_ = BeepDurationLimits.constrain(duration);
        excitation_period_ = 1.0F / BeepFrequencyLimits.constrain(frequency);
        time_to_next_excitation_ = excitation_period_;
    }

    Status getStatus() const override { return status_; }

    std::array<Scalar, NumDebugVariables> getDebugVariables() const override { return {}; }
};

}
