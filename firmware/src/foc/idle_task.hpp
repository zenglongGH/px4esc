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
class IdleTask : public ITask
{
    Status status_ = Status::Running;

public:
    IdleTask(const TaskContext& context)
    {
        if (!context.params.isValid() ||
            !context.hw_test_report.isSuccessful())
        {
            status_ = Status::Failed;
        }
    }

    const char* getName() const override { return "idle"; }

    void onMainIRQ(Const period,
                   const board::motor::Status& hw_status) override
    {
        (void) period;

        if (!hw_status.power_ok)
        {
            status_ = Status::Failed;
        }
    }

    std::pair<Vector<3>, bool> onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                                               Const inverter_voltage) override
    {
        (void) phase_currents_ab;
        (void) inverter_voltage;

        return {Vector<3>::Zero(), false};
    }

    Status getStatus() const override { return status_; }
};

}
