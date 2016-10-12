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

#include "motor_database.hpp"


namespace motor_database
{
namespace
{

using foc::MotorParameters;
using math::Scalar;

namespace common
{

constexpr Scalar SpinupDuration         = 2.0F;
constexpr Scalar MinElectricAngVel      = 200.0F;
constexpr unsigned NumStallsToLatch     = 10;

}

/**
 * Motor database entries go here.
 */
static const Entry g_entries[] =
{
    Entry("T-Motor MT2216-12", []()
    {
        MotorParameters p;
        p.nominal_spinup_duration = common::SpinupDuration;
        p.min_electrical_ang_vel  = common::MinElectricAngVel;
        p.min_current             = 0.3F;
        p.max_current             = 18.0F;
        p.current_ramp_amp_per_s  = 300.0F;
        p.phi                     = 1.06e-3F;
        p.rs                      = 0.11F;
        p.lq                      = 23e-6F;
        p.num_poles               = 14;
        p.num_stalls_to_latch     = common::NumStallsToLatch;
        return p;
    }),

    Entry("T-Motor U8-16", []()
    {
        MotorParameters p;
        p.nominal_spinup_duration = common::SpinupDuration;
        p.min_electrical_ang_vel  = common::MinElectricAngVel;
        p.min_current             = 0.5F;
        p.max_current             = 24.0F;
        p.current_ramp_amp_per_s  = 300.0F;
        p.phi                     = 3.938e-3F;
        p.rs                      = 0.1F;
        p.lq                      = 67e-6F;
        p.num_poles               = 28;
        p.num_stalls_to_latch     = common::NumStallsToLatch;
        return p;
    }),

    Entry("Maxon 339285", []()
    {
        MotorParameters p;
        p.nominal_spinup_duration = common::SpinupDuration;
        p.min_electrical_ang_vel  = common::MinElectricAngVel;
        p.min_current             = 0.2F;
        p.max_current             = 3.5F;
        p.current_ramp_amp_per_s  = 300.0F;
        p.phi                     = 1.814e-3F;
        p.rs                      = 0.232F;
        p.lq                      = 161e-6F;
        p.num_poles               = 16;
        p.num_stalls_to_latch     = common::NumStallsToLatch;
        return p;
    })
};


constexpr unsigned NumEntries = sizeof(g_entries) / sizeof(g_entries[0]);

}

Entry getByIndex(unsigned index)
{
    if (index < NumEntries)
    {
        return g_entries[index];
    }
    else
    {
        return Entry();
    }
}


Entry getByName(const Entry::Name& name)
{
    for (const auto& e : g_entries)
    {
        if (e.name.toLowerCase() == name.toLowerCase())
        {
            return e;
        }
    }
    return Entry();
}


unsigned getMaxIndex()
{
    return NumEntries - 1U;
}

}
