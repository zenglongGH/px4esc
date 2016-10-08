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
/**
 * Motor database entries go here.
 */
static const Entry g_entries[] =
{
    //               nominal_spinup_duration min_current spinup_current     phi                  num_spinup_attempts
    //                    min_electrical_ang_vel | max_current | current_ramp_amp_per_s rs     lq num_poles
    Entry("T-Motor MT2216-12",  {0.5F, 100.0F, 0.3F, 18.0F,  0.0F, 30.0F, 1.125e-3F, 0.110F,  23e-6F, 14, 10}),
    Entry("T-Motor U8-16",      {0.5F, 100.0F, 0.5F, 24.0F,  0.0F, 30.0F, 3.938e-3F, 0.100F,  67e-6F, 28, 10}),
    Entry("Maxon 339285",       {0.5F, 100.0F, 0.1F,  3.5F,  0.0F, 30.0F, 1.814e-3F, 0.232F, 161e-6F, 16, 10}),
    Entry("XAircraft 650",      {0.5F, 100.0F, 0.3F, 18.0F,  0.0F, 30.0F, 1.826e-3F, 0.070F,  25e-6F, 14, 5})
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
