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

#include <foc/foc.hpp>
#include <zubax_chibios/util/heapless.hpp>
#include <utility>


namespace motor_database
{
/**
 * One named motor model.
 */
struct Entry
{
    static constexpr unsigned MaxNameLength = 40;

    using Name = os::heapless::String<MaxNameLength>;

    Name name;
    foc::MotorParameters parameters;

    Entry() { }

    Entry(const Name& arg_name,
          const foc::MotorParameters& arg_parameters) :
        name(arg_name),
        parameters(arg_parameters)
    { }

    bool isEmpty() const { return name.empty(); }

    auto toString() const
    {
        os::heapless::String<300> out;
        out.append("Name : ");
        out.append(name);
        out.append("\n");
        out.append(parameters.toString());
        return out;
    }
};

/**
 * Return the requested entry, or an empty entry if the requested one could not be located.
 * Note that the name based lookup is case-insensitive.
 */
Entry getByIndex(unsigned index);
Entry getByName(const Entry::Name& name);

}
