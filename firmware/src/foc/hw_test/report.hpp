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

#include <cstdint>
#include <zubax_chibios/util/heapless.hpp>


namespace foc
{
namespace hw_test
{
/**
 * Output of the hardware testing task.
 */
class Report
{
    friend class HardwareTestingTask;

public:
    using Mask = std::uint16_t;

private:
    Mask mask_ = 0;

public:
    enum class ErrorFlag
    {
        InverterVoltageSensorError,
        InverterTemperatureSensorError,
        CurrentSensorsZeroOffsetError,
        PhaseAError,
        PhaseBError,
        PhaseCError,
        MotorNotConnected,
        InverterOverloadSignal,
        InverterFaultSignal
    };

    static constexpr Mask flag2mask(const ErrorFlag f) { return Mask(1U << unsigned(f)); }

    Mask getErrorMask() const { return mask_; }

    unsigned getNumberOfErrors() const
    {
        unsigned out = 0;
        for (unsigned i = 0; i < 32; i++)
        {
            if ((mask_ & (1U << i)) != 0)
            {
                out++;
            }
        }
        return out;
    }

    bool isSuccessful() const { return getErrorMask() == 0U; }

    auto toString() const
    {
        return os::heapless::format("NumErrors: %u, Code: 0x%04x 0b%s",
                                    getNumberOfErrors(),
                                    unsigned(mask_),
                                    os::heapless::intToString<2>(mask_).c_str());
    }
};

}
}
