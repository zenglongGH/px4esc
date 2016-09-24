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
#include <array>


namespace foc
{

class IRQDebugOutputBuffer
{
    static constexpr unsigned NumVariables = 5;

    std::array<Scalar, NumVariables> vars_{};
    std::array<bool, NumVariables> update_flags_{};


    IRQDebugOutputBuffer(const IRQDebugOutputBuffer&) = delete;
    IRQDebugOutputBuffer& operator=(const IRQDebugOutputBuffer&) = delete;

    IRQDebugOutputBuffer() { }

    static IRQDebugOutputBuffer& getInstance()
    {
        static IRQDebugOutputBuffer inst;
        return inst;
    }

public:
    /**
     * This function can be called from IRQ that wishes to display a value.
     */
    template <unsigned Index, typename Value>
    static void setVariableFromIRQ(const Value value)
    {
        static_assert(Index < NumVariables, "Index out of range");
        auto& self = getInstance();
        self.vars_[Index] = Scalar(value);
        self.update_flags_[Index] = true;
    }

    /**
     * This function can be called only from a regular thread context.
     * Note that the method uses no locking, this is intentional.
     * There are corner cases where it may skip a value due to race condition, but this is acceptable.
     */
    static void printIfNeeded()
    {
        auto& self = getInstance();
        if (std::any_of(self.update_flags_.begin(), self.update_flags_.end(), [](bool x) { return x; }))
        {
            std::printf("IRQ Vars:");
            for (unsigned i = 0; i < NumVariables; i++)
            {
                const bool updated = self.update_flags_[i];
                const auto value = self.vars_[i];
                self.update_flags_[i] = false;

                std::printf("   %u/%s: %g", i, updated ? "new" : "old", double(value));
            }
            std::puts("");
        }
    }
};

}
