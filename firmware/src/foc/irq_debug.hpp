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

#include <math/math.hpp>
#include <zubax_chibios/os.hpp>
#include <zubax_chibios/util/heapless.hpp>
#include <array>
#include <functional>


namespace foc
{

using math::Scalar;
using math::Const;

/**
 * Stores arbitrary values from IRQ to print them later into stdout.
 */
class IRQDebugOutputBuffer
{
public:
    using OutputCallback = std::function<void (const char*)>;

    static constexpr unsigned NumVariables = 5;

private:
    const char* string_ptr_ = nullptr;

    std::array<Scalar, NumVariables> vars_{};
    std::array<bool, NumVariables> update_flags_{};

    std::array<OutputCallback, 4> subscribers_;


    IRQDebugOutputBuffer(const IRQDebugOutputBuffer&) = delete;
    IRQDebugOutputBuffer& operator=(const IRQDebugOutputBuffer&) = delete;

    IRQDebugOutputBuffer() { }

    static IRQDebugOutputBuffer& getInstance()
    {
        static IRQDebugOutputBuffer inst;
        return inst;
    }

    static chibios_rt::Mutex& getMutex()
    {
        static chibios_rt::Mutex mu;
        return mu;
    }

    void broadcast(const os::heapless::String<>& s)
    {
        for (auto& x: subscribers_)
        {
            if (x)
            {
                x(s.c_str());
            }
        }
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
     * Alias without compile-time bounds check.
     */
    template <typename Value>
    static void setVariableFromIRQ(const unsigned index, const Value value)
    {
        auto& self = getInstance();
        self.vars_.at(index) = Scalar(value);
        self.update_flags_.at(index) = true;
    }

    /**
     * The name says it all. The string will NOT be copied, only the pointer will be stored.
     */
    static void setStringPointerFromIRQ(const char* const s)
    {
        getInstance().string_ptr_ = s;
    }

    /**
     * This function can be called only from a regular thread context.
     * Note that the method uses no locking, this is intentional.
     * There are corner cases where it may skip a value due to race condition, but this is acceptable.
     */
    static void poll()
    {
        os::MutexLocker locker(getMutex());

        auto& self = getInstance();

        if (const auto* s = self.string_ptr_)
        {
            self.string_ptr_ = nullptr;
            self.broadcast(s);
        }

        if (std::any_of(self.update_flags_.begin(), self.update_flags_.end(), [](bool x) { return x; }))
        {
            os::heapless::String<> s("Vars:");

            for (unsigned i = 0; i < NumVariables; i++)
            {
                if (self.update_flags_[i])
                {
                    self.update_flags_[i] = false;
                    s.concatenate("  ", i, ":", self.vars_[i]);
                }
            }

            self.broadcast(s);
        }
    }

    /**
     * Registers a method that will be invoked when a new message is generated.
     */
    static void addOutputCallback(const OutputCallback& cb)
    {
        os::MutexLocker locker(getMutex());

        for (auto& x: getInstance().subscribers_)
        {
            if (!x)
            {
                x = cb;
                return;
            }
        }

        DEBUG_LOG("COULD NOT INSTALL IRQ OUTPUT HANDLER\n");
        assert(false);
    }
};

/**
 * Stores quickly changing IRQ values and prints them into stdout.
 */
class IRQDebugPlotter
{
public:
    static constexpr unsigned NumVariables = 8;

private:
    std::array<Scalar, NumVariables> vars_ = {};

    mutable ::systime_t previous_time_systicks_ = 0;
    mutable std::uint64_t absolute_time_systicks_ = 0;

    Scalar getAbsoluteTimeInSeconds() const
    {
        // This is super wonky, should be improved someday
        const auto delta = chVTTimeElapsedSinceX(previous_time_systicks_);
        previous_time_systicks_ += delta;
        absolute_time_systicks_ += delta;
        return Scalar(absolute_time_systicks_) / Scalar(CH_CFG_ST_FREQUENCY);
    }

public:
    template <unsigned Index, typename Value>
    void set(const Value x)
    {
        static_assert(Index < NumVariables, "Debug variable index out of range");
        vars_[Index] = Scalar(x);
    }

    template <typename Container>
    void set(const Container cont)
    {
        std::copy_n(std::begin(cont), std::min(cont.size(), NumVariables), std::begin(vars_));
    }

    void print() const
    {
        std::array<Scalar, NumVariables> vars_copy;

        {
            AbsoluteCriticalSectionLocker locker;
            vars_copy = vars_;
        }

        std::printf("$%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    double(getAbsoluteTimeInSeconds()),
                    double(vars_copy[0]),
                    double(vars_copy[1]),
                    double(vars_copy[2]),
                    double(vars_copy[3]),
                    double(vars_copy[4]),
                    double(vars_copy[5]),
                    double(vars_copy[6]),
                    double(vars_copy[7]));
    }
};

}
