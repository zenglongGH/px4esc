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
#include <board/motor.hpp>
#include <zubax_chibios/util/heapless.hpp>


namespace foc
{

using math::Scalar;
using math::Const;
using math::Vector;
using board::motor::AbsoluteCriticalSectionLocker;


constexpr Scalar SquareRootOf3 = Scalar(1.7320508075688772);

/**
 * Simple counter with limited access to the variable for extra paranoia.
 */
class EventCounter
{
    std::uint64_t cnt_ = 0;

public:
    void increment() { cnt_++; }

    std::uint64_t get() const { return cnt_; }

    auto toString() const { return os::heapless::intToString(cnt_); }
};

/**
 * Constrains the angle within [0, Pi*2]
 */
inline math::Scalar constrainAngularPosition(math::Const x)
{
    constexpr math::Const Pi2 = math::Pi * 2.0F;

    if (x >= Pi2)
    {
        return x - Pi2;
    }
    else if (x < 0)
    {
        return x + Pi2;
    }
    else
    {
        return x;
    }
}

/**
 * @param flux_linkage  Positive, in Weber.
 * @param num_poles     Positive, even.
 * @return              KV if inputs are valid; zero and assertion failure if not.
 */
inline Scalar convertFluxLinkageToKV(Const flux_linkage,
                                     const unsigned num_poles)
{
    if ((flux_linkage > 0) &&
        (num_poles >= 2) &&
        (num_poles % 2 == 0))
    {
        return (20.0F * SquareRootOf3) / (math::Pi * flux_linkage * Scalar(num_poles));
    }
    else
    {
        assert(false);
        return 0;
    }
}

/**
 * @param kv            Positive, in MRPM/V; MRPM is mechanical RPM.
 * @param num_poles     Positive, even.
 * @return              Field flux linkage if inputs are valid; zero and assertion failure if not.
 */
inline Scalar convertKVToFluxLinkage(Const kv,
                                     const unsigned num_poles)
{
    if ((kv > 0) &&
        (num_poles >= 2) &&
        (num_poles % 2 == 0))
    {
        return (20.0F * SquareRootOf3) / (math::Pi * kv * Scalar(num_poles));
    }
    else
    {
        assert(false);
        return 0;
    }
}

}
