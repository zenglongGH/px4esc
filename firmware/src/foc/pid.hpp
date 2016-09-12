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
#include <cassert>


namespace foc
{
/**
 * Trivial PI controller.
 * Simulation:
 *
 *     p = 0.1;
 *     i = 0.1;
 *     d = 0.1;
 *     iLim = {-1, 0.1};
 *     iConstrain[x_] := Min[Max[x, iLim[[1]]], iLim[[2]]];
 *     integral = 0;
 *     prevError = 0;
 *     update[sp_, pv_, dt_] := Module[{error = sp - pv},
 *        Module[{p = error p, d = ((error - prevError) d)/dt},
 *         integral = iConstrain[integral + error*dt*i];
 *         prevError = error;
 *         p + integral + d]];
 *     xrange = Range[0, 20, 0.01];
 *     setpoints = Sin /@ xrange;
 *     procvar = Cos /@ xrange;
 *     outputs = update[#1, #2, 0.1] & @@@ ({setpoints, procvar}\[Transpose]);
 *     ListLinePlot[{setpoints, procvar, outputs}, PlotLegends -> Automatic]
 */
class ParallelPIController
{
    math::Const kp_;
    math::Const ki_;

    const math::Range<math::Scalar> integral_limits_;

    math::Scalar integral_ = 0;

public:
    ParallelPIController(math::Const p,
                         math::Const i,
                         const math::Range<math::Scalar>& integration_limits) :
        kp_(p),
        ki_(i),
        integral_limits_(integration_limits)
    {
        assert(p > 0);
        assert(i > 0);
        assert(integral_limits_.contains(0));
    }

    math::Scalar update(math::Const setpoint,
                        math::Const process_variable,
                        math::Const time_delta)
    {
        assert(time_delta > 0);

        math::Const error = setpoint - process_variable;

        math::Const p = error * kp_;

        // The I gain defines the speed of change of the integrated error, not its weight.
        // This enables us to change the I term at any moment without upsetting the output.
        integral_ = integral_limits_.constrain(integral_ + error * time_delta * ki_);

        return p + integral_;
    }
};

}
