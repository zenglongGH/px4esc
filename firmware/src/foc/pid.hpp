/****************************************************************************
*
*   Copyright (C) 2016  Zubax Robotics  <info@zubax.com>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

#pragma once

#include <math/math.hpp>
#include <cassert>


namespace foc
{
/**
 * See @ref PIController.
 */
struct PIControllerSettings
{
    math::Scalar p = math::Scalar(0);
    math::Scalar i = math::Scalar(0);

    math::Range<math::Scalar> integration_limits;

    PIControllerSettings() { }

    PIControllerSettings(math::Const p,
                         math::Const i,
                         const math::Range<math::Scalar>& integration_limits) :
        p(p),
        i(i),
        integration_limits(integration_limits)
    {
        assert(std::isfinite(p) && std::isfinite(i));
        assert(integration_limits.contains(math::Scalar(0)));
    }

    PIControllerSettings(math::Const p,
                         math::Const i,
                         math::Const integration_limit) :
        PIControllerSettings(p, i,
                             {-std::abs(integration_limit),
                               std::abs(integration_limit)})
    { }
};

/**
 * See @ref PIDController.
 */
struct PIDControllerSettings : public PIControllerSettings
{
    math::Scalar d = math::Scalar(0);

    PIDControllerSettings() { }

    PIDControllerSettings(math::Const p,
                          math::Const i,
                          math::Const d,
                          const math::Range<math::Scalar>& integration_limits) :
        PIControllerSettings(p, i, integration_limits),
        d(d)
    {
        assert(std::isfinite(d));
    }

    PIDControllerSettings(math::Const p,
                          math::Const i,
                          math::Const d,
                          math::Const integration_limit) :
        PIDControllerSettings(p, i, d,
                              {-std::abs(integration_limit),
                                std::abs(integration_limit)})
    { }
};

/**
 * Non-polymorphic base for PI and PID controllers.
 */
template <typename Settings>
class PIControllerBase
{
protected:
    Settings cfg_;

    math::Scalar integral_ = math::Scalar(0);

    PIControllerBase() { }

    PIControllerBase(const Settings& settings) : cfg_(settings) { }

public:
    const Settings& getSettings() const { return cfg_; }

    void setSettings(const Settings& settings) { cfg_ = settings; }
};

/**
 * Trivial PID controller.
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
class PIDController : public PIControllerBase<PIDControllerSettings>
{
    math::Scalar prev_error_ = math::Scalar(0);

    bool initialized_ = false;

public:
    PIDController() { }

    explicit PIDController(const PIDControllerSettings& settings) :
        PIControllerBase<PIDControllerSettings>(settings)
    { }

    math::Scalar update(math::Const setpoint,
                        math::Const process_variable,
                        math::Const time_delta)
    {
        assert(time_delta > 0);

        math::Const error = setpoint - process_variable;

        if (!initialized_)
        {
            initialized_ = true;
            prev_error_ = error;
        }

        math::Const p = error * cfg_.p;
        math::Const d = (error - prev_error_) * cfg_.d / time_delta;

        // The I gain defines the speed of change of the integrated error, not its weight.
        // This enables us to change the I term at any moment without upsetting the output.
        integral_ = cfg_.integration_limits.constrain(integral_ + error * time_delta * cfg_.i);

        prev_error_ = error;

        return p + integral_ + d;
    }
};

}
