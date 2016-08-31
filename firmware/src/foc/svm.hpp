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
#include <cstdint>
#include <cassert>
#include <utility>


namespace foc
{
/**
 * Space Vector PWM modulation.
 * Refer to the Dmitry's documents for theory.
 * Basics: https://en.wikipedia.org/wiki/Space_vector_modulation
 * Inspired by (this implementation is broken): http://blog.chinaaet.com/282280072/p/20721
 *
 * The following code was used for simulation:
 *
 *     performSVMTransform[{refAlpha_, refBeta_}] :=
 *       Module[{Ualpha = Sqrt[3] refAlpha, Ubeta = refBeta},
 *        X = Ubeta; Y = -((Ualpha + Ubeta)/2); Z = -((Ualpha - Ubeta)/2);
 *        sector = If[Y > 0,
 *          If[X > 0, 4, If[Z > 0, 3, 2]],
 *          If[X <= 0, 1, If[Z > 0, 5, 6]]];
 *        t1 = (-X - Y)/2; t2 = (X - Z)/2; t3 = (-Y - Z)/2;
 *        ta = {t1, t3, t2, t1, t3, t2}[[sector]]; tb = ta + Z; tc = ta + Y;
 *        {ta, tb, tc} // N];
 *     normalizeSVMPhaseVoltages[svmVector_, inverterVoltage_] := svmVector/inverterVoltage + 0.5;
 *     inverterVoltage = 100;
 *     range = Range[0, 8, .01];
 *     refAlpha = Sin[#1]*inverterVoltage & /@ range;
 *     refBeta = Sin[#1 + Pi/2]*inverterVoltage & /@ range;
 *     ListLinePlot[{refAlpha, refBeta}]
 *     transforms =
 *       Map[normalizeSVMPhaseVoltages[performSVMTransform[#1/2], inverterVoltage] &, {refAlpha, refBeta}\[Transpose]];
 *     ListLinePlot[transforms\[Transpose], PlotRange -> {Automatic, {0, 1}}]
 *
 * @param reference_voltage     Reference voltages alpha and beta.
 * @return                      Phase voltages centered around zero and the index of the current
 *                              electrical sector in the range [0, 5].
 */
inline std::pair<math::Vector<3>, std::uint_fast8_t>
performSpaceVectorTransform(const math::Vector<2>& reference_voltage)
{
    constexpr auto SquareRootOf3 = math::Scalar(1.7320508075688772);

    const auto ualpha = reference_voltage[0] * SquareRootOf3;
    const auto ubeta =  reference_voltage[1];

    const auto x = ubeta;
    const auto y = -(ualpha + ubeta) / 2.0F;
    const auto z = -(ualpha - ubeta) / 2.0F;

    const std::uint_fast8_t sector_index =
        (y > 0) ? ((x > 0) ? 3 : ((z > 0) ? 2 : 1)) : ((x <= 0) ? 0 : ((z > 0) ? 4 : 5));

    math::Scalar ta = 0;

    switch (sector_index)
    {
    case 0:
    case 3:
    {
        ta = (-x - y) / 2.0F;
        break;
    }
    case 1:
    case 4:
    {
        ta = (-y - z) / 2.0F;
        break;
    }
    case 2:
    case 5:
    {
        ta = (x - z) / 2.0F;
        break;
    }
    default:
    {
        assert(false);
    }
    }

    return {{ta, ta + z, ta + y}, sector_index};
}

/**
 * Converts the output of @ref performSpaceVectorTransform() to a vector that can be fed into the PWM driver.
 *
 * @param phase_voltages                Zero centered phase voltages.
 * @param inverter_voltage              Power stage supply voltage.
 * @return                              PWM setpoint vector; each component is in [0, 1].
 */
inline math::Vector<3> normalizePhaseVoltagesToPWMSetpoint(const math::Vector<3>& phase_voltages,
                                                           math::Const inverter_voltage)
{
    return (phase_voltages / inverter_voltage).array() + 0.5F;
}

}
