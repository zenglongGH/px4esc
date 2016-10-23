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
#include <math/math.hpp>


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
 *    performSVMTransform[{refAlpha_, refBeta_}, inverterVoltage_] :=
 *      Module[{Ualpha = Sqrt[3] refAlpha, Ubeta = refBeta}, X = Ubeta;
 *       Y = -((Ualpha + Ubeta)/2); Z = -((Ualpha - Ubeta)/2);
 *       sector = If[Y > 0, If[X > 0, 4, If[Z > 0, 3, 2]], If[X <= 0, 1, If[Z > 0, 5, 6]]];
 *       t1 = (-X - Y)/2; t2 = (X - Z)/2; t3 = (-Y - Z)/2;
 *       ta = {t1, t3, t2, t1, t3, t2}[[sector]]; tb = ta + Z; tc = ta + Y;
 *       Sqrt[3] {ta, tb, tc}/inverterVoltage + 0.5 // N];
 *
 *    inverterVoltage = 100;
 *    alphaBetaAmplitude = inverterVoltage/Sqrt[3];
 *    range = Range[0, 8, .01];
 *    refAlpha = Sin[#1]*alphaBetaAmplitude & /@ range;
 *    refBeta = Sin[#1 + Pi/2]*alphaBetaAmplitude & /@ range;
 *
 *    transforms = Map[performSVMTransform[#1, inverterVoltage] &, {refAlpha, refBeta}\[Transpose]];
 *    zero = Map[Total[#1]/3 &, transforms];
 *    phaseDiffAB = Map[#1[[1]] inverterVoltage - #1[[2]] inverterVoltage &, transforms];
 *
 *    GraphicsGrid[{
 *      {ListLinePlot[{refAlpha, refBeta}, GridLines -> Automatic,
 *        PlotRange -> {Automatic, {-inverterVoltage, inverterVoltage}},
 *        PlotLabel -> "\[Alpha]\[Beta] Voltage"],
 *       ListLinePlot[phaseDiffAB, GridLines -> Automatic,
 *        PlotRange -> {Automatic, {-inverterVoltage, inverterVoltage}},
 *        PlotLabel -> "Phase Difference A-B"]},
 *      {ListLinePlot[{transforms\[Transpose][[1]],
 *         transforms\[Transpose][[2]], transforms\[Transpose][[3]], zero},
 *        PlotRange -> {Automatic, {0, 1}}, GridLines -> Automatic,
 *        PlotLabel -> "PWM Setpoint"],
 *       ListLinePlot[
 *        inverterVoltage {transforms\[Transpose][[1]] - zero,
 *          transforms\[Transpose][[2]] - zero,
 *          transforms\[Transpose][[3]] - zero}, GridLines -> Automatic,
 *        PlotRange -> {Automatic, {-inverterVoltage, inverterVoltage}},
 *        PlotLabel -> "Phase Voltage"]}}]
 *
 * @param alpha_beta_voltage    Reference voltages alpha and beta.
 * @param inverter_voltage      Power stage supply voltage.
 * @return                      PWM setpoint vector (each component is in [0, 1]) and the index of the current
 *                              electrical sector in the range [0, 5].
 */
inline std::pair<Vector<3>, std::uint_fast8_t>
performSpaceVectorTransform(const Vector<2>& alpha_beta_voltage,
                            Const inverter_voltage)
{
    const auto ualpha = alpha_beta_voltage[0] * SquareRootOf3;
    const auto ubeta =  alpha_beta_voltage[1];

    const auto x = ubeta;
    const auto y = -(ualpha + ubeta) / 2.0F;
    const auto z = -(ualpha - ubeta) / 2.0F;

    const std::uint_fast8_t sector_index =
        (y > 0) ? ((x > 0) ? 3 : ((z > 0) ? 2 : 1)) : ((x <= 0) ? 0 : ((z > 0) ? 4 : 5));

    Scalar ta = 0;

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

    const Vector<3> raw_voltages { ta, ta + z, ta + y };

    const Vector<3> output = (SquareRootOf3 * raw_voltages / inverter_voltage).array() + 0.5F;

    return {output, sector_index};
}

/**
 * Accepts a PWM setpoint vector in [0, 1], returns corrected PWM setpoint.
 */
inline Vector<3> performDeadTimeCompensation(Vector<3> pwm_setpoint,
                                             const Vector<2>& phase_currents_ab,
                                             Const pwm_period,
                                             Const pwm_dead_time)
{
    static constexpr math::Range<> PWMRange(0.0F, 1.0F);

    Const currents[3] =
    {
        phase_currents_ab[0],
        phase_currents_ab[1],
       -phase_currents_ab.sum(),
    };

    Const correction = (pwm_dead_time / pwm_period) * 0.5F;

    for (unsigned i = 0; i < 3; i++)
    {
        pwm_setpoint[i] = PWMRange.constrain(pwm_setpoint[i] + std::copysign(correction, currents[i]));
    }

    return pwm_setpoint;
}

/**
 * Computes the maximum voltage that can be generated without exceeding the PWM limit.
 * Exceeding the PWM limit may cause the PWM modulation to obscure the ADC sampling intervals.
 */
inline Scalar computeLineVoltageLimit(Const inverter_voltage,
                                      Const max_pwm_value)
{
    assert((max_pwm_value >= 0.5F) && (max_pwm_value < 1.0F));
    return (inverter_voltage / SquareRootOf3) * max_pwm_value;
}

/**
 * Simplified Clarke transformation for balanced systems.
 * Overview: https://en.wikipedia.org/wiki/Alpha%E2%80%93beta_transformation
 *
 * Derivation from the full form (it looks ugly in plain text, use Mathematica):
 *
 *     FullSimplify[2/3 {{1, -(1/2), -(1/2)},
 *                       {0, \[Sqrt]3/2, -(\[Sqrt]3/2)},
 *                       {1/2, 1/2, 1/2}} . {{a},{b},{c}}, a + b + c == 0] // TraditionalForm
 *
 * Result:
 *
 *     a
 *     (b - c) / sqrt(3)
 *
 * Where C can be eliminated:
 *
 *     a
 *     (a + 2b) / sqrt(3)
 *
 * So the third component, if present, should be ignored.
 * Model:
 *
 *      performClarkeTransform[a_, b_] := {a, (a + 2 b)/\[Sqrt]3};
 */
inline Vector<2> performClarkeTransform(const Vector<2>& ab)
{
    Const alpha = ab[0];
    Const beta = (ab[0] + ab[1] * 2.0F) / SquareRootOf3;

    return { alpha, beta };
}

/**
 * Park transform, assming increasing Theta during direct rotation.
 */
inline Vector<2> performParkTransform(const Vector<2>& alpha_beta,
                                      const Vector<2>& angle_sincos)
{
    return { alpha_beta[0] * angle_sincos[1] + alpha_beta[1] * angle_sincos[0],
            -alpha_beta[0] * angle_sincos[0] + alpha_beta[1] * angle_sincos[1] };
}

/**
 * Inverse Park transform to the above defined.
 */
inline Vector<2> performInverseParkTransform(const Vector<2>& dq,
                                             const Vector<2>& angle_sincos)
{
    return { dq[0] * angle_sincos[1] - dq[1] * angle_sincos[0],
             dq[0] * angle_sincos[0] + dq[1] * angle_sincos[1] };
}

/* The code above was validated using the following script:

Theta = Range[0, 2 Pi, 0.017453292519943295];
currentLag = 10 \[Degree];
voltageAmplitude = 2;
currentAmplitude = 0.5;
phaseV = {Sin[Theta] voltageAmplitude, Sin[Theta - 120 \[Degree]] voltageAmplitude};
phaseI = {Sin[Theta - currentLag] currentAmplitude, Sin[Theta - 120 \[Degree] - currentLag] currentAmplitude};

IAlphaBeta = Map[performClarkeTransform[#1[[1]], #1[[2]]] &, phaseI\[Transpose]];
Idq = Map[performParkTransform[#1[[1]][[1]], #1[[1]][[2]], #1[[2]]] &, {IAlphaBeta, Theta}\[Transpose]];
Iinv = Map[performInverseParkTransform[#1[[1]][[1]], #1[[1]][[2]], #1[[2]]] &, {Idq, Theta}\[Transpose]];

ListLinePlot[{phaseV[[1]], phaseI[[1]]}, PlotLegends -> Automatic]
ListLinePlot[{IAlphaBeta\[Transpose][[1]], IAlphaBeta\[Transpose][[2]], Idq\[Transpose][[1]], Idq\[Transpose][[2]]},
             PlotLegends -> Automatic]
ListLinePlot[{Iinv\[Transpose][[1]], Iinv\[Transpose][[2]]}, PlotLegends -> Automatic]

 */

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

/**
 * Obviously, this function is equally applicable both to electrical and mechanical angular velocity.
 * @param radian_per_sec        Angular velocity in Rad/sec
 * @return                      Revolutions per minute
 */
constexpr inline Scalar convertAngularVelocityToRPM(Const radian_per_sec)
{
    return (radian_per_sec * 60.0F) / (math::Pi * 2.0F);
}

/**
 * This function is applicable to any quantity that measures the rotation rate.
 * @param rate          Rotation rate in any unit, e.g. Radian/sec, RPM, Hertz, etc.
 * @param num_poles     Number of magnetic poles in the rotor; positive, even.
 * @return              Scaled rotation rate in the same units.
 */
inline Scalar convertRotationRateElectricalToMechanical(Const rate,
                                                        const unsigned num_poles)
{
    if ((num_poles >= 2) &&
        (num_poles % 2 == 0))
    {
        return rate / Scalar(num_poles / 2U);
    }
    else
    {
        assert(false);
        return 0;
    }
}

}
