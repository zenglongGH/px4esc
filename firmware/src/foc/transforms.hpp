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


namespace foc
{
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
inline math::Vector<2> performClarkeTransform(const math::Vector<2>& ab)
{
    constexpr auto SquareRootOf3 = math::Scalar(1.7320508075688772);

    math::Const alpha = ab[0];
    math::Const beta = (ab[0] + ab[1] * 2.0F) / SquareRootOf3;

    return { alpha, beta };
}

/**
 * Park transform, assming increasing Theta during direct rotation.
 */
inline math::Vector<2> performParkTransform(const math::Vector<2>& alpha_beta,
                                            math::Const angle_sine,
                                            math::Const angle_cosine)
{
    return { alpha_beta[0] * angle_cosine + alpha_beta[1] * angle_sine,
            -alpha_beta[0] * angle_sine   + alpha_beta[1] * angle_cosine };
}

/**
 * Inverse Park transform to the above defined.
 */
inline math::Vector<2> performInverseParkTransform(const math::Vector<2>& dq,
                                                   math::Const angle_sine,
                                                   math::Const angle_cosine)
{
    return { dq[0] * angle_cosine - dq[1] * angle_sine,
             dq[0] * angle_sine   + dq[1] * angle_cosine };
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

}
