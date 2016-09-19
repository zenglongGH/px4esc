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
#include <zubax_chibios/util/heapless.hpp>


namespace foc
{
/**
 * Observer constants that are invariant to the motor model.
 * Model of the motor is defined separately.
 * All parameters here are set to reasonable default values.
 */
struct ObserverParameters
{
    math::DiagonalMatrix<4> Q  = math::makeDiagonalMatrix(100.0F,
                                                          100.0F,
                                                          5.0F,
                                                          1.0F);

    math::DiagonalMatrix<2> R  = math::makeDiagonalMatrix(2.0F,
                                                          2.0F);

    math::DiagonalMatrix<4> P0 = math::makeDiagonalMatrix(1000.0F,
                                                          1000.0F,
                                                          0.1F,      // We know that initial angular velocity is zero
                                                          5000.0F);  // We don't know initial angular position

    math::Scalar cross_coupling_compensation = 0.8F;

    auto toString() const
    {
        return os::heapless::format("Q diag : %s\n"
                                    "R diag : %s\n"
                                    "P0 diag: %s\n"
                                    "CC Comp: %.3f",
                                    math::toString(Q.diagonal()).c_str(),
                                    math::toString(R.diagonal()).c_str(),
                                    math::toString(P0.diagonal()).c_str(),
                                    double(cross_coupling_compensation));
    }
};

/**
 * Dmitry's ingenious observer.
 * Refer to the Simulink model for derivations.
 * All units are SI units (Weber, Henry, Ohm, Volt, Second, Radian).
 */
class Observer
{
public:
    enum class DirectionConstraint
    {
        None,
        Forward,
        Reverse
    };

private:
    math::Const phi_;
    math::Const ld_;
    math::Const lq_;
    math::Const r_;

    static constexpr unsigned StateIndexAngularVelocity = 2;
    static constexpr unsigned StateIndexAngularPosition = 3;

    math::Const cross_coupling_comp_;

    const math::Matrix<4, 4> Q_;
    const math::Matrix<2, 2> R_;

    const math::Matrix<2, 4> C_;

    DirectionConstraint direction_constraint_ = DirectionConstraint::None;

    // Filter states
    math::Vector<4> x_ = math::Vector<4>::Zero();
    math::Matrix<4, 4> P_;

public:
    Observer(const ObserverParameters& parameters,
             math::Const field_flux,
             math::Const stator_phase_inductance_direct,
             math::Const stator_phase_inductance_quadrature,
             math::Const stator_phase_resistance);

    void update(math::Const dt,
                const math::Vector<2>& idq,
                const math::Vector<2>& udq);

    void setDirectionConstraint(DirectionConstraint dc) { direction_constraint_ = dc; }

    math::Vector<2> getIdq() const { return x_.block<2, 1>(0, 0); }

    math::Scalar getAngularVelocity() const { return x_[StateIndexAngularVelocity]; }

    math::Scalar getAngularPosition() const { return x_[StateIndexAngularPosition]; }
};

}
