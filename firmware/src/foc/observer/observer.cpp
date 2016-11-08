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

#include "observer.hpp"
#include <zubax_chibios/os.hpp>


namespace foc
{
namespace observer
{

using math::makeMatrix;
using math::makeDiagonalMatrix;
using math::makeRow;


Observer::Observer(const Parameters& parameters,
                   Const flux_linkage,
                   Const stator_phase_inductance_direct,
                   Const stator_phase_inductance_quadrature,
                   Const stator_phase_resistance) :
    // Motor model
    phi_(flux_linkage),
    ld_(stator_phase_inductance_direct),
    lq_(stator_phase_inductance_quadrature),
    r_(stator_phase_resistance),

    // Filter constants
    cross_coupling_comp_(parameters.cross_coupling_compensation),
    C_(makeMatrix(makeRow(1, 0, 0, 0),
                  makeRow(0, 1, 0, 0))),

    // Filter state
    P_(parameters.P0),
    Q_(parameters.Q),
    R_(parameters.R),

    z_covariance_(Matrix<2, 2>::Identity())
{
    ASSERT_ALWAYS(os::float_eq::positive(phi_)  &&
                  os::float_eq::positive(ld_)   &&
                  os::float_eq::positive(lq_)   &&
                  os::float_eq::positive(r_)    &&
                  !os::float_eq::negative(cross_coupling_comp_));
}


void Observer::update(Const dt,
                      const Vector<2>& idq,
                      const Vector<2>& udq)
{
    /*
     * Creating aliases for the sake of better compatibility with the Matlab source.
     * They do not affect performance in any way - the optimization will throw them out.
     */
    const auto& Pin = P_;
    const auto& y = idq;
    Const Ts = dt;
    Const Id = idq[0];
    Const Iq = idq[1];
    Const Ld = ld_;
    Const Lq = lq_;
    Const komp = cross_coupling_comp_;
    Const ud = udq[0];
    Const uq = udq[1];
    Const R = r_;
    Const Fi = phi_;

    /*
     * Copy-pasted from Matlab source with minor syntax changes.
     */
    Const w = x_[StateIndexAngularVelocity];
    Const Theta = x_[StateIndexAngularPosition];

    Const Td = Ld / R;
    Const Tq = Lq / R;

    Matrix<4, 4> F = Matrix<4, 4>::Zero();
    F(0, 0) = (1.0F - Ts / Td);
    F(0, 1) = Ts * w * Lq / Ld;
    F(0, 2) = Ts * Lq * Iq / Ld;
    F(1, 0) = -Ts * w * Ld / Lq;
    F(1, 1) = (1.0F - Ts * (1.0F + komp) / Tq);
    F(1, 2) = Ts * (-Ld * Id / Lq - Fi / Lq);
    F(2, 2) = 1.0F;
    F(3, 2) = Ts;
    F(3, 3) = 1.0F;

    Vector<4> Xout;
    Xout[0] = Id + (ud / Ld - R * Id / Ld + w * Lq * Iq / Ld) * Ts;
    Xout[1] = Iq + (uq / Lq - R * Iq / Lq - w * Ld * Id / Lq - Fi * w / Lq) * Ts;
    Xout[2] = w;
    Xout[3] = Theta + w * Ts;

    const Matrix<4, 4> Pout = F * Pin * F.transpose() + Q_;

    const Matrix<4, 2> K = Pout * C_.transpose() * (C_ * Pout * C_.transpose() + R_).inverse();

    const Vector<2> z = y - C_ * Xout;

    z_covariance_.update(z * z.transpose());
    const Matrix<2, 2> z_cov = z_covariance_.getValue();

    Q_ = K * z_cov * K.transpose();
//    R_ = z_cov - C_ * Pout.inverse() * C_.transpose();    // This is horribly slow because of the inverse

    // TODO rewrite properly
    Q_(0, 0) = math::Range<>(  0.01F,   100.0F).constrain(Q_(0, 0));
    Q_(1, 1) = math::Range<>(  0.01F,   100.0F).constrain(Q_(1, 1));
    Q_(2, 2) = math::Range<>(100.0F,  10000.0F).constrain(Q_(2, 2));
    Q_(3, 3) = math::Range<>(  0.1F,    100.0F).constrain(Q_(3, 3));

//    R_(0, 0) = math::Range<>(  0.01F,   100.0F).constrain(R_(0, 0));
//    R_(1, 1) = math::Range<>(  0.01F,   100.0F).constrain(R_(1, 1));

    x_ = Xout + K * z;
    x_[StateIndexAngularPosition] = math::normalizeAngle(x_[StateIndexAngularPosition]);

    P_ = (Matrix<4, 4>::Identity() - K * C_) * Pout;

    /*
     * Constraint check
     */
    if (direction_constraint_ != DirectionConstraint::None)
    {
        if (((direction_constraint_ == DirectionConstraint::Forward) && (x_[StateIndexAngularVelocity] < 0)) ||
            ((direction_constraint_ == DirectionConstraint::Reverse) && (x_[StateIndexAngularVelocity] > 0)))
        {
            x_[StateIndexAngularVelocity] = 0.0F;
        }
    }
}

}
}
