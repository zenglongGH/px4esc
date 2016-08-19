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

#include "observer.hpp"
#include <zubax_chibios/config/config.hpp>


namespace foc
{
namespace
{

os::config::Param<float> g_param_cross_coupling_compensation("foc.obs.cc_comp", 0.5F, 0.0F, 10.0F);

os::config::Param<float> g_param_Q_11("foc.obs.q_11",      100.0F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_Q_22("foc.obs.q_22",      100.0F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_Q_33("foc.obs.q_33",     5000.0F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_Q_44("foc.obs.q_44",        5.0F,  1e-6F,  1e+6F);

os::config::Param<float> g_param_R_11("foc.obs.r_11",        2.0F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_R_22("foc.obs.r_22",        2.0F,  1e-6F,  1e+6F);

os::config::Param<float> g_param_P0_11("foc.obs.p0_11",      0.1F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_P0_22("foc.obs.p0_22",      0.1F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_P0_33("foc.obs.p0_33",      0.1F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_P0_44("foc.obs.p0_44",      0.1F,  1e-6F,  1e+6F);

} // namespace

using math::makeMatrix;
using math::makeDiagonalMatrix;
using math::makeRow;
using math::Const;
using math::Vector;
using math::Matrix;


math::Scalar Observer::constrainAngularPosition(Const x)
{
    constexpr Const Pi2 = math::Pi * 2.0F;

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

Observer::Observer(const ObserverParameters& parameters,
                   Const field_flux,
                   Const stator_phase_inductance_direct,
                   Const stator_phase_inductance_quadrature,
                   Const stator_phase_resistance) :
    // Motor model
    phi_(field_flux),
    ld_(stator_phase_inductance_direct),
    lq_(stator_phase_inductance_quadrature),
    r_(stator_phase_resistance),

    // Filter constants
    cross_coupling_comp_(parameters.cross_coupling_compensation),
    Q_(parameters.Q),
    R_(parameters.R),
    C_(makeMatrix(makeRow(1, 0, 0, 0),
                  makeRow(0, 1, 0, 0))),

    // Filter state
    x_(decltype(x_)::Zero()),
    P_(parameters.P0)
{
    assert(std::isfinite(phi_));
    assert(std::isfinite(ld_));
    assert(std::isfinite(lq_));
    assert(std::isfinite(r_));
    assert(std::isfinite(cross_coupling_comp_));
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
    F(1, 1) = (1.0F - Ts / Td);
    F(1, 2) = Ts * w * Lq / Ld;
    F(1, 3) = Ts * Lq * Iq / Ld;
    F(2, 1) = -Ts * w * Ld / Lq;
    F(2, 2) = (1.0F - Ts * (1.0F + komp) / Tq);
    F(2, 3) = Ts * (-Ld * Id / Lq - Fi / Lq);
    F(3, 3) = 1.0F;
    F(4, 3) = Ts;
    F(4, 4) = 1.0F;

    Vector<4> Xout;
    Xout[0] = Id + (ud / Ld - R * Id / Ld + w * Lq * Iq / Ld) * Ts;
    Xout[1] = Iq + (uq / Lq - R * Iq / Lq - w * Ld * Id / Lq - Fi * w / Lq) * Ts;
    Xout[2] = w;
    Xout[3] = Theta + w * Ts;

    const Matrix<4, 4> Pout = F * Pin * F.transpose() + Q_;

    const Matrix<4, 2> K = Pout * C_.transpose() * (C_ * Pout * C_.transpose() + R_).inverse();

    x_ = Xout + K * (y - C_ * Xout);
    x_[StateIndexAngularPosition] = constrainAngularPosition(x_[StateIndexAngularPosition]);

    P_ = (Matrix<4, 4>::Identity() - K * C_) * Pout;
}

}
