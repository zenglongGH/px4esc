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

#include "params.hpp"
#include <zubax_chibios/os.hpp>
#include <zubax_chibios/util/heapless.hpp>
#include <initializer_list>


namespace params
{
namespace
{

using Real = os::config::Param<float>;
using Natural = os::config::Param<unsigned>;
using Boolean = os::config::Param<bool>;


namespace controller
{

using D = foc::ControllerParameters;

Real g_spinup_duration    ("ctrl.spinup_sec",   D().nominal_spinup_duration,       0.1F,    10.0F);
Natural g_num_attempts    ("ctrl.num_attempt",  D().num_stalls_to_latch,              1, 10000000);
Real g_vm_bandwidth       ("ctrl.vmbandwidth",  D().voltage_modulator_bandwidth,  0.01F,     1.0F);
Boolean g_vm_ccic         ("ctrl.vm_cci_comp",  D().voltage_modulator_cross_coupling_inductance_compensation);

}

namespace motor
{

using D = foc::MotorParameters;

Natural g_num_poles       ("m.num_poles",       0,                               0,         200);
Real g_max_current        ("m.max_ampere",      0.0F,                         0.0F,      200.0F);
Real g_min_current        ("m.min_ampere",      0.0F,                         0.0F,       50.0F);
Real g_spinup_current     ("m.spinup_ampere",   0.0F,                         0.0F,       50.0F);
Real g_field_flux         ("m.phi_milliweber",  0.0F,                         0.0F, D::getPhiLimits().max * 1e3F);
Real g_phase_resistance   ("m.rs_ohm",          0.0F,                         0.0F, D::getRsLimits().max);
Real g_inductance_quadr   ("m.lq_microhenry",   0.0F,                         0.0F, D::getLqLimits().max * 1e6F);
Real g_min_electr_ang_vel ("m.min_eradsec",     D().min_electrical_ang_vel,  10.0F,     1000.0F);
Real g_current_ramp       ("m.ampere_per_sec",  D().current_ramp_amp_per_s,   0.1F,    10000.0F);
Real g_voltage_ramp       ("m.volt_per_sec",    D().voltage_ramp_volt_per_s, 0.01F,     1000.0F);

}

namespace motor_id
{

using Default = foc::motor_id::Parameters;

Real g_frac_of_max_current("mid.max_cur_frac",  Default().fraction_of_max_current,                     0.1F,    1.0F);
Real g_high_frequency     ("mid.hifreq_hertz",  Default().current_injection_frequency,               100.0F, 5000.0F);
Real g_phi_eradsec        ("mid.phi_eradsec",   Default().phi_estimation_electrical_angular_velocity, 50.0F, 5000.0F);

}

namespace observer
{

using Default = foc::observer::Parameters;

Real g_Q_11    ("obs.q_11",     Default().Q.diagonal()[0],  1e-6F,  1e+6F);
Real g_Q_22    ("obs.q_22",     Default().Q.diagonal()[1],  1e-6F,  1e+6F);
Real g_Q_33    ("obs.q_33",     Default().Q.diagonal()[2],  1e-6F,  1e+6F);
Real g_Q_44    ("obs.q_44",     Default().Q.diagonal()[3],  1e-6F,  1e+6F);

Real g_R_11    ("obs.r_11",     Default().R.diagonal()[0],  1e-6F,  1e+6F);
Real g_R_22    ("obs.r_22",     Default().R.diagonal()[1],  1e-6F,  1e+6F);

Real g_P0_11   ("obs.p0_11",    Default().P0.diagonal()[0], 1e-6F,  1e+6F);
Real g_P0_22   ("obs.p0_22",    Default().P0.diagonal()[1], 1e-6F,  1e+6F);
Real g_P0_33   ("obs.p0_33",    Default().P0.diagonal()[2], 1e-6F,  1e+6F);
Real g_P0_44   ("obs.p0_44",    Default().P0.diagonal()[3], 1e-6F,  1e+6F);

Real g_cross_coupling_comp("obs.crosscp_comp", Default().cross_coupling_compensation, 0.0F, 1.0F);

}


chibios_rt::Mutex g_mutex;

os::Logger g_logger("Params");


template <typename T, typename Src>
void assign(os::config::Param<T>& destination, const Src source)
{
    const int res = destination.set(source);

    if (res < 0)
    {
        g_logger.println("ERROR: COULD NOT SET PARAM '%s' TO '%s': ERROR %d",
                         destination.name, os::heapless::concatenate(source).c_str(), res);
    }
}

} // namespace


foc::Parameters readFOCParameters()
{
    os::MutexLocker locker(g_mutex);

    foc::Parameters out;

    {
        using namespace controller;
        out.controller.nominal_spinup_duration      = g_spinup_duration.get();
        out.controller.num_stalls_to_latch          = g_num_attempts.get();
        out.controller.voltage_modulator_bandwidth  = g_vm_bandwidth.get();
        out.controller.voltage_modulator_cross_coupling_inductance_compensation = g_vm_ccic.get();
        assert(out.controller.isValid());
    }
    {
        using namespace motor;
        out.motor.num_poles               = g_num_poles.get();
        out.motor.max_current             = g_max_current.get();
        out.motor.min_current             = g_min_current.get();
        out.motor.spinup_current          = g_spinup_current.get();
        out.motor.phi                     = g_field_flux.get() * 1e-3F;
        out.motor.rs                      = g_phase_resistance.get();
        out.motor.lq                      = g_inductance_quadr.get() * 1e-6F;
        out.motor.min_electrical_ang_vel  = g_min_electr_ang_vel.get();
        out.motor.current_ramp_amp_per_s  = g_current_ramp.get();
        out.motor.voltage_ramp_volt_per_s = g_voltage_ramp.get();
        out.motor.deduceMissingParameters();
        // May be invalid
    }
    {
        using namespace motor_id;
        out.motor_id.fraction_of_max_current = g_frac_of_max_current.get();
        out.motor_id.current_injection_frequency = g_high_frequency.get();
        out.motor_id.phi_estimation_electrical_angular_velocity = g_phi_eradsec.get();
        assert(out.motor_id.isValid());
    }
    {
        using namespace observer;
        out.observer.Q = math::makeDiagonalMatrix(g_Q_11.get(),
                                                  g_Q_22.get(),
                                                  g_Q_33.get(),
                                                  g_Q_44.get());
        out.observer.R = math::makeDiagonalMatrix(g_R_11.get(),
                                                  g_R_22.get());
        out.observer.P0 = math::makeDiagonalMatrix(g_P0_11.get(),
                                                   g_P0_22.get(),
                                                   g_P0_33.get(),
                                                   g_P0_44.get());
        out.observer.cross_coupling_compensation = g_cross_coupling_comp.get();
        assert(out.observer.isValid());
    }
    return out;
}

void writeFOCParameters(const foc::Parameters& obj)
{
    os::MutexLocker locker(g_mutex);

    {
        using namespace controller;
        assign(g_spinup_duration,   obj.controller.nominal_spinup_duration);
        assign(g_num_attempts,      obj.controller.num_stalls_to_latch);
        assign(g_vm_bandwidth,      obj.controller.voltage_modulator_bandwidth);
        assign(g_vm_ccic,           obj.controller.voltage_modulator_cross_coupling_inductance_compensation);
    }

    writeMotorParameters(obj.motor);

    {
        using namespace motor_id;
        assign(g_frac_of_max_current,       obj.motor_id.fraction_of_max_current);
        assign(g_high_frequency,            obj.motor_id.current_injection_frequency);
        assign(g_phi_eradsec,               obj.motor_id.phi_estimation_electrical_angular_velocity);
    }

    {
        using namespace observer;

        static const auto unpacker = [](const auto matrix, std::initializer_list<Real*> param_array)
        {
            int index = 0;
            for (auto& dst : param_array)
            {
                if (dst != nullptr)
                {
                    assign(*dst, matrix.diagonal()[index++]);
                }
            }
        };

        unpacker(obj.observer.Q, {&g_Q_11,
                                  &g_Q_22,
                                  &g_Q_33,
                                  &g_Q_44});
        unpacker(obj.observer.R, {&g_R_11,
                                  &g_R_22});
        unpacker(obj.observer.P0, {&g_P0_11,
                                   &g_P0_22,
                                   &g_P0_33,
                                   &g_P0_44});
        assign(g_cross_coupling_comp, obj.observer.cross_coupling_compensation);
    }
}

void writeMotorParameters(const foc::MotorParameters& obj)
{
    os::MutexLocker locker(g_mutex);

    using namespace motor;

    assign(g_num_poles,          obj.num_poles);
    assign(g_max_current,        obj.max_current);
    assign(g_min_current,        obj.min_current);
    assign(g_spinup_current,     obj.spinup_current);
    assign(g_field_flux,         obj.phi * 1e3F);
    assign(g_phase_resistance,   obj.rs);
    assign(g_inductance_quadr,   obj.lq * 1e6F);
    assign(g_min_electr_ang_vel, obj.min_electrical_ang_vel);
    assign(g_current_ramp,       obj.current_ramp_amp_per_s);
    assign(g_voltage_ramp,       obj.voltage_ramp_volt_per_s);
}

}
