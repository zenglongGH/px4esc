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
#include "transforms.hpp"
#include <math/math.hpp>
#include <cassert>


namespace foc
{
/**
 * Serial PI controller, Idq Current --> Udq Voltage.
 */
class CurrentPIController
{
    Const full_scale_current_;
    Const kp_;
    Const ki_;

    Scalar ui_ = 0;

public:
    CurrentPIController(Const Ls,
                        Const Rs,
                        Const max_current,
                        Const dt) :
        full_scale_current_(max_current * 3.0F),
        kp_((math::Pi * 2.0F * Ls) / (20.0F * dt)),
        ki_(dt * Rs / Ls)
    {
        assert(Ls > 0);
        assert(Rs > 0);
        assert(max_current > 0);
        assert(dt > 0);
    }

    Scalar computeVoltage(Const target_current,
                          Const real_current,
                          Const inverter_voltage)
    {
        Const voltage_limit = inverter_voltage * (SquareRootOf3 / 2.0F);
        const math::Range<> voltage_limits(-voltage_limit, voltage_limit);

        static constexpr math::Range<> UnityLimits(-1.0F, 1.0F);
        Const error = UnityLimits.constrain((target_current - real_current) / full_scale_current_);

        ui_ = voltage_limits.constrain(ui_ + ki_ * error);      // Sdelat' hotel grozu,

        return kp_ * (error + ui_);                             // a poluchil kozu
    }
};

/**
 * Generates rotating three phase voltage vector using measured and estimated parameters of the motor and Iq reference.
 */
template <unsigned IdqMovingAverageLength>
class ThreePhaseVoltageModulator
{
    /*
     * This constant limits the maximum PWM value.
     * Exceeding this value may cause the ADC samples to occur at the moment when FET are switching,
     * which leads to incorrect measurements.
     * TODO: This parameter is heavily hardware-dependent, so it should be provided by the board driver.
     */
    static constexpr Scalar PWMLimit = 0.8F;

    Const dt_;

    CurrentPIController pid_Id_;
    CurrentPIController pid_Iq_;

    math::SimpleMovingAverageFilter<IdqMovingAverageLength, Vector<2>> estimated_Idq_filter_;

    EventCounter Udq_normalization_count_;

public:
    struct Output
    {
        Scalar extrapolated_angular_position = 0;
        math::Vector<2> estimated_Idq{};
        math::Vector<2> reference_Udq{};
        math::Vector<3> pwm_setpoint{};
    };

    ThreePhaseVoltageModulator(Const stator_phase_inductance,
                               Const stator_phase_resistance,
                               Const max_current,
                               Const update_period) :
        dt_(update_period),
        pid_Id_(stator_phase_inductance, stator_phase_resistance, max_current, update_period),
        pid_Iq_(stator_phase_inductance, stator_phase_resistance, max_current, update_period),
        estimated_Idq_filter_(Vector<2>::Zero())
    { }

    Output update(const Vector<2>& phase_currents_ab,
                  Const inverter_voltage,
                  Const angular_velocity,
                  Const angular_position,
                  Const reference_Iq)
    {
        Output out;

        /*
         * Computing Idq, Udq
         */
        Const angle_sine   = math::sin(angular_position);
        Const angle_cosine = math::cos(angular_position);

        const auto estimated_I_alpha_beta = performClarkeTransform(phase_currents_ab);

        const Vector<2> new_Idq = performParkTransform(estimated_I_alpha_beta, angle_sine, angle_cosine);
        estimated_Idq_filter_.update(new_Idq);
        out.estimated_Idq = estimated_Idq_filter_.getValue();

        /*
         * Running PIDs, estimating reference voltage in the rotating reference frame
         */
        out.reference_Udq[0] = pid_Id_.computeVoltage(0.0F,
                                                      out.estimated_Idq[0],
                                                      inverter_voltage);

        out.reference_Udq[1] = pid_Iq_.computeVoltage(reference_Iq,
                                                      out.estimated_Idq[1],
                                                      inverter_voltage);

        Const Udq_magnitude_limit = inverter_voltage * PWMLimit;

        if (out.reference_Udq.norm() > Udq_magnitude_limit)
        {
            out.reference_Udq = out.reference_Udq.normalized() * Udq_magnitude_limit;

            Udq_normalization_count_.increment();
        }

        /*
         * Transforming back to the stationary reference frame, updating the PWM outputs
         * TODO: Dead time compensation
         */
        auto reference_U_alpha_beta = performInverseParkTransform(out.reference_Udq, angle_sine, angle_cosine);

        const auto pwm_setpoint_and_sector_number = performSpaceVectorTransform(reference_U_alpha_beta,
                                                                                inverter_voltage);
        // Sector number is not used
        out.pwm_setpoint = pwm_setpoint_and_sector_number.first;

        out.extrapolated_angular_position = constrainAngularPosition(angular_position + angular_velocity * dt_);

        return out;
    }

    EventCounter getUdqNormalizationCounter() const { return Udq_normalization_count_; }
};

}
