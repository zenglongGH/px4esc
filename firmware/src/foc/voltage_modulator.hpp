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
    CurrentPIController(Const Lq,
                        Const Rs,
                        Const max_current,
                        Const dt) :
        full_scale_current_(max_current * 3.0F),
        kp_((math::Pi * 2.0F * Lq) / (20.0F * dt)),
        ki_(dt * Rs / Lq)
    {
        assert(Lq > 0);
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

    void resetIntegrator()
    {
        ui_ = 0;
    }
};

/**
 * Generates rotating three phase voltage vector using measured and estimated parameters of the motor and Iq reference.
 */
template <unsigned IdqMovingAverageLength>
class ThreePhaseVoltageModulator
{
public:
    enum class DeadTimeCompensationPolicy
    {
        Disabled,
        Enabled
    };

    enum class CrossCouplingCompensationPolicy
    {
        Disabled,
        Enabled
    };

private:
    const DeadTimeCompensationPolicy dead_time_compensation_policy_;
    const CrossCouplingCompensationPolicy cross_coupling_compensation_policy_;

    board::motor::PWMParameters pwm_params_;

    Const Lq_;

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
        bool Udq_was_limited = false;
    };

    struct Setpoint
    {
        Scalar value = 0;

        enum class Mode : std::uint8_t
        {
            Iq,
            Uq
        } mode = Mode::Iq;
    };

    ThreePhaseVoltageModulator(Const Lq,
                               Const Rs,
                               Const max_current,
                               const board::motor::PWMParameters& pwm_params,
                               const DeadTimeCompensationPolicy dtcomp_policy,
                               const CrossCouplingCompensationPolicy cccomp_policy) :
        dead_time_compensation_policy_(dtcomp_policy),
        cross_coupling_compensation_policy_(cccomp_policy),
        pwm_params_(pwm_params),
        Lq_(Lq),
        pid_Id_(Lq, Rs, max_current, pwm_params_.period),
        pid_Iq_(Lq, Rs, max_current, pwm_params_.period),
        estimated_Idq_filter_(Vector<2>::Zero())
    { }

    Output onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                           Const inverter_voltage,
                           Const angular_velocity,
                           Const angular_position,
                           const Setpoint setpoint)
    {
        Output out;

        /*
         * Computing Idq, Udq
         */
        out.extrapolated_angular_position =
            math::normalizeAngle(angular_position + angular_velocity * pwm_params_.period);

        const auto angle_sincos = math::sincos(out.extrapolated_angular_position);

        const auto estimated_I_alpha_beta = performClarkeTransform(phase_currents_ab);

        const Vector<2> new_Idq = performParkTransform(estimated_I_alpha_beta, angle_sincos);
        estimated_Idq_filter_.update(new_Idq);
        out.estimated_Idq = estimated_Idq_filter_.getValue();

        /*
         * Running PIDs, estimating reference voltage in the rotating reference frame
         */
        out.reference_Udq[0] = pid_Id_.computeVoltage(0.0F,
                                                      out.estimated_Idq[0],
                                                      inverter_voltage);

        if (setpoint.mode == Setpoint::Mode::Iq)
        {
            out.reference_Udq[1] = pid_Iq_.computeVoltage(setpoint.value,
                                                          out.estimated_Idq[1],
                                                          inverter_voltage);
        }
        else if (setpoint.mode == Setpoint::Mode::Uq)
        {
            out.reference_Udq[1] = setpoint.value;
            pid_Iq_.resetIntegrator();
        }
        else
        {
            assert(false);
        }

        if (cross_coupling_compensation_policy_ == CrossCouplingCompensationPolicy::Enabled)
        {
            out.reference_Udq[0] -= angular_velocity * Lq_ * out.estimated_Idq[1];
            out.reference_Udq[1] += angular_velocity * Lq_ * out.estimated_Idq[0];
        }

        Const Udq_magnitude_limit = computeLineVoltageLimit(inverter_voltage, pwm_params_.upper_limit);

        if (out.reference_Udq.norm() > Udq_magnitude_limit)
        {
            out.reference_Udq = out.reference_Udq.normalized() * Udq_magnitude_limit;
            out.Udq_was_limited = true;
            Udq_normalization_count_.increment();
        }

        /*
         * Transforming back to the stationary reference frame, updating the PWM outputs
         */
        auto reference_U_alpha_beta = performInverseParkTransform(out.reference_Udq, angle_sincos);

        const auto pwm_setpoint_and_sector_number = performSpaceVectorTransform(reference_U_alpha_beta,
                                                                                inverter_voltage);
        // Sector number is not used
        if (dead_time_compensation_policy_ == DeadTimeCompensationPolicy::Enabled)
        {
            out.pwm_setpoint = performDeadTimeCompensation(pwm_setpoint_and_sector_number.first,
                                                           phase_currents_ab,
                                                           pwm_params_.period,
                                                           pwm_params_.dead_time);
        }
        else
        {
            out.pwm_setpoint = pwm_setpoint_and_sector_number.first;
        }

        return out;
    }

    EventCounter getUdqNormalizationCounter() const { return Udq_normalization_count_; }
};

}
