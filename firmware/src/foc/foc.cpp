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

#include "foc.hpp"
#include "svm.hpp"
#include "pid.hpp"
#include "transforms.hpp"

#include <board/board.hpp>
#include <board/motor.hpp>
#include <zubax_chibios/config/config.hpp>


/*
 * Documents:
 *  - http://cache.nxp.com/files/microcontrollers/doc/ref_manual/DRM148.pdf
 */
namespace foc
{

using math::Scalar;
using math::Const;
using math::Vector;
using board::motor::AbsoluteCriticalSectionLocker;

namespace
{

volatile State g_state = State::Idle;

MotorParameters g_motor_params;

ObserverParameters g_observer_params;

ControlMode g_control_mode;

volatile Scalar g_setpoint;

volatile Scalar g_setpoint_remaining_ttl;       ///< Seconds left before the setpoint will be zeroed, unless updated


struct ErrorCounter
{
    Error last_error = Error::None;
    std::uint32_t error_count = 0;

    void reset() volatile
    {
        last_error = Error::None;
        error_count = 0;
    }

    void registerError(Error e) volatile
    {
        AbsoluteCriticalSectionLocker locker;
        last_error = e;
        error_count++;
    }
} volatile g_error_counter;


struct Context
{
    const math::Range<> motor_current_limit;
    const math::Range<> motor_voltage_limit;

    Observer observer;

    Scalar angular_velocity = 0;                        ///< Radian per second, read in the fast IRQ
    Scalar angular_position = 0;                        ///< Radians [0, Pi*2]; extrapolated in the fast IRQ

    PIController pid_Id;                                ///< Used only in the fast IRQ
    PIController pid_Iq;                                ///< Used only in the fast IRQ

    Vector<2> estimated_Idq = Vector<2>::Zero();        ///< Ampere, updated from the fast IRQ
    Vector<2> reference_Udq = Vector<2>::Zero();        ///< Volt, updated from the fast IRQ

    Scalar reference_Iq = 0;                            ///< Ampere, read in the fast IRQ

    Context(const ObserverParameters& observer_params,
            math::Const field_flux,
            math::Const stator_phase_inductance_direct,
            math::Const stator_phase_inductance_quadrature,
            math::Const stator_phase_resistance,
            math::Const max_current,
            math::Const max_voltage,
            const PIControllerSettings& pid_settings_Id,
            const PIControllerSettings& pid_settings_Iq) :
        motor_current_limit(-max_current, max_current),
        motor_voltage_limit(-max_voltage, max_voltage),
        observer(observer_params,
                 field_flux,
                 stator_phase_inductance_direct,
                 stator_phase_inductance_quadrature,
                 stator_phase_resistance),
        pid_Id(pid_settings_Id),
        pid_Iq(pid_settings_Iq)
    { }
};

Context* g_context = nullptr;


void initializeContext()
{
    constexpr Scalar Pi2 = math::Pi * 2.0F;

    Const fast_irq_period = board::motor::getPWMPeriod();

    Const Rs = g_motor_params.r_ab / 2.0F;
    Const Ld = g_motor_params.l_ab / 2.0F;
    Const Lq = g_motor_params.l_ab / 2.0F;

    // TODO: Correct PID gain computation
    const PIControllerSettings pid_Id_settings((Pi2 * Ld) / (20.0F * fast_irq_period),
                                               0.1F,
                                               g_motor_params.max_current);

    const PIControllerSettings pid_Iq_settings((Pi2 * Lq) / (20.0F * fast_irq_period),
                                               0.1F,
                                               g_motor_params.max_current);

    alignas(16) static std::uint8_t context_storage[sizeof(Context)];

    g_context = new (context_storage) Context(g_observer_params,
                                              g_motor_params.field_flux,
                                              Ld,
                                              Lq,
                                              Rs,
                                              g_motor_params.max_current,
                                              g_motor_params.max_voltage,
                                              pid_Id_settings,
                                              pid_Iq_settings);
}

} // namespace


void init()
{
    {
        AbsoluteCriticalSectionLocker locker;

        g_state = State::Idle;
        g_setpoint = 0;
        g_setpoint_remaining_ttl = 0;
        g_error_counter.reset();
        g_context = nullptr;
    }

    board::motor::setActive(false);

    board::motor::beginCalibration();
}


void setMotorParameters(const MotorParameters& params)
{
    AbsoluteCriticalSectionLocker locker;
    g_motor_params = params;
}

MotorParameters getMotorParameters()
{
    AbsoluteCriticalSectionLocker locker;
    return g_motor_params;
}

void setObserverParameters(const ObserverParameters& params)
{
    AbsoluteCriticalSectionLocker locker;
    g_observer_params = params;
}

ObserverParameters getObserverParameters()
{
    AbsoluteCriticalSectionLocker locker;
    return g_observer_params;
}


void beginMotorIdentification(MotorIdentificationMode mode)
{
    AbsoluteCriticalSectionLocker locker;
    (void)mode;
}


State getState()
{
    return g_state;             // Atomic read, no locking
}


std::pair<Error, std::uint32_t> getLastErrorWithErrorCount()
{
    AbsoluteCriticalSectionLocker locker;
    return { g_error_counter.last_error, g_error_counter.error_count };
}


void setSetpoint(ControlMode control_mode,
                 Const value,
                 Const request_ttl)
{
    AbsoluteCriticalSectionLocker locker;

    g_control_mode = control_mode;

    switch (g_state)
    {
    case State::Idle:
    case State::Spinup:
    case State::Running:
    case State::MotorIdentification:    // Implying that the motor will start once identification is finished
    {
        // Normal handling
        g_setpoint = value;
        g_setpoint_remaining_ttl = request_ttl;
        break;
    }

    case State::Fault:
    {
        // When setting zero setpoint at the fault state, reset the state to idle.
        if (std::abs(value) < 1e-3F)
        {
            g_state = State::Idle;
        }
        break;
    }
    }
}

ControlMode getControlMode()
{
    return g_control_mode;      // Atomic read, no locking
}

void stop()
{
    AbsoluteCriticalSectionLocker locker;

    g_setpoint = 0;
    g_setpoint_remaining_ttl = 0;

    if (g_state == State::Fault)
    {
        g_state = State::Idle;
    }
}


Scalar getInstantCurrent()
{
    AbsoluteCriticalSectionLocker locker;
    return 0;
}

Scalar getInstantPower()
{
    AbsoluteCriticalSectionLocker locker;
    return 0;
}

Scalar getMaximumPower()
{
    AbsoluteCriticalSectionLocker locker;
    return 0;
}


void beep(Const frequency,
          Const duration)
{
    AbsoluteCriticalSectionLocker locker;
    (void)frequency;
    (void)duration;
}


void printStatusInfo()
{
}

}


namespace board
{
namespace motor
{

using namespace foc;


void handleMainIRQ(Const period)
{
    if (board::motor::isCalibrationInProgress())
    {
        g_state = State::Idle;
    }

    /*
     * It is guaranteed by the driver that the main IRQ is always invoked immediately after the fast IRQ
     * of the same period. This guarantees that the context struct contains the most recent data.
     * The Spinup/Running case must be processed first, because synchronization with the fast IRQ is important,
     * and because we care about latency.
     */
    if (g_state == State::Running ||
        g_state == State::Spinup)
    {
        /*
         * Making a local copy of state variables ASAP, before the next fast IRQ fires.
         */
        const auto Idq = g_context->estimated_Idq;
        const auto Udq = g_context->reference_Udq;

        /*
         * Running the observer, this takes forever.
         * By the time the observer has finished, the rotor has moved some angle forward, which we compensate.
         */
        g_context->observer.update(period, Idq, Udq);

        /*
         * Updating the state estimate.
         * Critical section is required because at this point we're no longer synchronized with the fast IRQ.
         */
        {
            AbsoluteCriticalSectionLocker locker;

            if (g_state == State::Running)
            {
                g_context->angular_velocity = g_context->observer.getAngularVelocity();

                // Angle delay compensation
                Const angle_slip = g_context->observer.getAngularVelocity() * period;
                g_context->angular_position =
                    constrainAngularPosition(g_context->observer.getAngularPosition() + angle_slip);
            }
            else
            {
                // In spinup mode, we run solely by extrapolation
                if (g_context->angular_velocity < 300.0F)
                {
                    g_context->angular_velocity += 50.0F * period;
                }
                else
                {
                    g_state = State::Running;
                }
            }
        }

        /*
         * Updating setpoint and handling termination condition.
         */
        g_setpoint_remaining_ttl -= period;
        if (g_setpoint_remaining_ttl <= 0.0F)
        {
            // Setpoint TTL expired, stopping
            g_setpoint = 0.0F;
        }

        if (os::float_eq::closeToZero(Scalar(g_setpoint)))
        {
            // Stopping
            {
                AbsoluteCriticalSectionLocker locker;

                g_state = State::Idle;

                g_context->~Context();
                g_context = nullptr;
            }

            board::motor::setActive(false);
        }
    }

    if (g_state == State::Idle)
    {
        const bool need_to_start = !os::float_eq::closeToZero(Scalar(g_setpoint));
        if (need_to_start)
        {
            initializeContext();

            g_context->reference_Iq = g_motor_params.start_current;

            board::motor::setActive(true);

            g_state = State::Spinup;
        }
    }
}


void handleFastIRQ(Const period,
                   const math::Vector<2>& phase_currents_ab,
                   Const inverter_voltage)
{
    const auto state = g_state;                 // Avoiding excessive volatile reads

    /*
     * In normal operating mode, we're using this IRQ to do the following:
     *  - Idq estimation
     *  - computation of Udq reference using PIDs
     *  - space vector modulation of PWM outputs
     *  - angle extrapolation between main IRQs
     */
    if (state == State::Running ||
        state == State::Spinup)
    {
        /*
         * Computing Idq, Udq
         */
        Const angle_sine   = math::sin(g_context->angular_position);
        Const angle_cosine = math::cos(g_context->angular_position);

        const auto estimated_I_alpha_beta = performClarkeTransform(phase_currents_ab);

        g_context->estimated_Idq = performParkTransform(estimated_I_alpha_beta, angle_sine, angle_cosine);

        /*
         * Running PIDs, estimating reference voltage in the rotating reference frame
         */
        g_context->reference_Udq[0] = g_context->pid_Id.update(0.0F,
                                                               g_context->estimated_Idq[0], period);

        g_context->reference_Udq[1] = g_context->pid_Iq.update(g_context->reference_Iq,
                                                               g_context->estimated_Idq[1], period);

        /*
         * Transforming back to the stationary reference frame, updating the PWM outputs
         * TODO: Dead time compensation
         */
        auto reference_U_alpha_beta = performInverseParkTransform(g_context->reference_Udq,
                                                                  angle_sine, angle_cosine);

        // Voltage limiting
        for (int i = 0; i <= 1; i++)
        {
            reference_U_alpha_beta[i] = g_context->motor_voltage_limit.constrain(reference_U_alpha_beta[i]);
        }

        const auto phase_voltages_and_sector_number = performSpaceVectorTransform(reference_U_alpha_beta);
        // Sector number is not used

        const auto pwm_setpoint = normalizePhaseVoltagesToPWMSetpoint(phase_voltages_and_sector_number.first,
                                                                      inverter_voltage);

        board::motor::setPWM(pwm_setpoint);

        /*
         * Position extrapolation
         */
        g_context->angular_position =
            constrainAngularPosition(g_context->angular_position + g_context->angular_velocity * period);
    }

    /*
     * Idle state handler.
     */
    if (state == State::Idle)
    {
        // TODO: beeping
    }
}

}
}
