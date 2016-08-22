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

Scalar g_setpoint;

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
    Observer observer;

    Scalar angular_velocity = 0;                ///< Radian per second
    Scalar angular_position = 0;                ///< Radians [0, Pi*2]; extrapolated in the fast IRQ
    Scalar angular_position_sine = 0;           ///< Updated from fast IRQ
    Scalar angular_position_cosine = 0;         ///< Updated from fast IRQ

    PIController pid_Id;
    PIController pid_Iq;

    Vector<2> reference_Udq = {0.0F, 0.0F};     ///< Volt
    Scalar reference_Iq = 0;                    ///< Ampere

    unsigned sector = 0;

    Context(const ObserverParameters& observer_params,
            math::Const field_flux,
            math::Const stator_phase_inductance_direct,
            math::Const stator_phase_inductance_quadrature,
            math::Const stator_phase_resistance,
            const PIControllerSettings& pid_settings_Id,
            const PIControllerSettings& pid_settings_Iq) :
        observer(observer_params,
                 field_flux,
                 stator_phase_inductance_direct,
                 stator_phase_inductance_quadrature,
                 stator_phase_resistance),
        pid_Id(pid_settings_Id),
        pid_Iq(pid_settings_Iq)
    { }
};

Context* volatile g_context = nullptr;

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


void handleMainIRQ(Const period,
                   const Vector<2>& phase_currents_ab,
                   Const inverter_voltage)
{
    (void)period;
    (void)phase_currents_ab;
    (void)inverter_voltage;
}

void handleFastIRQ(Const period)
{
    (void)period;
}

}
}
