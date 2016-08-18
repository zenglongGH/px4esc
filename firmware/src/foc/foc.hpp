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

#pragma once

#include <math/math.hpp>
#include <cstdint>


namespace foc
{
/**
 * Must be invoked in the first order, exactly once.
 * This function may trigger automatic identification.
 */
void init();

/**
 * State of the control logic.
 * Some of the functions may be unavailable in certain states.
 * TODO: Needs review.
 */
enum class State
{
    Standby,            ///< The control logic is doing nothing and is ready to accept requests.
    Busy,               ///< Internal activities are underway (e.g. calibration, auto ID), requests cannot be accepted.
    Spinup,             ///< The motor is starting.
    Running,            ///< The motor is running.
    Stalled,            ///< The motor stopped unexpectedly, possibly due to high mechanical load or software error.
    Locked,             ///< The motor was stalled too many times, controller locked up. Call @ref stop() to reset.
    InvalidConfig,      ///< The configuration is invalid. The motor cannot be started.
    HardwareFailure     ///< The hardware or the motor are dysfunctional. This is a terminal state.
};

/**
 * @ref State.
 */
State getState();

/**
 * Various control modes.
 * See the function definitions below for usages.
 */
enum class ControlMode
{
    RelativePower,      ///< Abstract units in [-1, 1]
    Torque,             ///< Newton meters
    AngularVelocity     ///< Radian per second
};

/**
 * Assigns new setpoint; the units depend on the selected control mode.
 * The value of zero stops the motor.
 * Negative values indicate reverse rotation.
 *
 * @param control_mode          See @ref ControlMode.
 * @param value                 Value depending on the control mode.
 * @param request_ttl           After this timeout (in seconds) the motor will be stopped automatically.
 */
void setSetpoint(ControlMode control_mode,
                 math::Const value,
                 math::Const request_ttl);

/**
 * See @ref ControlMode.
 * The control mode is selected via @ref setSetpoint().
 */
ControlMode getControlMode();

/**
 * Stops the motor normally if it is running.
 * Does nothing if the motor is not running.
 */
void stop();

/**
 * Returns the instant motor current in Amperes.
 */
math::Scalar getInstantCurrent();

/**
 * Returns the instant motor power in Watts.
 */
math::Scalar getInstantPower();

/**
 * Returns the maximum power that can be delivered to the motor.
 * This value is defined either by the ESC hardware or by the motor's specification.
 * In conjunction with @ref getInstantPower(), this value can be used to determine the instant power rating in percent.
 */
math::Scalar getMaximumPower();

/**
 * Generate sound using the motor windings.
 * The request MAY be ignored if the controller is in not in the Standby mode.
 * Units are SI (Hertz, seconds).
 */
void beep(math::Const frequency,
          math::Const duration);

/**
 * Returns the total number of errors encountered since the motor was last stopped or since reboot,
 * whichever was the last.
 */
std::uint32_t getErrorCount();

/**
 * Prints the current status information into stdout.
 * This command is mostly useful for debugging, diagnostics and tuning.
 */
void printStatusInfo();

}
