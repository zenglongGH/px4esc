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

#include "motor_parameters.hpp"
#include "motor_parameters_estimator.hpp"
#include "hardware_tester.hpp"
#include "observer.hpp"
#include <math/math.hpp>
#include <cstdint>
#include <utility>


namespace foc
{
/**
 * Must be invoked in the first order, exactly once.
 * This function may block for a few seconds.
 * All other API functions are non-blocking, except stated otherwise.
 */
void init();

/**
 * Motor parameters must be set after initialization before the motor can be started.
 * New parameters will take effect when the motor is restarted.
 */
void setMotorParameters(const MotorParameters& params);

MotorParameters getMotorParameters();

/**
 * Observer parameters can be set after initialization. They are initialized to reasonable values by default.
 * New parameters will take effect when the motor is restarted.
 */
void setObserverParameters(const ObserverParameters& params);

ObserverParameters getObserverParameters();

/**
 * Begins the asynchronous process of motor identification.
 * See @ref MotorIdentificationMode.
 * Completion of the process can be detected by means of monitoring the current state of the controller, see @ref State.
 * The identified parameters can be read via @ref getMotorParameters().
 */
void beginMotorIdentification(MotorIdentificationMode mode);

/**
 * State of the control logic.
 * Some of the functions may be unavailable in certain states.
 */
enum class State
{
    /**
     * The control logic is doing nothing and is ready to accept commands.
     * If the motor stalled or another error occurred, an error code will be set.
     */
    Idle,

    /**
     * Motor identification is in progress, commands cannot be accepted.
     * @ref beginMotorIdentification().
     */
    MotorIdentification,

    /**
     * Hardware testing is underway.
     * Possible outcomes:
     *  - Test passed                                   -> Idle
     *  - Test failed (hardware problems detected)      -> Fault
     */
    HardwareTesting,

    /**
     * The motor is starting, or some pre-start procedures are underway. This is a transient state.
     * Possible outcomes:
     *  - Started successfully  -> Running
     *  - Failed to start       -> Idle or Fault
     */
    Spinup,

    /**
     * The motor is running. Next state is normally Idle.
     */
    Running,

    /**
     * The controller has encountered a serious error and will not start the motor until the error is reset.
     * In order to reset the error, call @ref stop(), or set a zero setpoint (which is equivalent to calling stop()).
     */
    Fault
};

/**
 * @ref State.
 */
State getState();

/**
 * Error codes complement the states defined above with additional failure information.
 * Errors are just indications, they do not require any actions from the user except when the state is Fault.
 */
enum class Error
{
    None,
    SpinupUnsuccessful,
    HardwareFailure,
    InvalidMotorParameters
};

/**
 * Returns the code of the last encountered error and the total number of errors encountered since the motor was
 * last stopped or since reboot, whichever was the last. See @ref Error.
 */
std::pair<Error, std::uint32_t> getLastErrorWithErrorCount();

/**
 * Various control modes.
 * See the function definitions below for usages.
 */
enum class ControlMode
{
    RatiometricCurrent,         ///< Abstract units in [-1, 1]
    Current                     ///< Ampere
};

/**
 * Assigns new setpoint; the units depend on the selected control mode.
 * The value of zero stops the motor and clears the fault state, which is equivalent to calling @ref stop().
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
 * Clears the fault state if the motor is not running.
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
 * Prints the current status information into stdout.
 * This command is mostly useful for debugging, diagnostics and tuning.
 * The function makes blocking calls to printf() and may moderately disturb IRQ processing due to critical sections.
 */
void printStatusInfo();

/**
 * This command is intended for use with CLI plotting tool.
 * Refer to the project tools directory for more info.
 */
void plotRealTimeValues();

}
