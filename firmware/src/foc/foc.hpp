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
#include "current_setpoint_controller.hpp"
#include "hardware_tester.hpp"
#include "observer.hpp"
#include <math/math.hpp>
#include <cstdint>
#include <utility>


namespace foc
{
/**
 * General parameters not pertaining to the observer or the motor.
 */
struct ControllerParameters
{
    /// Preferred duration of spinup, real duration may slightly differ, seconds
    Scalar nominal_spinup_duration = 1.5F;

    /// If the rotor stalled this many times in a row, latch into FAULT state
    std::uint32_t num_stalls_to_latch = 10;

    /// Refer to the definition for details
    MotorIdentificationParameters motor_id;


    bool isValid() const
    {
        return math::Range<>(0.1F, 60.0F).contains(nominal_spinup_duration) &&
               num_stalls_to_latch > 0 &&
               motor_id.isValid();
    }

    auto toString() const
    {
        return os::heapless::format("Tspinup: %.1f sec\n"
                                    "Nslatch: %u\n"
                                    "Motor ID parameters:\n%s",
                                    double(nominal_spinup_duration),
                                    unsigned(num_stalls_to_latch),
                                    motor_id.toString().c_str());
    }
};

/**
 * Must be invoked in the first order, exactly once.
 * This function may block for a few seconds.
 * All other API functions are non-blocking, except stated otherwise.
 */
void init();

/**
 * Controller parameters have sensible values initialized by default.
 * They can be overwritten after initialization.
 * The new values should take effect immediately.
 */
void setControllerParameters(const ControllerParameters& params);

ControllerParameters getControllerParameters();

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
 * Begins the asynchronous process of hardware testing.
 * Completion of the process can be detected by means of monitoring the current state of the controller, see @ref State.
 * The result of the test can be obtained via @ref getLastHardwareTestReport().
 * A side effect of the test is that the HW driver will be recalibrated.
 */
void beginHardwareTest();

/**
 * See @ref beginHardwareTest().
 */
HardwareTester::TestReport getLastHardwareTestReport();

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
     * @ref beginHardwareTest().
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

inline const char* stateToString(const State s)
{
    switch (s)
    {
    case State::Idle:                return "Idle";
    case State::MotorIdentification: return "MotorID";
    case State::HardwareTesting:     return "HWTest";
    case State::Spinup:              return "Spinup";
    case State::Running:             return "Running";
    case State::Fault:               return "Fault";
    }

    return "BADSTATE";
}

/**
 * @ref State.
 */
State getState();

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
                 Const value,
                 Const request_ttl);

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
Scalar getInstantCurrent();

/**
 * Returns the instant relative power in percent of the maximum.
 */
Scalar getInstantDemandFactor();

/**
 * Returns the current mechanical angular velocity of the rotor in RPM.
 */
Scalar getInstantMechanicalRPM();

/**
 * Returns the number of all outstanding errors at the moment.
 * The exact semantics is yet to be defined; refer to the code to learn more.
 */
std::uint32_t getErrorCount();

/**
 * Generate sound using the motor windings.
 * The request MAY be ignored if the controller is in not in the Idle state.
 * Units are SI (Hertz, seconds).
 */
void beep(Const frequency,
          Const duration);

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

/**
 * Named debug values.
 * This is suitable for e.g. reporting via UAVCAN.
 */
using DebugKeyType = os::heapless::String<3>;
using DebugKeyValueType = std::pair<DebugKeyType, Scalar>;
constexpr unsigned NumDebugKeyValuePairs = 4;

std::array<DebugKeyValueType, NumDebugKeyValuePairs> getDebugKeyValuePairs();

}
