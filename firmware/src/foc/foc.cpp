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

#include "foc.hpp"
#include "pid.hpp"
#include "transforms.hpp"
#include "voltage_modulator.hpp"
#include "irq_debug_output.hpp"
#include "common.hpp"

#include <zubax_chibios/config/config.hpp>


/*
 * Documents:
 *  - http://cache.nxp.com/files/microcontrollers/doc/ref_manual/DRM148.pdf
 */
namespace foc
{

namespace
{
/*
 * Configuration constants
 */
constexpr unsigned IdqMovingAverageLength = 5;

constexpr Scalar MotorIdentificationCurrentFrequency = 300.0F;
constexpr Scalar MotorIdentificationPhiAngularVelocity = 150.0F;

/*
 * State variables
 */
volatile State g_state = State::Idle;

MotorParameters g_motor_params;

ObserverParameters g_observer_params;

ControlMode g_control_mode;

volatile Scalar g_setpoint;

volatile Scalar g_setpoint_remaining_ttl;       ///< Seconds left before the setpoint will be zeroed, unless updated

volatile std::uint32_t g_num_successive_rotor_stalls = 0;

board::motor::PWMHandle g_pwm_handle;

MotorIdentificationMode g_requested_motor_identification_mode;

HardwareTester::TestReport g_last_hardware_test_report;

EventCounter g_fast_irq_cycle_counter;


class DebugVariableTracer
{
    static constexpr unsigned NumVariables = 7;

#if DEBUG_BUILD
    Scalar vars_[NumVariables] = {};
#endif

public:
    template <unsigned Index, typename Value>
    void set(const Value x)
    {
        static_assert(Index < NumVariables, "Debug variable index out of range");
#if DEBUG_BUILD
        vars_[Index] = Scalar(x);
#else
        (void) x;
#endif
    }

    template <typename Container>
    void set(const Container cont)
    {
        std::copy_n(std::begin(cont), std::min(cont.size(), NumVariables), std::begin(vars_));
    }

    void print() const
    {
#if DEBUG_BUILD
        std::uint64_t pwm_cycles = 0;
        Scalar vars_copy[NumVariables] = {};

        {
            AbsoluteCriticalSectionLocker locker;
            pwm_cycles = g_fast_irq_cycle_counter.get();
            std::copy(std::begin(vars_), std::end(vars_), std::begin(vars_copy));
        }

        std::printf("$%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    double(pwm_cycles) * double(board::motor::getPWMPeriod()),
                    double(vars_copy[0]),
                    double(vars_copy[1]),
                    double(vars_copy[2]),
                    double(vars_copy[3]),
                    double(vars_copy[4]),
                    double(vars_copy[5]),
                    double(vars_copy[6]));
#endif
    }
} g_debug_tracer;


struct BeepCommand
{
    Scalar frequency = 0;
    Scalar duration  = 0;
} * g_beep_command = nullptr;


struct Context
{
    Observer observer;

    ThreePhaseVoltageModulator<IdqMovingAverageLength> modulator;

    CurrentSetpointController current_controller;

    Scalar angular_velocity = 0;                                ///< Radian per second, read in the fast IRQ
    Scalar angular_position = 0;                                ///< Radians [0, Pi*2]; extrapolated in the fast IRQ

    Vector<2> estimated_Idq = Vector<2>::Zero();                ///< Ampere, updated from the fast IRQ
    Vector<2> reference_Udq = Vector<2>::Zero();                ///< Volt, updated from the fast IRQ

    Scalar reference_Iq = 0;                                    ///< Ampere, read in the fast IRQ

    Scalar inverter_power = 0;                                 ///< Watt, updated from the main IRQ

    Context(const ObserverParameters& observer_params,
            Const field_flux,
            Const stator_phase_inductance,
            Const stator_phase_resistance,
            Const max_current,
            Const min_current,
            Const current_ramp,
            Const pwm_period,
            Const pwm_dead_time) :
        observer(observer_params,
                 field_flux,
                 stator_phase_inductance,
                 stator_phase_inductance,
                 stator_phase_resistance),
        modulator(stator_phase_inductance,
                  stator_phase_resistance,
                  max_current,
                  pwm_period,
                  pwm_dead_time,
                  modulator.DeadTimeCompensationPolicy::Disabled),
        current_controller(max_current,
                           min_current,
                           current_ramp)
    { }
};

Context* g_context = nullptr;


void initializeContext()
{
    Const Ls = g_motor_params.l_ab / 2.0F;
    Const Rs = g_motor_params.r_ab / 2.0F;

    alignas(16) static std::uint8_t context_storage[sizeof(Context)];
    std::fill(std::begin(context_storage), std::end(context_storage), 0);       // Paranoia time

    g_context = new (context_storage) Context(g_observer_params,
                                              g_motor_params.phi,
                                              Ls,
                                              Rs,
                                              g_motor_params.max_current,
                                              g_motor_params.min_current,
                                              g_motor_params.current_ramp_amp_per_s,
                                              board::motor::getPWMPeriod(),
                                              board::motor::getPWMDeadTime());
}


void doStop()
{
    AbsoluteCriticalSectionLocker locker;

    g_state = State::Idle;
    g_setpoint = 0;
    g_setpoint_remaining_ttl = 0;

    if (g_context != nullptr)
    {
        g_context->~Context();
        g_context = nullptr;
    }
}


bool isStatusOk()
{
    return g_last_hardware_test_report.isSuccessful() &&
           g_motor_params.isValid() &&
           board::motor::getStatus().isOkay() &&
           (g_num_successive_rotor_stalls < g_motor_params.num_stalls_to_latch);
}

} // namespace


void init()
{
    {
        AbsoluteCriticalSectionLocker locker;

        g_state = State::Idle;
        g_setpoint = 0;
        g_setpoint_remaining_ttl = 0;
        g_context = nullptr;
    }

    board::motor::beginCalibration();
}


void setMotorParameters(const MotorParameters& params)
{
    AbsoluteCriticalSectionLocker locker;
    g_motor_params = params;
    // Making sure the motor restarts if already running
    g_setpoint = 0;
    g_setpoint_remaining_ttl = 0;
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

    if (g_state == State::Idle ||
        g_state == State::Fault)
    {
        g_state = State::MotorIdentification;
        g_requested_motor_identification_mode = mode;
    }
}


void beginHardwareTest()
{
    AbsoluteCriticalSectionLocker locker;

    if (g_state == State::Idle ||
        g_state == State::Fault)
    {
        g_state = State::HardwareTesting;
    }
}


HardwareTester::TestReport getLastHardwareTestReport()
{
    AbsoluteCriticalSectionLocker locker;
    return g_last_hardware_test_report;
}


State getState()
{
    return g_state;             // Atomic read, no locking
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
    {
        // Normal handling
        g_setpoint = value;
        g_setpoint_remaining_ttl = request_ttl;
        break;
    }

    case State::MotorIdentification:
    case State::HardwareTesting:
    {
        break;          // Ignoring
    }

    case State::Fault:
    {
        // When setting zero setpoint at the fault state, reset the rotor stall counter
        if (std::abs(value) < 1e-3F)
        {
            g_num_successive_rotor_stalls = 0;
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
    Const voltage = board::motor::getInverterVoltage();

    if (os::float_eq::positive(voltage))
    {
        AbsoluteCriticalSectionLocker locker;
        if (g_context != nullptr)
        {
            return g_context->inverter_power / voltage;
        }
    }

    return 0;
}

Scalar getInstantDemandFactor()
{
    Const max_current = g_motor_params.max_current;

    if (os::float_eq::positive(max_current))
    {
        AbsoluteCriticalSectionLocker locker;
        if (g_context != nullptr)
        {
            return g_context->reference_Iq / max_current;
        }
    }

    return 0;
}

Scalar getInstantMechanicalRPM()
{
    Scalar electrical_rad_sec = 0;

    {
        AbsoluteCriticalSectionLocker locker;
        if (g_context != nullptr)
        {
            electrical_rad_sec = g_context->angular_velocity;
        }
    }

    const auto num_poles = g_motor_params.num_poles;

    if ((num_poles >= 2) &&
        (num_poles % 2 == 0))
    {
        return convertRotationRateElectricalToMechanical(convertAngularVelocityToRPM(electrical_rad_sec),
                                                         num_poles);
    }

    return 0;
}


void beep(Const frequency,
          Const duration)
{
    AbsoluteCriticalSectionLocker locker;

    if ((frequency > 0) &&
        (duration > 0))
    {
        static BeepCommand beep_command;

        beep_command.frequency = frequency;
        beep_command.duration  = duration;

        g_beep_command = &beep_command;
    }
}


void printStatusInfo()
{
    State state = State();
    ControlMode control_mode = ControlMode();
    Scalar setpoint = 0;
    std::uint32_t num_successive_rotor_stalls = 0;

    EventCounter Udq_normalizations;

    Scalar angular_velocity = 0;
    Vector<2> estimated_Idq = Vector<2>::Zero();
    Vector<2> reference_Udq = Vector<2>::Zero();
    Scalar reference_Iq = 0;
    Scalar inverter_power = 0;

    {
        AbsoluteCriticalSectionLocker locker;

        state           = g_state;
        control_mode    = g_control_mode;
        setpoint        = g_setpoint;

        num_successive_rotor_stalls = g_num_successive_rotor_stalls;

        if (g_context != nullptr)
        {
            Udq_normalizations = g_context->modulator.getUdqNormalizationCounter();

            angular_velocity    = g_context->angular_velocity;
            estimated_Idq       = g_context->estimated_Idq;
            reference_Udq       = g_context->reference_Udq;
            reference_Iq        = g_context->reference_Iq;
            inverter_power      = g_context->inverter_power;
        }
    }

    std::printf("State        : %d\n"
                "Control Mode : %d\n"
                "Setpoint     : %.3f\n"
                "Succ. Stalls : %u\n",
                int(state),
                int(control_mode),
                double(setpoint),
                unsigned(num_successive_rotor_stalls));

    std::printf("Udq Norm. Cnt: %s\n",
                Udq_normalizations.toString().c_str());

    Scalar mrpm = 0;

    if ((g_motor_params.num_poles >= 2) &&
        (g_motor_params.num_poles % 2 == 0))
    {
        mrpm = convertRotationRateElectricalToMechanical(convertAngularVelocityToRPM(angular_velocity),
                                                         g_motor_params.num_poles);
    }

    std::printf("Elect. AngVel: %.1f rad/s\n"
                "Mech. RPM    : %.1f MRPM\n"
                "Estimated Idq: %s\n"
                "Reference Udq: %s\n"
                "Reference Iq : %.1f\n"
                "Inverter Pwr : %.1f\n",
                double(angular_velocity),
                double(mrpm),
                math::toString(estimated_Idq).c_str(),
                math::toString(reference_Udq).c_str(),
                double(reference_Iq),
                double(inverter_power));
}


void plotRealTimeValues()
{
    g_debug_tracer.print();
    IRQDebugOutputBuffer::printIfNeeded();
}


std::array<DebugKeyValueType, NumDebugKeyValuePairs> getDebugKeyValuePairs()
{
    Vector<2> Idq = Vector<2>::Zero();
    Vector<2> Udq = Vector<2>::Zero();

    {
        AbsoluteCriticalSectionLocker locker;
        if (g_context != nullptr)
        {
            Idq = g_context->estimated_Idq;
            Udq = g_context->reference_Udq;
        }
    }

    return {
        DebugKeyValueType("Id", Idq[0]),
        DebugKeyValueType("Iq", Idq[1]),
        DebugKeyValueType("Ud", Udq[0]),
        DebugKeyValueType("Uq", Udq[1])
    };
}

}


namespace board
{
namespace motor
{

using namespace foc;


void handleMainIRQ(Const period)
{
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
        if (g_state == State::Spinup)
        {
            g_context->observer.setDirectionConstraint((g_setpoint > 0) ?
                                                       Observer::DirectionConstraint::Forward :
                                                       Observer::DirectionConstraint::Reverse);
        }
        else
        {
            g_context->observer.setDirectionConstraint(Observer::DirectionConstraint::None);
        }

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

                Const angle_slip = g_context->observer.getAngularVelocity() * period;
                g_context->angular_position =
                    constrainAngularPosition(g_context->observer.getAngularPosition() + angle_slip);

                g_context->reference_Iq =
                    g_context->current_controller.update(period,
                                                         g_setpoint,
                                                         g_control_mode,
                                                         g_context->reference_Iq,
                                                         g_context->angular_velocity);

                // Stopping if the angular velocity is too low
                if (std::abs(g_context->angular_velocity) < (g_motor_params.min_electrical_ang_vel * 0.25F))
                {
                    g_num_successive_rotor_stalls++;
                    g_state = State::Spinup;    // Dropping into the spinup mode to change direction if necessary
                }
            }
            else
            {
                Const variable_delta = period / g_motor_params.nominal_spinup_duration;

                // Direction change requests and current setpoint initialization
                if (((g_setpoint > 0) != (g_context->reference_Iq > 0)) ||
                    os::float_eq::closeToZero(g_context->reference_Iq))
                {
                    g_context->reference_Iq = g_motor_params.spinup_current;
                    if (g_setpoint < 0)
                    {
                        g_context->reference_Iq = -g_context->reference_Iq;
                    }
                }

                // Open loop acceleration (shall we get rid of the hard-coded multiplier?)
                if (std::abs(g_context->angular_velocity) <= (g_motor_params.min_electrical_ang_vel * 3.0F))
                {
                    Const increase = g_motor_params.min_electrical_ang_vel * variable_delta;

                    g_context->angular_velocity += (g_setpoint > 0) ? increase : -increase;
                }

                // Observer synchronization
                if (std::abs(g_context->angular_velocity) >= g_motor_params.min_electrical_ang_vel)
                {
                    {
                        // The current is steadily decreasing until observer synchronization is established.
                        // This also limits the maximum time alloted for spinup - when the current drops
                        // below the minimum, spinup will be aborted.
                        // Note that the current starts decreasing not synchronously with acceleration,
                        // but this behavior is intended - they don't need to be synchronized.
                        Const decrease = g_motor_params.spinup_current * variable_delta;
                        if (g_context->reference_Iq > 0)
                        {
                            g_context->reference_Iq -= decrease;
                        }
                        else
                        {
                            g_context->reference_Iq += decrease;
                        }
                    }

                    // Evaluating the observer synchronization precision
                    Const ang_pos_error = math::subtractAngles(g_context->observer.getAngularPosition(),
                                                               g_context->angular_position);
                    Const ang_vel_rel_error =
                        std::abs(g_context->observer.getAngularVelocity() - g_context->angular_velocity) /
                        std::abs(g_context->angular_velocity);

                    // Decision to hand-off control to the normal mode.
                    // Not sure if the hardcoded thresholds are okay.
                    const bool ang_pos_ok = std::abs(ang_pos_error) < math::convertDegreesToRadian(20.0F);
                    const bool ang_vel_ok = ang_vel_rel_error < 0.2F;
                    if (ang_vel_ok && ang_pos_ok)
                    {
                        g_state = State::Running;
                    }
                }

                // Fault detection
                if (std::abs(g_context->reference_Iq) < g_motor_params.min_current)
                {
                    g_num_successive_rotor_stalls++;
                    g_setpoint = 0;     // Stopping
                }
            }
        }

        /*
         * Stuff that does not require synchronization with the fast IRQ
         */
        g_context->inverter_power = (Udq.transpose() * Idq)[0] * 1.5F;

        g_debug_tracer.set<5>(g_context->observer.getAngularVelocity());
        g_debug_tracer.set<6>(g_context->inverter_power / board::motor::getInverterVoltage());

        /*
         * Updating setpoint and handling termination condition.
         */
        g_setpoint_remaining_ttl -= period;

        if (os::float_eq::closeToZero(g_setpoint) ||
            (g_setpoint_remaining_ttl <= 0.0F))
        {
            doStop();
            return;
        }
    }

    if (g_state == State::Idle)
    {
        // Retaining the Fault state if there's something wrong
        if (!isStatusOk())
        {
            g_setpoint = 0;             // No way
            g_state = State::Fault;
        }

        if (!os::float_eq::closeToZero(g_setpoint))
        {
            initializeContext();
            g_context->reference_Iq = 0;
            g_state = State::Spinup;
        }
    }

    if (g_state == State::Fault)
    {
        if (isStatusOk())
        {
            g_state = State::Idle;
        }
    }
}


void handleFastIRQ(Const period,
                   const Vector<2>& phase_currents_ab,
                   Const inverter_voltage)
{
    g_fast_irq_cycle_counter.increment();

    if (board::motor::isCalibrationInProgress())
    {
        if (g_state == State::Running ||
            g_state == State::Spinup)
        {
            g_state = State::Idle;
        }
    }

    /*
     * Normal operating mode
     */
    if (g_state == State::Running ||
        g_state == State::Spinup)
    {
        const auto output = g_context->modulator.onNextPWMPeriod(phase_currents_ab,
                                                                 inverter_voltage,
                                                                 g_context->angular_velocity,
                                                                 g_context->angular_position,
                                                                 g_context->reference_Iq);

        g_pwm_handle.setPWM(output.pwm_setpoint);

        g_context->estimated_Idq = output.estimated_Idq;
        g_context->reference_Udq = output.reference_Udq;

        g_context->angular_position = output.extrapolated_angular_position;

        g_debug_tracer.set<0>(g_context->reference_Udq[0]);
        g_debug_tracer.set<1>(g_context->reference_Udq[1]);
        g_debug_tracer.set<2>(g_context->estimated_Idq[0]);
        g_debug_tracer.set<3>(g_context->estimated_Idq[1]);
        g_debug_tracer.set<4>(g_context->reference_Iq);
    }

    /*
     * Motor identification
     */
    {
        static MotorParametersEstimator* estimator = nullptr;

        if (g_state == State::MotorIdentification)
        {
            alignas(16) static std::uint8_t estimator_storage[sizeof(MotorParametersEstimator)];

            if (estimator == nullptr)
            {
                board::motor::beginCalibration();

                estimator = new (estimator_storage)
                    MotorParametersEstimator(g_requested_motor_identification_mode,
                                             g_motor_params,
                                             MotorIdentificationCurrentFrequency,
                                             MotorIdentificationPhiAngularVelocity,
                                             period,
                                             board::motor::getPWMDeadTime());
            }

            const auto hw_status = board::motor::getStatus();

            if (board::motor::isCalibrationInProgress())
            {
                g_pwm_handle.release(); // Nothing to do, waiting for the calibration to end before continuing
            }
            else
            {
                // TODO: We can't check the general hardware status because FAULT tends to go up randomly.
                //       There might be a hardware bug somewhere. Investigate it later.
                //const bool hw_ok = hw_status.isOkay();
                const bool hw_ok = hw_status.power_ok && !hw_status.overload;

                if (hw_ok)
                {
                    const auto pwm_vector = estimator->onNextPWMPeriod(phase_currents_ab, inverter_voltage);

                    g_pwm_handle.setPWM(pwm_vector);

                    if (estimator->isFinished())
                    {
                        g_pwm_handle.release();

                        g_motor_params = estimator->getEstimatedMotorParameters();

                        if (g_motor_params.isValid())
                        {
                            g_state = State::Idle;
                        }
                        else
                        {
                            g_state = State::Fault;
                        }
                    }
                }
                else
                {
                    // Hardware error; aborting and switching into the fault state
                    g_pwm_handle.release();
                    g_state = State::Fault;

                    IRQDebugOutputBuffer::setVariableFromIRQ<0>(hw_status.power_ok);
                    IRQDebugOutputBuffer::setVariableFromIRQ<1>(hw_status.overload);
                    IRQDebugOutputBuffer::setVariableFromIRQ<2>(hw_status.fault);
                }
            }

            g_debug_tracer.set(estimator->getDebugValues());
            g_debug_tracer.set<6>(hw_status.current_sensor_gain);
        }
        else
        {
            if (estimator != nullptr)
            {
                estimator->~MotorParametersEstimator();
                estimator = nullptr;
            }
        }
    }

    /*
     * Hardware testing
     */
    {
        static HardwareTester* hardware_tester = nullptr;

        if (g_state == State::HardwareTesting)
        {
            alignas(16) static std::uint8_t hardware_tester_storage[sizeof(HardwareTester)];

            if (hardware_tester == nullptr)
            {
                board::motor::beginCalibration();

                // TODO: Pass valid parameter ranges from the board driver.
                hardware_tester = new (hardware_tester_storage) HardwareTester(period);
            }

            const auto hw_status = board::motor::getStatus();

            if (board::motor::isCalibrationInProgress())
            {
                g_pwm_handle.release(); // Nothing to do, waiting for the calibration to end before continuing
            }
            else
            {
                const auto pwm_vector = hardware_tester->onNextPWMPeriod(phase_currents_ab,
                                                                         inverter_voltage,
                                                                         hw_status.inverter_temperature,
                                                                         hw_status.overload,
                                                                         hw_status.fault);

                g_pwm_handle.setPWM(pwm_vector);

                if (hardware_tester->isFinished())
                {
                    g_pwm_handle.release();

                    g_last_hardware_test_report = hardware_tester->getTestReport();

                    if (g_last_hardware_test_report.isSuccessful())
                    {
                        g_state = State::Idle;
                    }
                    else
                    {
                        g_state = State::Fault;
                    }
                }
            }

            const auto filtered_currents = hardware_tester->getCurrentsFilter().getValue();

            g_debug_tracer.set<0>(phase_currents_ab[0]);
            g_debug_tracer.set<1>(phase_currents_ab[1]);
            g_debug_tracer.set<2>(inverter_voltage);
            g_debug_tracer.set<3>(filtered_currents[0]);
            g_debug_tracer.set<4>(filtered_currents[1]);
            g_debug_tracer.set<5>(math::convertKelvinToCelsius(hw_status.inverter_temperature));
            g_debug_tracer.set<6>(hw_status.current_sensor_gain);
        }
        else
        {
            if (hardware_tester != nullptr)
            {
                hardware_tester->~HardwareTester();
                hardware_tester = nullptr;
            }
        }
    }

    /*
     * Secondary states
     */
    if (g_state == State::Idle ||
        g_state == State::Fault)
    {
        if (!board::motor::isCalibrationInProgress() && g_pwm_handle.isUnique())
        {
            static std::uint64_t beeping_deadline;
            static std::uint64_t next_excitation_at;
            static std::uint32_t excitation_period;

            const auto current_cycle = g_fast_irq_cycle_counter.get();

            if (current_cycle < beeping_deadline)               // Beeping in progress
            {
                g_pwm_handle.setPWM(math::Vector<3>::Zero());

                if (current_cycle >= next_excitation_at)
                {
                    next_excitation_at += excitation_period;

                    static unsigned phase_selector;
                    phase_selector = (phase_selector + 1U) % 3U;

                    math::Vector<3> pwm_vector = math::Vector<3>::Zero();
                    pwm_vector[phase_selector] = 1.0F;

                    g_pwm_handle.setPWM(pwm_vector);
                }
            }
            else                                                // Beeping is not in progress, commencing if needed
            {
                g_pwm_handle.release();

                if (g_beep_command != nullptr)
                {
                    Const frequency = math::Range<>(100.0F, 15000.0F).constrain(g_beep_command->frequency);
                    Const duration  = math::Range<>(0, 3.0F).constrain(g_beep_command->duration);

                    excitation_period = std::uint32_t((1.0F / frequency) / period + 0.5F);
                    next_excitation_at = current_cycle;
                    beeping_deadline = current_cycle + std::uint64_t(duration / period + 0.5F);

                    g_beep_command = nullptr;
                }
            }
        }
        else
        {
            g_beep_command = nullptr;
            g_pwm_handle.release();     // Even if the phase was excited, the driver will handle everything properly.
        }
    }
}

}
}
