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

constexpr Scalar MinimumSpinupDurationFraction          = 0.1F;
constexpr Scalar MaximumSpinupDurationFraction          = 1.5F;
constexpr Scalar SpinupAngularVelocityHysteresis        = 3.0F;

/*
 * State variables
 */
volatile State g_state = State::Idle;

ControllerParameters g_controller_params;

MotorParameters g_motor_params;

ObserverParameters g_observer_params;

ControlMode g_requested_control_mode;

volatile Scalar g_setpoint;

volatile Scalar g_setpoint_remaining_ttl;       ///< Seconds left before the setpoint will be zeroed, unless updated

volatile std::uint32_t g_num_successive_rotor_stalls = 0;

board::motor::PWMHandle g_pwm_handle;

motor_id::Mode g_requested_motor_identification_mode;

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

    Scalar inverter_power = 0;                                  ///< Watt, updated from the main IRQ

    Scalar remaining_time_before_stall_detection_enabled = 0;

    Scalar spinup_time = 0;

    Scalar setpoint_Iq = 0;
    Scalar setpoint_Uq = 0;
    bool use_voltage_setpoint = false;

    struct LowPassFilteredValues
    {
        static constexpr Scalar InnovationWeight = 0.05F;

        static void update(Scalar& value, const Scalar& new_value)
        {
            value += InnovationWeight * (new_value - value);
        }

        Scalar inverter_power = 0;
        Scalar demand_factor = 0;
    } low_pass_filtered_values;

    Context(const ObserverParameters& observer_params,
            Const field_flux,
            Const Lq,
            Const Rs,
            Const max_current,
            Const min_current,
            Const min_voltage,
            Const current_ramp,
            Const voltage_ramp,
            Const pwm_period,
            Const pwm_dead_time) :
        observer(observer_params,
                 field_flux,
                 Lq,            // ASSUMPTION: Ld ~= Lq
                 Lq,
                 Rs),
        modulator(Lq,
                  Rs,
                  max_current,
                  pwm_period,
                  pwm_dead_time,
                  board::motor::getPWMUpperLimit(),
                  modulator.DeadTimeCompensationPolicy::Disabled,
                  modulator.CrossCouplingCompensationPolicy::Disabled),
        current_controller(max_current,
                           min_current,
                           min_voltage,
                           current_ramp,
                           voltage_ramp)
    { }
};

Context* g_context = nullptr;


void initializeContext()
{
    // Deriving min voltage from known parameters
    Scalar min_voltage = 0;

    if (g_motor_params.isValid())
    {
        Const min_mrpm =
            convertRotationRateElectricalToMechanical(
                convertAngularVelocityToRPM(g_motor_params.min_electrical_ang_vel),
                g_motor_params.num_poles);

        Const kv = convertFluxLinkageToKV(g_motor_params.phi, g_motor_params.num_poles);

        min_voltage = min_mrpm / kv;
    }

    IRQDebugOutputBuffer::setVariableFromIRQ<0>(min_voltage);

    // Sanity checks
    if (!std::isfinite(min_voltage) ||
        !math::Range<>(0.01F, 10.0F).contains(min_voltage))
    {
        min_voltage = 0.0F;
    }

    // Instantiation
    alignas(16) static std::uint8_t context_storage[sizeof(Context)];
    std::fill(std::begin(context_storage), std::end(context_storage), 0);       // Paranoia time

    g_context = new (context_storage) Context(g_observer_params,
                                              g_motor_params.phi,
                                              g_motor_params.lq,
                                              g_motor_params.rs,
                                              g_motor_params.max_current,
                                              g_motor_params.min_current,
                                              min_voltage,
                                              g_motor_params.current_ramp_amp_per_s,
                                              g_motor_params.voltage_ramp_volt_per_s,
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
           (g_num_successive_rotor_stalls < g_controller_params.num_stalls_to_latch);
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


void setControllerParameters(const ControllerParameters& params)
{
    AbsoluteCriticalSectionLocker locker;
    g_controller_params = params;
}

ControllerParameters getControllerParameters()
{
    AbsoluteCriticalSectionLocker locker;
    return g_controller_params;
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


void beginMotorIdentification(motor_id::Mode mode)
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

    g_requested_control_mode = control_mode;

    {
        const bool zero_setpoint = os::float_eq::closeToZero(value);

        const bool sign_flip = ((g_setpoint > 0) && (value < 0)) ||
                               ((g_setpoint < 0) && (value > 0));

        if (zero_setpoint || sign_flip)
        {
            g_num_successive_rotor_stalls = 0;
        }
    }

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
    case State::Fault:
    {
        break;          // Ignoring
    }
    }
}

ControlMode getControlMode()
{
    return g_requested_control_mode;      // Atomic read, no locking
}

void stop()
{
    AbsoluteCriticalSectionLocker locker;

    g_setpoint = 0;
    g_setpoint_remaining_ttl = 0;
    g_num_successive_rotor_stalls = 0;
}


Scalar getInstantCurrentFiltered()
{
    Const voltage = board::motor::getInverterVoltage();

    if (os::float_eq::positive(voltage))
    {
        AbsoluteCriticalSectionLocker locker;
        if (g_context != nullptr)
        {
            return g_context->low_pass_filtered_values.inverter_power / voltage;
        }
    }

    return 0;
}

Scalar getInstantDemandFactorFiltered()
{
    AbsoluteCriticalSectionLocker locker;
    if (g_context != nullptr)
    {
        return g_context->low_pass_filtered_values.demand_factor;
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

std::uint32_t getErrorCount()
{
    return g_num_successive_rotor_stalls;
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
    Scalar setpoint_Iq = 0;
    Scalar setpoint_Uq = 0;
    Scalar inverter_power = 0;
    Scalar demand_factor = 0;

    {
        AbsoluteCriticalSectionLocker locker;

        state           = g_state;
        control_mode    = g_requested_control_mode;
        setpoint        = g_setpoint;

        num_successive_rotor_stalls = g_num_successive_rotor_stalls;

        if (g_context != nullptr)
        {
            Udq_normalizations = g_context->modulator.getUdqNormalizationCounter();

            angular_velocity    = g_context->angular_velocity;
            estimated_Idq       = g_context->estimated_Idq;
            reference_Udq       = g_context->reference_Udq;
            setpoint_Iq         = g_context->setpoint_Iq;
            setpoint_Uq         = g_context->setpoint_Uq;
            inverter_power      = g_context->low_pass_filtered_values.inverter_power;
            demand_factor       = g_context->low_pass_filtered_values.demand_factor;
        }
    }

    std::printf("State        : %s [%d]\n"
                "Control Mode : %d\n"
                "Setpoint     : %.3f\n"
                "Succ. Stalls : %u\n",
                stateToString(state), int(state),
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
                "Estimated Idq: %s A\n"
                "Reference Udq: %s A\n"
                "Setpoint  Iq : %.1f A\n"
                "Setpoint  Uq : %.1f A\n"
                "Inverter Pwr : %.1f W\n"
                "Demand Factor: %.0f %%\n",
                double(angular_velocity),
                double(mrpm),
                math::toString(estimated_Idq).c_str(),
                math::toString(reference_Udq).c_str(),
                double(setpoint_Iq),
                double(setpoint_Uq),
                double(inverter_power),
                double(demand_factor) * 100.0);
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

                // Correcting the angle estimation latency, assuming that the observer runs for about half period.
                Const angle_slip = g_context->angular_velocity * (period * 0.5F);
                g_context->angular_position =
                    math::normalizeAngle(g_context->observer.getAngularPosition() + angle_slip);

                // Computing new setpoint using the appropriate control mode (current, voltage, RPM)
                if (g_requested_control_mode == ControlMode::RatiometricVoltage ||
                    g_requested_control_mode == ControlMode::Voltage)
                {
                    const auto max_voltage = computeLineVoltageLimit(board::motor::getInverterVoltage(),
                                                                     board::motor::getPWMUpperLimit());
                    g_context->use_voltage_setpoint = true;
                    g_context->setpoint_Iq = g_context->estimated_Idq[1];
                    g_context->setpoint_Uq =
                        g_context->current_controller.update(period,
                                                             g_setpoint,
                                                             g_requested_control_mode,
                                                             g_context->setpoint_Uq,
                                                             max_voltage,
                                                             g_context->angular_velocity);
                }
                else
                {
                    g_context->use_voltage_setpoint = false;
                    g_context->setpoint_Uq = g_context->reference_Udq[1];
                    g_context->setpoint_Iq =
                        g_context->current_controller.update(period,
                                                             g_setpoint,
                                                             g_requested_control_mode,
                                                             g_context->setpoint_Iq,
                                                             0.0F,
                                                             g_context->angular_velocity);
                }

                // Rotor stall detection
                if (g_context->remaining_time_before_stall_detection_enabled > 0)
                {
                    // We've just entered the running mode, stall detection is temporarily suppressed
                    g_context->remaining_time_before_stall_detection_enabled -= period;
                }
                else
                {
                    // Stopping if the angular velocity is too low
                    if (std::abs(g_context->angular_velocity) < g_motor_params.min_electrical_ang_vel)
                    {
                        g_state = State::Idle;      // We'll possibly switch back to spinup from idle later
                        g_num_successive_rotor_stalls++;
                    }
                }
            }
            else
            {
                // Change of direction while starting
                if ((g_setpoint > 0) != (g_context->setpoint_Iq > 0))
                {
                    g_context->setpoint_Iq = 0.0F;
                    g_context->spinup_time = 0;
                    g_context->use_voltage_setpoint = false;
                }

                // Slowly increasing current
                if (std::abs(g_context->setpoint_Iq) < g_motor_params.spinup_current)
                {
                    Const current_delta =
                        (g_motor_params.spinup_current / g_controller_params.nominal_spinup_duration) * period;

                    g_context->setpoint_Iq += std::copysign(current_delta, g_setpoint);
                }

                // State update
                g_context->angular_velocity = g_context->observer.getAngularVelocity();
                g_context->angular_position = g_context->observer.getAngularPosition();
                g_context->spinup_time += period;

                // Checking if spinup is finished, switching to Running if so
                const bool spinup_in_progress_long_enough = g_context->spinup_time >
                    (g_controller_params.nominal_spinup_duration * MinimumSpinupDurationFraction);

                const bool min_current_exceeded = std::abs(g_context->setpoint_Iq) > g_motor_params.min_current;

                if (spinup_in_progress_long_enough && min_current_exceeded)
                {
                    Const ang_vel_threshold = g_motor_params.min_electrical_ang_vel * SpinupAngularVelocityHysteresis;
                    if (std::abs(g_context->angular_velocity) > ang_vel_threshold)
                    {
                        // Running fast enough, switching to normal mode
                        g_state = State::Running;
                        g_context->remaining_time_before_stall_detection_enabled = g_context->spinup_time * 2.0F;
                    }
                }

                // Checking failure conditions
                Const spinup_timeout = g_controller_params.nominal_spinup_duration * MaximumSpinupDurationFraction;
                if (g_context->spinup_time > spinup_timeout)
                {
                    // Timed out
                    g_setpoint = 0;
                    g_num_successive_rotor_stalls++;
                }
            }
        }

        /*
         * Stuff that does not require synchronization with the fast IRQ
         */
        g_context->inverter_power = (Udq.transpose() * Idq)[0] * 1.5F;

        Context::LowPassFilteredValues::update(g_context->low_pass_filtered_values.inverter_power,
                                               g_context->inverter_power);

        Context::LowPassFilteredValues::update(g_context->low_pass_filtered_values.demand_factor,
                                               std::abs(g_context->estimated_Idq[1]) / g_motor_params.max_current);

        g_debug_tracer.set<5>(g_context->observer.getAngularVelocity());
        g_debug_tracer.set<6>((g_state == State::Spinup) ?
                              g_context->angular_velocity :
                              g_context->inverter_power / board::motor::getInverterVoltage());
#if 1
        {
            static Scalar prev_Iq = Idq[1];
            Const Iq_gradient = (Idq[1] - prev_Iq) / period;
            prev_Iq = Idq[1];

            Const angvel = (Udq[1] - g_motor_params.rs * Idq[1] - g_motor_params.lq * Iq_gradient) /
                           (g_motor_params.lq * Idq[0] + g_motor_params.phi);

            static Scalar filtered;
            filtered += 0.01F * (angvel - filtered);

            g_debug_tracer.set<4>(filtered);
        }
#else
        g_debug_tracer.set<4>(g_context->use_voltage_setpoint ? g_context->setpoint_Uq : g_context->setpoint_Iq);
#endif

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
            g_context->setpoint_Iq = 0;
            g_context->use_voltage_setpoint = true;
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
        decltype(g_context->modulator)::Setpoint setpoint;
        if (g_context->use_voltage_setpoint)
        {
            setpoint.mode  = setpoint.Mode::Uq;
            setpoint.value = g_context->setpoint_Uq;
        }
        else
        {
            setpoint.mode  = setpoint.Mode::Iq;
            setpoint.value = g_context->setpoint_Iq;
        }

        const auto output = g_context->modulator.onNextPWMPeriod(phase_currents_ab,
                                                                 inverter_voltage,
                                                                 g_context->angular_velocity,
                                                                 g_context->angular_position,
                                                                 setpoint);
        g_pwm_handle.setPWM(output.pwm_setpoint);

        g_context->estimated_Idq = output.estimated_Idq;
        g_context->reference_Udq = output.reference_Udq;
        g_context->angular_position = output.extrapolated_angular_position;

        g_debug_tracer.set<0>(g_context->reference_Udq[0]);
        g_debug_tracer.set<1>(g_context->reference_Udq[1]);
        g_debug_tracer.set<2>(g_context->estimated_Idq[0]);
        g_debug_tracer.set<3>(g_context->estimated_Idq[1]);
    }

    /*
     * Motor identification
     */
    {
        static motor_id::Estimator* estimator = nullptr;

        if (g_state == State::MotorIdentification)
        {
            alignas(16) static std::uint8_t estimator_storage[sizeof(motor_id::Estimator)];

            if (estimator == nullptr)
            {
                board::motor::beginCalibration();

                estimator = new (estimator_storage)
                    motor_id::Estimator(g_requested_motor_identification_mode,
                                        g_motor_params,
                                        g_controller_params.motor_id,
                                        period,
                                        board::motor::getPWMDeadTime(),
                                        board::motor::getPWMUpperLimit());
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
                estimator->~Estimator();
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

                const auto measurement_range = board::motor::getLimits().measurement_range;

                hardware_tester = new (hardware_tester_storage) HardwareTester(period,
                                                                               measurement_range.inverter_voltage,
                                                                               measurement_range.inverter_temperature);
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
