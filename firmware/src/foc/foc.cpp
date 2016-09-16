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
#include "svm.hpp"
#include "pid.hpp"
#include "transforms.hpp"
#include "voltage_modulator.hpp"
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

constexpr Scalar MotorIdentificationCurrent = 7.0F;
constexpr Scalar MotorIdentificationCurrentFrequency = 300.0F;
constexpr Scalar MotorIdentificationPhiAngularVelocity = 300.0F;

/*
 * State variables
 */
volatile State g_state = State::Idle;

MotorParameters g_motor_params;

ObserverParameters g_observer_params;

ControlMode g_control_mode;

volatile Scalar g_setpoint;

volatile Scalar g_setpoint_remaining_ttl;       ///< Seconds left before the setpoint will be zeroed, unless updated

board::motor::PWMHandle g_pwm_handle;

MotorIdentificationMode g_requested_motor_identification_mode;


struct ErrorCounter
{
    Error last_error = Error::None;
    std::uint32_t error_count = 0;

    void reset()
    {
        last_error = Error::None;
        error_count = 0;
    }

    void registerError(Error e)
    {
        AbsoluteCriticalSectionLocker locker;
        last_error = e;
        error_count++;
    }
} g_error_counter;


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

        std::printf("$%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
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
    const math::Range<> current_limit;

    Observer observer;

    ThreePhaseVoltageModulator<IdqMovingAverageLength> modulator;

    Scalar angular_velocity = 0;                                ///< Radian per second, read in the fast IRQ
    Scalar angular_position = 0;                                ///< Radians [0, Pi*2]; extrapolated in the fast IRQ

    math::Vector<2> estimated_Idq = math::Vector<2>::Zero();    ///< Ampere, updated from the fast IRQ
    math::Vector<2> reference_Udq = math::Vector<2>::Zero();    ///< Volt, updated from the fast IRQ

    Scalar reference_Iq = 0;                                    ///< Ampere, read in the fast IRQ

    Context(const ObserverParameters& observer_params,
            Const field_flux,
            Const stator_phase_inductance,
            Const stator_phase_resistance,
            Const max_current,
            Const fast_irq_period) :
        current_limit(-max_current, max_current),
        observer(observer_params,
                 field_flux,
                 stator_phase_inductance,
                 stator_phase_inductance,
                 stator_phase_resistance),
        modulator(stator_phase_inductance,
                  stator_phase_resistance,
                  max_current,
                  fast_irq_period)
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
                                              g_motor_params.field_flux,
                                              Ls,
                                              Rs,
                                              g_motor_params.max_current,
                                              board::motor::getPWMPeriod());
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

    if (g_state == State::Idle)
    {
        g_state = State::MotorIdentification;
        g_requested_motor_identification_mode = mode;
    }
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
    ErrorCounter error_counter;

    EventCounter Udq_normalizations;

    Scalar angular_velocity = 0;
    Vector<2> estimated_Idq = Vector<2>::Zero();
    Vector<2> reference_Udq = Vector<2>::Zero();
    Scalar reference_Iq = 0;

    {
        AbsoluteCriticalSectionLocker locker;

        state           = g_state;
        control_mode    = g_control_mode;
        setpoint        = g_setpoint;
        error_counter   = g_error_counter;

        if (g_context != nullptr)
        {
            Udq_normalizations = g_context->modulator.getUdqNormalizationCounter();

            angular_velocity    = g_context->angular_velocity;
            estimated_Idq       = g_context->estimated_Idq;
            reference_Udq       = g_context->reference_Udq;
            reference_Iq        = g_context->reference_Iq;
        }
    }

    std::printf("State        : %d\n"
                "Control Mode : %d\n"
                "Setpoint     : %.3f\n",
                int(state), int(control_mode), double(setpoint));

    std::printf("Last Error   : %d\n"
                "Error Count  : %lu\n",
                int(error_counter.last_error), error_counter.error_count);

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
                "Reference Iq : %.1f\n",
                double(angular_velocity),
                double(mrpm),
                math::toString(estimated_Idq).c_str(),
                math::toString(reference_Udq).c_str(),
                double(reference_Iq));
}


void plotRealTimeValues()
{
    g_debug_tracer.print();
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

        g_debug_tracer.set<5>(g_context->observer.getAngularVelocity());
        g_debug_tracer.set<6>(g_context->observer.getAngularPosition());

        /*
         * Updating the state estimate.
         * Critical section is required because at this point we're no longer synchronized with the fast IRQ.
         */
        {
            AbsoluteCriticalSectionLocker locker;

            g_context->angular_velocity = g_context->observer.getAngularVelocity();

            // Angle delay compensation
            Const angle_slip = g_context->observer.getAngularVelocity() * period;
            g_context->angular_position =
                constrainAngularPosition(g_context->observer.getAngularPosition() + angle_slip);

            if (g_state == State::Running)
            {
                if (g_control_mode == ControlMode::RatiometricCurrent)
                {
                    static constexpr math::Range<> UnityLimits(-1.0F, 1.0F);

                    g_context->reference_Iq = g_context->current_limit.max * UnityLimits.constrain(g_setpoint);
                }
                else if (g_control_mode == ControlMode::Current)
                {
                    g_context->reference_Iq = g_context->current_limit.constrain(g_setpoint);
                }
                else
                {
                    g_context->reference_Iq = 0;        // This is actually an error.
                }

                // Stopping if the angular velocity is too low
                if (std::abs(g_context->angular_velocity) < (g_motor_params.min_electrical_ang_vel / 2.0F))
                {
                    g_state = State::Spinup;    // Dropping into the spinup mode to change direction if necessary
                }
            }
            else
            {
                if ((g_setpoint > 0) != (g_context->reference_Iq > 0))
                {
                    g_context->reference_Iq = 0;        // Direction changed, reset
                }

                if (g_setpoint > 0)
                {
                    g_context->reference_Iq += g_motor_params.spinup_current_slope * period;
                }
                else
                {
                    g_context->reference_Iq -= g_motor_params.spinup_current_slope * period;
                }

                if (std::abs(g_context->angular_velocity) > g_motor_params.min_electrical_ang_vel &&
                    std::abs(g_context->reference_Iq) > g_motor_params.min_current)
                {
                    g_state = State::Running;
                }

                if (!g_context->current_limit.contains(g_context->reference_Iq))
                {
                    g_setpoint = 0;     // Stopping
                }
            }
        }

        /*
         * Updating setpoint and handling termination condition.
         */
        g_setpoint_remaining_ttl -= period;

        if (os::float_eq::closeToZero(Scalar(g_setpoint)) ||
            (g_setpoint_remaining_ttl <= 0.0F))
        {
            doStop();
            return;
        }
    }

    if (g_state == State::Idle)
    {
        if (!g_motor_params.isValid())
        {
            g_setpoint = 0;             // No way
        }

        const bool need_to_start = !os::float_eq::closeToZero(Scalar(g_setpoint));
        if (need_to_start)
        {
            initializeContext();
            g_error_counter.reset();

            g_context->reference_Iq = 0;

            g_state = State::Spinup;
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
        const auto output = g_context->modulator.update(phase_currents_ab,
                                                        inverter_voltage,
                                                        g_context->angular_velocity,
                                                        g_context->angular_position,
                                                        g_context->reference_Iq);

        g_pwm_handle.setPWM(output.pwm_setpoint);

        g_context->estimated_Idq = output.estimated_Idq;
        g_context->reference_Udq = output.reference_Udq;

        g_context->angular_position = output.extrapolated_angular_position;

        g_debug_tracer.set<0>(g_context->reference_Udq[0] * 1e3F);
        g_debug_tracer.set<1>(g_context->reference_Udq[1] * 1e3F);
        g_debug_tracer.set<2>(g_context->estimated_Idq[0] * 1e3F);
        g_debug_tracer.set<3>(g_context->estimated_Idq[1] * 1e3F);
        g_debug_tracer.set<4>(g_context->reference_Iq     * 1e3F);
    }

    /*
     * Motor identification
     */
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
                                         MotorIdentificationCurrent,
                                         MotorIdentificationCurrentFrequency,
                                         MotorIdentificationPhiAngularVelocity,
                                         period,
                                         board::motor::getPWMDeadTime());
        }

        if (board::motor::isCalibrationInProgress())
        {
            g_pwm_handle.release(); // Nothing to do, waiting for the calibration to end before continuing
        }
        else
        {
            const auto pwm_vector = estimator->onNextPWMPeriod(phase_currents_ab, inverter_voltage);

            g_pwm_handle.setPWM(pwm_vector);

            if (estimator->isFinished())
            {
                g_pwm_handle.release();

                g_motor_params = estimator->getEstimatedMotorParameters();

                g_state = g_motor_params.isValid() ? State::Idle : State::Fault;
            }
        }

        const auto filtered_currents = estimator->getCurrentsFilter().getValue();

        g_debug_tracer.set<0>(estimator->getStateVariables()[0]);
        g_debug_tracer.set<1>(estimator->getStateVariables()[1]);
        g_debug_tracer.set<2>(estimator->getStateVariables()[2]);
        g_debug_tracer.set<3>(estimator->getStateVariables()[3]);
        g_debug_tracer.set<4>(filtered_currents[0]);
        g_debug_tracer.set<5>(filtered_currents[1]);
        g_debug_tracer.set<6>(filtered_currents.norm());
    }

    /*
     * Secondary states
     */
    if (g_state == State::Idle ||
        g_state == State::Fault)
    {
        if (estimator != nullptr)
        {
            estimator->~MotorParametersEstimator();
            estimator = nullptr;
        }

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
