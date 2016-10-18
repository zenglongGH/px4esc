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
 * State variables
 */
volatile State g_state = State::Idle;

CompleteParameterSet g_params;

board::motor::PWMHandle g_pwm_handle;

HardwareTestingTask::TestReport g_last_hardware_test_report;


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
                    double(pwm_cycles) * double(board::motor::getPWMParameters().period),
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


void doStop()
{
    AbsoluteCriticalSectionLocker locker;

    g_state = State::Idle;
    g_setpoint = 0;
    g_setpoint_remaining_ttl = 0;

    if (g_c != nullptr)
    {
        g_c->~Context();
        g_c = nullptr;
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
        g_c = nullptr;
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


HardwareTestingTask::TestReport getLastHardwareTestReport()
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

}

void stop()
{
    AbsoluteCriticalSectionLocker locker;

    if (g_state == State::MotorIdentification)
    {
        g_state = State::Fault;     // Will switch back to Idle later from IRQ
    }
}


Scalar getInstantCurrentFiltered()
{
    Const voltage = board::motor::getInverterVoltage();

    if (os::float_eq::positive(voltage))
    {
        AbsoluteCriticalSectionLocker locker;
        if (g_c != nullptr)
        {
            return g_c->low_pass_filtered_values.inverter_power / voltage;
        }
    }

    return 0;
}

Scalar getInstantDemandFactorFiltered()
{
    AbsoluteCriticalSectionLocker locker;
    if (g_c != nullptr)
    {
        return g_c->low_pass_filtered_values.demand_factor;
    }

    return 0;
}

Scalar getInstantMechanicalRPM()
{
    Scalar electrical_rad_sec = 0;

    {
        AbsoluteCriticalSectionLocker locker;
        if (g_c != nullptr)
        {
            electrical_rad_sec = g_c->angular_velocity;
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

        if (g_c != nullptr)
        {
            Udq_normalizations = g_c->modulator.getUdqNormalizationCounter();

            angular_velocity    = g_c->angular_velocity;
            estimated_Idq       = g_c->estimated_Idq;
            reference_Udq       = g_c->reference_Udq;
            setpoint_Iq         = g_c->setpoint_Iq;
            setpoint_Uq         = g_c->setpoint_Uq;
            inverter_power      = g_c->low_pass_filtered_values.inverter_power;
            demand_factor       = g_c->low_pass_filtered_values.demand_factor;
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
        if (g_c != nullptr)
        {
            Idq = g_c->estimated_Idq;
            Udq = g_c->reference_Udq;
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
            g_c->setpoint_Iq = 0;
            g_c->use_voltage_setpoint = true;
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

    /*
     * Motor identification
     */
    if (g_state == State::MotorIdentification)
    {
        if (g_motor_parameter_estimator == nullptr)
        {
            AbsoluteCriticalSectionLocker locker;

            board::motor::beginCalibration();

            alignas(motor_id::MotorIdentificationTask)
            static std::uint8_t estimator_storage[sizeof(motor_id::MotorIdentificationTask)];

            g_motor_parameter_estimator = new (estimator_storage)
                motor_id::MotorIdentificationTask(g_requested_motor_identification_mode,
                                    g_motor_params,
                                    g_controller_params.motor_id,
                                    g_observer_params,
                                    board::motor::getPWMParameters());
        }

        const auto hw_status = board::motor::getStatus();

        if (!board::motor::isCalibrationInProgress())
        {
            AbsoluteCriticalSectionLocker::assertNotLocked();
            g_motor_parameter_estimator->onMainIRQ(period);

            AbsoluteCriticalSectionLocker locker;

            // TODO: We can't check the general hardware status because FAULT tends to go up randomly.
            //       There might be a hardware bug somewhere. Investigate it later.
            //if (hw_status.isOkay())
            if (hw_status.power_ok && !hw_status.overload)
            {
                if (g_motor_parameter_estimator->isFinished())
                {
                    g_motor_params = g_motor_parameter_estimator->getEstimatedMotorParameters();

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
                g_state = State::Fault;
                IRQDebugOutputBuffer::setVariableFromIRQ<0>(hw_status.power_ok);
                IRQDebugOutputBuffer::setVariableFromIRQ<1>(hw_status.overload);
                IRQDebugOutputBuffer::setVariableFromIRQ<2>(hw_status.fault);
            }
        }

        g_debug_tracer.set(g_motor_parameter_estimator->getDebugValues());
        g_debug_tracer.set<6>(hw_status.current_sensor_gain);
    }
    else
    {
        if (g_motor_parameter_estimator != nullptr)
        {
            AbsoluteCriticalSectionLocker locker;
            g_motor_parameter_estimator->~MotorIdentificationTask();
            g_motor_parameter_estimator = nullptr;
        }
    }

    /*
     * Hardware testing
     */
    {
        static HardwareTestingTask* hardware_tester = nullptr;

        if (g_state == State::HardwareTesting)
        {
            AbsoluteCriticalSectionLocker locker;

            alignas(16) static std::uint8_t hardware_tester_storage[sizeof(HardwareTestingTask)];

            if (hardware_tester == nullptr)
            {
                board::motor::beginCalibration();

                const auto measurement_range = board::motor::getLimits().measurement_range;

                hardware_tester = new (hardware_tester_storage) HardwareTestingTask(measurement_range.inverter_voltage,
                                                                               measurement_range.inverter_temperature);
            }

            const auto phase_currents_ab = board::motor::getPhaseCurrentsAB();
            const auto inverter_voltage = board::motor::getInverterVoltage();

            const auto hw_status = board::motor::getStatus();

            if (board::motor::isCalibrationInProgress())
            {
                g_pwm_handle.release(); // Nothing to do, waiting for the calibration to end before continuing
            }
            else
            {
                const auto pwm_vector = hardware_tester->update(period,
                                                                phase_currents_ab,
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
                AbsoluteCriticalSectionLocker locker;
                hardware_tester->~HardwareTestingTask();
                hardware_tester = nullptr;
            }
        }
    }
}


void handleFastIRQ(Const period,
                   const Vector<2>& phase_currents_ab,
                   Const inverter_voltage)
{
    if (board::motor::isCalibrationInProgress())
    {
        g_pwm_handle.release();

        if (g_state == State::Running ||
            g_state == State::Spinup)
        {
            g_state = State::Idle;
        }
    }
    else
    {

    }
}

}
}
