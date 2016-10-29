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
#include "transforms.hpp"
#include "voltage_modulator.hpp"
#include "irq_debug.hpp"

// Tasks:
#include "idle_task.hpp"
#include "fault_task.hpp"
#include "beeping_task.hpp"
#include "running_task.hpp"
#include "hw_test/task.hpp"
#include "motor_id/task.hpp"


/*
 * Documents:
 *  - http://cache.nxp.com/files/microcontrollers/doc/ref_manual/DRM148.pdf
 */
namespace foc
{
namespace
{

TaskContext g_context;

board::motor::PWMHandle g_pwm_handle;

IRQDebugPlotter g_debug_plotter;

using motor_id::MotorIdentificationTask;
using hw_test::HardwareTestingTask;

typedef TaskHandler
< IdleTask
, FaultTask
, BeepingTask
, RunningTask
, HardwareTestingTask
, MotorIdentificationTask
> TaskHandlerInstance;

TaskHandlerInstance g_task_handler([]() { return g_context; });


inline Scalar convertElectricalAngularVelocityToMechanicalRPM(Const eangvel)
{
    return convertRotationRateElectricalToMechanical(convertAngularVelocityToRPM(eangvel),
                                                     g_context.params.motor.num_poles);
}

} // namespace


void init(const Parameters& params)
{
    board::motor::beginCalibration();

    g_context.params = params;

    g_context.board.pwm     = board::motor::getPWMParameters();
    g_context.board.limits  = board::motor::getLimits();

    {
        AbsoluteCriticalSectionLocker locker;
        g_task_handler.select<IdleTask>();
    }

    DEBUG_LOG("FOC sizeof: %u %u %u %u\n",
              sizeof(g_task_handler), sizeof(MotorIdentificationTask), sizeof(g_context), sizeof(g_context.params));
}

void setParameters(const Parameters& params)
{
    AbsoluteCriticalSectionLocker locker;
    g_context.params = params;
    g_task_handler.from<IdleTask>().to<IdleTask>(); // Cycling to reload new configuration and check it
}

Parameters getParameters()
{
    AbsoluteCriticalSectionLocker locker;
    return g_context.params;
}

MotorParameters getMotorParameters()
{
    AbsoluteCriticalSectionLocker locker;
    return g_context.params.motor;
}

hw_test::Report getHardwareTestReport()
{
    AbsoluteCriticalSectionLocker locker;
    return g_context.hw_test_report;
}

void beginMotorIdentification(motor_id::Mode mode)
{
    g_task_handler.from<IdleTask, BeepingTask>().to<MotorIdentificationTask>(mode);
}

void beginHardwareTest()
{
    g_task_handler.from<IdleTask, BeepingTask, FaultTask>().to<HardwareTestingTask>();
}

bool isMotorIdentificationInProgress(MotorIdentificationStateInfo* out_info)
{
    if (out_info != nullptr)
    {
        AbsoluteCriticalSectionLocker locker;
        if (auto task = g_task_handler.as<MotorIdentificationTask>())
        {
            out_info->progress = task->getProgress();
            out_info->inverter_power_filtered = 0;
            out_info->mechanical_rpm = 0;
            return true;
        }
        return false;
    }
    else
    {
        return g_task_handler.is<MotorIdentificationTask>();
    }
}

bool isHardwareTestInProgress()
{
    return g_task_handler.is<HardwareTestingTask>();
}

bool isRunning(RunningStateInfo* out_info,
               bool* out_spinup_in_progress)
{
    AbsoluteCriticalSectionLocker locker;
    if (auto task = g_task_handler.as<RunningTask>())
    {
        if (out_info != nullptr)
        {
            out_info->stall_count = task->getNumSuccessiveStalls();

            const auto filt = task->getLowPassFilteredValues();
            out_info->inverter_power_filtered = filt.inverter_power;
            out_info->demand_factor_filtered = filt.demand_factor;

            out_info->mechanical_rpm =
                convertElectricalAngularVelocityToMechanicalRPM(task->getElectricalAngularVelocity());
        }

        if (out_spinup_in_progress != nullptr)
        {
            *out_spinup_in_progress = task->isSpinupInProgress();
        }
        return true;
    }
    return false;
}

bool isInactive(InactiveStateInfo* out_info)
{
    AbsoluteCriticalSectionLocker locker;
    if (g_task_handler.either<IdleTask, BeepingTask, FaultTask>())
    {
        if (out_info != nullptr)
        {
            if (auto task = g_task_handler.as<FaultTask>())
            {
                out_info->fault_code = task->getFaultCode();
            }
            else
            {
                out_info->fault_code = 0;
            }
        }
        return true;
    }
    return false;
}

ExtendedStatus getExtendedStatus()
{
    AbsoluteCriticalSectionLocker locker;
    return {
        g_task_handler.get().getName(),
        g_task_handler.getTaskSwitchCounter()
    };
}

void setSetpoint(ControlMode control_mode,
                 Const value,
                 Const request_ttl)
{
    AbsoluteCriticalSectionLocker locker;
    if (auto task = g_task_handler.as<RunningTask>())
    {
        task->setSetpoint(control_mode, value, request_ttl);
    }
    else
    {
        if (os::float_eq::closeToZero(value))
        {
            g_task_handler.from<FaultTask, MotorIdentificationTask>().to<IdleTask>();
        }
        else
        {
            g_task_handler.from<IdleTask, BeepingTask>().to<RunningTask>(control_mode, value, request_ttl);
        }
    }
}

void beep(Const frequency, Const duration)
{
    g_task_handler.from<IdleTask>().to<BeepingTask>(frequency, duration);
}

void plotRealTimeValues()
{
    g_debug_plotter.print();
    IRQDebugOutputBuffer::printIfNeeded();
}

std::array<DebugKeyValueType, NumDebugKeyValuePairs> getDebugKeyValuePairs()
{
    Vector<2> Idq = Vector<2>::Zero();
    Vector<2> Udq = Vector<2>::Zero();

    {
        AbsoluteCriticalSectionLocker locker;
        if (auto task = g_task_handler.as<RunningTask>())
        {
            Idq = task->getIdq();
            Udq = task->getUdq();
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
    const auto hw_status = board::motor::getStatus();

    static TaskHandlerInstance::SwitchCounter last_task_switch_counter;
    const auto new_task_switch_counter = g_task_handler.getTaskSwitchCounter();
    if (new_task_switch_counter != last_task_switch_counter)
    {
        AbsoluteCriticalSectionLocker locker;
        last_task_switch_counter = new_task_switch_counter;
        // OK we just switched task, cool.
        if (g_task_handler.get().isPreCalibrationRequired())
        {
            g_pwm_handle.release();
            board::motor::beginCalibration();
        }
    }

    if (!board::motor::isCalibrationInProgress())
    {
        auto& task = g_task_handler.get();

        const auto result = task.onMainIRQ(period, hw_status);

        if (result.finished)
        {
            AbsoluteCriticalSectionLocker locker;

            task.applyResultToGlobalContext(g_context);

            if (result.exit_code == result.ExitCodeOK)
            {
                assert(!g_task_handler.is<IdleTask>());         // Idle task shouldn't finish
                g_task_handler.select<IdleTask>();
            }
            else
            {
                assert(!g_task_handler.is<FaultTask>());        // Fault task shouldn't fail
                // Fault code consists of the task ID (takes the upper 4 bits) and its failure code (lower bits)
                const auto fault_code =
                    std::uint16_t((g_task_handler.getTaskID() << 12) | (result.exit_code & 0x0FFFU));
                g_task_handler.select<FaultTask>(fault_code);
            }
        }
        else
        {
            std::array<Scalar, ITask::NumDebugVariables> vars;
            {
                AbsoluteCriticalSectionLocker locker;
                vars = task.getDebugVariables();
            }
            g_debug_plotter.set(vars);
        }
    }
}


void handleFastIRQ(Const period,
                   const Vector<2>& phase_currents_ab,
                   Const inverter_voltage)
{
    (void) period;

    if (board::motor::isCalibrationInProgress())
    {
        g_pwm_handle.release();
    }
    else
    {
        const auto out = g_task_handler.get().onNextPWMPeriod(phase_currents_ab, inverter_voltage);
        if (out.second)
        {
            g_pwm_handle.setPWM(out.first);
        }
        else
        {
            g_pwm_handle.release();
        }
    }
}

}
}
