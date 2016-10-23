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

#include <foc/task.hpp>
#include <foc/voltage_modulator.hpp>
#include <foc/irq_debug.hpp>
#include <cstdint>


namespace foc
{
namespace motor_id
{
/**
 * Context of the whole identification process.
 * An object of this type is passed from task to task until the procedure is complete.
 * The virtual methods should be devirtualized by the optimizer.
 */
struct SubTaskContext : public TaskContext
{
    virtual ~SubTaskContext() { }

    virtual void setPWM(const Vector<3>& pwm) = 0;

    virtual void reportDebugVariables(const std::initializer_list<Scalar>& variables) = 0;

    /**
     * Returns monotonic time of constant rate but unknown phase.
     */
    virtual Scalar getTime() const = 0;
};

using SubTaskContextReference = SubTaskContext&;

/**
 * Interface of a motor ID task, e.g. resistance measurement.
 */
class ISubTask
{
public:
    enum class Status
    {
        InProgress,
        Succeeded,
        Failed
    };

    virtual ~ISubTask() { }

    /**
     * This method is invoked every main IRQ, which happens every N-th period of PWM.
     * @param period    Invocation period [seconds]
     */
    virtual void onMainIRQ(Const period) = 0;

    /**
     * This method is invoked at every PWM period, from the highest priority IRQ.
     * It preempts the main IRQ method.
     * This is the ONLY method that can be invoked from the PWM IRQ; all other methods can only be invoked
     * from the main IRQ.
     * @param phase_currents_ab
     * @param inverter_voltage
     */
    virtual void onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                                 Const inverter_voltage) = 0;

    /**
     * This method is always invoked from the main IRQ with the absolute critical section locked,
     * i.e. it cannot be interrupted by the PWM IRQ.
     */
    virtual Status getStatus() const = 0;

    /**
     * This method is always invoked from the main IRQ with the absolute critical section locked,
     * i.e. it cannot be interrupted by the PWM IRQ.
     */
    virtual MotorParameters getEstimatedMotorParameters() const = 0;
};

}
}
