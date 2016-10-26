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

#include "common.hpp"
#include "resistance.hpp"
#include "inductance.hpp"
#include "magnetic_flux.hpp"


namespace foc
{
namespace motor_id
{
/**
 * Motor identification logic.
 * Refer to the Dmitry's doc for derivations and explanation of what's going on here.
 */
class MotorIdentificationTask : public ITask
{
    struct ContextImplementation : public SubTaskContext
    {
        std::uint32_t pwm_period_counter = 0;
        Vector<3> pwm_output_vector = Vector<3>::Zero();
        std::array<Scalar, ITask::NumDebugVariables> debug_values_{};

        ContextImplementation(const TaskContext& cont)
        {
            static_cast<TaskContext&>(*this) = cont;
        }

        void setPWM(const Vector<3>& pwm) override
        {
            AbsoluteCriticalSectionLocker locker;
            pwm_output_vector = pwm;
        }

        void reportDebugVariables(const std::initializer_list<Scalar>& variables) override
        {
            AbsoluteCriticalSectionLocker locker;
            std::copy_n(variables.begin(),
                        std::min(variables.size(), debug_values_.size()),
                        debug_values_.begin());
        }

        Scalar getTime() const override
        {
            // Locking is not necessary because the read is atomic
            return Scalar(pwm_period_counter) * board.pwm.period;
        }
    } context_;

    static constexpr Result::ExitCode ExitCodeBadHardwareStatus     = Result::MaxExitCode - 0;
    static constexpr Result::ExitCode ExitCodeInvalidParameters     = Result::MaxExitCode - 1;

    MotorParameters result_;
    std::uint8_t next_task_index_ = 0;
    ISubTask* current_task_ = nullptr;
    void (MotorIdentificationTask::* const* const task_chain_)();

    alignas(32) std::uint8_t vinnie_the_pool_[std::max({
        sizeof(ResistanceTask),
        sizeof(InductanceTask),
        sizeof(MagneticFluxTask)
    })]{};

    void destroyCurrentTask()
    {
        if (current_task_ != nullptr)
        {
            current_task_->~ISubTask();
            current_task_ = nullptr;
        }
    }

    template <typename Task>
    void switchTask()
    {
        static_assert(sizeof(Task) <= sizeof(vinnie_the_pool_), "Vinnie the Pool is not large enough :(");
        destroyCurrentTask();
        std::fill(std::begin(vinnie_the_pool_), std::end(vinnie_the_pool_), 0);
        current_task_ = new (vinnie_the_pool_) Task(context_, result_);
    }

    static void (MotorIdentificationTask::* const* selectTaskChain(Mode mode))()
    {
        switch (mode)
        {
        case Mode::Static:
        {
            static constexpr void (MotorIdentificationTask::* chain[])() = {
                &MotorIdentificationTask::switchTask<ResistanceTask>,
                &MotorIdentificationTask::switchTask<InductanceTask>,
                nullptr
            };
            return chain;
        }
        case Mode::RotationWithoutMechanicalLoad:
        {
            static constexpr void (MotorIdentificationTask::* chain[])() = {
                &MotorIdentificationTask::switchTask<ResistanceTask>,
                &MotorIdentificationTask::switchTask<InductanceTask>,
                &MotorIdentificationTask::switchTask<MagneticFluxTask>,
                nullptr
            };
            return chain;
        }
        }
        assert(false);
        return nullptr;
    }

public:
    MotorIdentificationTask(const TaskContext& context,
                            Mode mode) :
        context_(context),
        result_(context.params.motor),
        task_chain_(selectTaskChain(mode))
    { }

    ~MotorIdentificationTask()
    {
        destroyCurrentTask();
    }

    const char* getName() const override { return "motor_id"; }

    Result onMainIRQ(Const period, const board::motor::Status& hw_status) override
    {
        // TODO: We can't check the general hardware status because FAULT tends to go up randomly.
        //       There might be a hardware bug somewhere. Investigate it later.
        //if (hw_status.isOkay())
        if (!hw_status.power_ok || hw_status.overload)
        {
            return Result::failure(ExitCodeBadHardwareStatus);
        }

        if (current_task_ == nullptr)
        {
            AbsoluteCriticalSectionLocker locker;

            // Making sure the parameters are sane
            if (!context_.params.motor_id.isValid())
            {
                return Result::failure(ExitCodeInvalidParameters);
            }

            // Switching to the next state
            const auto constructor = task_chain_[next_task_index_];
            if (constructor == nullptr)
            {
                return Result::success();
            }
            else
            {
                next_task_index_++;
                (this->*constructor)();
            }
        }
        else
        {
            // This is the only brief period of time where we aren't IRQ-safe.
            AbsoluteCriticalSectionLocker::assertNotLocked();
            current_task_->onMainIRQ(period);

            // Immediately once the business logic processing is finished, lock again
            AbsoluteCriticalSectionLocker locker;

            const auto status = current_task_->getStatus();
            if (status != ISubTask::Status::InProgress)
            {
                result_ = current_task_->getEstimatedMotorParameters();

                destroyCurrentTask();

                if (status == ISubTask::Status::Failed)
                {
                    assert(next_task_index_ > 0);
                    return Result::failure(next_task_index_);
                }
            }
        }
        return Result::inProgress();
    }

    std::pair<Vector<3>, bool> onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                                               Const inverter_voltage) override
    {
        AbsoluteCriticalSectionLocker::assertNotLocked();

        if (context_.pwm_period_counter < std::numeric_limits<decltype(context_.pwm_period_counter)>::max())
        {
            context_.pwm_period_counter++;
        }
        else
        {
            // This can't happen.
            assert(false);
            result_ = MotorParameters();
            return { Vector<3>::Zero(), false };
        }

        context_.pwm_output_vector.setZero();   // Default

        if (current_task_ != nullptr)
        {
            current_task_->onNextPWMPeriod(phase_currents_ab, inverter_voltage);
        }

        return {context_.pwm_output_vector, true};
    }

    void applyResultToGlobalContext(TaskContext& inout_context) const override
    {
        inout_context.params.motor = result_;
    }

    std::array<Scalar, NumDebugVariables> getDebugVariables() const override { return context_.debug_values_; }
};

}
}
