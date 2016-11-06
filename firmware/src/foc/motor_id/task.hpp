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
#include "flux_linkage.hpp"
#include "resistance.hpp"
#include "inductance.hpp"


namespace foc
{
namespace motor_id
{
/**
 * This part is internal to the motor ID task. Do not use it from outside.
 */
template <typename... TaskList>
class SubTaskSequencer
{
    SubTaskSequencer(const volatile SubTaskSequencer&) = delete;
    SubTaskSequencer(const volatile SubTaskSequencer&&) = delete;
    SubTaskSequencer& operator=(const volatile SubTaskSequencer&) = delete;
    SubTaskSequencer& operator=(const volatile SubTaskSequencer&&) = delete;

    typedef TypeEnumeration<TaskList...> Tasks;
    friend Tasks;   // This is needed for type resolution methods

    // We multiply the sequence length by 2 in order to allow repeating tasks, should it become necessary.
    static constexpr std::uint8_t MaxSequenceLength = sizeof...(TaskList) * 2;

    std::array<std::uint8_t, MaxSequenceLength> sequence_{};
    std::uint8_t sequence_length_ = 0;
    std::uint8_t current_task_index_ = 0;
    ISubTask* current_task_ = nullptr;

    alignas(Tasks::LargestAlignment) mutable std::uint8_t pool_[Tasks::LargestSize];

    SubTaskContext& context_ref_;
    MotorParameters& result_ref_;

    template <typename T>
    ISubTask* onTypeResolutionSuccess() const
    {
        static_assert(sizeof(T) <= sizeof(pool_), "Congratulations, you broke your C++ compiler!");
        return new (pool_) T(context_ref_, result_ref_);
    }

    ISubTask* onTypeResolutionFailure() const
    {
        chibios_rt::System::halt("motor_id::SubTaskSequencer TYPE RESOLVER");     // Oh how I miss RTTI
        return nullptr;
    }

    void destroyCurrentTask()
    {
        if (current_task_ != nullptr)
        {
            current_task_->~ISubTask();
            current_task_ = nullptr;
        }
    }

public:
    SubTaskSequencer(SubTaskContext& context_reference,
                     MotorParameters& result_reference) :
        context_ref_(context_reference),
        result_ref_(result_reference)
    { }

    ~SubTaskSequencer() { destroyCurrentTask(); }

    template <typename... TaskTypes>
    void setSequence()
    {
        static_assert(sizeof...(TaskTypes) > 0, "Zero length sequence is not allowed");
        static_assert(sizeof...(TaskTypes) <= MaxSequenceLength, "Sequence is too long");

        assert(sequence_length_ == 0);
        assert(current_task_index_ == 0);

        destroyCurrentTask();

        sequence_ = {Tasks::template getID<TaskTypes>()...};
        sequence_length_ = sizeof...(TaskTypes);
        current_task_index_ = 0;

        current_task_ = Tasks::findTypeByID(*this, current_task_index_);
    }

    bool selectNextTask()
    {
        if (current_task_index_ + 1 < sequence_length_)
        {
            destroyCurrentTask();
            current_task_index_++;
            current_task_ = Tasks::findTypeByID(*this, current_task_index_);
            return true;
        }
        return false;
    }

    ISubTask& getCurrentTask()
    {
        assert(current_task_ != nullptr);
        return *current_task_;
    }

    std::uint8_t getSequenceLength() const { return sequence_length_; }

    std::uint8_t getCurrentTaskIndex() const { return current_task_index_; }
};

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
        std::array<Scalar, ITask::NumDebugVariables> debug_values{};

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
                        std::min(variables.size(), debug_values.size()),
                        debug_values.begin());
        }

        Scalar getTime() const override
        {
            // Locking is not necessary because the read is atomic
            return Scalar(pwm_period_counter) * board.pwm.period;
        }
    } context_;

    static constexpr Result::ExitCode ExitCodeBadHardwareStatus     = Result::MaxExitCode - 0;
    static constexpr Result::ExitCode ExitCodeInvalidParameters     = Result::MaxExitCode - 1;
    static constexpr Result::ExitCode ExitCodeInvalidSequence       = Result::MaxExitCode - 2;
    static constexpr Result::ExitCode ExitCodeHardwareTestFailed    = Result::MaxExitCode - 3;

    const Mode mode_;

    MotorParameters result_;

    SubTaskSequencer
    < ResistanceTask
    , InductanceTask
    , FluxLinkageTask
    > sequencer_;

    bool started_ = false;
    bool processing_enabled_ = false;   ///< This is used instead of critical sections to gate PWM IRQ processing

public:
    MotorIdentificationTask(const TaskContext& context,
                            const Mode mode) :
        context_(context),
        mode_(mode),
        result_(context.params.motor),
        sequencer_(context_, result_)
    { }

    const char* getName() const override { return "motor_id"; }

    Result onMainIRQ(Const period, const board::motor::Status& hw_status) override
    {
        if (!started_)
        {
            // Note that we're not taking a critical section here.
            switch (mode_)
            {
            case Mode::Static:
            {
                sequencer_.setSequence<ResistanceTask, InductanceTask>();
                break;
            }
            case Mode::RotationWithoutMechanicalLoad:
            {
                sequencer_.setSequence<ResistanceTask, InductanceTask, FluxLinkageTask>();
                break;
            }
            default:
            {
                assert(false);
                return Result::failure(ExitCodeInvalidSequence);
            }
            }

            if (!context_.params.motor_id.isValid())
            {
                return Result::failure(ExitCodeInvalidParameters);
            }

            if (!context_.hw_test_report.isSuccessful())
            {
                return Result::failure(ExitCodeHardwareTestFailed);
            }

            started_ = true;
        }

        // TODO: We can't check the general hardware status because FAULT tends to go up randomly.
        //       There might be a hardware bug somewhere. Investigate it later.
        //if (hw_status.isOkay())
        if (!hw_status.power_ok || hw_status.overload)
        {
            return Result::failure(ExitCodeBadHardwareStatus);
        }

        AbsoluteCriticalSectionLocker::assertNotLocked();
        sequencer_.getCurrentTask().onMainIRQ(period);

        processing_enabled_ = true;     // This guarantees that the main IRQ is always served first after construction.

        ISubTask::Status status{};
        {
            AbsoluteCriticalSectionLocker locker;
            status = sequencer_.getCurrentTask().getStatus();
        }

        if (status != ISubTask::Status::InProgress)
        {
            {
                AbsoluteCriticalSectionLocker locker;
                result_ = sequencer_.getCurrentTask().getEstimatedMotorParameters();
                // Pausing processing to prevent race conditions. Will be restored on the next call.
                processing_enabled_ = false;
            }

            if (status == ISubTask::Status::Failed)
            {
                // +1 because we can't use zero - it would mean that there's no failure
                return Result::failure(Result::ExitCode(sequencer_.getCurrentTaskIndex() + 1));
            }

            /*
             * Construction of the next task may take a VERY long time (like 50+ microseconds), because some task
             * classes initialize large data structures or buffers. We don't want to take a critical section here.
             */
            if (!sequencer_.selectNextTask())
            {
                return Result::success();
            }
        }

        return Result::inProgress();
    }

    std::pair<Vector<3>, bool> onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                                               Const inverter_voltage) override
    {
        AbsoluteCriticalSectionLocker::assertNotLocked();

        if (!processing_enabled_)
        {
            return {};
        }

        if (context_.pwm_period_counter < std::numeric_limits<decltype(context_.pwm_period_counter)>::max())
        {
            context_.pwm_period_counter++;
        }
        else
        {
            chibios_rt::System::halt("MotorIdentificationTask TIME OVERFLOW");   // This can't happen.
            return {};
        }

        context_.pwm_output_vector.setZero();   // Default

        sequencer_.getCurrentTask().onNextPWMPeriod(phase_currents_ab, inverter_voltage);

        return {context_.pwm_output_vector, true};
    }

    void applyResultToGlobalContext(TaskContext& inout_context) const override
    {
        inout_context.params.motor = result_;
    }

    bool isPreCalibrationRequired() const override { return true; }

    std::array<Scalar, NumDebugVariables> getDebugVariables() const override { return context_.debug_values; }

    Scalar getProgress() const
    {
        // For purposes of progress estimation, we consider pre-calibration as a dedicated task in the sequencer
        if (!started_)
        {
            return 0.0F;
        }
        else
        {
            return Scalar(sequencer_.getCurrentTaskIndex() + 1) / Scalar(sequencer_.getSequenceLength() + 1);
        }
    }
};

}
}
