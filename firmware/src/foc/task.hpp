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
#include "parameters.hpp"
#include "hw_test/report.hpp"
#include <functional>


namespace foc
{
/**
 * Entity that holds all immutable data tasks may need to read (but never write).
 */
struct TaskContext
{
    Parameters params;

    hw_test::Report hw_test_report;

    struct Board
    {
        board::motor::PWMParameters pwm;
        board::motor::Limits limits;
    } board;
};

/**
 * State specific task generalization.
 */
class ITask
{
    ITask(const volatile ITask&) = delete;
    ITask(const volatile ITask&&) = delete;
    ITask& operator=(const volatile ITask&) = delete;
    ITask& operator=(const volatile ITask&&)= delete;

protected:
    ITask() { }

public:
    static constexpr unsigned NumDebugVariables = 7;

    /**
     * Task update result.
     * When finished, the exit code may be set to a non-zero value to indicate failure and its cause.
     * Exit codes are not globally unique, i.e. multiple tasks may use the same code for different reasons.
     * Maximum exit code is limited in order to enable extension by the Task ID value in the outer logic.
     */
    struct Result
    {
        using ExitCode = std::uint16_t;
        static constexpr ExitCode ExitCodeOK = 0;
        static constexpr ExitCode MaxExitCode = 0x0FFF;

        bool finished = false;
        ExitCode exit_code = 0;     ///< Non-zero exit code means that the task has failed.

        static Result inProgress()
        {
            return Result();
        }

        static Result success()
        {
            return {true, ExitCodeOK};
        }

        static Result failure(std::uint16_t exit_code)
        {
            assert(exit_code > 0);
            assert(exit_code <= MaxExitCode);
            return {true, exit_code};
        }
    };

    virtual ~ITask() { }

    virtual const char* getName() const = 0;

    /**
     * It is guaranteed by the driver that the main IRQ is always invoked immediately after the fast IRQ
     * of the same period.
     * It is GUARANTEED that once this method resutns Result::finished = true,
     * it will never be invoked again on the same instance.
     */
    virtual Result onMainIRQ(Const period, const board::motor::Status& hw_status) = 0;

    /**
     * This method is invoked from the highest priority IRQ, preempting the main IRQ.
     */
    virtual std::pair<Vector<3>, bool> onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                                                       Const inverter_voltage)
    {
        (void) phase_currents_ab;
        (void) inverter_voltage;
        return {Vector<3>::Zero(), false};
    }

    /**
     * This method gives the task a chance to modify the global context with the result of its work.
     * It is invoked once after the task reported that it has finished (even if failed).
     */
    virtual void applyResultToGlobalContext(TaskContext& inout_context) const
    {
        (void) inout_context;
    }

    /**
     * Returned values will be transferred over to the real time plotting logic.
     */
    virtual std::array<Scalar, NumDebugVariables> getDebugVariables() const
    {
        return {};
    }
};

/**
 * Implementation details, do not go there.
 */
namespace impl_
{

template <int ThisID, typename ThisType, typename... Tail>
struct TypeEnumerationEntry
{
    static constexpr int ID = ThisID;
    using Type = ThisType;

    using Next = typename std::conditional<(sizeof...(Tail) > 0),
                                           TypeEnumerationEntry<ThisID + 1, Tail...>,
                                           void>::type;

    template <typename T, bool>
    struct GetIDImpl;

    template <typename T>
    struct GetIDImpl<T, true>
    {
        static constexpr int Result = ID;
    };

    template <typename T>
    struct GetIDImpl<T, false>
    {
        typedef typename Next::template GetID<T> Forwarder;
        static constexpr int Result = Forwarder::Result;
    };

    template <typename T>
    struct GetID
    {
        static constexpr int Result = GetIDImpl<T, std::is_same<T, ThisType>::value>::Result;
    };
};

}   // namespace impl_

/**
 * Alexandrescu-inspired type list, used to substitute the lack of runtime type identification.
 */
template <typename... TypeList>
struct TypeEnumeration
{
    static constexpr unsigned Length = sizeof...(TypeList);

    static constexpr unsigned LargestAlignment  = std::max({alignof(TypeList)...});
    static constexpr unsigned LargestSize       = std::max({sizeof(TypeList)...});

    // Void is necessary in order to fully unfold the argument list
    using Head = impl_::TypeEnumerationEntry<0, TypeList..., void>;

    template <typename T>
    static constexpr int getID()
    {
        typedef typename Head::template GetID<T> Forwarder;
        return Forwarder::Result;
    }
};

/**
 * Helper class used for switching ITasks.
 * It is guaranteed that some task is always selected.
 */
template <typename... TaskList>
class TaskHandler
{
    class NullPlaceholderTask : public ITask
    {
        const char* getName() const override { return ""; }
        Result onMainIRQ(Const, const board::motor::Status&) override { return {}; }
    public:
        NullPlaceholderTask(const TaskContext&) { }
    };

    typedef TypeEnumeration<NullPlaceholderTask, TaskList...> Tasks;
    static_assert(Tasks::Length < 256, "Too many tasks");

    using ContextCloner = std::function<TaskContext ()>;

    alignas(Tasks::LargestAlignment) std::uint8_t vinnie_the_pool_[Tasks::LargestSize]{};
    ITask* ptr_ = nullptr;
    std::uint8_t task_id_ = 0;
    ContextCloner context_cloner_;

    void destroy()
    {
        if (ptr_ != nullptr)
        {
            ptr_->~ITask();
            ptr_ = nullptr;
        }
        task_id_ = 0;
    }

    template <typename... SwitchFrom>
    struct ConditionalSwitchHelper
    {
        TaskHandler* owner_;

        explicit ConditionalSwitchHelper(TaskHandler* pwner) : owner_(pwner) { }

        template <typename SwitchTo, typename... Args>
        void to(Args... args)
        {
            AbsoluteCriticalSectionLocker locker;
            if (owner_->either<SwitchFrom...>())
            {
                owner_->select<SwitchTo>(std::forward<Args>(args)...);
            }
        }
    };

public:
    TaskHandler(ContextCloner context_cloner) :
        context_cloner_(context_cloner)
    {
        assert(context_cloner_);
        select<NullPlaceholderTask>();
    }

    ~TaskHandler() { destroy(); }

    template <typename T, typename... Args>
    void select(Args... args)
    {
        AbsoluteCriticalSectionLocker::assertLocked();
        static_assert(sizeof(T) <= sizeof(vinnie_the_pool_),
                      "Pool is not large enough, probably this type is not registered");
        destroy();
        assert(ptr_ == nullptr);
        // And now you are my handler
        ptr_ = new (vinnie_the_pool_) T(context_cloner_(), std::forward<Args>(args)...);
        // And I, I will execute your demands
        task_id_ = Tasks::template getID<T>();
    }

    template <typename... SwitchFrom>
    ConditionalSwitchHelper<SwitchFrom...> from()
    {
        return ConditionalSwitchHelper<SwitchFrom...>(this);
    }

    template <typename T>
    bool is() const { return Tasks::template getID<T>() == task_id_; }

    template <typename Head, typename... Tail>
    typename std::enable_if<sizeof...(Tail) == 0, bool>::type either() const
    {
        return is<Head>();
    }

    template <typename Head, typename... Tail>
    typename std::enable_if<(sizeof...(Tail) > 0), bool>::type either() const
    {
        AbsoluteCriticalSectionLocker::assertLocked();
        return is<Head>() || either<Tail...>();
    }

    template <typename T>
    T* as() { return is<T>() ? static_cast<T*>(ptr_) : nullptr; }

    ITask& get()
    {
        assert(ptr_ != nullptr);
        return *ptr_;
    }

    std::uint8_t getTaskID() const { return task_id_; }
};

}
