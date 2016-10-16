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
 * Magical and hard-to-use class that estimates parameters of a given motor, such as magnetic flux and inductance.
 * The business logic of this class outstretches its tentacles of dependency to a lot of other stuff,
 * so it's really hard to encapsulate it cleanly. Or maybe I'm just a shitty architect, you never know.
 *
 * Refer to the Dmitry's doc for derivations and explanation of what's going on here.
 */
class Estimator
{
    struct ContextImplementation : public Context
    {
        std::uint64_t pwm_period_counter = 0;
        Vector<3> pwm_output_vector = Vector<3>::Zero();
        std::array<Scalar, 7> debug_values_{};

        ContextImplementation(const Parameters& config,
                              Const pwm_period,
                              Const pwm_dead_time,
                              Const pwm_upper_limit) :
            Context(config,
                    pwm_period,
                    pwm_dead_time,
                    pwm_upper_limit)
        { }

        void setPWM(const Vector<3>& pwm) override
        {
            pwm_output_vector = pwm;
        }

        void setDebugVariable(unsigned index, Const value) override
        {
            debug_values_.at(index) = value;
        }

        Scalar getTime() const override { return Scalar(pwm_period_counter) * pwm_period; }
    } context_;

    MotorParameters result_;
    bool finished_ = false;
    unsigned current_task_index_ = 0;
    ITask* current_task_ = nullptr;
    void (Estimator::* const* current_task_chain_)() = nullptr;

    alignas(32) std::uint8_t vinnie_the_pool_[std::max({
        sizeof(ResistanceTask),
        sizeof(InductanceTask),
        sizeof(MagneticFluxTask)
    })];

    void destroyCurrentTask()
    {
        if (current_task_ != nullptr)
        {
            current_task_->~ITask();
            current_task_ = nullptr;
        }
    }

    template <typename Task>
    void switchTask()
    {
        static_assert(sizeof(Task) <= sizeof(vinnie_the_pool_), "Vinnie the Pool is not large enough :(");
        destroyCurrentTask();
        current_task_ = new (vinnie_the_pool_) Task(context_, result_);
    }

    static void (Estimator::* const* selectTaskChain(Mode mode))()
    {
        switch (mode)
        {
        case Mode::Static:
        {
            static constexpr void (Estimator::* chain[3])() = {
                &Estimator::switchTask<ResistanceTask>,
                &Estimator::switchTask<InductanceTask>,
                nullptr
            };
            return chain;
        }
        case Mode::RotationWithoutMechanicalLoad:
        {
            static constexpr void (Estimator::* chain[4])() = {
                &Estimator::switchTask<ResistanceTask>,
                &Estimator::switchTask<InductanceTask>,
                &Estimator::switchTask<MagneticFluxTask>,
                nullptr
            };
            return chain;
        }
        }
        assert(false);
        return nullptr;
    }

public:
    Estimator(Mode mode,
              const MotorParameters& initial_parameters,
              const Parameters& config,
              Const pwm_period,
              Const pwm_dead_time,
              Const pwm_upper_limit) :
        context_(config,
                 pwm_period,
                 pwm_dead_time,
                 pwm_upper_limit),
        result_(initial_parameters),
        current_task_chain_(selectTaskChain(mode))
    {
        assert(config.isValid());
    }

    ~Estimator()
    {
        destroyCurrentTask();
    }

    /**
     * Must be invoked on every PWM period with appropriate measurements.
     * Returns the desired PWM setpoints, possibly zero.
     */
    Vector<3> onNextPWMPeriod(const Vector<2>& phase_currents_ab,
                              Const inverter_voltage)
    {
        context_.pwm_output_vector = Vector<3>::Zero(); // Default
        context_.pwm_period_counter++;

        if (current_task_ == nullptr)
        {
            const auto constructor = current_task_chain_[current_task_index_++];
            if (constructor == nullptr)
            {
                finished_ = true;
            }
            else
            {
                (this->*constructor)();
            }
        }
        else
        {
            current_task_->onNextPWMPeriod(phase_currents_ab, inverter_voltage);

            const auto status = current_task_->getStatus();
            if (status != ITask::Status::InProgress)
            {
                result_ = current_task_->getEstimatedMotorParameters();

                destroyCurrentTask();

                if (status == ITask::Status::Failed)
                {
                    finished_ = true;
                }
            }
        }

        return context_.pwm_output_vector;
    }

    bool isFinished() const { return finished_; }

    const MotorParameters& getEstimatedMotorParameters() const { return result_; }

    auto getDebugValues() const { return context_.debug_values_; }
};

}
}
