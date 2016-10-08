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

#include "aux_cmd_iface.hpp"
#include <zubax_chibios/os.hpp>
#include <params.hpp>
#include <unistd.h>
#include <foc/foc.hpp>


namespace aux_cmd_iface
{
namespace
{

enum class Command
{
    None,

    HardwareTest,
    MotorIDStatic,
    MotorIDRotating,

    ComeOnHowComeNewCppStandardsDontSupportEnumSizeTraits
};

os::config::Param<unsigned> g_param_cmd("exec_aux_command", 0, 0,
                                        unsigned(Command::ComeOnHowComeNewCppStandardsDontSupportEnumSizeTraits) - 1);

os::Logger g_logger("Aux Cmd Iface");


class Thread : public chibios_rt::BaseStaticThread<2048>
{
    static constexpr float WatchdogTimeout = 1.0F;

    os::watchdog::Timer watchdog_;


    void waitFor(float duration)
    {
        assert(duration < WatchdogTimeout);
        watchdog_.reset();
        ::usleep(unsigned(duration * 1e6F));
        watchdog_.reset();
    }

    void execute(const Command cmd)
    {
        class LoggingHelper
        {
            const ::systime_t started_at_ = chVTGetSystemTimeX();
            const Command command_;

        public:
            LoggingHelper(const Command cmd) :
                command_(cmd)
            {
                g_logger.println("Beginning execution of command %u...\n", unsigned(command_));
            }

            ~LoggingHelper()
            {
                const auto elapsed = float(ST2US(chVTTimeElapsedSinceX(started_at_))) / 1e6F;
                g_logger.println("Execution of command %u finished in %.1f seconds\n",
                              unsigned(command_), double(elapsed));
            }
        } logging_helper(cmd);

        switch (cmd)
        {
        case Command::HardwareTest:
        {
            foc::beginHardwareTest();
            while (foc::getState() == foc::State::HardwareTesting)
            {
                waitFor(0.01F);
            }
            g_logger.println("HW test result: %s", foc::getLastHardwareTestReport().toString().c_str());
            break;
        }

        case Command::MotorIDStatic:
        case Command::MotorIDRotating:
        {
            static const auto is_inactive = []()
            {
                const auto state = foc::getState();
                return state == foc::State::Idle ||
                       state == foc::State::Fault;
            };

            /*
             * Aborting immediately if busy
             */
            if (!is_inactive())
            {
                g_logger.puts("Bad state");
                break;
            }

            /*
             * Hardware testing
             */
            foc::beginHardwareTest();
            while (foc::getState() == foc::State::HardwareTesting)
            {
                waitFor(0.01F);
            }

            if (!foc::getLastHardwareTestReport().isSuccessful() ||
                !is_inactive())
            {
                g_logger.println("Bad state or HW test failure [%s]",
                                 foc::getLastHardwareTestReport().toString().c_str());
                break;
            }

            /*
             * Identification
             */
            if (cmd == Command::MotorIDStatic)
            {
                foc::beginMotorIdentification(foc::MotorIdentificationMode::Static);
            }
            else if (cmd == Command::MotorIDRotating)
            {
                foc::beginMotorIdentification(foc::MotorIdentificationMode::RotationWithoutMechanicalLoad);
            }
            else
            {
                assert(false);
                break;
            }

            while (foc::getState() == foc::State::MotorIdentification)
            {
                waitFor(0.01F);
            }

            /*
             * Saving the results
             */
            const auto result = foc::getMotorParameters();
            g_logger.println("Motor params:\n%s", result.toString().c_str());
            params::writeMotorParameters(result);

            break;
        }

        case Command::None:
        case Command::ComeOnHowComeNewCppStandardsDontSupportEnumSizeTraits:
        {
            g_logger.println("INVALID COMMAND %u\n", unsigned(cmd));
            break;
        }
        }
    }

    void main() override
    {
        watchdog_.startMSec(unsigned(WatchdogTimeout * 1e3F));

        setName("aux_cmd_iface");

        // First things first
        g_param_cmd.set(unsigned(Command::None));

        while (!os::isRebootRequested())
        {
            waitFor(0.1F);

            const auto cmd = Command(g_param_cmd.get());
            if (cmd != Command::None)
            {
                g_param_cmd.set(unsigned(Command::None));
                execute(cmd);
            }
        }

        g_logger.puts("Stopped\n");
    }

public:
    virtual ~Thread() { }
} g_thread;

}

void init()
{
    g_thread.start(NORMALPRIO - 5);
}

}
