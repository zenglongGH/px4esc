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
#include <motor_database/motor_database.hpp>
#include <unistd.h>
#include <foc/foc.hpp>
#include <uavcan_node/uavcan_node.hpp>


namespace aux_cmd_iface
{
namespace
{

constexpr int CmdNone                   = -1;

constexpr int CmdLoadMotorParamsFromDB  = 0;

constexpr int CmdHardwareTest           = 1000;
constexpr int CmdMotorIDStatic          = 1001;
constexpr int CmdMotorIDRotating        = 1002;


os::config::Param<int> g_param_cmd("exec_aux_command", -1, -1, 9999);

os::Logger g_logger("AuxCmdIface");


class Thread : public chibios_rt::BaseStaticThread<2048>
{
    static constexpr float WatchdogTimeout = 1.0F;

    mutable os::watchdog::Timer watchdog_;


    void waitFor(float duration) const
    {
        assert(duration < WatchdogTimeout);
        watchdog_.reset();
        ::usleep(unsigned(duration * 1e6F));
        watchdog_.reset();
    }

    template <typename... Args>
    void log(decltype(uavcan_node::LogLevel::INFO) level, const char* format, Args... args) const
    {
        uavcan_node::log(level, g_logger.getName(), format, args...);
    }

    void doLoadMotorParams(unsigned db_index) const
    {
        const auto entry = motor_database::getByIndex(db_index);
        log(uavcan_node::LogLevel::INFO, "MotorDB selected '%s'", entry.name.c_str());

        params::writeMotorParameters(entry.parameters);
    }

    void doHardwareTest() const
    {
        foc::beginHardwareTest();
        while (foc::getState() == foc::State::HardwareTesting)
        {
            waitFor(0.01F);
        }
        const auto report = foc::getLastHardwareTestReport();
        log(report.isSuccessful() ? uavcan_node::LogLevel::INFO : uavcan_node::LogLevel::WARNING,
            "HW test result: %s", report.toString().c_str());
    }

    void doMotorID(foc::MotorIdentificationMode mode) const
    {
        static const auto is_inactive = []()
        {
            const auto state = foc::getState();
            return state == foc::State::Idle ||
                   state == foc::State::Fault;
        };

        // Aborting immediately if busy
        if (!is_inactive())
        {
            g_logger.puts("Bad state");
            return;
        }

        // Hardware testing
        doHardwareTest();

        if (!foc::getLastHardwareTestReport().isSuccessful() ||
            !is_inactive())
        {
            g_logger.puts("Bad state or HW test failure");
            return;
        }

        // Identification
        foc::beginMotorIdentification(mode);

        while (foc::getState() == foc::State::MotorIdentification)
        {
            waitFor(0.01F);
        }

        // Saving the results
        const auto result = foc::getMotorParameters();
        g_logger.println("Motor params:\n%s", result.toString().c_str());
        params::writeMotorParameters(result);
    }

    void execute(const int cmd)
    {
        class LoggingHelper
        {
            const ::systime_t started_at_ = chVTGetSystemTimeX();
            const int command_;

        public:
            LoggingHelper(const int cmd) :
                command_(cmd)
            {
                g_logger.println("Beginning execution of command %d...", command_);
            }

            ~LoggingHelper()
            {
                const auto elapsed = float(ST2US(chVTTimeElapsedSinceX(started_at_))) / 1e6F;
                g_logger.println("Execution of command %d finished in %.1f seconds",
                                 command_, double(elapsed));
            }
        } logging_helper(cmd);

        if (cmd >= CmdLoadMotorParamsFromDB &&
            cmd <= (CmdLoadMotorParamsFromDB + int(motor_database::getMaxIndex())))
        {
            doLoadMotorParams(unsigned(cmd - CmdLoadMotorParamsFromDB));
        }
        else if (cmd == CmdHardwareTest)
        {
            doHardwareTest();
        }
        else if (cmd == CmdMotorIDStatic)
        {
            doMotorID(foc::MotorIdentificationMode::Static);
        }
        else if (cmd == CmdMotorIDRotating)
        {
            doMotorID(foc::MotorIdentificationMode::RotationWithoutMechanicalLoad);
        }
        else
        {
            g_logger.println("INVALID COMMAND %d", cmd);
        }
    }

    void main() override
    {
        watchdog_.startMSec(unsigned(WatchdogTimeout * 1e3F));

        setName("aux_cmd_iface");

        // First things first
        g_param_cmd.set(CmdNone);

        while (!os::isRebootRequested())
        {
            waitFor(0.1F);

            const auto cmd = g_param_cmd.get();
            if (cmd >= 0)
            {
                g_param_cmd.set(CmdNone);
                execute(cmd);
            }
        }

        g_logger.puts("Stopped");
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
