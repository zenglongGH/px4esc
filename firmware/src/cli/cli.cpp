/****************************************************************************
*
*   Copyright (C) 2016  Zubax Robotics  <info@zubax.com>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

#include "cli.hpp"

#include <board/board.hpp>
#include <bootloader_interface/bootloader_interface.hpp>
#include <uavcan_node/uavcan_node.hpp>

#include <zubax_chibios/os.hpp>
#include <zubax_chibios/config/config.hpp>
#include <zubax_chibios/util/shell.hpp>
#include <zubax_chibios/util/base64.hpp>

#include <cstdlib>


namespace cli
{
namespace
{

RebootRequestCallback g_reboot_request_callback;


class RebootCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "reboot"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        if (g_reboot_request_callback && g_reboot_request_callback("CLI command"))
        {
            ; // Nothing to do, request accepted
        }
        else
        {
            ios.print("ERROR: REBOOT REQUEST REJECTED\n");
        }
    }
} static cmd_reboot;


class ZubaxIDCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "zubax_id"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        ios.print("product_id   : '%s'\n", PRODUCT_ID_STRING);
        ios.print("product_name : '%s'\n", PRODUCT_NAME_STRING);

        {
            const auto fw_version = bootloader_interface::getFirmwareVersion();
            ios.print("fw_version   : '%u.%u'\n", fw_version.major, fw_version.minor);
            ios.print("fw_vcs_commit: 0x%08x\n", unsigned(fw_version.vcs_commit));
            ios.print("fw_build_date: %s\n", __DATE__);
            ios.print("fw_crc64we   : 0x%016llx\n", fw_version.image_crc64we);
        }

        {
            const auto hw_version = board::detectHardwareVersion();
            ios.print("hw_version   : '%u.%u'\n", hw_version.major, hw_version.minor);
        }

        {
            char base64_buf[os::base64::predictEncodedDataLength(std::tuple_size<board::DeviceSignature>::value) + 1];

            ios.print("hw_unique_id : '%s'\n", os::base64::encode(board::readUniqueID(), base64_buf));

            board::DeviceSignature signature;
            if (board::tryReadDeviceSignature(signature))
            {
                ios.print("hw_signature : '%s'\n", os::base64::encode(signature, base64_buf));
            }
        }
    }
} static cmd_zubax_id;


class ConfigCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "cfg"; }

    void execute(os::shell::BaseChannelWrapper&, int argc, char** argv) override
    {
        (void) os::config::executeCLICommand(argc - 1, &argv[1]);
    }
} static cmd_cfg;


class UAVCANCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "uavcan"; }

    void execute(os::shell::BaseChannelWrapper&, int, char**) override
    {
        uavcan_node::printStatusInfo();
    }
} static cmd_uavcan;


class PWMCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "pwm"; }

    void execute(os::shell::BaseChannelWrapper& ios, int argc, char** argv) override
    {
        if (argc < 2)
        {
            ios.print("Usage: %s { on | off | <A> <B> <C> }\n", argv[0]);
            ios.print("Where A, B, C are phase values in [0, 1]\n");
            return;
        }

        if (0 == std::strncmp("on", argv[1], 2))
        {
            board::motor::pwm::activate();
            board::motor::driver::setGateDriverEnabled(true);
            ios.print("PWM activated, gate driver enabled\n");
        }
        else if (0 == std::strncmp("off", argv[1], 3))
        {
            board::motor::pwm::deactivate();
            board::motor::driver::setGateDriverEnabled(false);
            ios.print("PWM deactivated, gate driver disabled\n");
        }
        else
        {
            constexpr math::Range<> range(0, 1);
            math::Vector<3> abc = math::Vector<3>::Zero();

            for (int i = 0; i < std::min(3, argc); i++)
            {
                // strtof() returns 0 on failure, which is just what we need
                using namespace std;
                abc[i] = range.constrain(strtof(argv[i + 1], nullptr));
            }

            board::motor::pwm::set(abc);
            ios.print("PWM set to  %.3f  %.3f  %.3f\n", double(abc[0]), double(abc[1]), double(abc[2]));
        }
    }
} static cmd_pwm;


class StateCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "state"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        ios.print("Driver failure indicators: %s\n",
                  board::motor::driver::readFailureIndicators().toString().c_str());

        ios.print("PWM: Frequency %.6f kHz   DeadTime %.1f nsec\n",
                  double(board::motor::pwm::getFrequency() * 1e-3F),
                  double(board::motor::pwm::getDeadTime() * 1e9F));
    }
} static cmd_state;


class CLIThread : public chibios_rt::BaseStaticThread<2048>
{
    os::shell::Shell<> shell_;

    void main() override
    {
        /*
         * TODO: Add USB support in the future. At the moment this is not possible, because USB requires 48 MHz
         *       clock, and the application requires 180 MHz core clock; both can be obtained only if SAI is
         *       used to clock the USB controller. Unfortunately, the current version of ChibiOS does not support
         *       this configuration, but the necessary code is already in the upstream, so we'll just wait until
         *       it gets released, and then add USB support here.
         */
        while (!os::isRebootRequested())
        {
            os::shell::BaseChannelWrapper wrapper(os::getStdIOStream());
            shell_.runFor(wrapper, 1000);
        }

        os::lowsyslog("CLI: Stopped\n");
    }

public:
    CLIThread()
    {
        (void) shell_.addCommandHandler(&cmd_reboot);
        (void) shell_.addCommandHandler(&cmd_zubax_id);
        (void) shell_.addCommandHandler(&cmd_cfg);
        (void) shell_.addCommandHandler(&cmd_uavcan);
        (void) shell_.addCommandHandler(&cmd_pwm);
        (void) shell_.addCommandHandler(&cmd_state);
    }

    virtual ~CLIThread() { }
} g_cli_thread;

} // namespace


void init(const RebootRequestCallback& reboot_callback)
{
    g_reboot_request_callback = reboot_callback;
    (void) g_cli_thread.start(LOWPRIO + 1);
}

}
