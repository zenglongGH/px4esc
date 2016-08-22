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

#include <foc/foc.hpp>
#include <foc/svm.hpp>

#include <cstdlib>
#include <unistd.h>


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
            board::motor::setActive(true);
            ios.print("Activated\n");
        }
        else if (0 == std::strncmp("off", argv[1], 3))
        {
            board::motor::setActive(false);
            ios.print("Deactivated\n");
        }
        else
        {
            if (board::motor::isActive())
            {
                constexpr math::Range<> range(0, 1);
                math::Vector<3> abc = math::Vector<3>::Zero();

                for (int i = 0; i < std::min(3, argc); i++)
                {
                    // strtof() returns 0 on failure, which is just what we need
                    using namespace std;
                    abc[i] = range.constrain(strtof(argv[i + 1], nullptr));
                }

                board::motor::setPWM(abc);
                ios.print("PWM set to  %.3f  %.3f  %.3f\n", double(abc[0]), double(abc[1]), double(abc[2]));
            }
            else
            {
                ios.print("Driver is not active, command ignored\n");
            }
        }
    }
} static cmd_pwm;


class StatusCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "status"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        ios.print("Motor control HW:\n");
        ios.print("%s---\n", board::motor::getStatus().toString().c_str());

        ios.print("PWM:\n"
                  "Frequency: %.6f kHz\n"
                  "DeadTime : %.1f nsec\n",
                  1e-3 / double(board::motor::getPWMPeriod()),
                  double(board::motor::getPWMDeadTime()) * 1e9);
    }
} static cmd_status;


class CalibrateCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "calibrate"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        board::motor::beginCalibration();

        while (board::motor::isCalibrationInProgress())
        {
            ::sleep(1);
        }

        const auto status = board::motor::getStatus();
        ios.print("Current zero offsets: %.3f, %.3f\n",
                  double(status.phase_current_zero_offset[0]),
                  double(status.phase_current_zero_offset[1]));
    }
} static cmd_calibrate;


class SpinCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "spin"; }

    void execute(os::shell::BaseChannelWrapper& ios, int argc, char** argv) override
    {
        constexpr float DefaultVoltage = 1.0F;

        if (argc <= 1)
        {
            ios.print("Spin the motor by means of blind rotation of the voltage vector.\n"
                      "This command is inteneded for testing and debugging purposes. Usage:\n");
            ios.print("\t%s <angular velocity, rad/sec electrical> [voltage magnitude]\n"
                      "Voltage magnitude defaults to %.fV. Press any key to stop rotation.\n",
                      argv[0], double(DefaultVoltage));
            return;
        }

        /*
         * Parsing the arguments
         */
        using namespace std;
        const float angular_velocity = strtof(argv[1], nullptr);
        if (os::float_eq::closeToZero(angular_velocity))
        {
            ios.print("Invalid angular velocity\n");
            return;
        }

        float voltage = DefaultVoltage;
        if (argc > 2)
        {
            voltage = strtof(argv[2], nullptr);
            if (voltage <= 0)
            {
                ios.print("Invalid voltage\n");
                return;
            }
        }

        /*
         * Spinning until keyhit
         */
        struct Activator
        {
            const bool original_state = board::motor::isActive();
            Activator()  { board::motor::setActive(true); }
            ~Activator() { board::motor::setActive(original_state); }
        } const volatile raii_activator;

        ios.print("Spinning at %.1f rad/s, %.1f V. Type any character to stop.\n",
                  double(angular_velocity), double(voltage));

        while (ios.getChar(0) > 0)
        {
            ;   // Clearing the input buffer
        }

        auto prev_ts = chVTGetSystemTimeX();
        float angle = 0.0F;

        float min_setpoint = 1.0F;
        float max_setpoint = 0.0F;
        float min_inverter_voltage = board::motor::getInverterVoltage();
        float max_inverter_voltage = board::motor::getInverterVoltage();

        while (ios.getChar(1) <= 0)
        {
            // Computing dt (it may be very small or even zero but that's alright)
            const auto new_ts = chVTGetSystemTimeX();
            const auto dt = float(ST2US(new_ts - prev_ts)) * 1e-6F;
            prev_ts = new_ts;

            // Computing PWM settings
            angle += angular_velocity * dt;
            if (angle >= 2.0F * math::Pi)
            {
                angle = 0.0F;
            }
            if (angle <= -2.0F * math::Pi)
            {
                angle = 0.0F;
            }

            const auto alpha = math::sin(angle) * voltage;
            const auto beta  = math::cos(angle) * voltage;
            const auto inverter_voltage = board::motor::getInverterVoltage();

            const auto setpoint =
                foc::normalizePhaseVoltagesToPWMSetpoint(foc::performSpaceVectorTransform({alpha, beta}),
                                                         inverter_voltage);

            board::motor::setPWM(setpoint);

            // Collecting statistics
            min_setpoint = std::min(min_setpoint, setpoint[0]);
            max_setpoint = std::max(max_setpoint, setpoint[0]);
            min_inverter_voltage = std::min(min_inverter_voltage, inverter_voltage);
            max_inverter_voltage = std::max(max_inverter_voltage, inverter_voltage);

            // This line allows to monitor the setpoint values using the serial plotting script
            //ios.print("$%.3f,%.3f,%.3f\n", double(setpoint[0]), double(setpoint[1]), double(setpoint[2]));
        }

        board::motor::setPWM(math::Vector<3>::Zero());

        ios.print("Stopped; min/max PWM setpoints: %.3f/%.3f; min/max inverter voltage: %.1f/%.1f\n",
                  double(min_setpoint), double(max_setpoint),
                  double(min_inverter_voltage), double(max_inverter_voltage));
    }
} static cmd_spin;


class RawSetpointCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "rsp"; }

    void execute(os::shell::BaseChannelWrapper& ios, int argc, char** argv) override
    {
        if (argc <= 1)
        {
            foc::stop();

            ios.print("Set raw setpoint in the range [-1, 1] (negative for reverse rotation).\n");
            ios.print("Execute without arguments to stop the motor.\n");
            ios.print("\t%s [setpoint=0]\n", argv[0]);
            return;
        }

        using namespace std;
        const float sp = strtof(argv[1], nullptr);

        const float ttl = 30.0F;

        foc::setSetpoint(foc::ControlMode::Relative, sp, ttl);
    }
} static cmd_raw_setpoint;


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
        (void) shell_.addCommandHandler(&cmd_status);
        (void) shell_.addCommandHandler(&cmd_calibrate);
        (void) shell_.addCommandHandler(&cmd_spin);
        (void) shell_.addCommandHandler(&cmd_raw_setpoint);
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
