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

#include "cli.hpp"

#include <board/board.hpp>
#include <bootloader_interface/bootloader_interface.hpp>
#include <uavcan_node/uavcan_node.hpp>

#include <zubax_chibios/os.hpp>
#include <zubax_chibios/config/config.hpp>
#include <zubax_chibios/util/shell.hpp>
#include <zubax_chibios/util/base64.hpp>

#include <foc/foc.hpp>
#include <foc/transforms.hpp>
#include <foc/irq_debug_output.hpp>
#include <motor_database/motor_database.hpp>
#include <params.hpp>

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
        static board::motor::PWMHandle pwm_handle;

        if (argc < 2)
        {
            pwm_handle.release();
            ios.print("Usage: %s { off | <A> [B [C]] }\n", argv[0]);
            ios.print("Where A, B, C are phase values in [0, 1]\n");
            return;
        }

        if (!pwm_handle.isUnique())
        {
            pwm_handle.release();
            ios.print("ERROR: Driver is already in use\n");
            return;
        }

        if (0 == std::strncmp("off", argv[1], 3))
        {
            pwm_handle.release();
            ios.print("PWM handle released (remaining: %u)\n",
                      board::motor::PWMHandle::getTotalNumberOfActiveHandles());
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

            pwm_handle.setPWM(abc);
            ios.print("PWM set to  %.3f  %.3f  %.3f\n", double(abc[0]), double(abc[1]), double(abc[2]));
        }
    }
} static cmd_pwm;


class StatusCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "status"; }

    void execute(os::shell::BaseChannelWrapper&, int, char**) override
    {
        foc::printStatusInfo();

        std::printf("\nController:\n%s\n", foc::getControllerParameters().toString().c_str());

        std::printf("\nMotor:\n%s\n", foc::getMotorParameters().toString().c_str());

        std::printf("\nObserver:\n%s\n", foc::getObserverParameters().toString().c_str());

        std::puts("\nMotor control HW:");
        board::motor::printStatus();

        std::printf("\nPWM:\n"
                    "Active handles: %u\n"
                    "Frequency     : %.6f kHz\n"
                    "DeadTime      : %.1f nsec\n",
                    board::motor::PWMHandle::getTotalNumberOfActiveHandles(),
                    1e-3 / double(board::motor::getPWMPeriod()),
                    double(board::motor::getPWMDeadTime()) * 1e9);
    }
} static cmd_status;


class BriefStatusCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "s"; }

    void execute(os::shell::BaseChannelWrapper&, int, char**) override
    {
        foc::printStatusInfo();
    }
} static cmd_brief_status;


class CalibrateCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "calibrate"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        board::motor::beginCalibration();

        while (board::motor::isCalibrationInProgress())
        {
            ios.putChar('.');
            ::sleep(1);
        }
        ios.puts(" Done.");
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
            ios.puts("Spin the motor by means of blind rotation of the voltage vector.\n"
                     "This command is inteneded for testing and debugging purposes. Usage:");
            ios.print("\t%s <angular velocity, rad/sec electrical> [voltage magnitude] [-p]\n"
                      "Voltage magnitude defaults to %.fV. Press any key to stop rotation.\n"
                      "Option -p will plot the real time values.\n",
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
            ios.print("ERROR: Invalid angular velocity\n");
            return;
        }

        float voltage = DefaultVoltage;
        bool do_plot = false;

        for (int i = 2; i < argc; i++)
        {
            const os::heapless::String<> arg(argv[i]);

            if (arg == "-p")
            {
                do_plot = true;
            }
            else
            {
                voltage = strtof(arg.c_str(), nullptr);
            }
        }

        if (voltage <= 0)
        {
            ios.print("ERROR: Invalid voltage\n");
            return;
        }

        /*
         * Gaining access to the driver
         */
        board::motor::PWMHandle pwm_handle;

        if (!pwm_handle.isUnique())
        {
            ios.print("ERROR: Driver is already in use\n");
            return;
        }

        /*
         * Spinning until keyhit
         */
        os::TemporaryPriorityChanger priority_adjustment_expert(HIGHPRIO - 5);

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
        float min_inverter_voltage = board::motor::getStatus().inverter_voltage;
        float max_inverter_voltage = min_inverter_voltage;

        while (ios.getChar(do_plot ? 0 : 1) <= 0)
        {
            // Computing dt (it may be very small or even zero but that's alright)
            const auto new_ts = chVTGetSystemTimeX();
            const auto dt = float(ST2US(new_ts - prev_ts)) * 1e-6F;
            prev_ts = new_ts;

            // Computing PWM settings
            angle = math::normalizeAngle(angle + angular_velocity * dt);

            const auto inverter_voltage = board::motor::getInverterVoltage();

            const auto Ualphabeta = foc::performInverseParkTransform({0.0F, voltage},
                                                                     math::sincos(angle));

            const auto setpoint = foc::performSpaceVectorTransform(Ualphabeta, inverter_voltage).first;

            pwm_handle.setPWM(setpoint);

            // Collecting statistics
            min_setpoint = std::min(min_setpoint, setpoint[0]);
            max_setpoint = std::max(max_setpoint, setpoint[0]);
            min_inverter_voltage = std::min(min_inverter_voltage, inverter_voltage);
            max_inverter_voltage = std::max(max_inverter_voltage, inverter_voltage);

            if (do_plot)
            {
                const math::Vector<3> modulated = (setpoint.array() - setpoint.mean()) * inverter_voltage;

                ios.print("$%.4f,%.3f,%.3f\n",
                          double(ST2US(std::uint64_t(chVTGetSystemTimeX()))) * 1e-6,
                          double(modulated[0]),
                          double(modulated[0] - modulated[1]));
            }
        }

        pwm_handle.setPWM(math::Vector<3>::Zero());

        ios.print("Stopped; min/max PWM setpoints: %.3f/%.3f; min/max inverter voltage: %.1f/%.1f\n",
                  double(min_setpoint), double(max_setpoint),
                  double(min_inverter_voltage), double(max_inverter_voltage));
    }
} static cmd_spin;


class SetpointCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "sp"; }

    void execute(os::shell::BaseChannelWrapper& ios, int argc, char** argv) override
    {
        if (argc <= 1)
        {
            foc::stop();

            ios.puts("Apply setpoint in the specified control mode; negative values specify reverse rotation.");
            ios.puts("The following control mode suffixes are supported (case insensitive):\n"
                     " - a      Current\n"
                     " - ra     Ratiometric Current\n"
                     " - v      Voltage\n"
                     " - (none) Ratiometric Voltage");
            ios.puts("Execute without arguments to stop the motor.");
            ios.puts("Option -p will plot the real time values.");
            ios.print("\t%s [setpoint=0 [a|ra|v] [-p]]\n", argv[0]);
            return;
        }

        // Parsing optional stuff
        auto control_mode = foc::ControlMode::RatiometricVoltage;      // This is the default
        bool do_plot = false;

        for (int i = 1; i < argc; i++)
        {
            const os::heapless::String<20> arg(argv[i]);

            if (arg == "-p")
            {
                do_plot = true;
            }

            if (arg.toLowerCase() == "a")  { control_mode = foc::ControlMode::Current; }
            if (arg.toLowerCase() == "ra") { control_mode = foc::ControlMode::RatiometricCurrent; }
            if (arg.toLowerCase() == "v")  { control_mode = foc::ControlMode::Voltage; }
        }

        using namespace std;
        float sp = strtof(argv[1], nullptr);

        switch (control_mode)
        {
        case foc::ControlMode::RatiometricCurrent:
        case foc::ControlMode::RatiometricVoltage:
        {
            static constexpr math::Range<> UnityLimits(-1.0F, 1.0F);

            if (!UnityLimits.contains(sp))
            {
                sp = 0;
                ios.print("ERROR: Setpoint out of range [%g, %g]\n",
                          double(UnityLimits.min), double(UnityLimits.max));
            }
            break;
        }

        case foc::ControlMode::Current:
        {
            const auto Imax = foc::getMotorParameters().max_current;
            const math::Range<> CurrentLimits(-Imax, Imax);
            if (!CurrentLimits.contains(sp))
            {
                sp = 0;
                ios.print("ERROR: Current out of range %s\n", CurrentLimits.toString().c_str());
            }
            break;
        }

        case foc::ControlMode::Voltage:
        {
            const auto Vmax = foc::computeLineVoltageLimit(board::motor::getInverterVoltage(),
                                                           board::motor::getPWMUpperLimit());
            const math::Range<> VoltageLimits(-Vmax, Vmax);
            if (!VoltageLimits.contains(sp))
            {
                sp = 0;
                ios.print("ERROR: Voltage out of range %s\n", VoltageLimits.toString().c_str());
            }
            break;
        }

        case foc::ControlMode::RatiometricMRPM:
        case foc::ControlMode::MRPM:
        {
            sp = 0;
            ios.puts("ERROR: CONTROL MODE NOT IMPLEMENTED YET");
            break;
        }

        default:
        {
            assert(false);
        }
        }

        const float ttl = 60.0F;                        // TODO: Configurable

        ios.print("Setpoint %.3f mode %d TTL %.1f s\n", double(sp), int(control_mode), double(ttl));

        foc::setSetpoint(control_mode, sp, ttl);

        if (do_plot)
        {
            ios.print("Press any key to stop\n");

            while (ios.getChar(1) > 0)
            {
                ;   // Clearing the input buffer
            }

            while (ios.getChar(0) <= 0)
            {
                foc::plotRealTimeValues();

                const auto state = foc::getState();
                if (state != foc::State::Running &&
                    state != foc::State::Spinup)
                {
                    break;
                }
            }

            foc::stop();
        }
    }
} static cmd_setpoint;


class BeepCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "beep"; }

    void execute(os::shell::BaseChannelWrapper& ios, int argc, char** argv) override
    {
        constexpr float DefaultDuration = 0.1F;

        if (argc <= 1)
        {
            ios.print("Make noise of the specified frequency [Hz] and duration [sec].\n");
            ios.print("\t%s <frequency> [duration=%.1f]\n", argv[0], double(DefaultDuration));
            return;
        }

        using namespace std;

        const float frequency = strtof(argv[1], nullptr);

        float duration = (argc > 2) ? strtof(argv[2], nullptr) : DefaultDuration;

        foc::beep(frequency, duration);
    }
} static cmd_beep;


class MotorIdentificationCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "motor_id"; }

    void execute(os::shell::BaseChannelWrapper& ios, int argc, char** argv) override
    {
        if (argc <= 1)
        {
            ios.print("Perform motor identification using the specified mode.\n");
            ios.print("Option -p will plot the real time values.\n");
            ios.print("\t%s static|rotating [-p]\n", argv[0]);
            return;
        }

        // Parsing optional stuff
        bool do_plot = false;

        for (int i = 1; i < argc; i++)
        {
            const os::heapless::String<> arg(argv[i]);

            if (arg == "-p")
            {
                do_plot = true;
            }
        }

        // Parsing mode
        const os::heapless::String<> mode_string(argv[1]);
        foc::MotorIdentificationMode mode{};
        if (mode_string == "static")
        {
            mode = foc::MotorIdentificationMode::Static;
        }
        else if (mode_string == "rotating")
        {
            mode = foc::MotorIdentificationMode::RotationWithoutMechanicalLoad;
        }
        else
        {
            ios.print("ERROR: Invalid identification mode: %s\n", mode_string.c_str());
            return;
        }

        // Running
        if (foc::getState() != foc::State::Idle &&
            foc::getState() != foc::State::Fault)
        {
            ios.puts("ERROR: Invalid state");
            return;
        }

        ios.puts("PRESS ANY KEY TO ABORT");

        while (ios.getChar(1) > 0)
        {
            ;   // Clearing the input buffer
        }

        foc::beginMotorIdentification(mode);

        bool aborted = false;

        while (foc::getState() == foc::State::MotorIdentification)
        {
            if (do_plot)
            {
                foc::plotRealTimeValues();
            }
            else
            {
                ios.putChar('.');
            }

            if (ios.getChar(do_plot ? 0 : 1000) > 0)
            {
                foc::stop();
                aborted = true;
            }

            foc::IRQDebugOutputBuffer::printIfNeeded();
        }

        // Handling the result
        if (aborted)
        {
            ios.puts("ABORTED");
        }
        else
        {
            ios.puts("Done");

            const auto params = foc::getMotorParameters();
            ios.puts(params.toString().c_str());

            if (params.isValid())
            {
                ios.puts("Overwriting custom motor params with identified values");
                params::writeMotorParameters(params);
            }
            else
            {
                ios.puts("Params are incomplete or identification failed, custom params are left unchanged");
            }
        }
    }
} static cmd_motor_identification;


class KVConvertCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "kvconv"; }

    void execute(os::shell::BaseChannelWrapper& ios, int argc, char** argv) override
    {
        if (argc < 3)
        {
            ios.print("Usage: %s <KV [MRPM/V] | field-flix-linkage [mWb]> <num-poles>\n", argv[0]);
            ios.print("Where: KV is in MRPM/V (MRPM - mechanical RPM),\n"
                      "       Flux linkage is in milli Weber\n");
            return;
        }

        using namespace std;

        const float kv_mWb = strtof(argv[1], nullptr);
        const long num_poles = strtol(argv[2], nullptr, 10);

        if ((kv_mWb <= 0.0F) ||
            (num_poles < 2) ||
            (num_poles % 2 != 0))
        {
            ios.print("ERROR: Invalid arguments\n");
            return;
        }

        ios.print("Num poles: %d\n", int(num_poles));

        ios.print("%.4f mWb --> %.1f MRPM/V \n",
                  double(kv_mWb),
                  double(foc::convertFluxLinkageToKV(kv_mWb * 1e-3F, unsigned(num_poles))));

        ios.print("%.1f MRPM/V --> %.4f mWb \n",
                  double(kv_mWb),
                  double(foc::convertKVToFluxLinkage(kv_mWb, unsigned(num_poles))) * 1e3);
    }
} static cmd_kv_convert;


class HardwareTestCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "hwtest"; }

    void execute(os::shell::BaseChannelWrapper& ios, int argc, char** argv) override
    {
        ios.print("Testing the hardware (option -p will plot real time values)...\n");

        // Parsing optional stuff
        bool do_plot = false;

        for (int i = 1; i < argc; i++)
        {
            const os::heapless::String<> arg(argv[i]);

            if (arg == "-p")
            {
                do_plot = true;
            }
        }

        // Running
        if (foc::getState() != foc::State::Idle &&
            foc::getState() != foc::State::Fault)
        {
            ios.print("ERROR: Invalid state\n");
            return;
        }

        foc::beginHardwareTest();

        if (do_plot)
        {
            while (foc::getState() == foc::State::HardwareTesting)
            {
                foc::plotRealTimeValues();
            }
            foc::plotRealTimeValues();
        }
        else
        {
            while (foc::getState() == foc::State::HardwareTesting)
            {
                ios.putChar('.');
                ::sleep(1);
            }
            ios.print(" Done.\n");
        }

        ios.print("%s\n", foc::getLastHardwareTestReport().toString().c_str());
    }
} static cmd_hardware_test;


class MotorDatabaseCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "motor_db"; }

    void execute(os::shell::BaseChannelWrapper& ios, int argc, char** argv) override
    {
        if (argc < 2)
        {
            ios.puts("Use this command to access the built-in motor database.");
            ios.print("Usage: %s (list|show|use) <entry-index>\n", argv[0]);
            ios.puts("Where: Entry index is a natural number.\n"
                     "       'list' prints all known motor profiles.\n"
                     "       'show' displays the specified motor profile, if such exists.\n"
                     "       'use' applies the specified motor profile.");
            return;
        }

        // This is weird but it greatly conserves stack.
        static const auto print = [](os::shell::BaseChannelWrapper& ios, auto item)
        {
            ios.puts(item.toString().c_str());
        };

        const auto command = os::heapless::String<40>(argv[1]);

        if (command == "list")
        {
            for (unsigned index = 0; true; index++)
            {
                const auto entry = motor_database::getByIndex(index);
                if (entry.isEmpty())
                {
                    break;
                }
                else
                {
                    if (index > 0)
                    {
                        ios.puts("");
                    }
                    ios.print("---------- Entry %02u ----------\n", index);
                    print(ios, entry);
                }
            }
        }
        else if (command == "show" ||
                 command == "use")
        {
            if (argc >= 3)
            {
                using namespace std;
                const auto entry = motor_database::getByIndex(unsigned(atoi(argv[2])));
                if (entry.isEmpty())
                {
                    ios.puts("Error: No such entry");
                }
                else
                {
                    print(ios, entry);
                    if (command == "use")
                    {
                        params::writeMotorParameters(entry.parameters);
                        foc::setMotorParameters(entry.parameters);
                        ios.puts("Motor parameters updated.");
                    }
                }
            }
            else
            {
                ios.puts("Error: Missing entry index.");
            }
        }
        else
        {
            ios.puts("Error: Invalid command.");
        }
    }
} static cmd_motor_database;


class PlotCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "plot"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        ios.puts("PRESS ANY KEY TO STOP PLOTTING");
        ::sleep(1);               // Making sure the human has enough time to read the message

        while (ios.getChar(1) > 0)
        {
            ;   // Clearing the input buffer
        }

        while (ios.getChar(0) <= 0)
        {
            foc::plotRealTimeValues();
        }
    }
} static cmd_plot;


class ThreadsCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "threads"; }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        static const char* const ThreadStateNames[] = { CH_STATE_NAMES };

        static const auto gauge_free_stack = [](const ::thread_t* tp)
        {
            const std::uint8_t* limit = reinterpret_cast<std::uint8_t*>(tp->p_stklimit);
            const unsigned current = reinterpret_cast<unsigned>(tp->p_ctx.r13);
            unsigned num_bytes = 0;
            while ((*limit++ == CH_DBG_STACK_FILL_VALUE) &&
                   (reinterpret_cast<unsigned>(limit) < current))
            {
                num_bytes++;
            }
            return num_bytes;
        };

        ios.puts("Name             State     FStk Prio");
        ios.puts("-------------------------------------");
        ::thread_t* tp = chRegFirstThread();
        do
        {
            ios.print("%-16s %-9s %-4u %-4u\n",
                      tp->p_name,
                      ThreadStateNames[tp->p_state],
                      gauge_free_stack(tp),
                      static_cast<unsigned>(tp->p_prio));
            tp = chRegNextThread(tp);
        }
        while (tp != nullptr);

        ios.puts("");

        ios.print("Stack reserved for IRQ handlers: %u\n", PORT_INT_REQUIRED_STACK);
    }
} static cmd_threads;


class CLIThread : public chibios_rt::BaseStaticThread<2048>
{
    os::shell::Shell<20> shell_;
    os::Logger logger{"CLI"};

    void main() override
    {
        setName("cli");

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
            shell_.runFor(wrapper, 100);

            foc::IRQDebugOutputBuffer::printIfNeeded();
        }

        logger.puts("Stopped");
    }

    static auto renderPrompt()
    {
        return os::heapless::concatenate<decltype(shell_)::Prompt::Capacity>(
            foc::stateToString(foc::getState()), "> ");
    }

public:
    CLIThread() :
        shell_(renderPrompt)
    {
        (void) shell_.addCommandHandler(&cmd_reboot);
        (void) shell_.addCommandHandler(&cmd_zubax_id);
        (void) shell_.addCommandHandler(&cmd_cfg);
        (void) shell_.addCommandHandler(&cmd_uavcan);
        (void) shell_.addCommandHandler(&cmd_pwm);
        (void) shell_.addCommandHandler(&cmd_status);
        (void) shell_.addCommandHandler(&cmd_brief_status);
        (void) shell_.addCommandHandler(&cmd_calibrate);
        (void) shell_.addCommandHandler(&cmd_spin);
        (void) shell_.addCommandHandler(&cmd_setpoint);
        (void) shell_.addCommandHandler(&cmd_beep);
        (void) shell_.addCommandHandler(&cmd_motor_identification);
        (void) shell_.addCommandHandler(&cmd_kv_convert);
        (void) shell_.addCommandHandler(&cmd_hardware_test);
        (void) shell_.addCommandHandler(&cmd_motor_database);
        (void) shell_.addCommandHandler(&cmd_plot);
        (void) shell_.addCommandHandler(&cmd_threads);
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
