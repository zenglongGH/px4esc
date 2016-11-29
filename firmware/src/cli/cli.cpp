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
#include <motor_database/motor_database.hpp>
#include <params.hpp>

#include <cstdlib>
#include <unistd.h>


namespace cli
{
namespace
{

RebootRequestCallback g_reboot_request_callback;

struct RAIIPlottingEnabler
{
    RAIIPlottingEnabler(bool en = true)
    {
        foc::setPlottingEnabled(en);
    }

    ~RAIIPlottingEnabler()
    {
        foc::setPlottingEnabled(false);
    }
};


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


class BriefStatusCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "s"; }

    void execute(os::shell::BaseChannelWrapper&, int, char**) override { execute(); }

public:
    void execute()
    {
        const auto voltage = board::motor::getInverterVoltage();

        bool printed = false;

        if (!printed)
        {
            foc::RunningStateInfo info;
            bool spinup_in_progress = false;
            if (foc::isRunning(&info, &spinup_in_progress))
            {
                std::printf("RUNNING, %s\n", spinup_in_progress ? "SPINUP" : "NORMAL");

                std::printf("%6.1f W     %6.2f A     %4.1f V\n",
                            double(info.inverter_power_filtered),
                            double(info.inverter_power_filtered / voltage),
                            double(voltage));

                std::printf("%6.0f RPMM  %3.0f %%    %5u stalls\n",
                            double(info.mechanical_rpm),
                            double(info.demand_factor_filtered * 100.0F),
                            static_cast<unsigned>(info.stall_count));
                printed = true;
            }
        }

        if (!printed)
        {
            foc::MotorIdentificationStateInfo info;
            if (foc::isMotorIdentificationInProgress(&info))
            {
                std::printf("MOTOR ID, %.0f %% complete\n", double(info.progress * 100.0F));
                std::printf("%5.1f W     %6.2f A     %4.1f V     %5.0f RPMM\n",
                            double(info.inverter_power_filtered),
                            double(info.inverter_power_filtered / voltage),
                            double(voltage),
                            double(info.mechanical_rpm));
                printed = true;
            }
        }

        if (!printed)
        {
            foc::InactiveStateInfo info;
            if (foc::isInactive(&info))
            {
                if (info.fault_code == info.FaultCodeNone)
                {
                    std::printf("IDLE, %llu task switchings\n",
                                static_cast<unsigned long long>(foc::getExtendedStatus().task_switch_count));
                }
                else
                {
                    std::printf("FAULT 0x%04x\n", info.fault_code);
                }
                printed = true;
            }
        }

        if (!printed)
        {
            std::puts("No status description available");
            printed = true;
        }

        assert(printed);

        {
            const auto kv = foc::getDebugKeyValuePairs();
            if (std::any_of(kv.begin(), kv.end(), [](auto& item) { return !item.first.empty(); }))
            {
                std::printf("Debug Values:");
                for (auto& x : kv)
                {
                    if (!x.first.empty())
                    {
                        std::printf("    %s %g", x.first.c_str(), double(x.second));
                    }
                }
                std::puts("");
            }
        }
    }

} static cmd_brief_status;


class StatusCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "status"; }

    void execute(os::shell::BaseChannelWrapper&, int, char**) override
    {
        cmd_brief_status.execute();

        std::puts("\nFOC Parameters:");
        std::puts(foc::getParameters().toString().c_str());

        std::puts("\nMotor control HW:");
        board::motor::printStatus();

        std::printf("\nPhase Currents AB: %s\n", math::toString(board::motor::getPhaseCurrentsAB()).c_str());

        const auto pwm_params = board::motor::getPWMParameters();

        std::printf("\nPWM:\n"
                    "Active handles: %u\n"
                    "Frequency     : %.6f kHz\n"
                    "DeadTime      : %.1f nsec\n",
                    board::motor::PWMHandle::getTotalNumberOfActiveHandles(),
                    1e-3 / double(pwm_params.period),
                    double(pwm_params.dead_time) * 1e9);
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
            ios.puts("Rotate voltage vector of the specified magnitude at the specified angular velocity.\n"
                     "This command is inteneded for testing and debugging purposes. Usage:");
            ios.print("\t%s <angular velocity, rad/sec electrical> [voltage magnitude] [-p] [-a accel]\n"
                      "Voltage magnitude defaults to %.fV. Press any key to stop rotation.\n"
                      "Option -p will plot real time data.\n"
                      "Option -a <accel> specified acceleration in rad/(sec*sec).\n",
                      argv[0], double(DefaultVoltage));
            return;
        }

        /*
         * Parsing the arguments
         */
        using namespace std;
        float angular_velocity = strtof(argv[1], nullptr);
        if (os::float_eq::closeToZero(angular_velocity))
        {
            ios.print("ERROR: Invalid angular velocity\n");
            return;
        }

        float voltage = DefaultVoltage;
        float acceleration = 0.0F;
        bool do_plot = false;

        {
            bool next_acceleration = false;

            for (int i = 2; i < argc; i++)
            {
                const os::heapless::String<> arg(argv[i]);

                if (next_acceleration)
                {
                    acceleration = strtof(argv[i], nullptr);
                    next_acceleration = false;
                    continue;
                }

                if (arg == "-p")
                {
                    do_plot = true;
                }
                else if (arg == "-a")
                {
                    next_acceleration = true;
                }
                else
                {
                    voltage = strtof(arg.c_str(), nullptr);
                }
            }

            if (next_acceleration)
            {
                ios.print("ERROR: Expected acceleration\n");
                return;
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

        ios.print("Spinning at %.1f rad/s, %.2f rad/s^2, %.1f V. Type any character to stop.\n",
                  double(angular_velocity), double(acceleration), double(voltage));

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

        while (ios.getChar(1) <= 0)
        {
            // Computing dt (it may be very small or even zero but that's alright)
            const auto new_ts = chVTGetSystemTimeX();
            const auto dt = float(ST2US(new_ts - prev_ts)) * 1e-6F;
            prev_ts = new_ts;

            // Computing PWM settings
            angle = math::normalizeAngle(angle + angular_velocity * dt);
            angular_velocity += acceleration * dt;

            const auto inverter_voltage = board::motor::getInverterVoltage();
            const auto angle_sincos = math::sincos(angle);

            const auto Ualphabeta = foc::performInverseParkTransform({0.0F, voltage}, angle_sincos);

            const auto setpoint = foc::performSpaceVectorTransform(Ualphabeta, inverter_voltage).first;

            pwm_handle.setPWM(setpoint);

            // Collecting statistics
            min_setpoint = std::min(min_setpoint, setpoint[0]);
            max_setpoint = std::max(max_setpoint, setpoint[0]);
            min_inverter_voltage = std::min(min_inverter_voltage, inverter_voltage);
            max_inverter_voltage = std::max(max_inverter_voltage, inverter_voltage);

            if (do_plot)
            {
                const auto Ialphabeta = foc::performClarkeTransform(board::motor::getPhaseCurrentsAB());
                const auto Idq = foc::performParkTransform(Ialphabeta, angle_sincos);

                // Ud, Uq, Id, Iq, angvel
                ios.print("$%.3f,0,%.2f,%.2f,%.2f,%.2f\n",
                          double(chVTGetSystemTimeX()) / double(CH_CFG_ST_FREQUENCY),  // TODO this overflows
                          double(voltage),
                          double(Idq[0]), double(Idq[1]),
                          double(angular_velocity));
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
            ios.puts("Option -p will plot real time data.");
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
                                                           board::motor::getPWMParameters().upper_limit);
            const math::Range<> VoltageLimits(-Vmax, Vmax);
            if (!VoltageLimits.contains(sp))
            {
                sp = 0;
                ios.print("ERROR: Voltage out of range %s\n", VoltageLimits.toString().c_str());
            }
            break;
        }

        case foc::ControlMode::RatiometricRPMM:
        case foc::ControlMode::RPMM:
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
            RAIIPlottingEnabler enabler;

            ios.puts("PRESS ANY KEY TO STOP");

            while (ios.getChar(1) > 0)
            {
                ;   // Clearing the input buffer
            }

            while (ios.getChar(100) <= 0)
            {
                if (!foc::isRunning())
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
            ios.puts("Perform motor identification using the specified mode.");
            ios.puts("Option -p will plot real time data.");
            ios.print("\t%s r_l | phi | fine | r_l_phi | r_l_phi_fine [-p]\n", argv[0]);
            ios.puts("Use mode r_l_phi_fine to perform full identification.");
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
        const os::heapless::String<20> mode_string(argv[1]);
        foc::motor_id::Mode mode{};
        if      (mode_string.toLowerCase() == "r_l")            { mode = foc::motor_id::Mode::R_L;                  }
        else if (mode_string.toLowerCase() == "phi")            { mode = foc::motor_id::Mode::Phi;                  }
        else if (mode_string.toLowerCase() == "fine")           { mode = foc::motor_id::Mode::FineTuning;           }
        else if (mode_string.toLowerCase() == "r_l_phi")        { mode = foc::motor_id::Mode::R_L_Phi;              }
        else if (mode_string.toLowerCase() == "r_l_phi_fine")   { mode = foc::motor_id::Mode::R_L_Phi_FineTuning;   }
        else
        {
            ios.print("ERROR: Invalid identification mode: %s\n", mode_string.c_str());
            return;
        }

        // Running
        if (!foc::isInactive())
        {
            ios.puts("ERROR: Invalid state");
            return;
        }

        ios.puts("PRESS ANY KEY TO ABORT");

        while (ios.getChar(1) > 0)
        {
            ;   // Clearing the input buffer
        }

        RAIIPlottingEnabler plotting_enabler(do_plot);

        foc::beginMotorIdentification(mode);

        bool aborted = false;

        foc::MotorIdentificationStateInfo info;
        while (foc::isMotorIdentificationInProgress(&info))
        {
            if (!do_plot)
            {
                ios.print("\r%u %% \r", unsigned(info.progress * 100.0F));
            }

            if (ios.getChar(500) > 0)
            {
                foc::stop();
                aborted = true;
            }
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
            ios.print("Usage: %s <KV [RPMM/V] | field-flix-linkage [mWb]> <num-poles>\n", argv[0]);
            ios.print("Where: KV is in RPMM/V (RPMM - mechanical RPM),\n"
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

        ios.print("%.4f mWb --> %.1f RPMM/V \n",
                  double(kv_mWb),
                  double(foc::convertFluxLinkageToKV(kv_mWb * 1e-3F, unsigned(num_poles))));

        ios.print("%.1f RPMM/V --> %.4f mWb \n",
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
        if (!foc::isInactive())
        {
            ios.print("ERROR: Invalid state\n");
            return;
        }

        if (board::motor::PWMHandle::getTotalNumberOfActiveHandles() != 0)
        {
            ios.print("ERROR: PWM handle is taken by another component [%u]\n",
                      board::motor::PWMHandle::getTotalNumberOfActiveHandles());
            return;
        }

        foc::beginHardwareTest();

        if (do_plot)
        {
            RAIIPlottingEnabler enabler;
            while (foc::isHardwareTestInProgress())
            {
                ::usleep(100000);
            }
        }
        else
        {
            while (foc::isHardwareTestInProgress())
            {
                ios.putChar('.');
                ::sleep(1);
            }
            ios.print(" Done.\n");
        }

        ios.print("%s\n", foc::getHardwareTestReport().toString().c_str());
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

    void execute(os::shell::BaseChannelWrapper& ios, int argc, char** argv) override
    {
        if (argc <= 1)
        {
            foc::setPlottingEnabled(false);
            ios.print("Plotting stopped.\n"
                      "To start plotting:\n"
                      "\t%s on\n", argv[0]);
        }
        else
        {
            if (os::heapless::String<30>(argv[1]).toLowerCase() == "on")
            {
                ios.puts("Plotting now. Execute without arguments to stop.");
                ::usleep(300000);
                foc::setPlottingEnabled(true);
            }
            else
            {
                ios.puts("ERROR: Invalid argument");
            }
        }
    }
} static cmd_plot;


class SystemInfoCommand : public os::shell::ICommandHandler
{
    const char* getName() const override { return "sysinfo"; }

    template <typename T>
    static double convertStatTickCountToSeconds(T val)
    {
        return double(val) / double(STM32_SYSCLK);
    }

    static void showTime(os::shell::BaseChannelWrapper& ios)
    {
        const auto sys_time = chibios_rt::System::getTimeX();
        ios.print("System time: %lu ticks / %g sec\n", sys_time, double(sys_time) / double(CH_CFG_ST_FREQUENCY));
    }

    static void showKernelStats(os::shell::BaseChannelWrapper& ios)
    {
        decltype(ch.kernel_stats) stats;

        {
            os::CriticalSectionLocker locker;
            stats = ch.kernel_stats;
        }

        static const auto show_once = [&ios](const char* name, const ::time_measurement_t& tm)
        {
            const auto average_timing = convertStatTickCountToSeconds(double(tm.cumulative) / double(tm.n));
            ios.print("%s: average %7.3f us     best %7.3f us     worst %7.3f us\n",
                      name,
                      average_timing * 1e6,
                      convertStatTickCountToSeconds(tm.best) * 1e6,
                      convertStatTickCountToSeconds(tm.worst) * 1e6);
        };

        ios.print("Sys IRQ handled: %lu\n", stats.n_irq);
        ios.print("Context switch.: %lu\n", stats.n_ctxswc);
        show_once("Thread critsect", stats.m_crit_thd);
        show_once("Sys IRQ critsec", stats.m_crit_isr);
    }

    static void showThreads(os::shell::BaseChannelWrapper& ios)
    {
        const char* const ThreadStateNames[] = { CH_STATE_NAMES };

        static const auto gauge_free_stack = [](const ::thread_t* tp)
        {
            const std::uint8_t* limit = reinterpret_cast<std::uint8_t*>(tp->wabase);
            const unsigned current = reinterpret_cast<unsigned>(tp->ctx.sp);
            unsigned num_bytes = 0;
            while ((*limit++ == CH_DBG_STACK_FILL_VALUE) &&
                   (reinterpret_cast<unsigned>(limit) < current))
            {
                num_bytes++;
            }
            return num_bytes;
        };

        std::uint64_t total_cumulative = 0;

        {
            ::thread_t* tp = chRegFirstThread();
            do
            {
                total_cumulative += tp->stats.cumulative;
                tp = chRegNextThread(tp);
            }
            while (tp != nullptr);
        }
        assert(total_cumulative > 0);

        ios.puts("                           Free         Avg  |     Timing Stat [ms]");
        ios.puts("Name             State     Stack  Prio  Load |   Avg     Best     Worst");
        ios.puts("---------------------------------------------+--------------------------");
        ::thread_t* tp = chRegFirstThread();
        do
        {
            decltype(tp->stats) stats;

            {
                os::CriticalSectionLocker locker;
                stats = tp->stats;
            }

            const auto average_load = unsigned((100 * stats.cumulative + 50) / total_cumulative);
            const auto average_timing = convertStatTickCountToSeconds(double(stats.cumulative) / double(stats.n));

            ios.print("%-16s %-9s %5u  %3u   %3u%% | %7.3f %7.3f %8.3f\n",
                      tp->name,
                      ThreadStateNames[tp->state],
                      gauge_free_stack(tp),
                      static_cast<unsigned>(tp->prio),
                      average_load,
                      average_timing * 1e3,
                      convertStatTickCountToSeconds(stats.best) * 1e3,
                      convertStatTickCountToSeconds(stats.worst) * 1e3);
            tp = chRegNextThread(tp);
        }
        while (tp != nullptr);

        ios.print("Stack reserved for IRQ: %u B\n", PORT_INT_REQUIRED_STACK);
    }

    void execute(os::shell::BaseChannelWrapper& ios, int, char**) override
    {
        showTime(ios);

        ios.puts("\nKernel Stats:");
        showKernelStats(ios);

        ios.puts("\nThreads:");
        showThreads(ios);
    }
} static cmd_sysinfo;


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
        }

        logger.puts("Stopped");
    }

    static auto renderPrompt()
    {
        return os::heapless::concatenate<decltype(shell_)::Prompt::Capacity>(
            foc::getExtendedStatus().current_task_name, "> ");
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
        (void) shell_.addCommandHandler(&cmd_sysinfo);
    }

    virtual ~CLIThread() { }
} g_cli_thread;

} // namespace


void init(const RebootRequestCallback& reboot_callback)
{
    g_reboot_request_callback = reboot_callback;
    (void) g_cli_thread.start(LOWPRIO + 3);
}

}
