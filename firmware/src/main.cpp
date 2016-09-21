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

#include <ch.hpp>
#include <hal.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <zubax_chibios/os.hpp>

#include "board/board.hpp"
#include "bootloader_interface/bootloader_interface.hpp"
#include "uavcan_node/uavcan_node.hpp"
#include "cli/cli.hpp"
#include "foc/foc.hpp"
#include "motor_database/motor_database.hpp"

#if __GNUC__ < 5
# error "GCC version 5.x or newer is required"
#endif


namespace app
{
namespace
{

constexpr unsigned WatchdogTimeoutMSec = 1500;

/*
 * Configuration parameters.                             Name                Default     Min       Max
 */
/*
 * Motor profile parameters.
 * All of these values should be provided by the motor manufacturer.
 * If not, automatic identification can be used (see below).
 * Note that most of the parameters are by default assigned invalid values.
 */
os::config::Param<float> g_config_motor_start_current   ("mot.min_curr",        2.0F,    0.1F,    50.0F);  // Ampere
os::config::Param<float> g_config_motor_max_current     ("mot.max_curr",      100.0F,   10.0F,   200.0F);  // Ampere
os::config::Param<float> g_config_motor_field_flux      ("mot.phi",             0.0F,    0.0F,    10.0F);  // Weber
os::config::Param<float> g_config_motor_resistance_ab   ("mot.r_ab",            0.0F,    0.0F,   100.0F);  // Ohm
os::config::Param<float> g_config_motor_inductance_ab   ("mot.l_ab",            0.0F,    0.0F,     1.0F);  // Henry
os::config::Param<unsigned> g_config_motor_num_poles    ("mot.num_poles",          0,       0,      200);

/*
 * Auto identification settings.
 * If automatic identification is selected, it will be performed once, and the identified parameters will be stored
 * as the appropriate configuration parameters. Once Auto ID is complete, the auto ID level configuration parameter
 * will be reset to zero automatically.
 */
os::config::Param<unsigned> g_config_motor_auto_id_level("mot.auto_id_lvl",        0,       0,        2);

/*
 * FOC observer parameters.
 */
// Cross-coupling compensation constant
os::config::Param<float> g_param_cross_coupling_compensation("foc.obs.cc_comp", 0.5F, 0.0F, 10.0F);
// Kalman filter Q matrix (process noise covariance)
os::config::Param<float> g_param_Q_11("foc.obs.q_11",      100.0F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_Q_22("foc.obs.q_22",      100.0F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_Q_33("foc.obs.q_33",     5000.0F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_Q_44("foc.obs.q_44",        5.0F,  1e-6F,  1e+6F);
// Kalman filter R matrix (measurement noise)
os::config::Param<float> g_param_R_11("foc.obs.r_11",        2.0F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_R_22("foc.obs.r_22",        2.0F,  1e-6F,  1e+6F);
// Kalman filter initial value of the P matrix (state estimate covariance)
os::config::Param<float> g_param_P0_11("foc.obs.p0_11",      0.1F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_P0_22("foc.obs.p0_22",      0.1F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_P0_33("foc.obs.p0_33",      0.1F,  1e-6F,  1e+6F);
os::config::Param<float> g_param_P0_44("foc.obs.p0_44",      0.1F,  1e-6F,  1e+6F);


/**
 * This callback is invoked when any component wants the application to restart. This is thread safe.
 */
bool onRebootRequested(const char* reason)
{
    os::lowsyslog("Main: Reboot requested; reason: %s\n", reason);
    os::requestReboot();
    return true;                // Should reject while the motor is running?
}

/**
 * This callback is invoked from the local UAVCAN node (from its own thread!) when the said node
 * receives a firmware update request. The objective here is to set up the bootloader and signal
 * rebooting; once the system is rebooted, the bootloader will know what to do.
 */
auto onFirmwareUpdateRequestedFromUAVCAN(
    const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>& request)
{
    /*
     * Checking preconditions
     * Should reject while the motor is running?
     */
    static bool already_in_progress = false;

    os::lowsyslog("Main: UAVCAN firmware update request from %d, source %d, path '%s'\n",
                  request.getSrcNodeID().get(), request.source_node_id, request.image_file_remote_path.path.c_str());

    if (already_in_progress)
    {
        os::lowsyslog("Main: UAVCAN firmware update is already in progress, rejecting\n");
        return uavcan::protocol::file::BeginFirmwareUpdate::Response::ERROR_IN_PROGRESS;
    }

    /*
     * Initializing the app shared structure with proper arguments
     */
    bootloader_interface::AppShared shared;
    shared.can_bus_speed = uavcan_node::getCANBusBitRate();
    shared.uavcan_node_id = uavcan_node::getNodeID().get();
    shared.uavcan_fw_server_node_id = request.source_node_id;
    shared.stay_in_bootloader = true;

    std::strncpy(static_cast<char*>(&shared.uavcan_file_name[0]),       // This is really messy
                 request.image_file_remote_path.path.c_str(),
                 shared.UAVCANFileNameMaxLength);
    shared.uavcan_file_name[shared.UAVCANFileNameMaxLength - 1] = '\0';

    static_assert(request.image_file_remote_path.path.MaxSize < shared.UAVCANFileNameMaxLength, "Err...");

    os::lowsyslog("Main: Bootloader args: CAN bus bitrate: %u, local node ID: %d\n",
                  unsigned(shared.can_bus_speed), shared.uavcan_node_id);

    /*
     * Commiting everything
     */
    bootloader_interface::writeSharedStruct(shared);

    os::requestReboot();

    already_in_progress = true;

    os::lowsyslog("Main: UAVCAN firmware update initiated\n");
    return uavcan::protocol::file::BeginFirmwareUpdate::Response::ERROR_OK;
}

/**
 * This is invoked once immediately after boot.
 */
os::watchdog::Timer init()
{
    /*
     * Board initialization
     */
    auto watchdog = board::init(WatchdogTimeoutMSec);

    const auto fw_version = bootloader_interface::getFirmwareVersion();

    const auto app_shared_read_result = bootloader_interface::readAndInvalidateSharedStruct();
    const auto& app_shared = app_shared_read_result.first;
    const auto app_shared_available = app_shared_read_result.second;

    if (app_shared_available)
    {
        os::lowsyslog("Main: Bootloader struct values: CAN bitrate: %u, UAVCAN Node ID: %u\n",
                      unsigned(app_shared.can_bus_speed), app_shared.uavcan_node_id);
    }
    else
    {
        os::lowsyslog("Main: Bootloader struct is NOT present\n");
    }

    /*
     * Motor initialization
     */
    board::motor::init();
    foc::init();

    // Power on self test
    os::lowsyslog("Main: Testing hardware...\n");

    foc::beginHardwareTest();

    while (foc::getState() == foc::State::HardwareTesting)
    {
        ::sleep(1);
    }

    os::lowsyslog("Hardware test result:\n%s\n\n", foc::getLastHardwareTestReport().toString().c_str());

    // TODO Motor parameter initialization should go here
    {
        foc::MotorParameters motor_params = motor_database::getByName("Maxon 339285").parameters;

        foc::setMotorParameters(motor_params);
    }

    os::lowsyslog("Motor params:\n%s\n\n", foc::getMotorParameters().toString().c_str());

    // TODO Observer parameter initialization should go here
    {
        foc::ObserverParameters observer_params;

        foc::setObserverParameters(observer_params);
    }

    os::lowsyslog("Observer params:\n%s\n\n", foc::getObserverParameters().toString().c_str());

    // Initializing to Idle
    foc::stop();

    /*
     * CLI initialization
     */
    cli::init(&onRebootRequested);

    /*
     * UAVCAN node initialization
     */
    uavcan_node::init(app_shared_available ? app_shared.can_bus_speed : 0,
                      app_shared_available ? app_shared.uavcan_node_id : 0,
                      {fw_version.major, fw_version.minor},
                      fw_version.image_crc64we,
                      fw_version.vcs_commit,
                      &onFirmwareUpdateRequestedFromUAVCAN,
                      &onRebootRequested);

    return watchdog;
}

}
}

namespace os
{

extern void applicationHaltHook()
{
    board::motor::emergency();
    board::setLEDRGB(255, 0, 0);
}

}


int main()
{
    auto watchdog = app::init();

    /*
     * Confirming initialization
     */
    chibios_rt::BaseThread::setPriority(LOWPRIO);

    foc::beep(5000.0F, 0.1F);
    ::usleep(100000);
    foc::beep(6000.0F, 0.1F);

    uavcan_node::notifyNodeInitializationComplete();

    /*
     * Main loop
     */
    std::uint8_t counter = 0;

    while (!os::isRebootRequested())
    {
        watchdog.reset();

        // Flying colors for testing
        board::setLEDRGB(counter, std::uint8_t(counter + 85 * 1), std::uint8_t(counter + 85 * 2));

        ::usleep(10000);
        counter++;
    }

    /*
     * Rebooting
     */
    board::motor::emergency();             // Just to be on the safe side

    os::lowsyslog("Main: GOING DOWN FOR REBOOT\n");

    watchdog.reset();

    ::sleep(1);         // Let other threads terminate properly

    board::restart();

    return 0;
}
