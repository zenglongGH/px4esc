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

#if __GNUC__ < 5
# error "GCC version 5.x or newer is required"
#endif


namespace app
{
namespace
{

constexpr unsigned WatchdogTimeoutMSec = 1500;

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
     * CLI initialization
     */
    cli::init(&onRebootRequested);

    /*
     * Motor initialization
     */
    // This is only for testing purposes, will be removed later
    board::motor::pwm::init(60000.0F, 5e-07F);
    board::motor::pwm::activate();

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
    board::motor::pwm::emergency();
    board::setLEDRGB(255, 0, 0);
}

}


int main()
{
    auto watchdog = app::init();

    chibios_rt::BaseThread::setPriority(LOWPRIO);

    std::uint8_t counter = 0;

    uavcan_node::notifyNodeInitializationComplete();

    while (!os::isRebootRequested())
    {
        watchdog.reset();

        // Flying colors for testing
        board::setLEDRGB(counter, std::uint8_t(counter + 85 * 1), std::uint8_t(counter + 85 * 2));

        board::motor::pwm::set({counter / 255.0F,
                                std::uint8_t(counter + 85 * 1) / 255.0F,
                                std::uint8_t(counter + 85 * 2) / 255.0F});

        ::usleep(10000);
        counter++;
    }

    /*
     * Rebooting
     */
    os::lowsyslog("Main: GOING DOWN FOR REBOOT\n");

    watchdog.reset();

    ::sleep(1);         // Let other threads terminate properly

    board::restart();

    return 0;
}
