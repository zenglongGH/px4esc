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
#include <zubax_chibios/platform/stm32/config_storage.hpp>

#include "board/board.hpp"
#include "bootloader_interface/bootloader_interface.hpp"
#include "uavcan_node/uavcan_node.hpp"
#include "cli/cli.hpp"
#include "foc/foc.hpp"
#include "motor_database/motor_database.hpp"
#include "params.hpp"

#if __GNUC__ < 5
# error "GCC version 5.x or newer is required"
#endif


namespace app
{
namespace
{

constexpr unsigned WatchdogTimeoutMSec = 1500;


os::config::Param<int> g_param_motor_db_entry("motor_db.entry",    -1,     -1, 10000);


/**
 * This wrapper prohibits flash access when normal operation cannot be interrupted.
 */
class CustomConfigStorageBackend : public os::stm32::ConfigStorageBackend
{
    static bool codeExecutionCanBeInterruptedNow()
    {
        const auto state = foc::getState();

        return (state == foc::State::Idle ||
                state == foc::State::Fault) &&
               (board::motor::PWMHandle::getTotalNumberOfActiveHandles() == 0) &&
               !board::motor::isCalibrationInProgress();
    }

    int write(std::size_t offset, const void* data, std::size_t len) override
    {
        // Raising priority allows to prevent race condition when accessing the inverter driver
        os::TemporaryPriorityChanger priority_adjustment_expert(HIGHPRIO);

        if (codeExecutionCanBeInterruptedNow() &&
            board::motor::suspend())
        {
            const int res = os::stm32::ConfigStorageBackend::write(offset, data, len);
            board::motor::unsuspend();
            DEBUG_LOG("Main: Flash write result: %d\n", res);
            return res;
        }
        else
        {
            os::lowsyslog("Main: FLASH WRITE DENIED\n");
            return -EACCES;
        }
    }

    int erase() override
    {
        os::TemporaryPriorityChanger priority_adjustment_expert(HIGHPRIO);

        if (codeExecutionCanBeInterruptedNow() &&
            board::motor::suspend())
        {
            const int res = os::stm32::ConfigStorageBackend::erase();
            board::motor::unsuspend();
            DEBUG_LOG("Main: Flash erase result: %d\n", res);
            return res;
        }
        else
        {
            os::lowsyslog("Main: FLASH ERASE DENIED\n");
            return -EACCES;
        }
    }

public:
    CustomConfigStorageBackend() :
        os::stm32::ConfigStorageBackend(reinterpret_cast<void*>(0x08008000),
                                        0x4000)
    { }
} g_config_storage_backend;


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
    auto watchdog = board::init(WatchdogTimeoutMSec, g_config_storage_backend);

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

    // Motor parameters initialization
    {
        if (g_param_motor_db_entry.get() >= 0)
        {
            unsigned index = unsigned(g_param_motor_db_entry.get());
            const auto entry = motor_database::getByIndex(index);

            os::lowsyslog("Main: Overwriting motor params from the selected Motor DB entry %u '%s'...\n",
                          index, entry.name.c_str());

            params::writeMotorParameters(entry.parameters);

            // Resetting the DB entry index - parameters are now stored in the custom configuration
            g_param_motor_db_entry.set(int(g_param_motor_db_entry.default_));
            os::lowsyslog("Main: Parameter '%s' has been reset back to %d\n",
                          g_param_motor_db_entry.name, int(g_param_motor_db_entry.default_));
        }

        os::lowsyslog("Main: Restoring motor params...\n");
        const auto motor_params = params::readMotorParameters();
        os::lowsyslog("%s\n\n", motor_params.toString().c_str());

        foc::setMotorParameters(motor_params);
    }

    // Observer parameters initialization
    {
        os::lowsyslog("Main: Restoring observer params...\n");
        const auto observer_params = params::readObserverParameters();
        os::lowsyslog("%s\n\n", observer_params.toString().c_str());

        foc::setObserverParameters(observer_params);
    }

    // Initializing to Idle
    foc::stop();

    board::motor::printStatus();

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
    constexpr unsigned LoopPeriodMSec = 100;

    auto next_step_at = chVTGetSystemTime();

    while (!os::isRebootRequested())
    {
        watchdog.reset();

        next_step_at += MS2ST(LoopPeriodMSec);
        os::sleepUntilChTime(next_step_at);
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
