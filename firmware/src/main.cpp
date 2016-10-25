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
#include "aux_cmd_iface.hpp"
#include "led_indicator.hpp"

#if __GNUC__ < 5
# error "GCC version 5.x or newer is required"
#endif


namespace app
{
namespace
{

constexpr unsigned WatchdogTimeoutMSec = 1500;

/**
 * This wrapper prohibits flash access when normal operation cannot be interrupted.
 */
class CustomConfigStorageBackend : public os::stm32::ConfigStorageBackend
{
    os::Logger logger{"CustomConfigStorageBackend"};

    int write(std::size_t offset, const void* data, std::size_t len) override
    {
        // Raising priority allows to prevent race condition when accessing the inverter driver
        os::TemporaryPriorityChanger priority_adjustment_expert(HIGHPRIO);

        if (canModifyStorageNow() &&
            board::motor::suspend())
        {
            const int res = os::stm32::ConfigStorageBackend::write(offset, data, len);
            board::motor::unsuspend();
            if (res < 0)
            {
                logger.println("Write error %d", res);
            }
            return res;
        }
        else
        {
            logger.println("WRITE DENIED");
            return -EACCES;
        }
    }

    int erase() override
    {
        os::TemporaryPriorityChanger priority_adjustment_expert(HIGHPRIO);

        if (canModifyStorageNow() &&
            board::motor::suspend())
        {
            const int res = os::stm32::ConfigStorageBackend::erase();
            board::motor::unsuspend();
            if (res < 0)
            {
                logger.println("Erase error %d", res);
            }
            return res;
        }
        else
        {
            logger.println("ERASE DENIED");
            return -EACCES;
        }
    }

public:
    CustomConfigStorageBackend() :
        os::stm32::ConfigStorageBackend(reinterpret_cast<void*>(0x08008000),
                                        0x4000)
    { }

    static bool canModifyStorageNow()
    {
        return foc::isInactive() &&
               (board::motor::PWMHandle::getTotalNumberOfActiveHandles() == 0) &&
               !board::motor::isCalibrationInProgress();
    }
} g_config_storage_backend;

os::Logger g_logger("Main");


/**
 * This callback is invoked when any component wants the application to restart. This is thread safe.
 */
bool onRebootRequested(const char* reason)
{
    g_logger.println("Reboot requested; reason: %s", reason);
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

    g_logger.println("UAVCAN firmware update request from %d, source %d, path '%s'",
                     request.getSrcNodeID().get(),
                     request.source_node_id,
                     request.image_file_remote_path.path.c_str());

    if (already_in_progress)
    {
        g_logger.println("UAVCAN firmware update is already in progress, rejecting");
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

    g_logger.println("Bootloader args: CAN bus bitrate: %u, local node ID: %d",
                     unsigned(shared.can_bus_speed), shared.uavcan_node_id);

    /*
     * Commiting everything
     */
    bootloader_interface::writeSharedStruct(shared);

    os::requestReboot();

    already_in_progress = true;

    g_logger.puts("UAVCAN firmware update initiated");
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

    board::setRGBLED(board::RGB::Ones());

    const auto fw_version = bootloader_interface::getFirmwareVersion();

    const auto app_shared_read_result = bootloader_interface::readAndInvalidateSharedStruct();
    const auto& app_shared = app_shared_read_result.first;
    const auto app_shared_available = app_shared_read_result.second;

    if (app_shared_available)
    {
        g_logger.println("Bootloader struct values: CAN bitrate: %u, UAVCAN Node ID: %u",
                         unsigned(app_shared.can_bus_speed), app_shared.uavcan_node_id);
    }
    else
    {
        g_logger.puts("Bootloader struct is NOT present");
    }

    /*
     * Motor initialization
     */
    board::motor::init();
    foc::init(params::readFOCParameters());

    // Power on self test
    g_logger.puts("Testing hardware...");

    foc::beginHardwareTest();

    while (foc::isHardwareTestInProgress())
    {
        ::sleep(1);
    }

    g_logger.puts(foc::getHardwareTestReport().toString().c_str());

    board::motor::printStatus();

    // Clearing faults
    foc::stop();

    /*
     * Interfaces
     */
    cli::init(&onRebootRequested);

    uavcan_node::init(app_shared_available ? app_shared.can_bus_speed : 0,
                      app_shared_available ? app_shared.uavcan_node_id : 0,
                      {fw_version.major, fw_version.minor},
                      fw_version.image_crc64we,
                      fw_version.vcs_commit,
                      &onFirmwareUpdateRequestedFromUAVCAN,
                      &onRebootRequested);

    aux_cmd_iface::init();

    return watchdog;
}

/**
 * Managing configuration parameters in the background from the main thread.
 */
class BackgroundConfigManager
{
    static constexpr float ReloadDelay          = 0.5F;
    static constexpr float SaveDelay            = 1.0F;
    static constexpr float SaveDelayAfterError  = 10.0F;

    os::Logger logger{"BackgroundConfigManager"};

    unsigned modification_counter_ = os::config::getModificationCounter();
    ::systime_t last_modification_ts_ = chVTGetSystemTimeX();
    bool pending_reload_ = false;
    bool pending_save_ = false;
    bool last_save_failed_ = false;

    bool just_reloaded_ = false;
    bool just_saved_ = false;

    float getTimeSinceModification() const
    {
        return float(ST2MS(chVTTimeElapsedSinceX(last_modification_ts_))) / 1e3F;
    }

    static bool doDestructiveTruthTest(bool& what)
    {
        if (what)
        {
            what = false;
            return true;
        }
        return false;
    }

    static void doReload()
    {
        foc::setParameters(params::readFOCParameters());
        // TODO: Reload some other parameters, e.g. UAVCAN
    }

public:
    void poll()
    {
        const auto new_mod_cnt = os::config::getModificationCounter();

        if (new_mod_cnt != modification_counter_)
        {
            modification_counter_ = new_mod_cnt;
            last_modification_ts_ = chVTGetSystemTimeX();
            pending_reload_ = true;
            pending_save_ = true;
        }

        if (pending_reload_)
        {
            if (getTimeSinceModification() > ReloadDelay)
            {
                pending_reload_ = false;
                just_reloaded_ = true;
                logger.println("Reloading [modcnt=%u]", modification_counter_);

                doReload();
            }
        }

        if (pending_save_)
        {
            if (getTimeSinceModification() > (last_save_failed_ ? SaveDelayAfterError : SaveDelay) &&
                g_config_storage_backend.canModifyStorageNow())
            {
                logger.println("Saving [modcnt=%u]", modification_counter_);
                const int res = os::config::save();
                if (res >= 0)
                {
                    pending_save_ = false;
                    last_save_failed_ = false;
                    just_saved_ = true;
                }
                else
                {
                    last_save_failed_ = true;
                    logger.println("SAVE ERROR %d '%s'", res, std::strerror(std::abs(res)));
                }
            }
        }
    }

    bool hasBeenReloaded() { return doDestructiveTruthTest(just_reloaded_); }

    bool hasBeenSaved() { return doDestructiveTruthTest(just_saved_); }
};


void updateUAVCANNodeStatus(const bool board_ok,
                            const std::uint8_t upper_byte_of_vendor_specific_status_code)
{
    uavcan_node::setNodeHealth(board_ok ? uavcan_node::NodeHealth::OK : uavcan_node::NodeHealth::Warning);

    std::uint8_t lower_byte_vssc = 0;

    /*
     * Note that the access is non-atomic - the state may change while we're walking through the checks.
     * This is why we fall back to the inactive state by default as a last resort.
     */
    if (foc::isRunning())
    {
        uavcan_node::setNodeMode(uavcan_node::NodeMode::Operational);
    }
    else if (foc::isMotorIdentificationInProgress() ||
             foc::isHardwareTestInProgress())
    {
        uavcan_node::setNodeMode(uavcan_node::NodeMode::Maintenance);
    }
    else
    {
        foc::InactiveStateInfo inactive_info;
        (void) foc::isInactive(&inactive_info);
        if (inactive_info.fault_code == inactive_info.FaultCodeNone)
        {
            uavcan_node::setNodeMode(uavcan_node::NodeMode::Operational);
        }
        else
        {
            // Fault! Keeping the mode unchanged for better diagnostics
            uavcan_node::setNodeHealth(uavcan_node::NodeHealth::Critical);
            lower_byte_vssc = inactive_info.fault_code;
        }
    }

    uavcan_node::setVendorSpecificNodeStatusCode(
        std::uint16_t((std::uint16_t(upper_byte_of_vendor_specific_status_code) << 8) |
                      lower_byte_vssc));
}

led_indicator::Pattern makeLEDPattern(const bool board_ok)
{
    static const board::RGB Red   (1.0F, 0,    0);
    static const board::RGB Yellow(1.0F, 1.0F, 0);
    static const board::RGB Green (0,    1.0F, 0);
    static const board::RGB Blue  (0,    0,    1.0F);

    constexpr float Dim = 0.05F;

    using Behavior = led_indicator::Pattern::Behavior;

    const auto base = board_ok ? Green : Yellow;
    bool spinup_in_progress = false;

    /*
     * Note that the access is non-atomic - the state may change while we're walking through the checks.
     * This is why we fall back to the inactive state by default as a last resort.
     */
    if (foc::isRunning(nullptr, &spinup_in_progress))
    {
        return {base, spinup_in_progress ? Behavior::Blinking : Behavior::Solid};
    }
    else if (foc::isMotorIdentificationInProgress())
    {
        return {Blue, Behavior::Solid};
    }
    else if (foc::isHardwareTestInProgress())
    {
        return {Blue, Behavior::Blinking};
    }
    else
    {
        foc::InactiveStateInfo inactive_info;
        (void) foc::isInactive(&inactive_info);
        if (inactive_info.fault_code == inactive_info.FaultCodeNone)
        {
            return {base * Dim, Behavior::Solid};
        }
        else
        {
            return {Red, Behavior::Blinking};
        }
    }
}

std::pair<bool, std::uint8_t> analyzeBoardHealth()
{
    const auto s = board::motor::getStatus();
    const auto lim = board::motor::getLimits().safe_operating_area;

    const bool ok = lim.inverter_temperature.contains(s.inverter_temperature) &&
                    lim.inverter_voltage.contains(s.inverter_voltage) &&
                    s.isOkay();
    const auto code = (unsigned(s.power_ok) << 0) |
                      (unsigned(s.overload) << 1) |
                      (unsigned(s.fault) << 2) |
                      (unsigned(board::motor::isCalibrationInProgress()) << 3);
    return {ok, std::uint8_t(code)};
}

}
}

namespace os
{

extern void applicationHaltHook()
{
    board::motor::emergency();
    board::setRGBLED({1.0F, 0, 0});
}

}


int main()
{
    auto watchdog = app::init();

    /*
     * Confirming initialization
     */
    chibios_rt::BaseThread::setPriority(LOWPRIO);

    ::usleep(10000);
    foc::beep(5000.0F, 0.1F);
    ::usleep(100000);
    foc::beep(6000.0F, 0.1F);

    /*
     * Main loop
     */
    constexpr unsigned LoopPeriodMSec = 150;

    led_indicator::Indicator led_indicator;     // Dependent on the loop period
    app::BackgroundConfigManager config_manager;

    auto next_step_at = chVTGetSystemTime();

    while (!os::isRebootRequested())
    {
        watchdog.reset();

        config_manager.poll();
        if (config_manager.hasBeenSaved())
        {
            uavcan_node::log(uavcan_node::LogLevel::INFO, app::g_logger.getName(), "Config Saved");
            led_indicator.flash();
        }
        if (config_manager.hasBeenReloaded())
        {
            led_indicator.flash();
        }

        const auto board_health = app::analyzeBoardHealth();

        led_indicator.setPattern(app::makeLEDPattern(board_health.first));
        led_indicator.onNextTimeFrame();

        // This will also switch the node away from the Initialization state
        app::updateUAVCANNodeStatus(board_health.first,
                                    board_health.second);

        next_step_at += MS2ST(LoopPeriodMSec);
        os::sleepUntilChTime(next_step_at);
    }

    /*
     * Rebooting
     */
    board::setRGBLED(board::RGB::Ones());

    board::motor::emergency();                  // Just to be on the safe side

    app::g_logger.println("GOING DOWN FOR REBOOT");

    watchdog.reset();

    ::sleep(1);                                 // Let other threads terminate properly

    board::restart();

    return 0;
}
