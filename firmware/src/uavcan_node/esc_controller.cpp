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

#include "esc_controller.hpp"
#include <uavcan/equipment/esc/RPMCommand.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include <zubax_chibios/os.hpp>
#include <foc/foc.hpp>
#include <cstdint>


namespace uavcan_node
{
namespace esc_controller
{
namespace
{

const auto StatusTransferPriority = uavcan::TransferPriority::fromPercent<75>();


os::config::Param<std::uint8_t> g_param_esc_index                  ("uavcan.esc_indx",     0,      0,      15);
os::config::Param<float>        g_param_esc_cmd_ttl                ("uavcan.esc_ttl",   0.3F,   0.1F,   10.0F);
os::config::Param<float>        g_param_esc_status_interval        ("uavcan.esc_si",   0.05F,  0.01F,    1.0F);
os::config::Param<float>        g_param_esc_status_interval_passive("uavcan.esc_sip",   0.5F,  0.01F,   10.0F);

os::config::Param<unsigned>     g_param_esc_raw_control_mode       ("uavcan.esc_rcm",
                                                                    unsigned(foc::ControlMode::RatiometricVoltage),
                                                                    foc::FirstRatiometricControlMode,
                                                                    foc::LastRatiometricControlMode);


uavcan::LazyConstructor<uavcan::Publisher<uavcan::equipment::esc::Status>> g_pub_status;
uavcan::LazyConstructor<uavcan::Publisher<uavcan::protocol::debug::KeyValue>> g_pub_key_value;
uavcan::LazyConstructor<uavcan::Timer> g_timer;

std::uint8_t g_self_index;
float g_command_ttl;
foc::ControlMode g_raw_control_mode;


void cbRawCommand(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RawCommand>& msg)
{
    if (msg.cmd.size() > g_self_index)
    {
        const float command =
            float(msg.cmd[g_self_index]) /
            float(uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max());

        foc::setSetpoint(g_raw_control_mode, command, g_command_ttl);
    }
}


void cbRPMCommand(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RPMCommand>& msg)
{
    (void) msg;
    // TODO: Closed loop operation
}


void cbTimer(const uavcan::TimerEvent& event)
{
    /*
     * Scheduling the next timer event depending on the current state
     */
    {
        static const float interval_normal  = g_param_esc_status_interval.get();
        static const float interval_passive = g_param_esc_status_interval_passive.get();

        const auto current_interval = uavcan::MonotonicDuration::fromUSec(
            std::uint64_t((foc::isInactive() ? interval_passive : interval_normal) * 1e6F));

        g_timer->startOneShotWithDeadline(event.scheduled_time + current_interval);
    }

    /*
     * Publishing status
     */
    {
        uavcan::equipment::esc::Status status;
        const auto hw_status = board::motor::getStatus();

        status.esc_index   = g_self_index;
        status.voltage     = hw_status.inverter_voltage;
        status.temperature = hw_status.inverter_temperature;

        foc::RunningStateInfo running_info;
        foc::MotorIdentificationStateInfo motor_id_info;

        if (foc::isRunning(&running_info))
        {
            status.error_count = running_info.stall_count;
            status.current     = running_info.inverter_power_filtered / hw_status.inverter_voltage;
            status.rpm         = static_cast<std::int32_t>(std::round(running_info.mechanical_rpm));

            status.power_rating_pct =
                std::uint8_t(std::min(running_info.demand_factor_filtered * 100.0F + 0.5F, 126.0F));
        }
        else if (foc::isMotorIdentificationInProgress(&motor_id_info))
        {
            status.current = motor_id_info.inverter_power_filtered / hw_status.inverter_voltage;
            status.rpm     = static_cast<std::int32_t>(std::round(motor_id_info.mechanical_rpm));
        }
        else
        {
            ; // Nothing to do
        }

        (void) g_pub_status->broadcast(status);
    }

    /*
     * Publishing debug parameters, only in debug builds
     */
#if defined(DEBUG_BUILD) && DEBUG_BUILD

    for (auto& kv : foc::getDebugKeyValuePairs())
    {
        uavcan::protocol::debug::KeyValue msg;
        msg.key = kv.first.c_str();
        msg.value = kv.second;
        (void) g_pub_key_value->broadcast(msg);
    }

#endif
}

} // namespace

int init(uavcan::INode& node)
{
    static bool initialized = false;
    ASSERT_ALWAYS(!initialized);
    initialized = true;

    /*
     * Configuration parameters
     */
    g_self_index  = g_param_esc_index.get();
    g_command_ttl = g_param_esc_cmd_ttl.get();
    g_raw_control_mode = foc::ControlMode(g_param_esc_raw_control_mode.get());

    /*
     * Publishers
     */
    g_pub_status.construct<uavcan::INode&>(node);
    int pub_init_res = g_pub_status->init(StatusTransferPriority);
    if (pub_init_res < 0)
    {
        return pub_init_res;
    }

    g_pub_key_value.construct<uavcan::INode&>(node);
    pub_init_res = g_pub_key_value->init(uavcan::TransferPriority::OneHigherThanLowest);
    if (pub_init_res < 0)
    {
        return pub_init_res;
    }

    /*
     * Subscribers
     */
    {
        static uavcan::Subscriber<uavcan::equipment::esc::RawCommand> sub_raw_command(node);

        const int res = sub_raw_command.start(&cbRawCommand);
        if (res < 0)
        {
            return res;
        }
    }

    {
        static uavcan::Subscriber<uavcan::equipment::esc::RPMCommand> sub_rpm_command(node);

        const int res = sub_rpm_command.start(&cbRPMCommand);
        if (res < 0)
        {
            return res;
        }
    }

    /*
     * Timer
     */
    g_timer.construct<uavcan::INode&>(node);
    g_timer->setCallback(&cbTimer);
    // Arbitrary delay to kickstart the process
    g_timer->startOneShotWithDelay(uavcan::MonotonicDuration::fromMSec(1000));

    return 0;
}

}
}
