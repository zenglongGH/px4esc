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

#include "uavcan_node.hpp"

#include <zubax_chibios/os.hpp>
#include <zubax_chibios/config/config.hpp>

#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/param_server.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>

#include <board/board.hpp>

#include <unistd.h>


namespace uavcan_node
{
namespace
{
/**
 * Hardcoded params.
 */
static constexpr unsigned MemoryPoolSize = 8192;
static constexpr unsigned RxQueueDepth = 32;
static constexpr unsigned NodeThreadPriority = (HIGHPRIO + NORMALPRIO) / 2;

/**
 * Node declarations.
 */
typedef uavcan::Node<MemoryPoolSize> Node;

uavcan_stm32::CanInitHelper<RxQueueDepth> g_can;        ///< CAN driver instance.

uavcan::protocol::SoftwareVersion g_firmware_version;
std::uint32_t g_can_bit_rate;
uavcan::NodeID g_node_id;
std::uint8_t g_node_status_mode = uavcan::protocol::NodeStatus::MODE_OPERATIONAL;
std::uint8_t g_node_status_health = uavcan::protocol::NodeStatus::HEALTH_OK;

/**
 * Runtime configuration parameters.
 */
os::config::Param<std::uint8_t> g_param_node_id("uavcan.node_id",       0,      0,      125);

/**
 * Implementation details.
 * Functions that return references to statics are designed this way as means to implement late initialization.
 */
Node& getNode()
{
    static Node node(g_can.driver, uavcan_stm32::SystemClock::instance());
    return node;
}

//uavcan::ParamServer& getParamServer()
//{
//    static uavcan::ParamServer server(getNode());
//    return server;
//}

/**
 * Node thread.
 */
class NodeThread : public chibios_rt::BaseStaticThread<4096>
{
    os::watchdog::Timer wdt_;

    void init_can()
    {
        int res = 0;
        do
        {
            wdt_.reset();
            ::sleep(1);

            auto bitrate = g_can_bit_rate;
            const bool autodetect = bitrate == 0;

            res = g_can.init([]() { ::usleep(g_can.getRecommendedListeningDelay().toUSec()); }, bitrate);
            if (res >= 0)
            {
                g_can_bit_rate = bitrate;
                os::lowsyslog("UAVCAN: CAN inited at %u bps\n", unsigned(g_can_bit_rate));
            }
            else if (autodetect && (res == -uavcan_stm32::ErrBitRateNotDetected))
            {
                ; // Nothing to do
            }
            else
            {
                os::lowsyslog("UAVCAN: Could not init CAN; status: %d, autodetect: %d, bitrate: %u\n",
                              res, int(autodetect), unsigned(bitrate));
            }
        }
        while (res < 0);

        assert(g_can_bit_rate > 0);
    }

    void init_node()
    {
        /*
         * Filling node info
         */
        getNode().setName(PRODUCT_ID_STRING);

        getNode().setSoftwareVersion(g_firmware_version);

        {
            uavcan::protocol::HardwareVersion hwver;

            const auto hw_version = board::detectHardwareVersion();
            hwver.major = hw_version.major;
            hwver.minor = hw_version.minor;

            const auto uid = board::readUniqueID();
            std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));

            board::DeviceSignature signature;
            if (board::tryReadDeviceSignature(signature))
            {
                std::copy(std::begin(signature), std::end(signature),
                          std::back_inserter(hwver.certificate_of_authenticity));
            }

            getNode().setHardwareVersion(hwver);
        }

        /*
         * Starting the node
         */
        while (true)
        {
            const int uavcan_start_res = getNode().start();
            if (uavcan_start_res >= 0)
            {
                break;
            }
            os::lowsyslog("UAVCAN: Node init failure: %i, will retry\n", uavcan_start_res);
            ::sleep(1);
        }
        assert(getNode().isStarted());

        /*
         * Configuring node ID
         */
        if (g_param_node_id.get() > 0 || g_node_id.isUnicast())         // Node ID is already known
        {
            // Config takes precedence over hint
            getNode().setNodeID((g_param_node_id.get() > 0) ? g_param_node_id.get() : g_node_id);
            os::lowsyslog("UAVCAN: Using static node ID %d\n", int(getNode().getNodeID().get()));
        }
        else
        {
            uavcan::DynamicNodeIDClient dnid_client(getNode());

            const int start_res = dnid_client.start(getNode().getNodeStatusProvider().getHardwareVersion().unique_id);
            if (start_res < 0)
            {
                assert(false);
                board::die(start_res);  // Should never happen
            }

            os::lowsyslog("UAVCAN: Waiting for dynamic node ID allocation...\n");

            while (!dnid_client.isAllocationComplete())
            {
                getNode().spin(uavcan::MonotonicDuration::fromMSec(100));
                wdt_.reset();
            }

            os::lowsyslog("UAVCAN: Dynamic node ID %d allocated by %d\n",
                          int(dnid_client.getAllocatedNodeID().get()),
                          int(dnid_client.getAllocatorNodeID().get()));

            getNode().setNodeID(dnid_client.getAllocatedNodeID());
        }

        /*
         * Initializing the business logic
         */
//        getNode().setRestartRequestHandler(&restart_request_handler);
//
//        int res = get_param_server().start(&param_manager);
//        if (res < 0)
//        {
//            board::die(res);
//        }
//
//        res = init_esc_controller(getNode());
//        if (res < 0)
//        {
//            board::die(res);
//        }
//
//        res = init_indication_controller(getNode());
//        if (res < 0)
//        {
//            board::die(res);
//        }
//
//        res = get_begin_firmware_update_server().start(&handle_begin_firmware_update_request);
//        if (res < 0)
//        {
//            board::die(res);
//        }
//
//        enumeration_handler_.construct<uavcan::INode&>(getNode());
//        res = enumeration_handler_->start();
//        if (res < 0)
//        {
//            board::die(res);
//        }

        os::lowsyslog("UAVCAN: Node started, ID %i\n", int(getNode().getNodeID().get()));
    }

    void main() override
    {
        wdt_.startMSec(10000);
        setName("uavcan");

        init_can();

        wdt_.reset();

        init_node();

        while (!os::isRebootRequested())
        {
            wdt_.reset();

            getNode().getNodeStatusProvider().setHealth(g_node_status_health);
            getNode().getNodeStatusProvider().setMode(g_node_status_mode);

            const int spin_res = getNode().spin(uavcan::MonotonicDuration::fromMSec(100));
            if (spin_res < 0)
            {
                os::lowsyslog("UAVCAN: Spin: %d\n", spin_res);
            }
        }

        os::lowsyslog("UAVCAN: Goodbye\n");
        (void) getNode().spin(uavcan::MonotonicDuration::fromMSec(10));
    }

public:
    virtual ~NodeThread() { }
} g_node_thread;

}

void init(std::uint32_t bit_rate_hint,
          std::uint8_t node_id_hint,
          std::pair<std::uint8_t, std::uint8_t> firmware_version_major_minor,
          std::uint64_t firmware_image_crc64we,
          std::uint32_t firmware_vcs_commit)
{
    g_can_bit_rate = bit_rate_hint;
    g_node_id = node_id_hint;

    g_firmware_version.major = firmware_version_major_minor.first;
    g_firmware_version.minor = firmware_version_major_minor.second;
    g_firmware_version.image_crc = firmware_image_crc64we;
    g_firmware_version.vcs_commit = firmware_vcs_commit;
    g_firmware_version.optional_field_flags =
        g_firmware_version.OPTIONAL_FIELD_FLAG_IMAGE_CRC | g_firmware_version.OPTIONAL_FIELD_FLAG_VCS_COMMIT;

    (void) g_node_thread.start(NodeThreadPriority);
}

void setNodeStatus(NodeStatus ns)
{
    switch (ns)
    {
    case NodeStatus::OK:
    {
        g_node_status_health = uavcan::protocol::NodeStatus::HEALTH_OK;
        break;
    }
    case NodeStatus::Warning:
    {
        g_node_status_health = uavcan::protocol::NodeStatus::HEALTH_WARNING;
        break;
    }
    case NodeStatus::Critical:
    {
        g_node_status_health = uavcan::protocol::NodeStatus::HEALTH_CRITICAL;
        break;
    }
    }
}

std::uint8_t getNodeID()
{
    return g_node_id.isUnicast() ? g_node_id.get() : 0;
}

std::uint32_t getCANBusBitRate()
{
    return g_can_bit_rate;
}

}
