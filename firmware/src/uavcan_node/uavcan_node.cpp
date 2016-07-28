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
#include <uavcan/protocol/restart_request_server.hpp>

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
static constexpr unsigned FixedBitrateInitTimeoutSec = 10;

/**
 * Node declarations.
 */
typedef uavcan::Node<MemoryPoolSize> Node;

uavcan_stm32::CanInitHelper<RxQueueDepth> g_can;        ///< CAN driver instance.

uavcan::protocol::SoftwareVersion g_firmware_version;
std::uint32_t g_can_bit_rate;
uavcan::NodeID g_node_id;
std::uint8_t g_node_status_mode = uavcan::protocol::NodeStatus::MODE_INITIALIZATION;
std::uint8_t g_node_status_health = uavcan::protocol::NodeStatus::HEALTH_OK;

/**
 * Runtime configuration parameters.
 */
os::config::Param<std::uint8_t> g_param_node_id("uavcan.node_id",       0,      0,      125);

/**
 * Callbacks.
 */
FirmwareUpdateRequestCallback g_on_firmware_update_requested;
RebootRequestCallback g_on_reboot_requested;

/**
 * Implementation details.
 * Functions that return references to statics are designed this way as means to implement late initialization.
 */
Node& getNode()
{
    static Node node(g_can.driver, uavcan_stm32::SystemClock::instance());
    return node;
}

uavcan::ParamServer& getParamServer()
{
    static uavcan::ParamServer server(getNode());
    return server;
}

/**
 * Param access server
 * TODO: Rewrite to use pure C++ API to Zubax ChibiOS.
 */
class ParamManager : public uavcan::IParamManager
{
    void convert(float native_value, ConfigDataType native_type, Value& out_value) const
    {
        if (native_type == CONFIG_TYPE_BOOL)
        {
            out_value.to<Value::Tag::boolean_value>() = !uavcan::isCloseToZero(native_value);
        }
        else if (native_type == CONFIG_TYPE_INT)
        {
            out_value.to<Value::Tag::integer_value>() = static_cast<std::int64_t>(native_value);
        }
        else if (native_type == CONFIG_TYPE_FLOAT)
        {
            out_value.to<Value::Tag::real_value>() = native_value;
        }
        else
        {
            ; // Invalid type - leave empty
        }
    }

    void convert(float native_value, ConfigDataType native_type, NumericValue& out_value) const
    {
        if (native_type == CONFIG_TYPE_INT)
        {
            out_value.to<NumericValue::Tag::integer_value>() = static_cast<std::int64_t>(native_value);
        }
        else if (native_type == CONFIG_TYPE_FLOAT)
        {
            out_value.to<NumericValue::Tag::real_value>() = native_value;
        }
        else
        {
            ; // Not applicable - leave empty
        }
    }

    void getParamNameByIndex(Index index, Name& out_name) const override
    {
        const char* name = configNameByIndex(index);
        if (name != nullptr)
        {
            out_name = name;
        }
    }

    void assignParamValue(const Name& name, const Value& value) override
    {
        float native_value = 0.F;

        if (value.is(Value::Tag::boolean_value))
        {
            native_value = (*value.as<Value::Tag::boolean_value>()) ? 1.F : 0.F;
        }
        else if (value.is(Value::Tag::integer_value))
        {
            native_value = static_cast<float>(*value.as<Value::Tag::integer_value>());
        }
        else if (value.is(Value::Tag::real_value))
        {
            native_value = *value.as<Value::Tag::real_value>();
        }
        else
        {
            return;
        }

        (void)configSet(name.c_str(), native_value);
    }

    void readParamValue(const Name& name, Value& out_value) const override
    {
        ConfigParam descr;
        const int res = configGetDescr(name.c_str(), &descr);
        if (res >= 0)
        {
            convert(configGet(name.c_str()), descr.type, out_value);
        }
    }

    void readParamDefaultMaxMin(const Name& name, Value& out_default,
                                NumericValue& out_max, NumericValue& out_min) const override
    {
        ConfigParam descr;
        const int res = configGetDescr(name.c_str(), &descr);
        if (res >= 0)
        {
            convert(descr.default_, descr.type, out_default);
            convert(descr.max, descr.type, out_max);
            convert(descr.min, descr.type, out_min);
        }
    }

    int saveAllParams() override
    {
        return configSave();
    }

    int eraseAllParams() override
    {
        return configErase();
    }
} g_param_manager;

/**
 * Restart handler
 */
class RestartRequestHandler : public uavcan::IRestartRequestHandler
{
    bool handleRestartRequest(uavcan::NodeID request_source) override
    {
        uavcan::MakeString<40>::Type str("Request from node ");
        str.appendFormatted("%d", request_source.get());

        if (g_on_reboot_requested)
        {
            return g_on_reboot_requested(str.c_str());
        }
        else
        {
            os::lowsyslog("UAVCAN: REBOOT REQUEST HANDLER NOT SET\n");
            return false;
        }
    }
} g_restart_request_handler;

/**
 * Firmware update server
 */
auto& getBeginFirmwareUpdateServer()
{
    static uavcan::ServiceServer<uavcan::protocol::file::BeginFirmwareUpdate,
        void (*)(const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>&,
                 uavcan::protocol::file::BeginFirmwareUpdate::Response&)> srv(getNode());
    return srv;
}

void handleBeginFirmwareUpdateRequest(
    const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>& request,
    uavcan::protocol::file::BeginFirmwareUpdate::Response& response)
{
    assert(g_can_bit_rate > 0);
    assert(g_node_id.isUnicast());

    if (g_on_firmware_update_requested)
    {
        response.error = g_on_firmware_update_requested(request);
    }
    else
    {
        os::lowsyslog("UAVCAN: FIRMWARE UPDATE HANDLER NOT SET\n");
        response.error = response.ERROR_UNKNOWN;
        response.optional_error_message = "Not supported by application";
    }
}

/**
 * Node thread.
 */
class NodeThread : public chibios_rt::BaseStaticThread<4096>
{
    os::watchdog::Timer wdt_;

    void initCAN()
    {
        int res = 0;

        unsigned fixed_failures_cnt = 0;

        do
        {
            wdt_.reset();
            ::sleep(1);

            auto bitrate = g_can_bit_rate;
            const bool autodetect = bitrate == 0;

            res = g_can.init([]() { ::usleep(::useconds_t(g_can.getRecommendedListeningDelay().toUSec())); }, bitrate);
            if (res >= 0)
            {
                g_can_bit_rate = bitrate;
            }
            else if (autodetect && (res == -uavcan_stm32::ErrBitRateNotDetected))
            {
                ; // Nothing to do
            }
            else
            {
                os::lowsyslog("UAVCAN: Could not init CAN; status: %d, autodetect: %d, bitrate: %u\n",
                              res, int(autodetect), unsigned(bitrate));

                if (!autodetect)
                {
                    fixed_failures_cnt++;

                    if (fixed_failures_cnt >= FixedBitrateInitTimeoutSec)
                    {
                        os::lowsyslog("UAVCAN: Too many CAN init errors, falling back to autodetect\n");
                        g_can_bit_rate = 0;
                    }
                }
            }
        }
        while (res < 0);

        assert(g_can_bit_rate > 0);
        os::lowsyslog("UAVCAN: CAN inited at %u bps\n", unsigned(g_can_bit_rate));
    }

    void initNode()
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

        g_node_id = getNode().getNodeID();

        /*
         * Initializing the business logic
         */
        getNode().setRestartRequestHandler(&g_restart_request_handler);

        int res = getParamServer().start(&g_param_manager);
        if (res < 0)
        {
            board::die(res);
        }

        res = getBeginFirmwareUpdateServer().start(&handleBeginFirmwareUpdateRequest);
        if (res < 0)
        {
            board::die(res);
        }

        // TODO: ESC API
        // TODO: Indication API
        // TODO: Enumeration API

        os::lowsyslog("UAVCAN: Node started, ID %i\n", int(getNode().getNodeID().get()));
    }

    void main() override
    {
        wdt_.startMSec(10000);
        setName("uavcan");

        initCAN();

        wdt_.reset();

        initNode();

        assert(g_can_bit_rate > 0);
        assert(g_node_id.isUnicast());

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
          std::uint32_t firmware_vcs_commit,
          const FirmwareUpdateRequestCallback& on_firmware_update_requested,
          const RebootRequestCallback& on_reboot_requested)
{
    g_can_bit_rate = bit_rate_hint;
    g_node_id = node_id_hint;

    g_firmware_version.major = firmware_version_major_minor.first;
    g_firmware_version.minor = firmware_version_major_minor.second;
    g_firmware_version.image_crc = firmware_image_crc64we;
    g_firmware_version.vcs_commit = firmware_vcs_commit;
    g_firmware_version.optional_field_flags =
        g_firmware_version.OPTIONAL_FIELD_FLAG_IMAGE_CRC | g_firmware_version.OPTIONAL_FIELD_FLAG_VCS_COMMIT;

    g_on_firmware_update_requested = on_firmware_update_requested;
    g_on_reboot_requested = on_reboot_requested;

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

void notifyNodeInitializationComplete()
{
    // Atomic write, no need for locking
    g_node_status_mode = uavcan::protocol::NodeStatus::MODE_OPERATIONAL;
}

uavcan::NodeID getNodeID()
{
    return g_node_id;
}

std::uint32_t getCANBusBitRate()
{
    return g_can_bit_rate;
}

}
