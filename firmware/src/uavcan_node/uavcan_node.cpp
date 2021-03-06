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

#include "uavcan_node.hpp"
#include "esc_controller.hpp"

#include <zubax_chibios/os.hpp>
#include <zubax_chibios/config/config.hpp>

#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/param_server.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include <uavcan/protocol/restart_request_server.hpp>

#include <board/board.hpp>
#include <foc/foc.hpp>

#include <unistd.h>


namespace uavcan_node
{
namespace
{
/**
 * Hardcoded params.
 */
static constexpr unsigned MemoryPoolSize = 8192;
static constexpr unsigned RxQueueDepth = 254;           ///< Can be safely reduced if we're tight on memory
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
std::uint16_t g_vendor_specific_status = 0;

// TODO: This flag thing is horrible and needs to be redesigned, but I'm not sure how
volatile bool g_do_print_status = false;

/**
 * Runtime configuration parameters.
 */
os::config::Param<std::uint8_t> g_param_node_id("uavcan.node_id",       0,      0,      125);

/**
 * Callbacks.
 */
FirmwareUpdateRequestCallback g_on_firmware_update_requested;
RebootRequestCallback g_on_reboot_requested;

os::Logger g_logger("UAVCAN");

/// An experiment in minimalism.
class LogMessageQueue
{
    chibios_rt::Mutex mutex_;
    std::array<std::pair<bool, uavcan::protocol::debug::LogMessage>, 2> buffer_{};
    bool write_pos_ = false;

public:
    void push(const uavcan::protocol::debug::LogMessage& msg)
    {
        os::MutexLocker locker(mutex_);
        buffer_[int(write_pos_)] = {true, msg};
        write_pos_ = !write_pos_;
    }

    bool pop(uavcan::protocol::debug::LogMessage& out_msg)
    {
        if (std::any_of(buffer_.begin(), buffer_.end(), [](auto& x) { return x.first; }))
        {
            os::MutexLocker locker(mutex_);
            for (auto& x : buffer_)
            {
                if (x.first)
                {
                    x.first = false;
                    out_msg = x.second;
                    return true;
                }
            }
        }
        return false;
    }
} g_log_message_queue_;

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
            g_logger.puts("REBOOT REQUEST HANDLER NOT SET");
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
        g_logger.puts("FIRMWARE UPDATE HANDLER NOT SET");
        response.error = response.ERROR_UNKNOWN;
        response.optional_error_message = "Not supported by application";
    }
}

/**
 * Log sink that prints to the system log.
 */
class LogSink : public uavcan::ILogSink
{
    void log(const uavcan::protocol::debug::LogMessage& message) override
    {
        g_logger.println("LogSink: [%u] %s: %s",
                         message.level.value,
                         message.source.c_str(),
                         message.text.c_str());
    }
} g_log_sink;

/**
 * Node thread.
 */
class NodeThread : public chibios_rt::BaseStaticThread<4096>
{
    os::watchdog::Timer wdt_;

    void printStatus()
    {
        std::printf("CAN bitrate: %lu\n", g_can_bit_rate);
        std::printf("Node ID:     %u\n", g_node_id.get());
        std::printf("Node mode:   %u\n", g_node_status_mode);
        std::printf("Node health: %u\n", g_node_status_health);

        const auto perf = getNode().getDispatcher().getTransferPerfCounter();

        const auto pool_capacity = getNode().getAllocator().getBlockCapacity();
        const auto pool_peak_usage = getNode().getAllocator().getPeakNumUsedBlocks();

        uavcan::CanIfacePerfCounters iface_perf[uavcan::MaxCanIfaces];
        std::uint8_t num_ifaces = 0;
        for (num_ifaces = 0; num_ifaces < getNode().getDispatcher().getCanIOManager().getNumIfaces(); num_ifaces++)
        {
            iface_perf[num_ifaces] = getNode().getDispatcher().getCanIOManager().getIfacePerfCounters(num_ifaces);
        }

        std::printf("Memory pool capacity:   %u blocks\n", pool_capacity);
        std::printf("Memory pool peak usage: %u blocks\n", pool_peak_usage);

        std::printf("Transfers RX/TX: %llu / %llu\n", perf.getRxTransferCount(), perf.getTxTransferCount());
        std::printf("Transfer errors: %llu\n", perf.getErrorCount());

        for (std::uint8_t i = 0; i < num_ifaces; i++)
        {
            std::printf("CAN iface %u:\n", i);
            std::printf("    Frames RX/TX: %llu / %llu\n", iface_perf[i].frames_rx, iface_perf[i].frames_tx);
            std::printf("    RX overflows: %lu\n", g_can.driver.getIface(i)->getRxQueueOverflowCount());
            std::printf("    Errors:       %llu\n", iface_perf[i].errors);
        }
    }

    void pollCommandFlags()     // TODO: This is ugly, needs to be refactored later!
    {
        if (g_do_print_status)
        {
            printStatus();
            g_do_print_status = false;
        }
    }

    void initCAN()
    {
        int res = 0;

        unsigned fixed_failures_cnt = 0;

        do
        {
            wdt_.reset();
            ::sleep(1);
            pollCommandFlags();

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
                g_logger.println("Could not init CAN; status: %d, autodetect: %d, bitrate: %u",
                                 res, int(autodetect), unsigned(bitrate));

                if (!autodetect)
                {
                    fixed_failures_cnt++;

                    if (fixed_failures_cnt >= FixedBitrateInitTimeoutSec)
                    {
                        g_logger.puts("Too many CAN init errors, falling back to autodetect");
                        g_can_bit_rate = 0;
                    }
                }
            }
        }
        while (res < 0);

        assert(g_can_bit_rate > 0);
        g_logger.println("CAN inited at %u bps", unsigned(g_can_bit_rate));
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

        getNode().getLogger().setExternalSink(&g_log_sink);
        getNode().getLogger().setLevel(LogLevel::INFO);

        /*
         * Starting the node
         */
        while (true)
        {
            pollCommandFlags();

            const int uavcan_start_res = getNode().start();
            if (uavcan_start_res >= 0)
            {
                break;
            }
            g_logger.println("Node init failure: %i, will retry", uavcan_start_res);
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
            g_logger.println("Using static node ID %d", int(getNode().getNodeID().get()));
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

            g_logger.puts("Waiting for dynamic node ID allocation...");

            while (!dnid_client.isAllocationComplete())
            {
                pollCommandFlags();
                getNode().spin(uavcan::MonotonicDuration::fromMSec(100));
                wdt_.reset();
            }

            g_logger.println("Dynamic node ID %d allocated by %d",
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

        res = esc_controller::init(getNode());
        if (res < 0)
        {
            board::die(res);
        }

        // TODO: Indication API
        // TODO: Enumeration API

        g_logger.println("Node started, ID %i", int(getNode().getNodeID().get()));
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
            getNode().getNodeStatusProvider().setVendorSpecificStatusCode(g_vendor_specific_status);

            const int spin_res = getNode().spin(uavcan::MonotonicDuration::fromMSec(100));
            if (spin_res < 0)
            {
                g_logger.println("Spin: %d", spin_res);
            }

            pollCommandFlags();

            uavcan::protocol::debug::LogMessage log_message;
            if (g_log_message_queue_.pop(log_message))
            {
                const int result = getNode().getLogger().log(log_message);
                if (result < 0)
                {
                    g_logger.println("Log: %d", result);
                }
            }
        }

        g_logger.puts("Goodbye");
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

void setNodeHealth(NodeHealth ns)
{
    switch (ns)
    {
    case NodeHealth::OK:
    {
        g_node_status_health = uavcan::protocol::NodeStatus::HEALTH_OK;
        break;
    }
    case NodeHealth::Warning:
    {
        g_node_status_health = uavcan::protocol::NodeStatus::HEALTH_WARNING;
        break;
    }
    case NodeHealth::Critical:
    {
        g_node_status_health = uavcan::protocol::NodeStatus::HEALTH_CRITICAL;
        break;
    }
    }
}

void setNodeMode(NodeMode mode)
{
    switch (mode)
    {
    case NodeMode::Operational:
    {
        g_node_status_mode = uavcan::protocol::NodeStatus::MODE_OPERATIONAL;
        break;
    }
    case NodeMode::Maintenance:
    {
        g_node_status_mode = uavcan::protocol::NodeStatus::MODE_MAINTENANCE;
        break;
    }
    }
}

void setVendorSpecificNodeStatusCode(std::uint16_t value)
{
    g_vendor_specific_status = value;
}

void log(const uavcan::StorageType<LogLevel::FieldTypes::value>::Type level,
         const char* const source,
         const char* const text)
{
    uavcan::protocol::debug::LogMessage msg;
    msg.level.value = level;
    msg.source = source;
    msg.text   = text;
    g_log_message_queue_.push(msg);
}

uavcan::NodeID getNodeID()
{
    return g_node_id;
}

std::uint32_t getCANBusBitRate()
{
    return g_can_bit_rate;
}

void printStatusInfo()
{
    // Ugh this is wrong
    g_do_print_status = true;

    for (int i = 0; i < 20; i++)
    {
        ::usleep(100000);
        if (!g_do_print_status)
        {
            break;
        }
    }
}

}
