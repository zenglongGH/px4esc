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

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <cstdint>
#include <utility>
#include <functional>


namespace uavcan_node
{
/**
 * Refer to @ref setNodeHealth().
 */
enum class NodeHealth
{
    OK,
    Warning,
    Critical
};

/**
 * Refer to @ref setNodeMode().
 */
enum class NodeMode
{
    Operational,
    Maintenance
};

/**
 * A callback of this type will be invoked FROM THE NODE THREAD if the node receives a firmware update request.
 * The callback should make the firmware reboot into the bootloader with proper arguments.
 * Return value is the error code; refer to uavcan::protocol::file::BeginFirmwareUpdate::Response for details
 * (or see the UAVCAN specification).
 */
using FirmwareUpdateRequestCallback =
    std::function<uavcan::StorageType<uavcan::protocol::file::BeginFirmwareUpdate::Response::FieldTypes::error>::Type
                  (const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>&)>;

/**
 * A callback of this type is invoked FROM THE NODE THREAD when the node needs the application to reboot itself
 * (e.g. by external request). This has no relation to the firmware update procedure.
 * The return value should be true if the node can be rebooted, false otherwise. This value will be returned to the
 * requesting node.
 */
using RebootRequestCallback = std::function<bool (const char* reason)>;

/**
 * Starts the local UAVCAN node in a dedicated thread.
 * This function cannot fail.
 *
 * @param bit_rate_hint                 CAN bus bitrate to use; zero if no hint is available.
 *                                      Typically this value should be provided by the bootloader.
 *
 * @param node_id_hint                  Node ID to use; use broadcast or invalid node ID if no hint is available.
 *                                      Typically this value should be provided by the bootloader.
 *
 * @param firmware_version_major_minor  Pair of major and minor firmware version numbers.
 *
 * @param firmware_image_crc64we        CRC-64-WE of the firmware image.
 *
 * @param firmware_vcs_commit           Commit hash of the firmware sources.
 */
void init(std::uint32_t bit_rate_hint,
          std::uint8_t node_id_hint,
          std::pair<std::uint8_t, std::uint8_t> firmware_version_major_minor,
          std::uint64_t firmware_image_crc64we,
          std::uint32_t firmware_vcs_commit,
          const FirmwareUpdateRequestCallback& on_firmware_update_requested,
          const RebootRequestCallback& on_reboot_requested);

/**
 * Sets the status of the local node.
 * The values are translated into the UAVCAN-defined node status code.
 */
void setNodeHealth(NodeHealth ns);
void setNodeMode(NodeMode mode);
void setVendorSpecificNodeStatusCode(std::uint16_t value);

/**
 * Returns current node ID (possibly invalid if it is not yet known).
 */
uavcan::NodeID getNodeID();

/**
 * Returns current CAN bus bitrate, if set, zero otherwise.
 */
std::uint32_t getCANBusBitRate();

/**
 * Prints node status information into stdout.
 */
void printStatusInfo();

}
