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

#pragma once

#include <cstdint>
#include <utility>


namespace uavcan_node
{
/**
 * Refer to setNodeStatus().
 */
enum class NodeStatus
{
    OK,
    Warning,
    Critical
};

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
          std::uint32_t firmware_vcs_commit);

/**
 * Sets the status of the local node. The NodeStatus value is translated into the UAVCAN-defined node status code.
 */
void setNodeStatus(NodeStatus ns);

/**
 * Signal the local node that the application has completed initialization and is ready to perform its functions.
 */
void notifyNodeInitializationComplete();

/**
 * Returns current node ID, if set, zero otherwise.
 */
std::uint8_t getNodeID();

/**
 * Returns current CAN bus bitrate, if set, zero otherwise.
 */
std::uint32_t getCANBusBitRate();

}
