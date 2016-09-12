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

#include <cstdint>
#include <utility>


namespace bootloader_interface
{
/**
 * This struct is used to exchange data between the bootloader and the application.
 * Its format allows for future extensions.
 */
struct AppShared
{
    std::uint32_t reserved_a = 0;                               ///< Reserved for future use
    std::uint32_t reserved_b = 0;                               ///< Reserved for future use

    /*
     * UAVCAN part
     */
    std::uint32_t can_bus_speed = 0;                            ///< App <-> Bootloader
    std::uint8_t uavcan_node_id = 0;                            ///< App <-> Bootloader
    std::uint8_t uavcan_fw_server_node_id = 0;                  ///< App --> Bootloader

    static constexpr std::uint8_t UAVCANFileNameMaxLength = 201;
    char uavcan_file_name[UAVCANFileNameMaxLength] = {};        ///< App --> Bootloader

    /*
     * General part
     */
    bool stay_in_bootloader = false;                            ///< App --> Bootloader
};

static_assert(sizeof(AppShared) <= 240, "AppShared may be larger than the amount of allocated memory");
static_assert(sizeof(bool) == 1, "Please redefine bool as uint8_t in the shared struct (should never happen on ARM)");

/**
 * Descriptor of the firmware that is currently being executed.
 * This descriptor is used (at least) by the UAVCAN node and CLI interface.
 */
struct FirmwareVersion
{
    std::uint64_t image_crc64we = 0;    ///< CRC-64-WE, same as in libuavcan
    std::uint32_t vcs_commit = 0;       ///< 32-bit commit hash of the firmware source
    std::uint8_t major = 0;             ///< Major version number
    std::uint8_t minor = 0;             ///< Minor version number
};

/**
 * Returns version info of the currently running firmware image.
 * This function relies on the firmware image post-processing; refer to the build system docs for details.
 */
FirmwareVersion getFirmwareVersion();

/**
 * Reads the bootloader-app shared structure from the shared memory location and returns it by value.
 * The operation will fail if the structure is not written correctly (e.g. if the bootloader didn't provide any
 * information or if it was using wrong structure layout due to version mismatch, or whatever).
 * The success of the operation is indicated in the second member of the returned pair.
 * Note that the structure will be invalidated after read to prevent deja-vu.
 */
std::pair<AppShared, bool> readAndInvalidateSharedStruct();

/**
 * Writes the bootloader-app shared data structure.
 * This function cannot fail.
 */
void writeSharedStruct(const AppShared& shared);

}
