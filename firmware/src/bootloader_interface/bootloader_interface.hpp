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


namespace bootloader_interface
{
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
 * Reads the bootloader's shared data structure.
 * This function must be invoked before the CAN hardware is initialized.
 */
void init();

/**
 * Returns version info of the currently running firmware image.
 * This function relies on the firmware image post-processing; refer to the build system docs for details.
 */
FirmwareVersion getFirmwareVersion();

/**
 * If known, returns the bit rate value inherited from the bootloader.
 * If unknown, return zero.
 */
std::uint32_t getInheritedCANBusBitRate();

/**
 * If known, returns the node ID value inherited from the bootloader.
 * If unknown, returns zero.
 */
std::uint8_t getInheritedUAVCANNodeID();

/**
 * Initializes the shared data structure with given values.
 */
void passParametersToBootloader(std::uint32_t can_bus_bit_rate, std::uint8_t uavcan_node_id);

}
