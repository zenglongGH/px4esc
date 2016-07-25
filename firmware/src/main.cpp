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

#include <ch.hpp>
#include <hal.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <zubax_chibios/os.hpp>
#include <Eigen/Eigen>

#include "board/board.hpp"
#include "bootloader_interface/bootloader_interface.hpp"
#include "uavcan/uavcan_node.hpp"

#if __GNUC__ < 5
# error "GCC version 5.x or newer is required"
#endif

// Testing
using Scalar = float;
template <int Rows, int Cols> using Matrix = Eigen::Matrix<Scalar, Rows, Cols>;
template <int Size> using Vector = Matrix<Size, 1>;


namespace app
{
namespace
{

constexpr unsigned WatchdogTimeoutMSec = 1500;


os::watchdog::Timer init()
{
    /*
     * Board initialization
     */
    auto watchdog = board::init(WatchdogTimeoutMSec);

    bootloader_interface::init();
    const auto fw_version = bootloader_interface::getFirmwareVersion();

    /*
     * UAVCAN node initialization
     */
    uavcan_node::init(bootloader_interface::getInheritedCANBusBitRate(),
                      bootloader_interface::getInheritedUAVCANNodeID(),
                      {fw_version.major, fw_version.minor},
                      fw_version.image_crc64we,
                      fw_version.vcs_commit);

    return watchdog;
}

}
}


int main()
{
    /*
     * Initializing
     */
    auto watchdog = app::init();

    chibios_rt::BaseThread::setPriority(NORMALPRIO);

    std::uint8_t counter = 0;

    while (true)
    {
        watchdog.reset();

        // Flying colors for testing
        board::setLEDRGB(counter, std::uint8_t(counter + 85 * 1), std::uint8_t(counter + 85 * 2));

        ::usleep(10000);
        counter++;
    }
}
