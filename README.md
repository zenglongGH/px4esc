# PX4ESC

PX4ESC is a state-of-the-art open source motor controller firmware for electrically propelled aircraft.

## Features

* Highly efficient field-oriented control
(over 15% improvement in efficiency compared to propulsion systems based on the traditional six-step commutation method).
* Excellent response characteristics.
* [CAN bus interface](http://uavcan.org/) with optional dual bus redundancy for safety critical applications.
* Communication interfaces are compatible with the open [DroneCode](https://www.dronecode.org/) standards,
both at the physical and protocol levels.
* The firmware is suitable for motors of different power levels, from tens of watts up to kilowatts.
* Reliability -
the code is implemented in C++14 with partial adherence to the MISRA guidelines and is statically verified.
* Low cost -
the project is built on an STM32 microcontroller using open source or low cost development tools.

## Publications

List of publications that refer to PX4ESC:

* [Efficient BLDC controller for critical UAV applications (Pavel Kirienko, Dmitry Ramensky, Anton Krol; IMAV 2015)](https://zubax.com/files/zubax_imav_2015.pdf)
* [PX4ESC and Sapog - open source motor controller firmware (Pavel Kirienko; DroneCode Unconference 2016)](https://zubax.com/files/px4esc_and_sapog_elc_2016.pdf)

Feel free to send pull requests extending this list.

## Repository Layout

The repository contains the following directories:

* `firmware` - the firmware, all its dependencies and main documentation are located here.
This directory has a dedicated README file.
* `tools` - a set of tools that may be useful for developers. They may have no relevance to the end users.
Each tool may have a dedicated README file.

## Development Tools

### Software

The major software tools used in the development process are listed below:

* GNU ARM Toolchain version 5.4 or newer (C++14 support is required)
* Eagle CAD version 7.2 or newer

We recommend Eclipse Neon or newer as an IDE.

### Hardware

The following tools may be needed for development work:

* JTAG/SWD debugger compatible with STM32 and supported by the toolchain.
* High speed USB-UART adapter.
* USB-CAN adapter with DroneCode-compatible connectors.

The listed tools are available from Zubax Robotics:

* [DroneCode Probe](https://zubax.com/product/dronecode-probe)
* [Zubax Babel](https://zubax.com/product/zubax-babel)

## License

### Firmware

The firmware is distributed under the terms of the three-clause BSD license, which is provided below.

> Copyright (c) 2016  Zubax Robotics OU  <info@zubax.com>
>
> All rights reserved.
>
> Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
> following conditions are met:
>
> 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
>    disclaimer.
>
> 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
>    following disclaimer in the documentation and/or other materials provided with the distribution.
>
> 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
>    products derived from this software without specific prior written permission.
>
> THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
> INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
> DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
> SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
> SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
> WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
> USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

### Hardware

Reference hardware sources will be published later under an open source license.
Please stay tuned for updates.
