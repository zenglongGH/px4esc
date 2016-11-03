# PX4ESC

[![Join the chat at https://gitter.im/Zubax/general](https://img.shields.io/badge/GITTER-join%20chat-green.svg)](https://gitter.im/Zubax/general)
<a href="https://scan.coverity.com/projects/px4esc">
  <img alt="Coverity Scan Build Status" src="https://scan.coverity.com/projects/10506/badge.svg"/>
</a>

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

## Compatible Hardware

This list contains all known hardware designs that can run the PX4ESC firmware.
We accept extensions to this list via pull requests.

* [Pixhawk ESC v1.6](http://www.auav.co/product-p/pixhawkesc16dev.htm)
  * Voltage: 9~26 V (intended to support 55 V, but a hardware bug limits the maximum voltage)
  * Dual CAN bus, USB, UART, RCPWM
* Zubax Macaroni 400 (coming soon!)
  * Voltage: 9~52 V (3~12 S Li-ion battery)
  * Power capability: 400 W continuous, 1200 W peak
  * Dual CAN bus, USB, UART, RCPWM

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

## Basic Usage

### Initial Setup

Connect the motor to your ESC hardware and power it on.
Once powered up the first time, the board will start blinking red,
indicating that it's not ready to run because it lacks a valid model of the connected motor.

Connect to the board using either CAN bus, UART, or USB.

#### Connecting via CAN bus

If you chose to use CAN bus and you have a CAN adapter for your PC,
it is recommended to use the [UAVCAN GUI Tool](http://uavcan.org/GUI_Tool).

All configuration parameters are exposed via the standard UAVCAN node configuration services.
There is one special configuration parameter named `exec_aux_command`,
which serves as a rudimentary command execution interface.
Writing the following numbers will trigger actions as defined:

* 0..X - load a pre-defined motor model under the specified index.
* 1000 - perform the hardware self-test.
* 1001 - perform motor identification, static mode.
* 1002 - perform motor identification, free rotation mode.

#### Connecting via CLI

The CLI (command line interface) is exposed via UART at 115200-8N1.
**At the moment, the USB interface is not yet supported in the firmware**
(the operating system lacks necessary drivers; they should be released before the end of 2016).

The following commands will be of the most interest
(most of them will print usage info when executed without arguments):

* `help`        - list all available commands.
* `sp`          - assign setpoint to the motor controller.
* `motor_db`    - view the database of predefined motor profiles and load motor models from it.
* `motor_id`    - begin motor identification.
* `s`, `status` - print status information, brief and detailed, respectively.
* `spin`        - spin the rotor in an open loop at the specified electrical angular rate.
This command is useful for testing.
* `cfg`         - view or modify configuration parameters.
All configuration parameters are stored automatically upon modification, no additional actions needed.
* `kvconv`      - convert between magnetic flux linkage (a.k.a. Phi) and KV.
* `sysinfo`     - display general status information of the operating system.

Note that some commands can accept the `-p` argument, in which case they will print real-time values
in a special format that is understood by the serial plotting tool, located in the `tools` directory
in this repository.

When submitting bug reports, please always include outputs of the commands `status` and `sysinfo`
after the failure occurred.

### Specifying the Motor Parameters

Using whatever interface available to you (e.g. CAN, CLI),
assign proper values to the following configuration parameters:

* `m.max_ampere` - maximum phase current [ampere].
* `m.num_poles` - number of magnetic poles (number of permanent magnets on the rotor).
This is always an even number.

All parameters whose names start with `m.` are related to the model of the motor.
You may want to explicitly specify some other parameters as well, but that is rarely needed.

Alternatively, you may want to load a pre-defined model from the database,
if parameters for your motor are available there.

If you happen to have high quality motors for which a decent specification is available (e.g. Maxon),
you can simply copy-paste their parameters from the specification into the corresponding
configuration parameters, and skip the identification step described next.
Many cheaper motors, however, do not have an exhaustive specification, or their parameters may vary so widely
the specification will not serve much purpose anyway (e.g. T-Motor).
In this case please proceed to the Motor Identification section below.

### Performing Motor Identification

Make sure the board is powered from a stable power supply (e.g. lab power supply or a fully charged battery),
and that the supply voltage is about the same that will be used during normal operation.
Make extra sure the rotor is not connected to any mechanical load (e.g. propeller) and that it can rotate freely.
**Even a slightest mechanical load applied to the rotor will render the results of motor identification invalid.**
The motor itself must be mounted firmly.

Start the motor identification process using your communication interface as described above.
Make sure you're using identification in free rotation mode.
The LED will turn blue. The software will take a few minutes to examine the motor. Sit tight.
During the process the motor may twitch, squeak, vibrate, and slowly rotate.
**Make sure nothing unnecessary is in contact with the motor during identification.**

Once identification is finished, a report will be printed to the CLI
(if you don't have a CLI connection and can't see the report, that's fine),
and the LED indicator will turn from blue to either of the following colors:

* Green - identification was successful, or the software believes so.
The configuration parameters will be automatically updated with the identified values.
* Blinking red - identification was unsuccessful, or there was a hardware failure.
The configuration parameters will be left intact.
* Solid red - you're the chosen one.
Your mission is to submit a ticket with a detailed description of your setup and your actions prior to this moment.

You may want to write the identified parameters down somewhere, so you could easily reprogram them later, if needed,
not having to perform identification again.
If you have 10 minutes to spare, it is recommended to run identification 3 times in a row and make sure that all runs
return similar parameters.

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
