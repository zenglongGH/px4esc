PX4ESC Firmware
===============

## Development

Coding conventions used in this project are available at <https://github.com/Zubax/zubax_style_guide>.

### Static Analysis using Coverity Scan

Download the [Coverity Build Tool](https://scan.coverity.com/download?tab=cxx),
make sure its `bin` directory is listed in the PATH environment variable,
then build the firmware using the script `coverity_scan_build.sh`.
Once the build is finished, submit the resulting archive to Coverity Scan.

## Building

Development requires a machine with Linux or OSX; if you're using Windows, you're on your own.

Clone the repository and initialize all submodules, then run `make`:

```bash
git submodule update --init --recursive
cd firmware
make -j8 RELEASE=1  # Omit RELEASE=1 to build in debug mode with extra runtime checks
```

Debug mode will be selected by default.
Debug builds perform additional runtime state checks at the cost of somewhat lower performance.
Release builds can be selected with additional argument to make: `RELEASE=1`.

The results of the build can be found in the `build` directory, which will contain the following entities:

* Binary file `*.application.bin`.
This file contains the application binary itself with correct bootloader signature,
which allows to load it onto the board via the bootloader.
* Binary file `*.compound.bin`.
This file contains the application binary (above) prepended with the bootloader image.
This file can be used to flash an empty board for the first time.
* ELF file `compound.elf`.
This ELF contains the application, the bootloader, the correct bootloader signature,
and the application's debug information.
It can be loaded with a GDB debugger to perform symbol debugging using the script
`./zubax_chibios/tools/blackmagic_flash.sh`.

## MCU Usage

### Timers

The following list documents the current usage of hardware timers.

Timer   | Resolution| Usage
--------|-----------|--------------------------------------------------------------------------------------------------
TIM1    | 16        | Motor PWM
TIM2    | 32        | *Not used*
TIM3    | 16        | RGB LED PWM
TIM4    | 16        | *Not used*
TIM5    | 32        | ChibiOS system tick (see `STM32_ST_USE_TIMER`); must be 32-bit
TIM6    | 16        | *Not used*
TIM7    | 16        | Libuavcan STM32 driver
TIM8    | 16        | ADC triggering
TIM9    | 16        | RCPWM input and tachometer pulse input

## Third-party Dependencies

### PX4 UAVCAN Bootloader

Currently we're using the PX4 UAVCAN bootloader.
Its sources can be found here: <https://github.com/Zubax/PX4Firmware>, branch `px4esc_bootloader`.
This repository includes a prebuilt binary named `bootloader.bin`.
In the future we may migrate to our own implementation that will also support upgrading via USB.

### Eigen Library

We're using a clean fork of Eigen (without custom patches) as a submodule.
The fork is just a mirror of the upstream Mercurial repository.

### Zubax ChibiOS Extensions

The firmware is based on [ChibiOS/RT](http://chibios.org) RTOS with custom extensions developed by Zubax Robotics,
known as Zubax ChibiOS.
The extensions include things like configuration parameter management, command line interface,
implementation of algorithms such as Base64, etc.

In the future the project may migrate to the NuttX RTOS, provided that it adequately supports C++14 and
its kernel is preemptive.
At the moment, the sole motivation for abandonment of ChibiOS/RT is its
[somewhat restrictive licensing policies](http://www.chibios.org/dokuwiki/doku.php?id=chibios:licensing:start).
However, it should be kept in mind that if IP security is of a concern, the restrictive nature of GPL3 can be
circumvented by means of purchasing the unlimited commercial license of ChibiOS/RT,
which at the time of writing this cost about 8000 &euro;.
