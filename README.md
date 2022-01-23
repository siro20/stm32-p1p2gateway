# P1/P2 decoding on STM32L031

This is an example application for the STM32L031 ARM microcontrollers
that compiles with GNU tools.

It decodes the P1/P2 protocol used by Daikin heat pumps received on PB0
and forwards it to the VUART.

It serves as a quick-start for those who do not wish to use an IDE, but rather
develop in a text editor of choice and build from the command line.

It makes use of the nano newlib as the Cortex-M0 only has 32k of flash.

## Target Overview

  - `all`       Builds the target ELF binary.
  - `program`   Flashes the ELF binary to the target board.
  - `debug`     Launches GDB and connects to the target.
  - `clean`     Remove all files and directories which have been created during the compilation.

## Installing

Checkout the submodules:

    git submodule update --init --checkout

Before building, you must install the GNU compiler toolchain.
I'm using the the `gnu-none-eabi` triple shipped with recent Debian and Ubuntu versions:

    sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi

You also might want to install some other libraries and debuggers:

    sudo apt-get install openocd gdb-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib

## Source code

Your source code has to be put in the `src` directory.
Dont forget to add your source files in the Makefile.

## Acknowledgement

Based on [stv0g/stm32cube-gcc](https://github.com/stv0g/stm32cube-gcc)'s STM32Cube Makefile project.
Many thanks to Steffen Vogel for this great Makefile project.
