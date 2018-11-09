# TizenRT

[![License](https://img.shields.io/badge/licence-Apache%202.0-brightgreen.svg?style=flat)](LICENSE)
[![Build Status](https://travis-ci.org/Samsung/TizenRT.svg?branch=master)](https://travis-ci.org/Samsung/TizenRT)

TizenRT is lightweight RTOS-based platform to support low-end IoT devices.  
Please find project details at [Wiki](https://github.com/Samsung/TizenRT/wiki) especially **[documentations page](https://github.com/Samsung/TizenRT/wiki/Documentations)**.


## 1. Getting the toolchain

### For ARM architecture:

Install the OS specific toolchain. Supported OS Type's are "linux" and "mac".  
Get the build in binaries and libraries, [gcc-arm-none-eabi-6-2017-q1-update-*OS Type*.tar.bz2](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads/6-2017-q1-update)  
Untar the gcc-arm-none-eabi-6-2017-q1-update-*OS Type*.tar.bz2 and export the path like

```bash
tar xvjf gcc-arm-none-eabi-6-2017-q1-update-[OS Type].tar.bz2
export PATH=<Your Toolchain PATH>:$PATH
```
Be aware that recommanded toolchain is fully working on 64bits machine.

### For Xtensa architecture, esp32 board:

Get the build in binaries and libraries

64-bit Linux：

[xtensa-esp32-elf-linux64-1.22.0-80-g6c4433a-5.2.0.tar.gz](https://dl.espressif.com/dl/xtensa-esp32-elf-linux64-1.22.0-80-g6c4433a-5.2.0.tar.gz)

32-bit Linux：

[xtensa-esp32-elf-linux32-1.22.0-80-g6c4433a-5.2.0.tar.gz](https://dl.espressif.com/dl/xtensa-esp32-elf-linux32-1.22.0-80-g6c4433a-5.2.0.tar.gz)

```bash
tar xvjf xtensa-esp32-elf-[linux*]-1.22.0-80-g6c4433a-5.2.0.tar.gz
export PATH=<Your Toolchain PATH>:$PATH
```

## 2. Getting TizenRT source code

```bash
git clone https://github.com/Samsung/TizenRT.git
cd TizenRT
TIZENRT_BASEDIR="$PWD"
```
**Note**: To contribute in this community, you need to clone your forked private repository instead.  
          Github guides this by [working-with-forks](https://help.github.com/articles/working-with-forks).

## 3. How to Build

Configure the build from $TIZENRT_BASEDIR/os/tools directory

```bash
cd os
./tools/configure.sh <board>/<configuration_set>
```

This command retrieves the specific pre-configured file named defconfig and Make.defs according to \<board\>/\<configuration_set\>.  
You can see collection of all configuration files at *$TIZENRT_BASEDIR/build/configs*.  
To check all pre-defined configurations, type as follows:

```bash
./configure.sh --help
```

After configuring above, configuration can be modified through *make menuconfig* from *$TIZENRT_BASEDIR/os*.
Howerver, menuconfig isn't ready for Xtensa architecture esp32 board. Please do not use it for esp32 board.

```bash
cd ..
make menuconfig
```

Refer [kconfig-frontend installation](docs/HowtoInstallKconfigFrontend.md) to use *menuconfig*.

Export your toolchain:

```bash
cd os
source setenv.sh <Your Toolchain PATH>
```
Or, 

```bash
export PATH=<Your Toolchain PATH>:$PATH
```

Finally, initiate build by make from *$TIZENRT_BASEDIR/os*.
```bash
make
```

## 4. How to burn images

After compiling,  "tinyara.bin" are built in the $TIZENRT_BASEDIR/build/ directory.
Burn the image by performing the following commands in $TIZENRT_BASEDIR/os/:

Burn ALL the images:

 ```bash
cd $TIZENRT_BASEDIR/os
make download ALL
```

Burn the bootloader:

 ```bash
cd $TIZENRT_BASEDIR/os
make download BOOTLOADER
```

Burn the TizenRT image:

 ```bash
cd $TIZENRT_BASEDIR/os
make download APP
```

