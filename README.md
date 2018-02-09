# Snapdragon Navigator<sup>TM</sup> Flight Control Interface

## Table of Contents

1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Version Compatibility](#version-compatibility)
4. [Cloning and building the code](#cloning-and-building-the-code)
5. [Running the examples](#running-the-examples)
6. [Generating the documentation](#generating-the-documentation)
7. [FAQ](#faq)

## Introduction

Snapdragon Navigator<sup>TM</sup> is a flight controller that runs on
the Qualcomm Snapdragon Flight<sup>TM</sup> platform. Detailed information
about its capabilities can be found on the [Qualcomm Developer Network
site](https://developer.qualcomm.com/hardware/snapdragon-flight/sd-navigator).

[Snapdragon Navigator Flight Control Interface](https://github.com/ATLFlight/snav_fci)
is an example of an abstraction layer between the Snapdragon Navigator API and higher-level
programs. It is intended to isolate application developers from certain
low-level details of the Snapdragon Navigator API.

To get started with the documentation, see [snav_fci::FlightControlInterface](https://github.com/pages/ATLFlight/snav_fci/classsnav__fci_1_1_flight_control_interface.html).

Several examples are provided to demonstrate how to use FlightControlInterface:
  - [hello_snav.cpp](https://github.com/ATLFlight/snav_fci/blob/master/examples/hello_snav.cpp)
  - [basic_waypoint_example.cpp](https://github.com/ATLFlight/snav_fci/blob/master/examples/basic_waypoint_example.cpp)
  - [multithreaded_waypoint_example.cpp](https://github.com/ATLFlight/snav_fci/blob/master/examples/multithreaded_waypoint_example.cpp)
  - [orbit_example.cpp](https://github.com/ATLFlight/snav_fci/blob/master/examples/orbit_example.cpp)

## Prerequisites

### Hardware

This example requires the following hardware:

* [Qualcomm Snapdragon Flight
  Kit](https://shop.intrinsyc.com/collections/product-development-kits/products/qualcomm-snapdragon-flight-sbc)
* [Qualcomm Electronic Speed Control (ESC)
  board](https://shop.intrinsyc.com/collections/dragonboard-accessories/products/qualcomm-electronic-speed-control-board)
* Drone frame, motors, and propellers; such as the [Dragon Drone Development
  Kit](https://worldsway.com/product/dragon-drone-development-kit/)

Note that if you're using the Dragon DDK, a URDF ros package is available at:
[dragon_ddk_description](https://github.com/ATLFlight/dragon_ddk_description)

### Software

This example requires the following software:

* [Plaform Image from Intrynsic
  3.1.3.1](https://support.intrinsyc.com/attachments/download/1597/Flight_3.1.3.1_JFlash.zip)
* [Platform Addon from Intrynsic
  3.1.3.1](https://support.intrinsyc.com/attachments/download/1571/Flight_3.1.3.1_qcom_flight_controller_hexagon_sdk_add_on.zip)
* [Qualcomm Snapdragon Navigator Flight Controller SDK
  1.2.53.1](https://developer.qualcomm.com/download/snapdragon-flight/navigator-v1.2.53.1.deb)
* [Qualcomm Machine Vision SDK
  1.1.4](https://developer.qualcomm.com/download/machine-vision/machine-vision-sdk-v1.1.4.deb)
* [Qualcomm Snapdragon ESC Firmware
  1.2.0](https://developer.qualcomm.com/download/snapdragon-flight/navigator-controller-esc-firmware-v1.2.0)
* CMake: install on target with `sudo apt-get install cmake`
* Eigen: install on target with `sudo apt-get install libeigen3-dev`

## Version Compatibility

The table below summarizes what versions of Snapdragon Navigator are compatible
with a given release of Flight Control Interface.

| Flight Control Interface Version | Snapdragon Navigator Version |
| -------------------------------- | ---------------------------- |
| v1.0                             | >= v1.2.53.1                 |

## Cloning and building the code

These instructions assume that your Snapdragon Flight board is connected to the
internet, but, alternatively, you can clone this repo on an internet-connected
computer and then push the files to your Snapdragon Flight board. These
instructions also assume that the compilation is done on the Snapdragon Flight
board.

### Clone this repo

    adb shell
    cd /home/linaro
    git clone https://github.com/ATLFlight/snav_fci.git

### Build the code

    cd snav_fci
    mkdir build
    cd build
    cmake ..
    make -j4

A library called `libsnav_fci.so` is built and put in the `lib` directory.
Examples appear in the `bin` directory.

## Running the examples

### Running Snapdragon Navigator simulator

It is a good idea to test the examples in simulation before attempting any real
flights. In general, this is a powerful way to test API programs without
risking damage to any vehicles or the environment that could be caused in a
real flight test.  To do this, run `snav` in simulation mode:

    sudo stop snav
    sudo snav -w 1000

Although the motors do not spin in simulation mode, it is still prudent to
remove the propellers while doing bench-top testing to avoid accidents.

### Test communication with Snapdragon Navigator

From another shell, run the `hello_snav` example to test whether or not you
are able to communicate with Snapdragon Navigator.

    cd /home/linaro/snav_fci/build/bin
    ./hello_snav

You should see a message print out about successfully connecting to Snapdragon
Navigator, and, if you have ESCs and motors connected, you should observe
the status LEDs changing color and hear beeps indicating that Snapdragon
Navigator started receiving commands.

For example, here is the output from a successful run (output is identical
for both sim and non-sim):

    linaro@linaro-developer:~/snav_fci/build/bin$ ./hello_snav
    The apps <--> dsp offset = [1513353394762546617 ](ns) Apps ClockType[CLOCK_REALTIME]
    [INFO] Launched tx thread (id: 3020944384)
    [INFO] Launched rx thread (id: 3012555776)
    Successfully connected to Snapdragon Navigator
    [INFO] tx thread terminated normally
    [INFO] rx thread terminated normally

Running this example successfully confirms that the flight stack is running
normally and is accepting API commands. This is a good sanity test of your
installation of Snapdragon Navigator and Flight Control Interface.

### Test example programs in simulation with snav_inspector

With Snapdragon Navigator running in simulation mode, you can run any of the
examples and observe the live simulated results using `snav_inspector`, a tool
included in the Snapdragon Navigator package. For more information on how to
use this tool, please refer to the [Snapdragon Navigator User
Guide](https://developer.qualcomm.com/hardware/snapdragon-flight/sd-navigator/tools).

For example, run the `basic_waypoint_example` program and observe the `pos_vel`
group in `snav_inspector`. You should observe the `pos_vel.position_estimated`
field tracking the `pos_vel.position_desired` field as the simulated vehicle
executes the waypoint mission.

### Visualize simulated flight using ROS and rviz

You can also test the example programs by visualizing the simulated flight
in `rviz` using ROS. You can use `snav_ros` to accomplish this. Follow the
instructions in the [snav_ros](https://github.com/ATLFlight/snav_ros)
[README](https://github.com/ATLFlight/snav_ros/blob/master/README.md)
to get up and running.

### Real flight test

Once you are confident that the example programs are working properly in
simulation, you can perform a real flight test. Kill the `snav` simulator and
start `snav` normally with the `start snav` command.

Before running any of the programs, place the vehicle on flat ground that has
plenty of visual texture. Make sure there is plenty of open space in all
directions. If you have a Spektrum RC, be prepared to take over in the event
of an anomaly.

## Generating the documentation

Doxygen is used to generate documentation for this repo. You can generate it
yourself as follows:

    cd /home/linaro/snav_fci
    doxygen Doxyfile

Autogenerated HTML files are put in the `docs` directory and can be viewed
using a web browser. Simply open `index.html` in a browser to get started.

## FAQ

### I am getting an error message about not being able to initialize SnavCachedData

    Update checksum_failure. [f0eb75d8,a4d0a2d1] Possible RPC issue.
    [ERROR] int snav_fci::FlightControlInterface::initialize(snav_fci::FlightControlInterface::Permissions) -- could not init SnavCachedData

This error message usually indicates that Snapdragon Navigator is not actively
running.  Start Snapdragon Navigator using the `start snav` command for normal
operation or the `snav -w` command for simulation and then rerun the program.

