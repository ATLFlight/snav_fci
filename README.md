# Qualcomm Navigator<sup>TM</sup> Flight Control Interface

## Table of Contents

1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Version Compatibility](#version-compatibility)
4. [Changelog](CHANGELOG.md)
5. [Cloning and building the code](#cloning-and-building-the-code)
6. [Running the examples](#running-the-examples)
7. [Documentation](#documentation)
8. [FAQ](#faq)

## Introduction

Qualcomm Navigator<sup>TM</sup> is a flight controller that runs on
the Qualcomm Flight<sup>TM</sup> platform. Detailed information
about its capabilities can be found on the [Qualcomm Developer Network
site](https://developer.qualcomm.com/hardware/qualcomm-flight/qualcomm-navigator).

[Qualcomm Navigator Flight Control Interface](https://github.com/ATLFlight/snav_fci)
is an example of an abstraction layer between the Qualcomm Navigator API and higher-level
programs. It is intended to isolate application developers from certain
low-level details of the Qualcomm Navigator API.

To get started with the documentation, see [snav_fci::FlightControlInterface](https://atlflight.github.io/snav_fci/classsnav__fci_1_1_flight_control_interface.html).

Several examples are provided to demonstrate how to use FlightControlInterface:
  - [hello_snav.cpp](https://github.com/ATLFlight/snav_fci/blob/master/examples/hello_snav.cpp)
  - [basic_waypoint_example.cpp](https://github.com/ATLFlight/snav_fci/blob/master/examples/basic_waypoint_example.cpp)
  - [multithreaded_waypoint_example.cpp](https://github.com/ATLFlight/snav_fci/blob/master/examples/multithreaded_waypoint_example.cpp)
  - [orbit_example.cpp](https://github.com/ATLFlight/snav_fci/blob/master/examples/orbit_example.cpp)
  - [nonblocking_trajectory_example.cpp](https://github.com/ATLFlight/snav_fci/blob/master/examples/nonblocking_trajectory_example.cpp)

## Prerequisites

### Hardware

This example requires the following hardware:

* [Qualcomm Flight
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
* [Qualcomm Navigator Flight Controller SDK
  1.2.58](https://developer.qualcomm.com/download/qualcomm-flight/navigator-v1.2.58.deb)
* [Qualcomm Machine Vision SDK
  1.1.8](https://developer.qualcomm.com/download/machine-vision/machine-vision-sdk-v1.1.8.deb)
* [Qualcomm ESC Firmware
  1.2.0](https://developer.qualcomm.com/download/qualcomm-flight/navigator-controller-esc-firmware-v1.2.0.deb)
* CMake: install on target with `sudo apt-get install cmake`
* Eigen: install on target with `sudo apt-get install libeigen3-dev`

## Version Compatibility

The table below summarizes what versions of Qualcomm Navigator are compatible
with a given release of Flight Control Interface.

| Flight Control Interface Version | Qualcomm Navigator Version   |
| -------------------------------- | ---------------------------- |
| v2.0                             | >= v1.2.58                   |
| v1.0                             | >= v1.2.53.1                 |

## Cloning and building the code

These instructions assume that your Qualcomm Flight board is connected to the
internet, but, alternatively, you can clone this repo on an internet-connected
computer and then push the files to your Qualcomm Flight board. These
instructions also assume that the compilation is done on the Qualcomm Flight
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

### Running Qualcomm Navigator simulator

It is a good idea to test the examples in simulation before attempting any real
flights. In general, this is a powerful way to test API programs without
risking damage to any vehicles or the environment that could be caused in a
real flight test.  To do this, run `snav` in simulation mode:

    sudo stop snav
    sudo snav -w 1000

Although the motors do not spin in simulation mode, it is still prudent to
remove the propellers while doing bench-top testing to avoid accidents.

### Test communication with Qualcomm Navigator

From another shell, run the `hello_snav` example to test whether or not you
are able to communicate with Qualcomm Navigator.

    cd /home/linaro/snav_fci/build/bin
    ./hello_snav

You should see a message print out about successfully connecting to Qualcomm
Navigator, and, if you have ESCs and motors connected, you should observe
the status LEDs changing color and hear beeps indicating that Qualcomm
Navigator started receiving commands.

For example, here is the output from a successful run (output is identical
for both sim and non-sim):

    linaro@linaro-developer:~/snav_fci/build/bin$ ./hello_snav
    The apps <--> dsp offset = [1513353394762546617 ](ns) Apps ClockType[CLOCK_REALTIME]
    [INFO] Launched tx thread (id: 3020944384)
    [INFO] Launched rx thread (id: 3012555776)
    Successfully connected to Qualcomm Navigator
    [INFO] tx thread terminated normally
    [INFO] rx thread terminated normally

Running this example successfully confirms that the flight stack is running
normally and is accepting API commands. This is a good sanity test of your
installation of Qualcomm Navigator and Flight Control Interface.

### Test example programs in simulation with snav_inspector

With Qualcomm Navigator running in simulation mode, you can run any of the
examples and observe the live simulated results using `snav_inspector`, a tool
included in the Qualcomm Navigator package. For more information on how to
use this tool, please refer to the [Qualcomm Navigator User
Guide](https://developer.qualcomm.com/hardware/qualcomm-flight/qualcomm-navigator/tools).

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

## Documentation

### Access the docs

Doxygen documentation for the most recent release is available at
[https://atlflight.github.io/snav_fci/](https://atlflight.github.io/snav_fci/)

### Generating the docs

You can generate the documentation yourself as follows:

    cd /home/linaro/snav_fci
    doxygen Doxyfile

Autogenerated HTML files are put in the `docs` directory and can be viewed
using a web browser. Simply open `index.html` in a browser to get started.

## FAQ

### I am getting an error message about not being able to initialize SnavCachedData

    Update checksum_failure. [f0eb75d8,033e2d1a] Possible RPC issue.
    [ERROR] snav_fci::FlightControlInterface::FlightControlInterface(const snav_fci::FlightControlInterface::Permissions&)
    terminate called after throwing an instance of 'std::runtime_error'
      what():  could not init SnavCachedData
    Aborted

This error message usually indicates that Qualcomm Navigator is not actively
running.  Start Qualcomm Navigator using the `start snav` command for normal
operation or the `snav -w` command for simulation and then rerun the program.

### I, like Ricky Bobby, wanna go fast

Qualcomm Navigator has a number of parameters that directly and indirectly
affect the maximum attainable velocity and acceleration. These parameters are
set relatively conservatively by default. If you are interested in high-speed,
aggressive trajectory-following, you may wish to adjust these parameters in
order to reach higher velocities and accelerations.

`SN_VIO_POS_HOLD_MODE` is the only mode supported when using the
trajectory-tracking input, so only parameters relevant to that use case are
discussed here. However, similar parameters exist for other modes and can be
adjusted for faster flight when using the RC command input. Refer to the
Qualcomm Navigator User Guide for detailed documentation on parameters.

The `vio_mode_xy_gain` and the `height_control_z_vel_gain` parameters in the
`input_interpreter_params` group dictate the maximum attainable XY and Z
speeds, respectively. The XY and Z velocity feedforward factors used in control
can be set using the `velocity_ff_factor_vio` and `velocity_z_ff_factor_vio`
parameters, respectively, in the `position_control_params` group, where a value
of 0 eliminates the feedforward control and a value of 1 corresponds to full
feedforward control.

Acceleration is slightly more complicated since it is related to the tilt angle
and thrust-to-weight ratio of the vehicle. The maximum theoretical acceleration
in the horizontal plane (Z component = 0) can be approximated as `gravity *
tan(max_tilt_angle)` for idealized flight. The `max_vio_control_tilt_angle`
parameter in the `position_control_params` group dictates the maximum tilt
angle for VIO mode. The `max_acc_allowed` parameter in the
`velocity_smoother_params` group specifies the maximum allowed acceleration
to move the desired position in the XY plane.

Keep in mind that the vertical component of thrust is reduced when the tilt
angle is non-zero, meaning that at some angle the vehicle is not able to
maintain altitude due to its finite thrust. This critical angle may be
approximated as `acos(1/(T/W))`, where `T/W` is the thrust-to-weight ratio of
the vehicle. The thrust-to-weight ratio may be calculated by dividing the
`max_thrust` parameter of the `rc_params` group by the `basethrust` parameter
of the `position_control_params` group. Furthermore, keep in mind that in windy
conditions a non-zero tilt angle is required to maintain zero acceleration.

The maximum theoretical vertical acceleration (XY components = 0) can be
approximated as `((T/W)-1) * gravity`, where `T/W` is the thrust-to-weight
ratio of the vehicle. However, this assumes zero tilt angle, so the actual
achievable acceleration in Z is a function of the tilt angle at any given time.

Once the Qualcomm Navigator parameters have been updated, you should update
the `max_allowed*` values in snav_fci::PlannerConfig to reflect these changes.
Then, try increasing the `average_speed*` parameters in snav_fci::PlannerConfig
to generate more aggressive trajectories if using
TimestampStrategy::AVERAGE_SPEED; otherwise, manually adjust the timestamps
to obtain the desired trajectory.

