#
#   Copyright (c) 2018 John A. Dougherty. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name ATLFlight nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
#

cmake_minimum_required(VERSION 2.8.11)
project(snav_fci)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_FLAGS_DEBUG "-std=gnu++11 -Werror -Wall -pedantic ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "-std=gnu++11 -Werror -Wall -pedantic ${CMAKE_CXX_FLAGS_RELEASE}")

set(default_build_type "Release")
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "${default_build_type}")
endif()

message("build type: ${CMAKE_BUILD_TYPE}")

include_directories(${CMAKE_SOURCE_DIR}
  /usr/include/eigen3
)

add_library(snav_fci SHARED imp/src/flight_control_interface.cpp
  imp/src/planner.cpp
  imp/src/waypoint.cpp
)

target_link_libraries(snav_fci snav_arm snav_traj_gen)

add_executable(hello_snav examples/hello_snav.cpp)
target_link_libraries(hello_snav snav_fci)

add_executable(basic_waypoint_example examples/basic_waypoint_example.cpp)
target_link_libraries(basic_waypoint_example snav_fci)

add_executable(multithreaded_waypoint_example examples/multithreaded_waypoint_example.cpp)
target_link_libraries(multithreaded_waypoint_example snav_fci)

add_executable(orbit_example examples/orbit_example.cpp)
target_link_libraries(orbit_example snav_fci)

add_executable(nonblocking_trajectory_example examples/nonblocking_trajectory_example.cpp)
target_link_libraries(nonblocking_trajectory_example snav_fci)

