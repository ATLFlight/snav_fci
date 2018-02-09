/****************************************************************************
 * Copyright (c) 2018 John A. Dougherty. All rights reserved.
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
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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
 * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
 ****************************************************************************/

/** @file */

#include <iostream>

#include "api/flight_control_interface.hpp"

/**
 * This example demonstrates how to execute a simple waypoint-following
 * mission using FlightControlInterface.
 *
 * The mission looks like this:
 *   Takeoff -> Go To (2, 0, 2) m -> Go To (0, 0, 2) m -> Land
 */
int main(int argc, char* argv[])
{
  using FCI = snav_fci::FlightControlInterface;

  FCI fci;
  if (fci.initialize(FCI::Permissions::READ_WRITE) != FCI::Return::SUCCESS)
  {
    std::cout << "Error initializing FlightControlInterface for READ_WRITE" << std::endl;
    return -1;
  }

  /**
   * Make the mission relative to the vehicle's position and orientation when
   * the propellers start spinning. Without these lines, the mission is
   * relative to the vehicle's position and orientation when Snapdragon
   * Navigator first initializes.
   */
  snav_fci::TxConfig tx_config;
  tx_config.waypoint_frame_parent = snav_fci::ReferenceFrame::LAUNCH;
  fci.configure_tx(tx_config);

  if (fci.connect() != FCI::Return::SUCCESS) return -2;

  fci.takeoff();
  fci.go_to_waypoint(snav_fci::Waypoint(Eigen::Vector3f(2, 0, 2)));
  fci.go_to_waypoint(snav_fci::Waypoint(Eigen::Vector3f(0, 0, 2)));

  /**
   * Add a final waypoint at the same location as the last waypoint but with
   * zero yaw so that the vehicle returns to its initial orientation prior to
   * landing. snav_fci::WaypointConfig::YawType must be changed for this
   * waypoint since the default config points in the direction of travel and
   * would ignore any yaw setpoint given.
   */
  snav_fci::WaypointConfig wpconf;
  wpconf.yaw_type = snav_fci::WaypointConfig::YawType::WAYPOINT;
  fci.go_to_waypoint(snav_fci::Waypoint(snav_fci::StateVector(Eigen::Vector3f(0, 0, 2), 0), wpconf));

  fci.land();

  return 0;
}

