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

#include <array>
#include <csignal>
#include <iostream>
#include <unistd.h>

#include "api/flight_control_interface.hpp"

// Global variable triggered by sigint to stop orbiting
bool gKeepGoing = true;

void signal_handler(int sig_num)
{
  std::cout << "Received SIGINT!" << std::endl;
  gKeepGoing = false;
}

/**
 * This is an example of circling a point of interest at a constant radius
 * and speed while yawing toward the center of the circle (always facing the
 * point of interest). In this example, the point of interest is the takeoff
 * location.
 *
 * This example demonstrates how to use some high-level aspects of
 * snav_fci::FlightControlInterface together with some lower-level
 * functionality (computing rate commands).
 */
int main(int argc, char* argv[])
{
  using FCI = snav_fci::FlightControlInterface;

  FCI fci(FCI::Permissions::READ_WRITE);

  // Configure FlightControlInterface to send commands at 200 Hz relative
  // to the LAUNCH frame
  snav_fci::TxConfig tx_config;
  tx_config.tx_rate = 200;
  tx_config.waypoint_frame_parent = snav_fci::ReferenceFrame::LAUNCH;
  fci.configure_tx(tx_config);

  fci.connect();

  /**
   * Parameters used to define the orbit maneuver:
   *   - kOrbitRadius: distance in XY plane from the point of interest to
   *     maintain
   *   - kOrbitHeight: constant height used for the manuever
   *   - kOrbitSpeed: lateral (XY plane) speed used for the maneuver
   *   - kOrbitMaxLinAccel: constant acceleration used for ramping up/down
   *     speed
   */
  const float kOrbitRadius = 1.25; // m
  const float kOrbitHeight = 2.0; // m
  const float kOrbitSpeed = 1.0; // m/s
  const float kOrbitMaxLinAccel = 0.25; // m/s/s

  /**
   * Start the maneuver! Phases of the mission are:
   *   1. Ascend to kOrbitHeight
   *   2. Translate in XY plane to (kOrbitRadius, 0, kOrbitHeight)
   *   3. Circle the takeoff location (0, 0), maintaining a constant distance
   *      of kOrbitRadius and a constant height of kOrbitHeight
   *   4. Upon receiving SIGINT, translate in XY plane to the takeoff location
   *      (0, 0, kOrbitHeight)
   *   5. Land
   *
   * Phases 1, 2, 4, and 5 use high-level functions of FlightControlInterface.
   * Phase 3 directly computes velocity and yaw rate commands and sends the
   * result to FlightControlInterface to be sent to Qualcomm Navigator.
   */

  // Phase 1
  fci.takeoff(snav_fci::TakeoffConfig(kOrbitHeight));

  // Phase 2
  fci.go_to_waypoint(snav_fci::Waypoint(
        Eigen::Vector3f(kOrbitRadius, 0, kOrbitHeight)));

  // Phase 3
  std::cout << "Beginning orbit...[Ctrl-C to end]" << std::endl;
  signal(SIGINT, signal_handler);
  while (FCI::ok())
  {
    SnavCachedData snav_data = fci.get_snav_cached_data();

    // t_ld_xy is a 2D vector from LAUNCH frame to DESIRED frame in XY
    std::array<float, 2> t_ld_xy =
    {
      snav_data.pos_vel.position_desired[0] - snav_data.pos_vel.t_el[0],
      snav_data.pos_vel.position_desired[1] - snav_data.pos_vel.t_el[1]
    };

    float t_ld_xy_norm = sqrtf(t_ld_xy[0]*t_ld_xy[0] + t_ld_xy[1]*t_ld_xy[1]);

    // Let e3 = [0; 0; 1] (up)
    // Then e3 cross [t_ld_xy; 0] gives direction of velocity vector in XY plane
    std::array<float, 3> vel_direction = {-t_ld_xy[1], t_ld_xy[0], 0};
    float norm = sqrtf(vel_direction[0]*vel_direction[0]+vel_direction[1]*vel_direction[1]);
    vel_direction[0] = vel_direction[0] / norm;
    vel_direction[1] = vel_direction[1] / norm;

    // Due to estimation and timing errors, a correction needs to be applied
    // to the analytic idealized velocity direction to prevent the vehicle from
    // either spiraling inward or outward. A correction angle is defined to
    // rotate the idealized velocity direction slightly inward or outward to
    // maintain a constant radius.
    //
    // 0.5 rad/m (gain) * 1 m (error) = 0.5 rad or ~29 degree correction angle
    const float kCorrectionAngleGain = 0.5; // rad/m
    float correction_angle = kCorrectionAngleGain * (t_ld_xy_norm - kOrbitRadius);

    // Rotate the idealized velocity direction about Z axis by the correction angle
    vel_direction[0] = cos(correction_angle)*vel_direction[0]
      - sin(correction_angle)*vel_direction[1];
    vel_direction[1] = sin(correction_angle)*vel_direction[0]
      + cos(correction_angle)*vel_direction[1];

    static float xy_vel_norm = 0;
    const float kDt = 1.0 / static_cast<float>(tx_config.tx_rate);
    if (gKeepGoing)
    {
      // Ramp up to kOrbitSpeed using kOrbitMaxLinAccel
      if (xy_vel_norm < kOrbitSpeed)
      {
        xy_vel_norm += kOrbitMaxLinAccel * kDt;
      }
    }
    else
    {
      // Ramp down to 0 using kOrbitMaxLinAccel
      if (xy_vel_norm > 0)
      {
        xy_vel_norm -= kOrbitMaxLinAccel * kDt;
      }
    }

    // Cap xy_vel_norm
    if (xy_vel_norm > kOrbitSpeed)
    {
      xy_vel_norm = kOrbitSpeed;
    }
    else if (xy_vel_norm < 0)
    {
      xy_vel_norm = 0;
      // Once we hit 0, time to exit this phase
      break;
    }

    // Assemble velocity vector
    std::array<float, 3> velocity = {0, 0, 0};
    velocity[0] = xy_vel_norm * vel_direction[0];
    velocity[1] = xy_vel_norm * vel_direction[1];

    // Yaw toward center of circle
    float yaw_rate = 0;
    float desired_yaw_angle = atan2(-t_ld_xy[1], -t_ld_xy[0]);
    float yaw_error = desired_yaw_angle - snav_data.pos_vel.yaw_estimated;
    if (yaw_error > M_PI) yaw_error -= 2*M_PI;
    else if (yaw_error < -M_PI) yaw_error += 2*M_PI;

    // Choice of yaw gain here is fairly arbitrary, but 3 seems to work well...
    const float kYawPGain = 3; // 1/s
    yaw_rate = kYawPGain*yaw_error;

    // Convert velocity and yaw_rate commands into RcCommand
    snav_fci::RcCommand rc_command;
    fci.convert_velocity_to_rc_command(velocity,
        yaw_rate, rc_command);

    // Update command being sent by FlightControlInterface
    fci.set_tx_command(rc_command);

    const unsigned int kSleepTimeUs = static_cast<unsigned int>(kDt * 1e6);
    usleep(kSleepTimeUs);
  }

  if (FCI::ok())
  {
    std::cout << "Orbit complete! Returning to launch position for landing." << std::endl;

    // Phase 4
    fci.go_to_waypoint(snav_fci::Waypoint(Eigen::Vector3f(0, 0, kOrbitHeight)));
    snav_fci::WaypointConfig wpconf;
    wpconf.yaw_type = snav_fci::WaypointConfig::YawType::WAYPOINT;
    fci.go_to_waypoint(snav_fci::Waypoint(snav_fci::StateVector(
            Eigen::Vector3f(0, 0, kOrbitHeight), 0), wpconf));

    // Phase 5
    fci.land();
  }

  return 0;
}

