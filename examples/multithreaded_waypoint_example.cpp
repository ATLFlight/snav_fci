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

#include <atomic>
#include <iomanip>
#include <iostream>
#include <unistd.h>
#include <thread>
#include <vector>

#include "api/flight_control_interface.hpp"

std::atomic_bool gMissionInProgress(true);

/**
 * @brief Task to command the vehicle to execute a waypoint mission
 *
 * This thread blocks while the vehicle executes the commanded maneuvers.
 */
void mission_task()
{
  using FCI = snav_fci::FlightControlInterface;

  FCI fci;
  if (fci.initialize(FCI::Permissions::READ_WRITE) != FCI::Return::SUCCESS)
  {
    std::cout << "Error initializing FlightControlInterface for READ_WRITE" << std::endl;
    gMissionInProgress = false;
    return;
  }

  /**
   * Configure snav_fci::FlightControlInterface in this thread since this
   * object has snav_fci::FlightControlInterface::Permissions::READ_WRITE and
   * will launch both the tx and rx threads if not already launched. Other
   * threads will wait for this to happen before calling
   * snav_fci::FlightControlInterface::connect() using the
   * snav_fci::FlightControlInterface::wait_for_configure() function.
   */
  snav_fci::TxConfig tx_config;
  tx_config.tx_rate = 100; // Hz
  if (fci.configure_tx(tx_config) != FCI::Return::SUCCESS)
  {
    std::cout << "Error configuring tx" << std::endl;
    gMissionInProgress = false;
    return;
  }

  snav_fci::RxConfig rx_config;
  rx_config.rx_rate = 250; // Hz
  if (fci.configure_rx(rx_config) != FCI::Return::SUCCESS)
  {
    std::cout << "Error configuring rx" << std::endl;
    return;
  }

  if (fci.connect() != FCI::Return::SUCCESS) return;

  /**
   * Take advantage of snav_fci::TakeoffConfig to configure the takeoff.
   * Set take off height to 2 m, min takeoff speed to 0.05 m/s, max takeoff
   * speed to 1 m/s, and max linear acceleration magnitude to 0.25 m/s^2.
   */
  if (fci.takeoff(snav_fci::TakeoffConfig(2, 0.05, 1, 0.25)) != FCI::Return::SUCCESS)
  {
    std::cout << "Error in takeoff!" << std::endl;
    gMissionInProgress = false;
    return;
  }

  // Configure waypoints
  snav_fci::WaypointConfig wpconf;
  wpconf.max_linear_velocity_norm = 5; // m/s
  wpconf.max_linear_acceleration_norm = 1; // m/s/s
  wpconf.max_yaw_velocity_norm = M_PI; // rad/s
  wpconf.max_yaw_acceleration_norm = M_PI/2; // rad/s/s

  /**
   * Define the waypoints for this mission, keeping in mind that the vehicle
   * is at (0, 0, 2) m after takeoff phase ends:
   *
   *   1. Go to (4, 0, 2) m
   *   2. Go to (4, 4, 2) m
   *   3. Go to (0, 4, 2) m
   *   4. Go to (0, 0, 2) m (back to start)
   *
   * The result is a square in XY plane with constant Z.
   */
  snav_fci::Waypoint wp1(Eigen::Vector3f(4,0,2), wpconf);
  snav_fci::Waypoint wp2(Eigen::Vector3f(4,4,2), wpconf);
  snav_fci::Waypoint wp3(Eigen::Vector3f(0,4,2), wpconf);
  snav_fci::Waypoint wp4(Eigen::Vector3f(0,0,2), wpconf);

  // Make the vehicle yaw to face its starting orientation before landing
  wpconf.yaw_type = snav_fci::WaypointConfig::YawType::WAYPOINT;
  snav_fci::Waypoint wp5(snav_fci::StateVector(Eigen::Vector3f(0, 0, 2), 0), wpconf);

  std::vector<snav_fci::Waypoint> waypoints = {wp1, wp2, wp3, wp4, wp5};

  int wp_cntr = 1;
  for (auto itr = waypoints.begin(); itr != waypoints.end(); ++itr)
  {
    if (fci.go_to_waypoint(*itr) != FCI::Return::SUCCESS)
    {
      std::cout << "Error encountered during waypoint #" << wp_cntr << std::endl;
      gMissionInProgress = false;
      return;
    }
    else { ++wp_cntr; }
  }

  fci.land(snav_fci::LandingConfig(0.3));

  gMissionInProgress = false;
}

/**
 * @brief Task to receive and display telemetry data
 *
 * This thread loops at a slow rate and prints useful telemetry data without
 * interfering with the more critical `mission_task` thread.
 */
void telemetry_task()
{
  using FCI = snav_fci::FlightControlInterface;

  FCI fci;
  if (fci.initialize(FCI::Permissions::READ_ONLY) != FCI::Return::SUCCESS)
  {
    std::cout << "Error initializing FlightControlInterface for READ_ONLY" << std::endl;
    return;
  }

  // Wait until FlightControlInterface has been configured before calling
  // connect() to make sure that the correct configuration is used when
  // spawning the rx thread if it hasn't already been spawned
  fci.wait_for_configure();
  if (fci.connect() != FCI::Return::SUCCESS) return;

  SnavCachedData snav_data;
  std::memset(&snav_data, 0, sizeof(SnavCachedData));
  fci.get_snav_cached_data(snav_data);
  int64_t t0 = snav_data.pos_vel.time;
  while (snav_fci::FlightControlInterface::rx_ok() && gMissionInProgress)
  {
    if (fci.get_snav_cached_data(snav_data) != FCI::Return::SUCCESS) return;

    // Get estimated and desired state w.r.t. ReferenceFrame::WAYPOINT
    snav_fci::StateVector estimated_state, desired_state;
    fci.get_estimated_state(snav_data, estimated_state);
    fci.get_desired_state(snav_data, desired_state);

    std::cout << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "    ----snav-telemetry------------" << std::endl;
    std::cout << std::setw(25) << "time"
      << std::setw(20) << (snav_data.pos_vel.time - t0) / 1e6 << std::setw(5) << " s"
      <<std::endl;
    std::cout << std::setw(25) << "x y z yaw des"
      << std::setw(10) << desired_state.position[0] << std::setw(5) << " m"
      << std::setw(10) << desired_state.position[1] << std::setw(5) << " m"
      << std::setw(10) << desired_state.position[2] << std::setw(5) << " m"
      << std::setw(10) << desired_state.yaw << std::setw(5) << " rad"
      << std::endl;
    std::cout << std::setw(25) << "x y z yaw est"
      << std::setw(10) << estimated_state.position[0] << std::setw(5) << " m"
      << std::setw(10) << estimated_state.position[1] << std::setw(5) << " m"
      << std::setw(10) << estimated_state.position[2] << std::setw(5) << " m"
      << std::setw(10) << estimated_state.yaw << std::setw(5) << " rad"
      << std::endl;
    std::cout << std::setw(25) << "vx vy vz des"
      << std::setw(10) << desired_state.velocity[0] << std::setw(5) << " m/s"
      << std::setw(10) << desired_state.velocity[1] << std::setw(5) << " m/s"
      << std::setw(10) << desired_state.velocity[2] << std::setw(5) << " m/s"
      << std::endl;
    std::cout << std::setw(25) << "vx vy vz est"
      << std::setw(10) << estimated_state.velocity[0] << std::setw(5) << " m/s"
      << std::setw(10) << estimated_state.velocity[1] << std::setw(5) << " m/s"
      << std::setw(10) << estimated_state.velocity[2] << std::setw(5) << " m/s"
      << std::endl;
    std::cout << "    ------------------------------" << std::endl;
    std::cout << std::endl;
    usleep(1e5); // loop at about 10 Hz
  }
}

/**
 * @brief Task to update the transform between
 * snav_fci::ReferenceFrame::WAYPOINT and its parent
 *
 * In this example, the transform is updated using the transform to
 * snav_fci::ReferenceFrame::LAUNCH, which makes the mission relative to the
 * takeoff position and orientation. This same behavior can be achieved simply
 * by setting snav_fci::TxConfig::waypoint_frame_parent to
 * snav_fci::ReferenceFrame::LAUNCH, but this example is intended to
 * demonstrate how to use the
 * snav_fci::FlightControlInterface::set_waypoint_frame_tf() function. A
 * typical use of this function would be to update
 * snav_fci::ReferenceFrame::WAYPOINT using some form of global localization,
 * such as AprilTags, in order to navigate in a global frame.
 */
void frame_update_task()
{
  using FCI = snav_fci::FlightControlInterface;

  FCI fci;
  if (fci.initialize(FCI::Permissions::READ_ONLY) != FCI::Return::SUCCESS)
  {
    std::cout << "Error initializing FlightControlInterface for READ_ONLY" << std::endl;
    return;
  }

  // Wait until FlightControlInterface has been configured before calling
  // connect() to make sure that the correct configuration is used when
  // spawning the rx thread if it hasn't already been spawned
  fci.wait_for_configure();
  if (fci.connect() != FCI::Return::SUCCESS) return;

  SnavCachedData snav_data;
  std::memset(&snav_data, 0, sizeof(SnavCachedData));
  while (snav_fci::FlightControlInterface::rx_ok() && gMissionInProgress)
  {
    if (fci.get_snav_cached_data(snav_data) != FCI::Return::SUCCESS) return;
    if (snav_data.pos_vel.launch_tf_is_valid)
    {
      Eigen::Quaternionf q_el(Eigen::AngleAxisf(snav_data.pos_vel.yaw_el,
            Eigen::Vector3f::UnitZ()));
      Eigen::Vector3f t_el(snav_data.pos_vel.t_el[0],
          snav_data.pos_vel.t_el[1],
          snav_data.pos_vel.t_el[2]);
      fci.set_waypoint_frame_tf(q_el, t_el);
    }
    usleep(1e4); // loop at about 100 Hz
  }
}

/**
 * @brief This is an example of a multithreaded waypoint-following mission
 *
 * It also does a better job of checking return codes provided by
 * snav_fci::FlightControlInterface than basic_waypoint_example.cpp.
 *
 * This example uses three threads:
 * 1. `mission_thread`: commands the vehicle to go to waypoints
 * 2. `frame_update_thread`: updates the transform between
 *    ReferenceFrame::WAYPOINT and its parent such that the mission is relative
 *    to the launch position and orientation; note that this is just intended
 *    to serve as an example of using the
 *    snav_fci::FlightControlInterface::set_waypoint_frame_tf() function and
 *    would normally by achieved by simply using
 *    snav_fci::TxConfig::waypoint_frame_parent
 * 3. `telemetry_thread`: retrieves data from Snapdragon Navigator and prints
 *    it to `stdout`
 *
 * A similar multithreaded architecture may be suitable for a host of other use
 * cases and is more easily achieved using snav_fci::FlightControlInterface
 * compared to using the Snapdragon Navigator API directly.
 */
int main(int argc, char* argv[])
{
  // Launch the threads
  std::cout << "Launching mission task thread" << std::endl;
  std::thread mission_thread(mission_task);
  std::cout << "Launching frame update task thread" << std::endl;
  std::thread frame_update_thread(frame_update_task);
  std::cout << "Launching telemetry task thread" << std::endl;
  std::thread telemetry_thread(telemetry_task);

  // Wait for the threads to complete
  mission_thread.join();
  std::cout << "Mission thread returned" << std::endl;
  frame_update_thread.join();
  std::cout << "Frame update thread returned" << std::endl;
  telemetry_thread.join();
  std::cout << "Telemetry thread returned" << std::endl;

  return 0;
}

