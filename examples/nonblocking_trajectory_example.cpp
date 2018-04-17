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

#include <csignal>
#include <iostream>

#include "api/flight_control_interface.hpp"

void signal_handler(int sig_num)
{
  std::cout << "Received SIGINT!" << std::endl;
  snav_fci::FlightControlInterface::preempt_current_action();
}

/**
 * @brief This is an example of a trajectory-following mission
 *
 * It uses nonblocking versions of action functions to simplify the flow
 * of the program to a single thread (multiple threads are managed internal
 * to FCI). It also demonstrates the use of the
 * FlightControlInterface::preempt_current_action() function by
 * handling the SIGINT signal.
 */
int main(int argc, char* argv[])
{
  signal(SIGINT, signal_handler);

  using FCI = snav_fci::FlightControlInterface;

  FCI fci(FCI::Permissions::READ_WRITE);

  // Configure FlightControlInterface to send commands at 200 Hz and relative
  // to the ReferenceFrame::LAUNCH frame.
  snav_fci::TxConfig tx_config;
  tx_config.tx_rate = 200;
  tx_config.waypoint_frame_parent = snav_fci::ReferenceFrame::LAUNCH;

  // For trajectory tracking, these options must be used
  tx_config.desired_mode = SN_VIO_POS_HOLD_MODE;
  tx_config.use_traj_tracking = true;

  fci.configure_tx(tx_config);

  fci.connect();

  // Use minimum snap optimization to generate the trajectory
  snav_fci::PlannerConfig planner_config;
  planner_config.traj_type = snav_fci::PlannerConfig::TrajType::MIN_SNAP;
  fci.configure_planner(planner_config);

  // Define the waypoints of the trajectory.
  // FlightControlInterface will automatically set the first and last
  // waypoints to be fully constrained (a requirement of generating the
  // trajectory). Other waypoints use the default constraints for a Waypoint
  // since they are not set here.
  //
  // The resulting trajectory resembles a figure-eight shape.
  std::vector<snav_fci::Waypoint> waypoints;
  const float kTrajStartHeight = 2.5;
  waypoints.push_back(snav_fci::Waypoint(Eigen::Vector3f(0,0,kTrajStartHeight)));
  waypoints.push_back(snav_fci::Waypoint(Eigen::Vector3f(1,1,kTrajStartHeight+0.5)));
  waypoints.push_back(snav_fci::Waypoint(Eigen::Vector3f(2,0,kTrajStartHeight+1.0)));
  waypoints.push_back(snav_fci::Waypoint(Eigen::Vector3f(1,-1,kTrajStartHeight+0.5)));
  waypoints.push_back(snav_fci::Waypoint(Eigen::Vector3f(0,0,kTrajStartHeight)));
  waypoints.push_back(snav_fci::Waypoint(Eigen::Vector3f(-1,1,kTrajStartHeight-0.5)));
  waypoints.push_back(snav_fci::Waypoint(Eigen::Vector3f(-2,0,kTrajStartHeight-1.0)));
  waypoints.push_back(snav_fci::Waypoint(Eigen::Vector3f(-1,-1,kTrajStartHeight-0.5)));
  waypoints.push_back(snav_fci::Waypoint(Eigen::Vector3f(0,0,kTrajStartHeight)));

  fci.preload_waypoints(waypoints);

  // Takeoff phase
  fci.takeoff_nb();
  std::cout << "Taking off..." << std::endl;

  // While the maneuver is being executed, get some telemetry and print it to
  // screen
  SnavCachedData snav_data;
  std::memset(&snav_data, 0, sizeof(SnavCachedData));
  while (FCI::ok() && fci.get_current_action() == FCI::Action::TAKEOFF)
  {
    snav_data = fci.get_snav_cached_data();
    snav_fci::StateVector state_est = fci.get_estimated_state(snav_data);
    std::cout << "altitude = " << state_est.position[2] << " m " << std::endl;
    usleep(1e5);
  }

  fci.wait_on_action();

  if (fci.get_last_action_result() != FCI::Return::SUCCESS)
  {
    std::cout << "Takeoff did not return success; exiting" << std::endl;
    return -1;
  }
  std::cout << "Done!" << std::endl;

  // Trajectory phase
  // This is an example of defining an absolute start time for the mission.
  // In this example, the absolute start time is simply defined as the current
  // wall clock time plus 3 seconds. For a multi-robot system, one might
  // imagine determing the absolute start time on a host machine before sending
  // to all of the robots for a synchronized start.
  // If you don't care about an absolute start time and want the mission to
  // start immediately, simply omit the start time argument or set it equal to
  // zero.
  std::chrono::time_point<std::chrono::system_clock> t_now_tp = std::chrono::system_clock::now();
  std::chrono::time_point<std::chrono::system_clock> t_start_tp = t_now_tp
    + std::chrono::seconds(3);
  double t_start = (std::chrono::duration_cast<std::chrono::nanoseconds>
    (t_start_tp.time_since_epoch()).count()) / 1e9;
  fci.execute_mission_nb(t_start);
  std::cout << "Executing mission at " << t_start << "..." << std::endl;
  while (FCI::ok() && fci.get_current_action() == FCI::Action::EXECUTE_MISSION)
  {
    float time = fci.get_trajectory_time();
    std::vector<snav_fci::Waypoint> waypoints;
    fci.get_waypoints(waypoints);
    snav_fci::Planner::Status status = fci.get_planner_status();
    if (waypoints.size() > 0)
    {
      std::cout << "t = " << time << " s, "
        << "planner status = " << snav_fci::Planner::get_status_string(status)
        << std::endl;
      std::cout << "waypoints:" << std::endl;
      for (auto&& itr = waypoints.begin(); itr != waypoints.end(); ++itr)
      {
        std::string marker("    ");
        if (itr->status == snav_fci::Waypoint::Status::ENROUTE)
        {
          marker = "--> ";
        }
        std::cout << marker << itr - waypoints.begin() << ". " << *itr << std::endl;
      }
      std::cout << std::endl;
    }
    usleep(1e5);
  }

  fci.wait_on_action();

  if (fci.get_last_action_result() != FCI::Return::SUCCESS)
  {
    std::cout << "execute mission did not return success; exiting" << std::endl;
    return -1;
  }
  std::cout << "Done!" << std::endl;

  // Yaw to face starting orientation
  snav_fci::WaypointConfig wpconf;
  wpconf.yaw_type = snav_fci::WaypointConfig::YawType::WAYPOINT;
  std::cout << "Yawing to face start..." << std::endl;
  fci.go_to_waypoint_nb(snav_fci::Waypoint(snav_fci::StateVector(Eigen::Vector3f(0, 0, kTrajStartHeight), 0), wpconf));

  //
  // Do stuff here. When you're done, call wait_on_action() before continuing
  // to verify that the action has completed.
  //

  fci.wait_on_action();

  if (fci.get_last_action_result() != FCI::Return::SUCCESS)
  {
    std::cout << "Go to waypoint did not return success; exiting" << std::endl;
    return -1;
  }
  std::cout << "Done!" << std::endl;

  // Land phase
  fci.land_nb();
  std::cout << "Landing..." << std::endl;

  //
  // Do stuff here. When you're done, call wait_on_action() before continuing
  // to verify that the action has completed.
  //

  fci.wait_on_action();

  if (fci.get_last_action_result() != FCI::Return::SUCCESS)
  {
    std::cout << "Land did not return success; exiting" << std::endl;
    return -1;
  }
  std::cout << "Done!" << std::endl;

  return 0;
}


