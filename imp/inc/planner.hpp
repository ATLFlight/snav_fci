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

#ifndef SNAV_FCI_PLANNER_HPP_
#define SNAV_FCI_PLANNER_HPP_

#include <cstdbool>

#include "api/config/planner_config.hpp"
#include "api/config/waypoint_config.hpp"
#include "api/state_vector.hpp"
#include "api/waypoint.hpp"
#include "imp/inc/waypoint_convergence.hpp"

namespace snav_fci
{

class Planner
{
public:
  // Overall planner status for mission execution
  enum class Status
  {
    IDLE,
    ENROUTE,
    COMPLETE
  };

  // Constructor
  Planner();
  Planner(PlannerConfig config);

  // Destructor
  ~Planner();

  void reset();

  // set config
  void set_config(PlannerConfig conf);
  // add a single waypoint
  void add_waypoint(Waypoint wp);
  // add a vector of waypoints
  void add_waypoints(std::vector<Waypoint> wpv);
  // get a pointer to the current active waypoint
  Waypoint* get_active_waypoint();
  // number of waypoints remaining (not completed)
  int waypoints_remaining();
  // current planner status
  Status get_status();

  // get desired state/action at time t, given current state
  int get_desired_state(float t, const StateVector& current_state,
      StateVector& desired_state);

private:
  // recalculate optimal trajectory
  void calculate_path(StateVector state, float t);
  // advance to next waypoint
  void advance_waypoint();
  // get time of previous waypoint
  float last_waypoint_time();

private:
  const float kEpsilon_ = 1e-6;
  const float kMinDt_ = 1e-3;
  const float kMaxDt_ = 1e-1;

  std::vector<Waypoint> waypoints_;
  PlannerConfig config_;
  Status status_;
  size_t active_waypoint_idx_;
  bool path_ready_;
  WaypointConvergence wp_convergence_;
  bool initialized_;
  float mission_time_last_secs_;
  StateVector state_command_world_;
  StateVector state_measured_last_;
  float yaw_desired_;
};

} // namespace snav_fci

#endif // SNAV_FCI_PLANNER_HPP_

