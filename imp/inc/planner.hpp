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

#include <atomic>
#include <cstdbool>
#include <map>
#include <mutex>
#include <thread>

#include "api/config/planner_config.hpp"
#include "api/config/waypoint_config.hpp"
#include "api/state_vector.hpp"
#include "api/waypoint.hpp"
#include "imp/inc/waypoint_convergence.hpp"
#include "imp/inc/snav_trajectory_thread_safe.hpp"

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

  Planner();

  Planner(PlannerConfig config);

  ~Planner();

  void reset();

  int set_waypoints(const std::vector<Waypoint>& wpv);

  int waypoints_remaining() const;

  Status get_status() const;

  std::vector<Waypoint> get_input_waypoints() const
  {
    std::lock_guard<std::mutex> lock(wp_mutex_);
    return waypoints_;
  }

  std::vector<Waypoint> get_optimized_waypoints() const
  {
    std::vector<Waypoint> output;
    std::vector<Waypoint> entrance = entrance_traj_.get_optimized_waypoints();
    std::vector<Waypoint> main = main_traj_.get_optimized_waypoints();
    output.insert(output.end(), entrance.begin(), entrance.end());
    output.insert(output.end(), main.begin(), main.end());
    return output;
  }

  int get_entrance_trajectory(snav_traj_gen::SnavTrajectory& traj) const
  {
    if (!entrance_traj_.optimized()) { return -1; }
    if (!entrance_traj_.satisfies_constraints()) { return -2; }
    traj = entrance_traj_.get_trajectory();
    return 0;
  }

  int get_trajectory(snav_traj_gen::SnavTrajectory& traj) const
  {
    if (!main_traj_.optimized()) { return -1; }
    if (!main_traj_.satisfies_constraints()) { return -2; }
    traj = main_traj_.get_trajectory();
    return 0;
  }

  int get_last_optimized_trajectory(snav_traj_gen::SnavTrajectory& traj) const
  {
    if (!main_traj_.optimized()) { return -1; }
    traj = main_traj_.get_trajectory();
    return 0;
  }

  int get_last_optimized_entrance_trajectory(snav_traj_gen::SnavTrajectory& traj) const
  {
    if (!entrance_traj_.optimized()) { return -1; }
    traj = entrance_traj_.get_trajectory();
    return 0;
  }

  // get desired state/action at time t, given current state
  int get_desired_state(float t, const StateVector& current_state,
      StateVector& desired_state);

  // recalculate optimal trajectory
  int calculate_path(StateVector state);

  static std::string get_status_string(Status status);

  void set_config(const PlannerConfig& config)
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = config;
  }

  PlannerConfig get_config() const
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
  }

private:

  // Disallow copy and assign
  Planner(const Planner&) = delete;
  Planner& operator=(const Planner&) = delete;

  void assign_waypoint_timestamps(std::vector<Waypoint>& wps);
  bool verify_timestamps_monotonically_increase(const std::vector<Waypoint>& wps) const;
  int safe_optimize(SnavTrajectoryThreadSafe& traj, bool loop);

  Waypoint* get_active_waypoint();
  void advance_waypoint();
  float last_waypoint_time();
  bool constraint_satisfied(float max_opt, float max_allowed, std::string field);

  const float kEpsilon_ = 1e-6;
  const float kMinDt_ = 1e-3;
  const float kMaxDt_ = 1e-1;

  std::vector<Waypoint> waypoints_;
  std::vector<Waypoint> entrance_waypoints_;
  mutable std::mutex wp_mutex_;

  PlannerConfig config_;
  mutable std::mutex config_mutex_;

  std::atomic<Status> status_;
  size_t active_waypoint_idx_;
  WaypointConvergence wp_convergence_;
  bool initialized_;
  float mission_time_last_secs_;
  float loop_time_start_;
  unsigned int num_loops_;
  float loop_duration_;
  StateVector state_command_world_;
  StateVector state_measured_last_;
  float yaw_desired_;
  SnavTrajectoryThreadSafe main_traj_;
  SnavTrajectoryThreadSafe entrance_traj_;
};

} // namespace snav_fci

#endif // SNAV_FCI_PLANNER_HPP_

