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

#include "imp/inc/planner.hpp"

#include <algorithm>
#include <iostream>

namespace snav_fci
{

float wrap_yaw(float yaw)
{
  float yaw_wrapped = 0;
  float unwrapped_yaw_plus_pi = yaw + M_PI;
  if(unwrapped_yaw_plus_pi < 0)
  {
    yaw_wrapped = (2*M_PI-fmod(-unwrapped_yaw_plus_pi, (2*M_PI))) -M_PI;
  }
  else
  {
    yaw_wrapped = fmod(unwrapped_yaw_plus_pi, (2*M_PI)) -M_PI;
  }
  return yaw_wrapped;
}

Planner::Planner()
{
  reset();
}

Planner::Planner(PlannerConfig conf):Planner()
{
  set_config(conf);
}

Planner::~Planner() {}

void Planner::reset()
{
  std::lock_guard<std::mutex> lock(wp_mutex_);
  status_ = Status::IDLE;
  active_waypoint_idx_ = 0;
  waypoints_.clear();
  entrance_waypoints_.clear();
  main_traj_.reset();
  entrance_traj_.reset();
  initialized_ = false;
  mission_time_last_secs_ = 0;
  loop_time_start_ = 0;
  num_loops_ = 0;
  loop_duration_ = 0;
  yaw_desired_ = 0;
}

Planner::Status Planner::get_status() const
{
  return status_.load();
}

int Planner::set_waypoints(const std::vector<Waypoint>& wpv)
{
  // Right now, waypoints can only be set prior to the mission start
  if (status_ != Status::IDLE) return -1;

  std::lock_guard<std::mutex> lock(wp_mutex_);

  waypoints_ = wpv;

  if (waypoints_.size() <= 0)
  {
    main_traj_.reset();
    entrance_traj_.reset();
    return 0;
  }

  // Add placeholder to be later filled with current state
  entrance_waypoints_.clear();
  entrance_waypoints_.push_back(Waypoint());
  // Next waypoint is the first of the given set
  entrance_waypoints_.push_back(waypoints_.at(0));

  entrance_traj_.set_waypoints(entrance_waypoints_);
  main_traj_.set_waypoints(waypoints_);

  return 0;
}

void Planner::assign_waypoint_timestamps(std::vector<Waypoint>& wps)
{
  /**
   * Assign timestamps to the waypoints if appropriate
   */
  PlannerConfig config = get_config();
  if (config.timing == PlannerConfig::TimestampStrategy::AVERAGE_SPEED)
  {
    for (auto&& itr = wps.begin()+1; itr != wps.end(); ++itr)
    {
      Eigen::Vector2f distance_xy(itr->position(0)-(itr-1)->position(0),
          itr->position(1)-(itr-1)->position(1));
      float distance_xy_norm = (distance_xy).norm();
      float dt_xy = distance_xy_norm / config.average_speed_xy;
      float distance_z_norm = fabs(itr->position(2)-(itr-1)->position(2));
      float dt_z = distance_z_norm / config.average_speed_z;
      itr->time = (itr-1)->time + std::max(dt_xy, dt_z);
    }
  }
}

bool Planner::verify_timestamps_monotonically_increase(
    const std::vector<Waypoint>& wps) const
{
  for (auto&& itr = wps.begin()+1; itr != wps.end(); ++itr)
  {
    if (itr->time <= (itr-1)->time)
    {
      std::cout << "Timestamps not monotonically increasing" << std::endl;
      return false;
    }
  }

  return true;
}

int Planner::safe_optimize(SnavTrajectoryThreadSafe& traj, bool loop)
{
  int deriv_num;
  PlannerConfig config = get_config();
  if (config.traj_type == PlannerConfig::TrajType::MIN_ACC) deriv_num = 2;
  else if (config.traj_type == PlannerConfig::TrajType::MIN_JERK) deriv_num = 3;
  else deriv_num = 4;

  int ret_code = 0;
  traj.clear_traj();
  auto t_before = std::chrono::steady_clock::now();
  float cost = traj.optimize(deriv_num, loop);
  auto t_after = std::chrono::steady_clock::now();
  std::chrono::duration<float> elapsed = t_after - t_before;
  std::cout << "optimized cost = " << cost << "; optimization took "
    << elapsed.count() << " s" << std::endl;

  if (cost < 0)
  {
    ret_code = -3;
  }
  else
  {
    const float kDerivMaxSampleDt = 0.01;
    traj.calculate_derivative_maximums(kDerivMaxSampleDt);
    snav_traj_gen::SnavTrajectory snav_traj = traj.get_trajectory();

    ret_code = 0;
    if (!constraint_satisfied(snav_traj.max_velocity_xy, config.max_allowed_vel_xy,
          "vel xy")) ret_code = -2;
    if (!constraint_satisfied(snav_traj.max_velocity_z, config.max_allowed_vel_z,
          "vel z")) ret_code = -2;
    if (!constraint_satisfied(snav_traj.max_acceleration_xy, config.max_allowed_acc_xy,
          "acc xy")) ret_code = -2;
    if (!constraint_satisfied(snav_traj.max_acceleration_z, config.max_allowed_acc_z,
          "acc z")) ret_code = -2;
    if (!constraint_satisfied(snav_traj.max_jerk_xy, config.max_allowed_jerk_xy,
          "jerk xy")) ret_code = -2;
    if (!constraint_satisfied(snav_traj.max_jerk_z, config.max_allowed_jerk_z,
          "jerk z")) ret_code = -2;

    if (ret_code == 0) traj.set_constraints_met();
  }

  return ret_code;
}

int Planner::calculate_path(StateVector current_state)
{
  PlannerConfig config = get_config();
  if(config.traj_type == PlannerConfig::TrajType::SHORTEST_PATH)
  {
    // Shortest path implementation -- nothing to do here (all local planning)
    return 0;
  }

  // Update state of first waypoint of entrance traj to be current state
  std::vector<Waypoint> entrance_waypoints = entrance_traj_.get_waypoints();
  entrance_waypoints.begin()->position = current_state.position;
  entrance_waypoints.begin()->velocity = current_state.velocity;
  entrance_waypoints.begin()->acceleration = current_state.acceleration;
  entrance_waypoints.begin()->jerk = current_state.jerk;
  entrance_waypoints.begin()->time = 0.0; // relative time of 0
  entrance_waypoints.begin()->fix_all(); // 1st wp is fully constrained
  (entrance_waypoints.end()-1)->fix_all(); // last wp is fully constrained

  std::vector<Waypoint> main_waypoints = main_traj_.get_waypoints();

  if (config.loop)
  {
    // Loop overwrites last waypoint and sets it equal to first internal to
    // traj gen library, so do it here prior to assigning times
    (main_waypoints.end()-1)->position = main_waypoints.begin()->position;
    std::lock_guard<std::mutex> lock(wp_mutex_);
    (waypoints_.end()-1)->position = (main_waypoints.end()-1)->position;
  }
  else
  {
    main_waypoints.begin()->fix_all();
    (main_waypoints.end()-1)->fix_all();
  }

  assign_waypoint_timestamps(entrance_waypoints);
  main_waypoints.begin()->time = (entrance_waypoints.end()-1)->time;
  assign_waypoint_timestamps(main_waypoints);

  if (!verify_timestamps_monotonically_increase(entrance_waypoints)) return -1;
  if (!verify_timestamps_monotonically_increase(main_waypoints)) return -1;

  if (config.loop)
  {
    loop_duration_ = (main_waypoints.end()-1)->time - main_waypoints.begin()->time;
  }

  entrance_traj_.set_waypoints(entrance_waypoints);
  main_traj_.set_waypoints(main_waypoints);

  // Propagate time updates to the planner's waypoints also
  {
    std::lock_guard<std::mutex> lock(wp_mutex_);
    for (auto&& it = entrance_waypoints_.begin(); it != entrance_waypoints_.end(); ++it)
    {
      it->time = entrance_waypoints.at(it-entrance_waypoints_.begin()).time;
    }

    for (auto&& it = waypoints_.begin(); it != waypoints_.end(); ++it)
    {
      it->time = main_waypoints.at(it-waypoints_.begin()).time;
    }

  }

  // First optimize main traj
  int ret_code = safe_optimize(main_traj_, config.loop);

  if (main_traj_.satisfies_constraints())
  {
    // Now optimize the entrance trajectory
    std::vector<Waypoint> optimized_wps = main_traj_.get_optimized_waypoints();
    (entrance_waypoints.end()-1)->velocity = optimized_wps.begin()->velocity;
    (entrance_waypoints.end()-1)->acceleration = optimized_wps.begin()->acceleration;
    (entrance_waypoints.end()-1)->jerk = optimized_wps.begin()->jerk;
    entrance_traj_.set_waypoints(entrance_waypoints);
    ret_code = safe_optimize(entrance_traj_, false); // entrance never loops
  }

  return ret_code;
}

void Planner::advance_waypoint()
{
  std::lock_guard<std::mutex> lock(wp_mutex_);

  waypoints_[active_waypoint_idx_].status = Waypoint::Status::COMPLETE;

  if(active_waypoint_idx_ < waypoints_.size()-1)
  {
    waypoints_[++active_waypoint_idx_].status = Waypoint::Status::ENROUTE;
  }
  else
  {
    if (get_config().loop && get_config().traj_type != PlannerConfig::TrajType::SHORTEST_PATH)
    {
      active_waypoint_idx_ = 1;
      loop_time_start_ = ++num_loops_ * loop_duration_;
      for (auto&& it = waypoints_.begin(); it != waypoints_.end(); ++it)
      {
        it->status = Waypoint::Status::INACTIVE;
        it->time += loop_duration_;
      }
      waypoints_[active_waypoint_idx_].status = Waypoint::Status::ENROUTE;
    }
    else
    {
      status_ = Status::COMPLETE;
    }
  }

  wp_convergence_.reset();
}

int Planner::waypoints_remaining() const
{
  std::lock_guard<std::mutex> lock(wp_mutex_);

  int result = 0;
  for (auto&& it = waypoints_.begin(); it != waypoints_.end(); ++it)
  {
    if (it->status != Waypoint::Status::COMPLETE) ++result;
  }

  return result;
}

Waypoint* Planner::get_active_waypoint()
{
  std::lock_guard<std::mutex> lock(wp_mutex_);

  if( active_waypoint_idx_ >= waypoints_.size() )
  {
    return nullptr;
  }

  return &(waypoints_[active_waypoint_idx_]);
}

// get action at time, given current state
int Planner::get_desired_state(float t, const StateVector& state_measured,
    StateVector& desired_state)
{
  if (!main_traj_.ready() || !entrance_traj_.ready())
  {
    if (calculate_path(state_measured) != 0) return -3;
  }

  // change in time since last call
  float dt = 0;
  if(!initialized_)
  {
    state_command_world_.position = state_measured.position;
    state_command_world_.velocity.setZero();
    state_command_world_.acceleration.setZero();
    state_command_world_.yaw = state_measured.yaw;
    state_command_world_.yaw_rate = 0;
    desired_state = state_command_world_;
    mission_time_last_secs_ = t;
    initialized_ = true;
    return 0;
  }
  else
  {
    dt = t - mission_time_last_secs_;
    mission_time_last_secs_ = t;
  }

  // Bound dt
  if (dt < kMinDt_) dt = kMinDt_;
  else if (dt > kMaxDt_) dt = kMaxDt_;

  Waypoint* active_waypoint = get_active_waypoint();
  if (!active_waypoint)
  {
    return -2;
  }

  status_ = Status::ENROUTE;
  active_waypoint->status = Waypoint::Status::ENROUTE;

  Eigen::Vector2f velocity_for_yaw_control(0, 0);
  WaypointConfig wpconf = active_waypoint->get_config();

  // Shortest path implementation (local planning only)
  if(get_config().traj_type == PlannerConfig::TrajType::SHORTEST_PATH)
  {
    // position error
    Eigen::Vector3f position_error = active_waypoint->position - state_measured.position;
    Eigen::Vector3f position_error_normalized = Eigen::Vector3f();
    if(position_error.norm() < kEpsilon_)
      position_error_normalized.setZero();
    else
      position_error_normalized = position_error/position_error.norm();

    // compute maximum possible velocity capable of stopping under maximum acceleration
    float accel_limited_velocity = sqrtf(2*wpconf.max_linear_acceleration_norm*position_error.norm());

    // bound velocity max
    float max_velocity = std::min( wpconf.max_linear_velocity_norm, accel_limited_velocity );

    // set desired velocity (with max allowable magnitude)
    Eigen::Vector3f velocity_desired = position_error_normalized*max_velocity;
    Eigen::Vector3f velocity_command_delta = velocity_desired - state_command_world_.velocity;
    Eigen::Vector3f velocity_command_delta_unit = Eigen::Vector3f();
    if(velocity_command_delta.norm() < kEpsilon_)
    {
      velocity_command_delta_unit.setZero();
    }
    else
    {
      velocity_command_delta_unit = velocity_command_delta/velocity_command_delta.norm();
    }
    Eigen::Vector3f accel_applied = velocity_command_delta_unit*wpconf.max_linear_acceleration_norm;
    Eigen::Vector3f velocity_new = state_command_world_.velocity + accel_applied*dt;

    // bound by max velocity option
    if( velocity_new.norm() > wpconf.max_linear_velocity_norm )
    {
      velocity_new = (velocity_new/velocity_new.norm())*wpconf.max_linear_velocity_norm;
    }

    // prevent overshoot if nearing waypoint (i.e. in the next dt)
    Eigen::Vector3f stop_velocity = position_error/dt;
    if( velocity_new.norm() > stop_velocity.norm() )
    {
      velocity_new = stop_velocity;
    }

    state_command_world_.position = state_command_world_.position + state_command_world_.velocity * dt;
    state_command_world_.velocity = velocity_new;
    state_command_world_.acceleration.setZero(); // could set this by differentiating velocity

    Eigen::Vector3f motion_vector = state_measured.position
      - state_measured_last_.position;

    // Check to see if waypoint was passed between this iteration and the last
    Eigen::Vector3f motion_vector_unit = motion_vector/motion_vector.norm();
    Eigen::Vector3f waypoint_vector_last = active_waypoint->position - state_measured_last_.position;
    Eigen::Vector3f waypoint_projected = motion_vector_unit*(waypoint_vector_last.dot( motion_vector_unit ));
    Eigen::Vector3f waypoint_orthogonal = waypoint_projected - waypoint_vector_last;

    float waypoint_inline = waypoint_projected.norm();
    float waypoint_toline = waypoint_orthogonal.norm();

    // should we compare against the line, the previous point, or this point?
    float waypoint_distance;
    if( waypoint_inline < 0 )
    {
      waypoint_distance = waypoint_vector_last.norm();
    }
    else if( waypoint_inline > motion_vector.norm() )
    {
      waypoint_distance = position_error.norm();
    }
    else
    {
      waypoint_distance = waypoint_toline;
    }

    wp_convergence_.update_position_error(waypoint_distance);
    wp_convergence_.update_velocity_error(state_measured.velocity.norm());

    velocity_for_yaw_control[0] = state_command_world_.velocity[0];
    velocity_for_yaw_control[1] = state_command_world_.velocity[1];
  }
  else
  {
    if(t > active_waypoint->time) advance_waypoint();

    if (status_ != Planner::Status::COMPLETE)
    {
      snav_traj_gen::StateVector state_command_world;
      snav_traj_gen::SnavTrajectory snav_traj =
      (t >= main_traj_.get_waypoints().begin()->time) ?
        main_traj_.get_trajectory() : entrance_traj_.get_trajectory();

      int sample_ret = snav_traj.sample(state_command_world, t - loop_time_start_);
      if (sample_ret != 0)
      {
        status_ = Planner::Status::COMPLETE;
        if (sample_ret == -1)
        {
          std::cout << "time out of bounds in traj sample call" << std::endl;
        }
        else if (sample_ret == -2)
        {
          std::cout << "traj not optimized in traj sample call" << std::endl;
        }
      }
      else
      {
        state_command_world_ = state_command_world;
      }

      const float kNearFuture = 0.4; // seconds in the future
      snav_traj_gen::StateVector state_command_world_future;
      float t_future = t - loop_time_start_ + kNearFuture;
      float sample_time = (t_future >= (snav_traj.optimized_wps.end()-1)->time)
        ? t - loop_time_start_ : t_future;
      if (snav_traj.sample(state_command_world_future, sample_time) == 0)
      {
        velocity_for_yaw_control[0] = state_command_world_future.velocity[0];
        velocity_for_yaw_control[1] = state_command_world_future.velocity[1];
        // else this vector is default constructed to 0s
      }
    }
    else
    {
      state_command_world_.velocity.setZero();
      state_command_world_.acceleration.setZero();
      state_command_world_.jerk.setZero();
      velocity_for_yaw_control.setZero();
    }
  }

  // ============================================================
  //                        yaw control
  // ============================================================
  switch(wpconf.yaw_type)
  {
    // set yaw to face forward along the motion vector
    case WaypointConfig::YawType::FORWARD:
      if(velocity_for_yaw_control.norm() > 0.1)
      {
        yaw_desired_ = atan2(velocity_for_yaw_control[1], velocity_for_yaw_control[0]);
      }
      break;
      // set yaw according to waypoint config
    case WaypointConfig::YawType::WAYPOINT:
      yaw_desired_ = active_waypoint->yaw;
      break;
  }

  float yaw_error = wrap_yaw(yaw_desired_ - state_measured.yaw);
  float yaw_error_abs = fabs(yaw_error);

  const float kYawPGain = 3.0;
  state_command_world_.yaw_rate = kYawPGain * yaw_error;

  // increment commanded yaw angle by desired velocity * rate
  state_command_world_.yaw = state_command_world_.yaw
    + state_command_world_.yaw_rate * dt;
  state_command_world_.yaw = wrap_yaw(state_command_world_.yaw);

  // bound maximum yaw rate
  if( state_command_world_.yaw_rate < -wpconf.max_yaw_velocity_norm)
    state_command_world_.yaw_rate = -wpconf.max_yaw_velocity_norm;
  if( state_command_world_.yaw_rate > wpconf.max_yaw_velocity_norm)
    state_command_world_.yaw_rate = wpconf.max_yaw_velocity_norm;

  if(get_config().traj_type == PlannerConfig::TrajType::SHORTEST_PATH)
  {
    wp_convergence_.update_yaw_error(yaw_error_abs);

    if (wp_convergence_.converged())
    {
      advance_waypoint();
    }
  }


  state_measured_last_ = state_measured;

  desired_state = state_command_world_;
  return 0;
}

std::string Planner::get_status_string(Status status)
{
  std::string result;
  if (status == Status::IDLE)
  {
    result = "IDLE";
  }
  else if (status == Status::ENROUTE)
  {
    result = "ENROUTE";
  }
  else if (status == Status::COMPLETE)
  {
    result = "COMPLETE";
  }
  else
  {
    result = "UNDEFINED";
  }
  return result;
}

bool Planner::constraint_satisfied(float max_opt, float max_allowed, std::string field)
{
  if (max_opt > max_allowed)
  {
    std::cout << "traj max " << field << " of " << max_opt
      << " exceeds max allowed " << field << " of " << max_allowed
      << std::endl;
    return false;
  }

  return true;
}

} // namespace snav_fci

