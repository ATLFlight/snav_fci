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
  config_ = conf;
}

// Destructor
Planner::~Planner()
{

}

void Planner::reset()
{
  waypoints_.clear();
  status_ = Status::IDLE;
  active_waypoint_idx_ = 0;
  path_ready_ = false;
  initialized_ = false;
  mission_time_last_secs_ = 0;
  yaw_desired_ = 0;
}

// adding goals
void Planner::add_waypoint(Waypoint wp)
{
  waypoints_.push_back( wp );
  path_ready_ = false;
  if (status_ == Status::COMPLETE)
  {
    advance_waypoint();
  }
}

Planner::Status Planner::get_status ()
{
  return status_ ;
}

void Planner::set_config(PlannerConfig conf)
{
  config_ = conf;
}

void Planner::add_waypoints(std::vector<Waypoint> wpv)
{
  for (auto it = wpv.begin(); it != wpv.end(); it++)
  {
    waypoints_.push_back(*it);
  }
  path_ready_ = false;
}

void Planner::calculate_path(StateVector current_state, float t_now)
{
  // Shortest path implementation -- nothing to do here (all local planning)
  if(config_.traj_type == PlannerConfig::TrajType::SHORTEST_PATH)
  {
    return;
  }

  path_ready_ = true;
}

void Planner::advance_waypoint()
{
  waypoints_[active_waypoint_idx_].status = Waypoint::Status::COMPLETE;
  if( active_waypoint_idx_ < waypoints_.size()-1 )
  {
    waypoints_[++active_waypoint_idx_].status = Waypoint::Status::ENROUTE;
  }
  else
  {
    status_ = Status::COMPLETE;
  }

  wp_convergence_.reset();
}

int Planner::waypoints_remaining()
{
  int num = waypoints_.size() - (active_waypoint_idx_+1);
  if (get_active_waypoint())
  {
    if (get_active_waypoint()->status!= Waypoint::Status::COMPLETE) num++;
  }
  return num;
}

Waypoint* Planner::get_active_waypoint()
{
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
  // ensure trajectory has been optimized
  if(!path_ready_)
  {
    calculate_path(state_measured, t);
  }

  // change in time since last call
  float dt = 0;
  if(!initialized_)
  {
    state_command_world_.position = state_measured.position;
    state_command_world_.yaw = state_measured.yaw;
    mission_time_last_secs_ = t;
    initialized_ = true;
    return -1;
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

  // position error
  Eigen::Vector3f position_error = active_waypoint->position - state_measured.position;
  Eigen::Vector3f position_error_normalized = Eigen::Vector3f();
  if(position_error.norm() < kEpsilon_)
    position_error_normalized.setZero();
  else
    position_error_normalized = position_error/position_error.norm();

  Eigen::Vector3f motion_vector = state_measured.position
    - state_measured_last_.position;

  // current measured velocity
  Eigen::Vector3f measured_velocity = state_measured.velocity;
  Eigen::Vector2f measured_velocity_xy(measured_velocity[0],
      measured_velocity[1]);

  // ============================================================
  //                        yaw control
  // ============================================================
  WaypointConfig wpconf = active_waypoint->get_config();
  switch(wpconf.yaw_type)
  {
    // set yaw to face forward along the motion vector
    case WaypointConfig::YawType::FORWARD:
      if(measured_velocity_xy.norm() > 0.1)
      {
        yaw_desired_ = atan2( measured_velocity[1], measured_velocity[0] );
      }
      break;
    // set yaw according to waypoint config
    case WaypointConfig::YawType::WAYPOINT:
      yaw_desired_ = active_waypoint->yaw;
      break;
  }

  float yaw_error = wrap_yaw(yaw_desired_ - state_measured.yaw);
  float yaw_error_abs = fabs(yaw_error);

  // determine which direction to turn
  float yaw_rate_direction = 0;
  const float kYawEpsilon = 1e-4;
  if (yaw_error < -kYawEpsilon) yaw_rate_direction = -1;
  if (yaw_error > kYawEpsilon) yaw_rate_direction = 1;

  // determine max anglular velocity that supports stopping at given angle
  float max_vel_yaw_close_mag = sqrtf(2*wpconf.max_yaw_acceleration_norm*yaw_error_abs);

  // compute desired angular velocity to reach goal location
  float des_ang_velocity = std::min(max_vel_yaw_close_mag, wpconf.max_yaw_velocity_norm);

  // compute change in velocity required to reach desired yaw rate and it's magnitude
  float des_vel_change_yaw = yaw_rate_direction*des_ang_velocity
    - state_command_world_.yaw_rate;

  // compute direction to acclerate in yaw
  float yaw_acceleration_direction = 0;
  if (des_vel_change_yaw < -kYawEpsilon) yaw_acceleration_direction = -1;
  if (des_vel_change_yaw > kYawEpsilon) yaw_acceleration_direction = 1;

  // compute yaw acceleration to use
  float accel_applied_yaw = yaw_acceleration_direction*wpconf.max_yaw_acceleration_norm;
  // increment commanded angular velocity by desired accleration*dt
  state_command_world_.yaw_rate = state_command_world_.yaw_rate
      + accel_applied_yaw*dt;

  // increment commanded yaw angle by desired velocity * rate
  state_command_world_.yaw = state_command_world_.yaw
    + state_command_world_.yaw_rate * dt;
  state_command_world_.yaw = wrap_yaw(state_command_world_.yaw);

  // bound maximum yaw rate
  if( state_command_world_.yaw_rate < -wpconf.max_yaw_velocity_norm)
    state_command_world_.yaw_rate = -wpconf.max_yaw_velocity_norm;
  if( state_command_world_.yaw_rate > wpconf.max_yaw_velocity_norm)
    state_command_world_.yaw_rate = wpconf.max_yaw_velocity_norm;

  // Shortest path implementation (local planning only)
  if(config_.traj_type == PlannerConfig::TrajType::SHORTEST_PATH)
  {
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
    state_command_world_.velocity = velocity_new;

    state_command_world_.position = state_command_world_.position + state_command_world_.velocity * dt;

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

} // namespace snav_fci

