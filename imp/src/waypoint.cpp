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

#include "api/waypoint.hpp"

#include <cmath>

namespace snav_fci
{

Waypoint::Waypoint()
{
  constrained.setConstant(false);
  status = Status::INACTIVE;
  time = NAN;
  set_deriv_constraints(FIX_POS);
  configured_ = false;
}

Waypoint::Waypoint(Eigen::Vector3f xyz): Waypoint()
{
  position = xyz;
}

Waypoint::Waypoint(Eigen::Vector3f xyz, WaypointConfig conf) : Waypoint()
{
  position = xyz;
  set_config(conf);
}

Waypoint::Waypoint(StateVector s): Waypoint()
{
  position = s.position;
  velocity = s.velocity;
  acceleration = s.acceleration;
  jerk = s.jerk;
  yaw = s.yaw;
  yaw_rate = s.yaw_rate;
  yaw_acceleration = s.yaw_acceleration;
}

Waypoint::Waypoint(StateVector state, WaypointConfig conf): Waypoint(state)
{
  set_config(conf);
}

Waypoint::Waypoint(const snav_traj_gen::Waypoint& waypoint) : Waypoint()
{
  position[0] = waypoint.position[0];
  position[1] = waypoint.position[1];
  position[2] = waypoint.position[2];
  velocity[0] = waypoint.velocity[0];
  velocity[1] = waypoint.velocity[1];
  velocity[2] = waypoint.velocity[2];
  acceleration[0] = waypoint.acceleration[0];
  acceleration[1] = waypoint.acceleration[1];
  acceleration[2] = waypoint.acceleration[2];
  jerk[0] = waypoint.jerk[0];
  jerk[1] = waypoint.jerk[1];
  jerk[2] = waypoint.jerk[2];
  time = waypoint.time;
  constrained[0] = waypoint.constrained[0];
  constrained[1] = waypoint.constrained[1];
  constrained[2] = waypoint.constrained[2];
  constrained[3] = waypoint.constrained[3];
}

void Waypoint::set_config(WaypointConfig conf)
{
  config_ = conf;
  configured_ = true;
}

void Waypoint::set_deriv_constraints(int mask)
{
  if(mask & FIX_POS)
    constrained[0] = true;
  else
    constrained[0] = false;

  if(mask & FIX_VEL)
    constrained[1] = true;
  else
    constrained[1] = false;

  if(mask & FIX_ACC)
    constrained[2] = true;
  else
    constrained[2] = false;

  if(mask & FIX_JERK)
    constrained[3] = true;
  else
    constrained[3] = false;
}

void Waypoint::fix_all()
{
  set_deriv_constraints(Waypoint::FIX_POS | Waypoint::FIX_VEL | Waypoint::FIX_ACC | Waypoint::FIX_JERK);
}

void Waypoint::fix_none()
{
  set_deriv_constraints(!(Waypoint::FIX_POS | Waypoint::FIX_VEL | Waypoint::FIX_ACC | Waypoint::FIX_JERK));
}

std::ostream& operator<<(std::ostream &strm, const Waypoint &wp)
{
  return strm << "(" << wp.position[0] << ","
    << wp.position[1] << ","
    << wp.position[2] << ") @ t = "
    << wp.time << "; constraints = " << wp.constrained[0]
    << ", " << wp.constrained[1]
    << ", " << wp.constrained[2]
    << ", " << wp.constrained[3]
    << "; status = " << Waypoint::get_status_string(wp.status);
}

std::string Waypoint::get_status_string(const Status& status)
{
  std::string result = "";
  if (status == Waypoint::Status::COMPLETE)
  {
    result = "COMPLETE";
  }
  else if (status == Waypoint::Status::ENROUTE)
  {
    result = "ENROUTE";
  }
  else if (status == Waypoint::Status::INACTIVE)
  {
    result = "INACTIVE";
  }
  else
  {
    result ="UNDEFINED";
  }

  return result;
}


snav_traj_gen::Waypoint Waypoint::get_traj_gen_waypoint() const
{
  snav_traj_gen::Waypoint output;
  output.position[0] = position[0];
  output.position[1] = position[1];
  output.position[2] = position[2];
  output.velocity[0] = velocity[0];
  output.velocity[1] = velocity[1];
  output.velocity[2] = velocity[2];
  output.acceleration[0] = acceleration[0];
  output.acceleration[1] = acceleration[1];
  output.acceleration[2] = acceleration[2];
  output.jerk[0] = jerk[0];
  output.jerk[1] = jerk[1];
  output.jerk[2] = jerk[2];
  output.time = time;
  output.constrained[0] = constrained[0];
  output.constrained[1] = constrained[1];
  output.constrained[2] = constrained[2];
  output.constrained[3] = constrained[3];
  return output;
}

} // namespace snav_fci

