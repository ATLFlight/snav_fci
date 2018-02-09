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

namespace snav_fci
{

Waypoint::Waypoint()
{
  status = Status::INACTIVE;
  constrained.setConstant(true);
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

std::ostream& operator<<(std::ostream &strm, const Waypoint &wp)
{
  return strm << "(" << wp.position[0] << ","
    << wp.position[1] << ","
    << wp.position[2] << ")";
}

} // namespace snav_fci

