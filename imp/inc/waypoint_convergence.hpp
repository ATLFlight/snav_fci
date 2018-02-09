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

#ifndef SNAV_FCI_WAYPOINT_CONVERGENCE_HPP_
#define SNAV_FCI_WAYPOINT_CONVERGENCE_HPP_

#include <cmath>

namespace snav_fci
{

class WaypointConvergence
{
public:

  WaypointConvergence()
  {
    reset();
  }

  inline void reset()
  {
    position_ = false;
    velocity_ = false;
    yaw_ = false;
  }

  inline bool converged() const
  {
    return position_ && velocity_ && yaw_;
  }

  inline void update_position_error(float err)
  {
    update(err, kPositionConvergeThreshold_, position_);
  }

  inline void update_velocity_error(float err)
  {
    update(err, kVelocityConvergeThreshold_, velocity_);
  }

  inline void update_yaw_error(float err)
  {
    update(err, kYawConvergeThreshold_, yaw_);
  }

private:

  void update(float err, float threshold, bool& result)
  {
    fabs(err) < threshold ? result = true : result = false;
  }

  const float kPositionConvergeThreshold_ = 5e-2; // m
  const float kVelocityConvergeThreshold_ = 1e-2; // m/s
  const float kYawConvergeThreshold_ = 1e-2; // rad
  bool position_;
  bool velocity_;
  bool yaw_;
};

} // namespace snav_fci

#endif // SNAV_FCI_WAYPOINT_CONVERGENCE_HPP_

