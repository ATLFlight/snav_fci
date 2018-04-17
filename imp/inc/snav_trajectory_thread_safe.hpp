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

#ifndef SNAV_FCI_SNAV_TRAJECTORY_THREAD_SAFE_HPP_
#define SNAV_FCI_SNAV_TRAJECTORY_THREAD_SAFE_HPP_

#include <mutex>
#include <thread>
#include <vector>

#include "snav/snav_traj_gen.hpp"

namespace snav_fci
{

class SnavTrajectoryThreadSafe
{
public:
  SnavTrajectoryThreadSafe()
  {
    optimized_ = false;
    traj_up_to_date_ = false;
    satisfies_constraints_ = false;
    cost_ = -1;
  }

  ~SnavTrajectoryThreadSafe() {}

  void set_waypoints(const std::vector<Waypoint>& wps)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    input_wps_ = wps;
    traj_up_to_date_ = false;
  }

  std::vector<Waypoint> get_waypoints() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return input_wps_;
  }

  std::vector<Waypoint> get_optimized_waypoints() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Waypoint> wps;
    for (auto&& itr = traj_.optimized_wps.begin(); itr != traj_.optimized_wps.end(); ++itr)
    {
      wps.push_back(Waypoint(*itr));
    }
    return wps;
  }

  float optimize(const int& deriv_num, const bool& loop)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<snav_traj_gen::Waypoint> traj_gen_wps;
    for (auto&& itr = input_wps_.begin(); itr != input_wps_.end(); ++itr)
    {
      traj_gen_wps.push_back(itr->get_traj_gen_waypoint());
    }

    try
    {
      // Wrap optimize() in try / catch as it may throw an exception
      cost_ = traj_.optimize(traj_gen_wps, deriv_num, loop);
    }
    catch (...)
    {
      // Catch all exceptions
      cost_ = -1;
    }

    if (cost_ > 0)
    {
      optimized_ = true;
      traj_up_to_date_ = true;
    }
    return cost_;
  }

  void set_constraints_met()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    satisfies_constraints_ = true;
  }

  void calculate_derivative_maximums(float dt)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    traj_.calculate_derivative_maximums(dt);
  }

  snav_traj_gen::SnavTrajectory get_trajectory() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return traj_;
  }

  float get_cost() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return cost_;
  }

  bool optimized() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return optimized_;
  }

  bool up_to_date() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return traj_up_to_date_;
  }

  bool satisfies_constraints() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return satisfies_constraints_;
  }

  bool ready() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return optimized_ && satisfies_constraints_; // && up_to_date_
  }

  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    traj_ = snav_traj_gen::SnavTrajectory();
    input_wps_.clear();
    optimized_ = false;
    traj_up_to_date_ = false;
    satisfies_constraints_ = false;
    cost_ = -1;
  }

  void clear_traj()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    traj_ = snav_traj_gen::SnavTrajectory();
    optimized_ = false;
    traj_up_to_date_ = false;
    satisfies_constraints_ = false;
    cost_ = -1;
  }

private:
  snav_traj_gen::SnavTrajectory traj_;
  mutable std::mutex mutex_;
  std::vector<Waypoint> input_wps_;
  bool optimized_;  // Has the trajectory ever been optimized
  bool traj_up_to_date_;
  bool satisfies_constraints_;
  float cost_;
};

} // namespace snav_fci

#endif // SNAV_FCI_SNAV_TRAJECTORY_THREAD_SAFE_HPP_

