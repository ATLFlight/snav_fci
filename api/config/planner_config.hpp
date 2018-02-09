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

#ifndef SNAV_FCI_PLANNER_CONFIG_HPP_
#define SNAV_FCI_PLANNER_CONFIG_HPP_

namespace snav_fci
{

/**
 * @brief Structure containing options for the Planner
 */
class PlannerConfig
{
public:

  /**
   * @brief Enum used to handle different methods of generating trajectories
   */
  enum class TrajType
  {
    SHORTEST_PATH,  /**< Straight line to waypoint with trapezoidal velocity
                         profile; purely local, computes instantaneous command
                         at each step **/

    /**
     * Other strategies for generating trajectories exist, such as
     * minimizing snap.
     */
  };

  /**
   * @brief Default constructor
   */
  PlannerConfig()
  {
    traj_type = TrajType::SHORTEST_PATH;
  }

  /**
   * @overload
   */
  PlannerConfig(TrajType traj_type)
  {
    this->traj_type = traj_type;
  }

  /**
   * @brief Specifies the trajectory-generation method for an instance of
   * Planner
   */
  TrajType traj_type;
};

} // namespace snav_api

#endif // SNAV_FCI_PLANNER_CONFIG_HPP_

