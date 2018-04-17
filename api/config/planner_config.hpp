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
    SHORTEST_PATH = 0,  /**< Straight line to waypoint with trapezoidal velocity
                         profile; purely local, computes instantaneous command
                         at each step **/
    MIN_ACC,  /**< Optimize trajectory by minimizing acceleration */
    MIN_JERK, /**< Optimize trajectory by minimizing jerk */
    MIN_SNAP,   /**< Optimize trajectory by minimizing snap */
  };

  /**
   * @brief Enum used to specify the timestamp strategy when generating
   * trajectories
   */
  enum class TimestampStrategy
  {
    USE_PROVIDED = 0, /**< Use the timestamps provided with the waypoints as is **/
    AVERAGE_SPEED, /**< Assign timestamps assuming vehicle travels along the
                     shortest path between waypoints at an average speed
                     defined by average_speed_xy and average_speed_z, ignoring
                     any timestamps provided with the waypoints. **/
  };

  /**
   * @brief Default constructor
   */
  PlannerConfig()
  {
    traj_type = TrajType::SHORTEST_PATH;
    timing = TimestampStrategy::AVERAGE_SPEED;
    average_speed_xy = 0.5;
    average_speed_z = 0.3;
    max_allowed_vel_xy = 3.0; // these are sane values based on the Qualcomm
                              // Navigator default params for the Dragon DDK
    max_allowed_vel_z = 1.0;
    max_allowed_acc_xy = 6.0; // theoretical max approx. equals:
                              //   gravity * tan(max_tilt_angle)
                              // in idealized and simplified scenario
    max_allowed_acc_z = 6.0; // theoretical max approx. equals:
                             //   ((thrust / weight) - 1) * gravity
                             // in idealized and simplified scenario
    max_allowed_jerk_xy = 50; // setting somewhat arbitrarily high
    max_allowed_jerk_z = 50; // setting somewhat arbitrarily high
    loop = false; // Warning: this must be false until bug in snav_traj_gen
                  // is fixed
  }

  /**
   * @overload
   */
  PlannerConfig(TrajType traj_type) : PlannerConfig()
  {
    this->traj_type = traj_type;
  }

  /**
   * @brief Specifies the trajectory-generation method for an instance of
   * Planner
   */
  TrajType traj_type;

  TimestampStrategy timing; /**< Strategy to use for waypoint timestamps in
                              trajectory generation */
  float average_speed_xy; /**< Average speed in the XY plane in m/s used when
                            assigning timestamps to waypoints in the following
                            calculation which assumes straight-line paths: t =
                            distance in XY plane / average_speed_xy. Planner
                            uses max of time given by XY calculation and time
                            given by Z calculation. */
  float average_speed_z; /**< Average vertical speed in m/s used when assigning
                           timestamps to waypoints in the following calculation
                           which assumes straight-line paths: t = vertical
                           distance / average_speed_z. Planner uses max of time
                           given by XY calculation and time given by Z
                           calculation. */
  float max_allowed_vel_xy; /**< Maximum velocity in the XY plane permitted by
                              planner in m/s. Note that Qualcomm Navigator
                              dictates the max velocities allowed in position
                              control modes, so increasing this number is not
                              sufficient for the vehicle to attain higher
                              horizontal velocities. */
  float max_allowed_vel_z; /**< Maximum vertical velocity permitted by the
                             planner in m/s. Note that Qualcomm Navigator
                             dictates the max vertical velocity allowed in
                             height control modes, so increasing this number is
                             not sufficient for the vehicle to attain higher
                             vertical velocities. */
  float max_allowed_acc_xy; /**< Maximum acceleration in the XY plane permitted
                              by the planner in m/s^2. Note that the actual
                              achievable horizontal acceleration is limited by
                              the max tilt angle permitted by Qualcomm
                              Navigator, so increasing this number is not
                              sufficient for the vehicle to attain higher
                              horizontal accelerations. */
  float max_allowed_acc_z; /**< Maximum vertical acceleration permitted by the
                             planner in m/s^2. Note that this number is limited
                             by the vehicle's thrust-to-weight ratio. */
  float max_allowed_jerk_xy; /**< Maximum allowed jerk in the XY plane
                               permitted by the planner in m/s^3. */
  float max_allowed_jerk_z; /**< Maximum allowed vertical jerk permitted by the
                              planner in m/s^3. */
  bool loop; /**< Whether or not the trajectory should loop. WARNING: a bug
               in the traj gen library must be resolved before this option
               can be used successfully */
};

} // namespace snav_api

#endif // SNAV_FCI_PLANNER_CONFIG_HPP_

