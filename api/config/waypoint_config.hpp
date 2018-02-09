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

#ifndef SNAV_FCI_WAYPOINT_CONFIG_HPP_
#define SNAV_FCI_WAYPOINT_CONFIG_HPP_

#include <cmath>

namespace snav_fci
{

/**
 * @brief Structure containing waypoint options
 */
class WaypointConfig
{
public:

  /**
   * @brief Enum used to handle different ways of handling yaw tracking
   */
  enum class YawType
  {
    FORWARD,  /**< Desired yaw points in the direction of travel **/
    WAYPOINT, /**< Desired yaw is fixed and specified **/
  };

  /**
   * @brief Default constructor
   */
  WaypointConfig()
  {
    max_linear_velocity_norm = 0.5;
    max_linear_acceleration_norm = 0.25;
    max_yaw_velocity_norm = 1.0;
    max_yaw_acceleration_norm = 0.5;
    yaw_type = YawType::FORWARD;
  }

  /**
   * @overload
   */
  WaypointConfig(float max_linear_velocity_norm,
      float max_linear_acceleration_norm, float max_angular_velocity_norm,
      float max_angular_acceleration_norm, YawType yaw_type)
  {
    this->max_linear_velocity_norm = fabs(max_linear_velocity_norm);
    this->max_linear_acceleration_norm = fabs(max_linear_acceleration_norm);
    this->max_yaw_velocity_norm = fabs(max_angular_velocity_norm);
    this->max_yaw_acceleration_norm = fabs(max_angular_acceleration_norm);
    this->yaw_type = yaw_type;
  }

  /**
   * @brief Maximum allowed linear velocity magnitude in m/s
   *
   * Only used for PlannerConfig::TrajType::SHORTEST_PATH
   */
  float max_linear_velocity_norm;

  /**
   * @brief Maximum allowed linear acceleration magnitude in m/s/s
   *
   * Only used for PlannerConfig::TrajType::SHORTEST_PATH
   */
  float max_linear_acceleration_norm;

  /**
   * @brief Maximum allowed yaw velocity magnitude in rad/s
   */
  float max_yaw_velocity_norm;

  /**
   * @brief Maximum allowed yaw acceleration magnitude in rad/s/s
   */
  float max_yaw_acceleration_norm;

  /**
   * @brief Specifies how desired yaw is handled for this waypoint
   */
  YawType yaw_type;
};

} // namespace snav_fci

#endif // SNAV_FCI_WAYPOINT_CONFIG_HPP_

