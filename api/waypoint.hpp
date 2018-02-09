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

#ifndef SNAV_FCI_WAYPOINT_HPP_
#define SNAV_FCI_WAYPOINT_HPP_

#include <Eigen/Dense>

#include "api/config/waypoint_config.hpp"
#include "api/state_vector.hpp"

namespace snav_fci
{

/**
 * @brief Structure containing position and yaw along with their derivatives
 * and some metadata
 */
class Waypoint : public StateVector
{
public:

  enum class Status
  {
    COMPLETE,
    ENROUTE,
    INACTIVE,
  };

  enum Constraint
  {
    FIX_POS=1,
    FIX_VEL=2,
    FIX_ACC=4,
    FIX_JERK=8,
  };

  /**
   * @brief Construct a waypoint with default state (set to zeros) and options
   */
  Waypoint();

  /**
   * @brief Construct a waypoint from a position vector
   *
   * Higher derivatives are set to zero and default options are used.
   */
  Waypoint(Eigen::Vector3f position);

  /**
   * @brief Construct a waypoint from a position vector and configuration
   *
   * Higher derivatives are set to zero.
   */
  Waypoint(Eigen::Vector3f position, WaypointConfig conf);

  /**
   * @brief Construct a waypoint from a full state vector
   *
   * Default options are used.
   */
  Waypoint(StateVector state);

  /**
   * @brief Construct a waypoint from a full state vector and configuration
   */
  Waypoint(StateVector state, WaypointConfig conf);

  void set_deriv_constraints(int mask);

  /**
   * @brief Set the waypoint configuration
   *
   * @param[in] config Desired configuration
   */
  void set_config(WaypointConfig config);

  /**
   * @brief Verify whether or not an instance of Waypoint has had its
   * configuration changed from the default
   *
   * @return
   *   - true if the instance has had its configuration set
   *   - false if the instance has not had its configuration set (will use
   *     default)
   */
  inline bool is_configured() const
  {
    return configured_;
  }

  /**
   * @brief Get the waypoint configuration
   */
  inline WaypointConfig get_config() const
  {
    return config_;
  }

  Eigen::Matrix<bool,4,1> constrained; /**< Boolean vector specifying whether
                                         the derivative is fixed **/
  Status status;

  friend std::ostream& operator<<(std::ostream &strm, const Waypoint &wp);

private:
  WaypointConfig config_;
  bool configured_;

};

} // namespace snav_fci

#endif // SNAV_FCI_WAYPOINT_HPP_

