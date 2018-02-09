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

#ifndef SNAV_FCI_TRAJECTORY_COMMAND_HPP_
#define SNAV_FCI_TRAJECTORY_COMMAND_HPP_

#include <array>

#include "api/state_vector.hpp"
#include "api/tx_commands/tx_command.hpp"

namespace snav_fci
{

/**
 * @brief Structure containing the inputs to the
 * sn_send_trajectory_tracking_command() Snapdragon Navigator API function
 */
class TrajectoryCommand : public TxCommand
{
public:

  /**
   * @brief Default constructor
   */
  TrajectoryCommand()
  {
    mode_ = Mode::TRAJECTORY;
    controller = SN_POSITION_CONTROL_VIO;
    options = SN_TRAJ_DEFAULT;
    position.fill(0);
    velocity.fill(0);
    acceleration.fill(0);
    yaw = 0;
    yaw_rate = 0;
  }

  TrajectoryCommand(const StateVector& sv) : TrajectoryCommand()
  {
    position[0] = sv.position[0];
    position[1] = sv.position[1];
    position[2] = sv.position[2];
    velocity[0] = sv.velocity[0];
    velocity[1] = sv.velocity[1];
    velocity[2] = sv.velocity[2];
    acceleration[0] = sv.acceleration[0];
    acceleration[1] = sv.acceleration[1];
    acceleration[2] = sv.acceleration[2];
    yaw = sv.yaw;
    yaw_rate = sv.yaw_rate;
  }

  SnPositionController controller; /**< Desired position controlller. **/
  SnTrajectoryOptions options;   /**< Options for trajectory tracking. **/
  std::array<float, 3> position; /**< Desired position in meters. **/
  std::array<float, 3> velocity; /**< Desired velocity in meters per second. **/
  std::array<float, 3> acceleration; /**< Desired acceleration in meters per
                                          second. **/
  float yaw; /**< Desired yaw in radians. **/
  float yaw_rate; /**< Desired yaw rate in radians per second. **/
};

} // namespace snav_fci

#endif // SNAV_FCI_TRAJECTORY_COMMAND_HPP_

