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

#ifndef SNAV_FCI_THRUST_ATT_ANG_VEL_COMMAND_HPP_
#define SNAV_FCI_THRUST_ATT_ANG_VEL_COMMAND_HPP_

#include <array>

#include "api/tx_commands/tx_command.hpp"

namespace snav_fci
{

/**
 * @brief Structure containing the inputs to the
 * sn_send_thrust_att_ang_vel_command() Snapdragon Navigator API function
 */
class ThrustAttAngVelCommand : public TxCommand
{
public:

  /**
   * @brief Default constructor
   */
  ThrustAttAngVelCommand()
  {
    mode_ = Mode::THRUST_ATT_ANG_VEL;
    thrust = 0;
    quaternion[0] = 1;
    quaternion[1] = 0;
    quaternion[2] = 0;
    quaternion[3] = 0;
    angular_velocity[0] = 0;
    angular_velocity[1] = 0;
    angular_velocity[2] = 0;
  }

  float thrust; /**< Desired thrust in grams. **/
  std::array<float, 4> quaternion; /**< Desired attitude specified as a
                                        quaternion. The first element of the
                                        array should contain the scalar part,
                                        followed by X, Y, and Z components. **/
  std::array<float, 3> angular_velocity; /**< Desired angular velocity vector
                                              expressed in the body frame
                                              in radians per second. **/
};

} // namespace snav_fci

#endif // SNAV_FCI_THRUST_ATT_ANG_VEL_COMMAND_HPP_



