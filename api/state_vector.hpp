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

#ifndef SNAV_FCI_STATE_VECTOR_HPP_
#define SNAV_FCI_STATE_VECTOR_HPP_

#include <Eigen/Dense>

#include "api/displacement_derivs.hpp"

namespace snav_fci
{

/**
 * @brief Structure containing position and yaw along with their derivatives
 */
class StateVector : public DisplacementDerivs
{
public:

  /**
   * @brief Construct a StateVector initialized to all zeros
   */
  StateVector()
  {
    yaw = 0.;
    yaw_rate = 0.;
    yaw_acceleration = 0.;
  }

  StateVector(Eigen::Vector3f position) : StateVector()
  {
    this->position = position;
  }

  StateVector(Eigen::Vector3f position, float yaw) : StateVector(position)
  {
    this->yaw = yaw;
  }

  float yaw;              /**< Yaw angle [rad] **/
  float yaw_rate;         /**< Yaw rate [rad/s] **/
  float yaw_acceleration; /**< Yaw acceleration [rad/s^2] **/

};

} // namespace snav_fci

#endif // SNAV_FCI_STATE_VECTOR_HPP_


