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

#ifndef SNAV_FCI_TX_CONFIG_HPP_
#define SNAV_FCI_TX_CONFIG_HPP_

#include "api/reference_frame.hpp"

namespace snav_fci
{

/**
 * @brief Structure containing options for the TX thread
 */
class TxConfig
{
public:

  /**
   * @brief Default constructor
   */
  TxConfig()
  {
    tx_rate = 50;
    desired_mode = SN_POS_HOLD_MODE;
    desired_pos_est_type = SN_POS_EST_TYPE_VIO;
    strict_pos_est_type_checking = false;
    use_traj_tracking = false;
    waypoint_frame_parent = ReferenceFrame::ESTIMATION;
    num_motors = 4; // default to quadrotor
  }

  int tx_rate; /**< Loop rate of tx thread in Hz **/
  SnMode desired_mode; /**< Desired mode of Snapdragon Navigator **/
  SnPosEstType desired_pos_est_type; /**< Desired type of position estimate,
                                          e.g. VIO, GPS, DFT **/
  bool strict_pos_est_type_checking; /**< Whether or not the desired position
                                          estimate type should be enforced **/
  bool use_traj_tracking; /**< Whether or not the trajectory tracking interface
                               should be used instead of the RC command
                               interface **/
  ReferenceFrame waypoint_frame_parent; /**< Parent frame of
                                          ReferenceFrame::WAYPOINT frame **/
  size_t num_motors; /**< Number of motors connected; only used when calling
                          FlightControlInterface::set_tx_command() with either
                          RpmCommand or PwmCommand **/
};

} // namespace snav_fci

#endif // SNAV_FCI_TX_CONFIG_HPP_


