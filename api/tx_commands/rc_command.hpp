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

#ifndef SNAV_FCI_RC_COMMAND_HPP_
#define SNAV_FCI_RC_COMMAND_HPP_

#include <array>

#include "snav/snapdragon_navigator.h"

#include "api/tx_commands/tx_command.hpp"

namespace snav_fci
{

/**
 * @brief Structure containing the inputs to the sn_send_rc_command()
 * Qualcomm Navigator API function
 */
class RcCommand : public TxCommand
{
public:

  /**
   * @brief Default constructor
   */
  RcCommand()
  {
    mode_ = Mode::RC;
    type = SN_RC_POS_HOLD_CMD;
    options = RC_OPT_LINEAR_MAPPING;
    commands.fill(0);
    commands[2] = -1; // safer to set to -1 for modes that control thrust,
                      // since -1 corresponds to min thrust in those modes
  }

  static const size_t kNumRcCommands = 4; /**< Number of RC commands is 4 as
                                            defined by the Qualcomm Navigator
                                            API */
  SnRcCommandType type; /**< Specifies how the commands should be interpreted. */
  SnRcCommandOptions options; /**< Options for RC commands. */
  std::array<float, kNumRcCommands> commands; /**< Dimensionless commands in
                                                the range [-1, 1]. */
};

} // namespace snav_fci

#endif // SNAV_FCI_RC_COMMAND_HPP_

