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

#ifndef SNAV_FCI_STATE_INFO_HPP_
#define SNAV_FCI_STATE_INFO_HPP_

#include <mutex>

#include "snav/snapdragon_navigator.h"

namespace snav_fci
{

class StateInfo
{
public:
  StateInfo()
  {
    desired_mode = SN_POS_HOLD_MODE;
    desired_input_cmd_type = SN_INPUT_CMD_TYPE_RC;
    enforce_props_state = false;
    expected_props_state = SN_PROPS_STATE_NOT_SPINNING;
    strict_pos_est_type_checking = false;
    desired_pos_est_type = SN_POS_EST_TYPE_VIO;
  }

  SnMode desired_mode;
  SnInputCommandType desired_input_cmd_type;
  bool enforce_props_state;
  SnPropsState expected_props_state;
  bool strict_pos_est_type_checking;
  SnPosEstType desired_pos_est_type;
};

class StateInfoThreadSafe
{
public:
  StateInfoThreadSafe() {}
  ~StateInfoThreadSafe() {}

  inline void set(StateInfo input)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_info_ = input;
  }

  inline StateInfo get()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_info_;
  }

private:
  StateInfo state_info_;
  std::mutex mutex_;
};

} // namespace snav_fci

#endif // SNAV_FCI_STATE_INFO_HPP_

