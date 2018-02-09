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

#ifndef SNAV_FCI_TX_COMMAND_THREAD_SAFE_HPP_
#define SNAV_FCI_TX_COMMAND_THREAD_SAFE_HPP_

#include <mutex>

#include "api/tx_commands/pwm_command.hpp"
#include "api/tx_commands/rc_command.hpp"
#include "api/tx_commands/rpm_command.hpp"
#include "api/tx_commands/thrust_att_ang_vel_command.hpp"
#include "api/tx_commands/trajectory_command.hpp"

namespace snav_fci
{

class TxCommandThreadSafe
{
public:
  TxCommandThreadSafe()
  {
    rpm_cmd_ptr_ = nullptr;
    pwm_cmd_ptr_ = nullptr;
    tx_cmd_ptr_ = nullptr;
  }

  ~TxCommandThreadSafe()
  {
    delete rpm_cmd_ptr_;
    delete pwm_cmd_ptr_;
  }

  void initialize(size_t num_motors)
  {
    rpm_cmd_ptr_ = new RpmCommand(num_motors);
    pwm_cmd_ptr_ = new PwmCommand(num_motors);
  }

  void set_tx_command_mode(TxCommand::Mode mode)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode == TxCommand::Mode::RC)
    {
      tx_cmd_ptr_ = &rc_cmd_;
    }
    else if (mode == TxCommand::Mode::RPM)
    {
      tx_cmd_ptr_ = rpm_cmd_ptr_;
    }
    else if (mode == TxCommand::Mode::PWM)
    {
      tx_cmd_ptr_ = pwm_cmd_ptr_;
    }
    else if (mode == TxCommand::Mode::THRUST_ATT_ANG_VEL)
    {
      tx_cmd_ptr_ = &thrust_att_ang_vel_cmd_;
    }
    else if (mode == TxCommand::Mode::TRAJECTORY)
    {
      tx_cmd_ptr_ = &traj_cmd_;
    }
  }

  TxCommand::Mode get_tx_command_mode()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    TxCommand::Mode result = TxCommand::Mode::UNDEFINED;
    if (tx_cmd_ptr_) result = tx_cmd_ptr_->get_mode();
    return result;
  }

  int set(const TxCommand& input)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!tx_cmd_ptr_) return -1;
    if (tx_cmd_ptr_->get_mode() != input.get_mode()) return -2;

    if (input.get_mode() == TxCommand::Mode::RC)
    {
      rc_cmd_ = static_cast<const RcCommand&>(input);
    }
    else if (input.get_mode() == TxCommand::Mode::RPM)
    {
      *rpm_cmd_ptr_ = static_cast<const RpmCommand&>(input);
    }
    else if (input.get_mode() == TxCommand::Mode::PWM)
    {
      *pwm_cmd_ptr_ = static_cast<const PwmCommand&>(input);
    }
    else if (input.get_mode() == TxCommand::Mode::THRUST_ATT_ANG_VEL)
    {
      thrust_att_ang_vel_cmd_ = static_cast<const ThrustAttAngVelCommand&>(input);
    }
    else if (input.get_mode() == TxCommand::Mode::TRAJECTORY)
    {
      traj_cmd_ = static_cast<const TrajectoryCommand&>(input);
    }
    else
    {
      return -3;
    }

    return 0;
  }

  void get(RcCommand& cmd)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd = rc_cmd_;
  }

  void get(RpmCommand& cmd)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (rpm_cmd_ptr_) cmd = *rpm_cmd_ptr_;
  }

  void get(PwmCommand& cmd)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pwm_cmd_ptr_) cmd = *pwm_cmd_ptr_;
  }

  void get(ThrustAttAngVelCommand& cmd)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd = thrust_att_ang_vel_cmd_;
  }

  void get(TrajectoryCommand& cmd)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd = traj_cmd_;
  }

private:
  RcCommand rc_cmd_;
  RpmCommand* rpm_cmd_ptr_;
  PwmCommand* pwm_cmd_ptr_;
  ThrustAttAngVelCommand thrust_att_ang_vel_cmd_;
  TrajectoryCommand traj_cmd_;
  TxCommand* tx_cmd_ptr_;
  std::mutex mutex_;
};

} // namespace snav_fci

#endif // SNAV_FCI_TX_COMMAND_THREAD_SAFE_HPP_



