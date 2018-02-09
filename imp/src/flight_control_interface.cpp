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

#include "api/flight_control_interface.hpp"

#include <chrono>
#include <iostream>
#include <sstream>
#include <unistd.h>

namespace snav_fci
{

std::mutex FlightControlInterface::connection_mutex_;
std::mutex FlightControlInterface::command_mutex_;
TxConfig FlightControlInterface::tx_config_;
bool FlightControlInterface::configured_tx_ = false;
RxConfig FlightControlInterface::rx_config_;
bool FlightControlInterface::configured_rx_ = false;
std::condition_variable FlightControlInterface::configured_rx_cv_;
TxCommandThreadSafe FlightControlInterface::tcts_;
TransformThreadSafe FlightControlInterface::tf_ep_;
TransformThreadSafe FlightControlInterface::tf_pw_;
TransformThreadSafe FlightControlInterface::tf_ev_;
StateVector FlightControlInterface::desired_setpoint_wrt_e_;
StateVector FlightControlInterface::desired_setpoint_wrt_w_;
SnavCachedDataThreadSafe FlightControlInterface::scdts_;
StateInfoThreadSafe FlightControlInterface::sits_;
std::thread FlightControlInterface::tx_thread_;
std::atomic_bool FlightControlInterface::tx_ok_(false);
std::thread FlightControlInterface::rx_thread_;
std::atomic_bool FlightControlInterface::rx_ok_(false);
bool FlightControlInterface::write_access_taken_ = false;
int FlightControlInterface::num_connections_ = 0;
std::atomic_bool FlightControlInterface::verbose_(true);

FlightControlInterface::FlightControlInterface()
{
  initialized_ = false;
  connected_ = false;
  permissions_ = Permissions::NONE;
}

FlightControlInterface::~FlightControlInterface()
{
  if (connected_)
  {
    disconnect();
  }
}

FlightControlInterface::Return FlightControlInterface::initialize(const Permissions requested_access)
{
  // Lock mutex to prevent race conditions in determining who gets write access
  std::lock_guard<std::mutex> lock(connection_mutex_);

  if (initialized_)
  {
    print_error(__PRETTY_FUNCTION__, "already initialized");
    return Return::ALREADY_INITIALIZED;
  }

  if (scdts_.initialize() != 0)
  {
    print_error(__PRETTY_FUNCTION__, "could not init SnavCachedData");
    return Return::SNAV_CACHED_DATA_ERROR;
  }

  if (requested_access == Permissions::READ_WRITE)
  {
    if (write_access_taken_)
    {
      print_error(__PRETTY_FUNCTION__, "write access already taken");
      return Return::WRITE_ACCESS_TAKEN;
    }
    else
    {
      write_access_taken_ = true;
    }
  }

  permissions_ = requested_access;
  initialized_ = true;

  return Return::SUCCESS;
}

FlightControlInterface::Return FlightControlInterface::configure_tx(const TxConfig& tx_config)
{
  std::lock_guard<std::mutex> lock(connection_mutex_);

  if (!initialized_)
  {
    print_error(__PRETTY_FUNCTION__, "object is not initialized");
    return Return::NOT_INITIALIZED;
  }

  if (permissions_ == Permissions::READ_ONLY)
  {
    print_error(__PRETTY_FUNCTION__, "READ_ONLY instance cannot configure tx");
    return Return::NO_WRITE_ACCESS;
  }

  if (tx_ok_)
  {
    print_error(__PRETTY_FUNCTION__, "connection already established");
    return Return::ALREADY_CONNECTED;
  }

  TxCommand::Mode tx_cmd_mode;
  SnRcCommandType rc_cmd_type;
  SnInputCommandType desired_input_cmd_type;
  if (infer_metadata_from_mode(tx_config.desired_mode,
        tx_config.use_traj_tracking, tx_cmd_mode, desired_input_cmd_type,
        rc_cmd_type) != 0)
  {
    print_error(__PRETTY_FUNCTION__, "invalid config, using default");
    return Return::INVALID_CONFIG;
  }

  tx_config_ = tx_config;
  StateInfo state_info = sits_.get();
  state_info.desired_mode = tx_config.desired_mode;
  state_info.desired_input_cmd_type = desired_input_cmd_type;
  state_info.strict_pos_est_type_checking = tx_config.strict_pos_est_type_checking;
  state_info.desired_pos_est_type = tx_config.desired_pos_est_type;

  tcts_.initialize(tx_config_.num_motors);
  tcts_.set_tx_command_mode(tx_cmd_mode);

  if (tx_cmd_mode == TxCommand::Mode::RC)
  {
    RcCommand cmd;
    cmd.type = rc_cmd_type;
    tcts_.set(cmd);
  }

  sits_.set(state_info);
  configured_tx_ = true;

  return Return::SUCCESS;
}

FlightControlInterface::Return FlightControlInterface::configure_rx(const RxConfig& rx_config)
{
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);

    if (!initialized_)
    {
      print_error(__PRETTY_FUNCTION__, "object is not initialized");
      return Return::NOT_INITIALIZED;
    }

    if (rx_ok_)
    {
      print_error(__PRETTY_FUNCTION__, "connection already established");
      return Return::ALREADY_CONNECTED;
    }

    rx_config_ = rx_config;
    configured_rx_ = true;
  }

  // Alert waiting threads that the rx_config has been set and it is now OK
  // to spawn the rx thread if it has not already been spawned
  configured_rx_cv_.notify_all();

  return Return::SUCCESS;
}

void FlightControlInterface::wait_for_configure()
{
  std::unique_lock<std::mutex> lock(connection_mutex_);
  // Read-only threads only need to worry about rx config, since they cannot
  // spawn the tx thread themselves
  configured_rx_cv_.wait(lock, []{return configured_rx_;});
}

FlightControlInterface::Return FlightControlInterface::connect()
{
  std::unique_lock<std::mutex> lock(connection_mutex_);

  if (!initialized_)
  {
    print_error(__PRETTY_FUNCTION__, "object is not initialized");
    return Return::NOT_INITIALIZED;
  }

  if (connected_)
  {
    print_error(__PRETTY_FUNCTION__, "object is already connected");
    return Return::ALREADY_CONNECTED;
  }

  if (permissions_ == Permissions::READ_WRITE
      && tx_thread_.get_id() == std::thread::id())
  {
    // thread is not yet associated with a running thread, so launch it
    if (!configured_tx_)
    {
      lock.unlock();
      configure_tx(tx_config_);
      lock.lock();
    }
    tx_ok_ = true;
    tx_thread_ = std::thread(&FlightControlInterface::tx_loop);
    std::stringstream stream;
    stream << "Launched tx thread (id: " << tx_thread_.get_id() << ") @ "
      << tx_config_.tx_rate << " Hz";
    print_info(stream.str());
  }

  if ((permissions_ == Permissions::READ_WRITE
        || permissions_ == Permissions::READ_ONLY)
      && rx_thread_.get_id() == std::thread::id())
  {
    // thread is not yet associated with a running thread, so launch it
    if (!configured_rx_)
    {
      lock.unlock();
      configure_rx(rx_config_);
      lock.lock();
    }
    rx_ok_ = true;
    rx_thread_ = std::thread(&FlightControlInterface::rx_loop);
    std::stringstream stream;
    stream << "Launched rx thread (id: " << rx_thread_.get_id() << ") @ "
      << rx_config_.rx_rate << " Hz";
    print_info(stream.str());
  }

  connected_ = true;
  ++num_connections_;

  return Return::SUCCESS;
}

FlightControlInterface::Return FlightControlInterface::disconnect()
{
  std::lock_guard<std::mutex> lock(connection_mutex_);

  if (!initialized_)
  {
    print_error(__PRETTY_FUNCTION__, "object is not initialized");
    return Return::NOT_INITIALIZED;
  }

  if (!connected_)
  {
    print_error(__PRETTY_FUNCTION__, "object is not connected");
    return Return::NOT_CONNECTED;
  }

  if (permissions_ == Permissions::READ_WRITE
      && tx_thread_.get_id() != std::thread::id())
  {
    tx_ok_ = false;
    tx_thread_.join();
    write_access_taken_ = false;
    print_info("tx thread terminated normally");
  }

  if ((permissions_ == Permissions::READ_ONLY
        || permissions_ == Permissions::READ_WRITE)
      && rx_thread_.get_id() != std::thread::id()
      && num_connections_ <= 1)
  {
    rx_ok_ = false;
    rx_thread_.join();
    print_info("rx thread terminated normally");
  }

  connected_ = false;
  --num_connections_;

  return Return::SUCCESS;
}

FlightControlInterface::Return FlightControlInterface::start_props()
{
  std::lock_guard<std::mutex> lock(command_mutex_);

  Return ret_code = basic_comm_checks(__PRETTY_FUNCTION__);
  if (ret_code != Return::SUCCESS)
  {
    return ret_code;
  }

  ret_code = Return::CONNECTION_LOST;
  if (FlightControlInterface::ok())
  {
    bool finished = false;
    do_not_enforce_props_state_constraint();
    while (!finished && FlightControlInterface::ok())
    {
      static int spinup_try_cntr = 0;
      if (scdts_.get().general_status.props_state == SN_PROPS_STATE_NOT_SPINNING)
      {
        sn_spin_props();
        if (++spinup_try_cntr > 10)
        {
          print_error(__PRETTY_FUNCTION__, "could not start props");
          finished = true;
          ret_code = Return::PROP_START_FAILURE;
        }
      }
      else if (scdts_.get().general_status.props_state == SN_PROPS_STATE_SPINNING)
      {
        finished = true;
        ret_code = Return::SUCCESS;
      }

      usleep(2e4);
    }
  }

  if (ret_code == Return::CONNECTION_LOST)
  {
    print_error(__PRETTY_FUNCTION__, "lost connection");
  }

  return ret_code;
}

FlightControlInterface::Return FlightControlInterface::stop_props()
{
  std::lock_guard<std::mutex> lock(command_mutex_);

  Return ret_code = basic_comm_checks(__PRETTY_FUNCTION__);
  if (ret_code != Return::SUCCESS)
  {
    return ret_code;
  }

  ret_code = Return::CONNECTION_LOST;
  if (FlightControlInterface::ok())
  {
    bool finished = false;
    while (!finished && FlightControlInterface::ok())
    {
      static int stop_try_cntr = 0;
      if (scdts_.get().general_status.props_state == SN_PROPS_STATE_SPINNING
          && scdts_.get().general_status.on_ground == 1)
      {
        // Snapdragon Navigator has determined that vehicle is on ground,
        // so it is safe to kill the propellers
        sn_stop_props();
        if (++stop_try_cntr > 10)
        {
          print_error(__PRETTY_FUNCTION__, "could not stop props");
          ret_code = Return::PROP_STOP_FAILURE;
          finished = true;
        }
      }

      if (scdts_.get().general_status.props_state == SN_PROPS_STATE_NOT_SPINNING)
      {
        finished = true;
        ret_code = Return::SUCCESS;
      }

      usleep(2e4);
    }
  }

  if (ret_code == Return::CONNECTION_LOST)
  {
    print_error(__PRETTY_FUNCTION__, "lost connection");
  }

  return ret_code;
}

FlightControlInterface::Return FlightControlInterface::takeoff()
{
  return takeoff(TakeoffConfig());
}

FlightControlInterface::Return FlightControlInterface::takeoff(const TakeoffConfig& takeoff_config)
{
  std::unique_lock<std::mutex> lock(command_mutex_);

  Return ret_code = basic_comm_checks(__PRETTY_FUNCTION__);
  if (ret_code != Return::SUCCESS) return ret_code;

  if (scdts_.get().general_status.props_state == SN_PROPS_STATE_STARTING
      || scdts_.get().general_status.props_state == SN_PROPS_STATE_SPINNING)
  {
    print_error(__PRETTY_FUNCTION__, "props are already spinning");
    return Return::IN_FLIGHT;
  }
  else
  {
    // Blocking call to start props
    lock.unlock();
    if (start_props() != Return::SUCCESS) return Return::PROP_START_FAILURE;
    lock.lock();
  }

  ret_code = Return::CONNECTION_LOST;
  bool finished = false;
  enforce_props_state_constraint(SN_PROPS_STATE_SPINNING);
  while (!finished && FlightControlInterface::ok())
  {
    SnavCachedData snav_data = scdts_.get();
    if (snav_data.general_status.props_state == SN_PROPS_STATE_SPINNING)
    {
      // Take off and smoothly slow down to desired takeoff altitude
      float t_wd = snav_data.pos_vel.position_desired[2]
        - (tf_ep_.get_translation()[2] + tf_pw_.get_translation()[2]);
      float desired_altitude_wrt_w = takeoff_config.height;
      float z_error_takeoff = desired_altitude_wrt_w - t_wd;

      if (z_error_takeoff < 0) { z_error_takeoff=0; }

      float z_vel_des = sqrt(z_error_takeoff)
        * sqrt(2*takeoff_config.max_linear_acceleration_norm)
        + takeoff_config.min_takeoff_speed;

      if(z_vel_des > takeoff_config.max_takeoff_speed)
      {
        z_vel_des = takeoff_config.max_takeoff_speed;
      }

      if (t_wd > desired_altitude_wrt_w)
      {
        finished = true;
        z_vel_des = 0;
        ret_code = Return::SUCCESS;
      }

      float dt = 1.0 / static_cast<float>(tx_config_.tx_rate);
      Eigen::Vector3f des_vel_wrt_e(0, 0, z_vel_des);
      Eigen::Vector3f des_pos_wrt_e
        = desired_setpoint_wrt_e_.position + des_vel_wrt_e * dt;
      desired_setpoint_wrt_e_.position = des_pos_wrt_e;
      desired_setpoint_wrt_e_.velocity = des_vel_wrt_e;
      desired_setpoint_wrt_e_.yaw_rate = 0;

      if (update_tx_command_pos_ctrl(__PRETTY_FUNCTION__) != 0)
      {
        ret_code = Return::NOT_SUPPORTED;
        finished = true;
      }
    }

    usleep(1.0 / tx_config_.tx_rate * 1e6);
  }

  if (ret_code == Return::CONNECTION_LOST)
  {
    print_error(__PRETTY_FUNCTION__, "lost connection");
  }

  return ret_code;
}

FlightControlInterface::Return FlightControlInterface::land()
{
  return land(LandingConfig());
}

FlightControlInterface::Return FlightControlInterface::land(const LandingConfig& landing_config)
{
  std::unique_lock<std::mutex> lock(command_mutex_);

  Return ret_code = basic_comm_checks(__PRETTY_FUNCTION__);
  if (ret_code != Return::SUCCESS) return ret_code;

  if (scdts_.get().general_status.props_state != SN_PROPS_STATE_SPINNING)
  {
    print_error(__PRETTY_FUNCTION__, "not in flight");
    return Return::NOT_IN_FLIGHT;
  }

  ret_code = Return::CONNECTION_LOST;
  bool finished = false;
  while (!finished && FlightControlInterface::ok())
  {
    float dt = 1.0 / static_cast<float>(tx_config_.tx_rate);
    Eigen::Vector3f des_vel_wrt_e(0, 0, -landing_config.max_landing_speed);
    Eigen::Vector3f des_pos_wrt_e
      = desired_setpoint_wrt_e_.position + des_vel_wrt_e * dt;
    desired_setpoint_wrt_e_.position = des_pos_wrt_e;
    desired_setpoint_wrt_e_.velocity = des_vel_wrt_e;
    desired_setpoint_wrt_e_.yaw_rate = 0;

    if (update_tx_command_pos_ctrl(__PRETTY_FUNCTION__) != 0)
    {
      ret_code = Return::NOT_SUPPORTED;
      finished = true;
    }

    if (scdts_.get().general_status.on_ground == 1)
    {
      lock.unlock();
      Return stop_props_result = stop_props();
      lock.lock();
      if (stop_props_result == Return::SUCCESS)
      {
        finished = true;
        ret_code = Return::SUCCESS;
        do_not_enforce_props_state_constraint();
      }
      else
      {
        finished = true;
        ret_code = Return::PROP_STOP_FAILURE;
      }
    }

    usleep(1.0 / tx_config_.tx_rate * 1e6);
  }

  if (ret_code == Return::CONNECTION_LOST)
  {
    print_error(__PRETTY_FUNCTION__, "lost connection");
  }

  return ret_code;
}

FlightControlInterface::Return FlightControlInterface::go_to_waypoint(const Waypoint& wp)
{
  std::lock_guard<std::mutex> lock(command_mutex_);

  Return ret_code = basic_comm_checks(__PRETTY_FUNCTION__);
  if (ret_code != Return::SUCCESS) return ret_code;

  if (scdts_.get().general_status.props_state != SN_PROPS_STATE_SPINNING)
  {
    print_error(__PRETTY_FUNCTION__, "not in flight");
    return Return::NOT_IN_FLIGHT;
  }

  planner_.reset();
  planner_.add_waypoint(wp);

  auto t_start = std::chrono::steady_clock::now();
  auto t_now = t_start;

  ret_code = Return::CONNECTION_LOST;
  bool finished = false;
  enforce_props_state_constraint(SN_PROPS_STATE_SPINNING);
  while (!finished && FlightControlInterface::ok())
  {
    SnavCachedData snav_data = scdts_.get();
    if (snav_data.general_status.props_state == SN_PROPS_STATE_SPINNING)
    {
      StateVector setpoint_wrt_e;

      setpoint_wrt_e.position = Eigen::Vector3f(
          snav_data.pos_vel.position_desired[0],
          snav_data.pos_vel.position_desired[1],
          snav_data.pos_vel.position_desired[2]);

      setpoint_wrt_e.velocity = Eigen::Vector3f(
          snav_data.pos_vel.velocity_desired[0],
          snav_data.pos_vel.velocity_desired[1],
          snav_data.pos_vel.velocity_desired[2]);

      setpoint_wrt_e.yaw = snav_data.pos_vel.yaw_desired;

      // Transform measured state from ESTIMATION frame into WAYPOINT frame
      StateVector setpoint_wrt_w
        = tf_pw_.get_transform().inverse() * tf_ep_.get_transform().inverse()
        * setpoint_wrt_e;

      // Get current time
      t_now = std::chrono::steady_clock::now();
      std::chrono::duration<float> elapsed = t_now - t_start;

      // Get output of path planner
      // input: current setpoint
      // output: desired setpoint
      int result = planner_.get_desired_state(elapsed.count(),
          setpoint_wrt_w, desired_setpoint_wrt_w_);
      if (result == -2)
      {
        finished = true;
        ret_code = Return::FAILURE;
      }

      // Transform desired state from WAYPOINT frame into ESTIMATION frame
      // for control
      desired_setpoint_wrt_e_ = tf_ep_.get_transform() * tf_pw_.get_transform()
        * desired_setpoint_wrt_w_;

      if (update_tx_command_pos_ctrl(__PRETTY_FUNCTION__) != 0)
      {
        ret_code = Return::NOT_SUPPORTED;
        finished = true;
      }

      // If reached waypoint is met, increment waypoint
      if (planner_.get_status() == Planner::Status::COMPLETE)
      {
        finished = true;
        ret_code = Return::SUCCESS;
      }
    }

    usleep(1.0 / tx_config_.tx_rate * 1e6);
  }

  if (ret_code == Return::CONNECTION_LOST)
  {
    print_error(__PRETTY_FUNCTION__, "lost connection");
  }

  return ret_code;
}

FlightControlInterface::Return FlightControlInterface::set_waypoint_frame_tf(
    const Eigen::Quaternionf& q_pw, const Eigen::Vector3f& t_pw)
{
  if (!initialized_)
  {
    print_error(__PRETTY_FUNCTION__, "object is not initialized");
    return Return::NOT_INITIALIZED;
  }
  tf_pw_.set(q_pw, t_pw);
  return Return::SUCCESS;
}

FlightControlInterface::Return FlightControlInterface::get_waypoint_frame_tf(
    Eigen::Quaternionf& q_pw, Eigen::Vector3f& t_pw) const
{
  if (!initialized_)
  {
    print_error(__PRETTY_FUNCTION__, "object is not initialized");
    return Return::NOT_INITIALIZED;
  }
  q_pw = tf_pw_.get_rotation();
  t_pw = tf_pw_.get_translation();
  return Return::SUCCESS;
}

FlightControlInterface::Return FlightControlInterface::set_tx_command(const TxCommand& cmd)
{
  std::lock_guard<std::mutex> lock(command_mutex_);

  Return ret_code = basic_comm_checks(__PRETTY_FUNCTION__);
  if (ret_code != Return::SUCCESS) return ret_code;

  if (tcts_.set(cmd) != 0)
  {
    print_error(__PRETTY_FUNCTION__, "improper tx command mode");
    ret_code = Return::IMPROPER_TX_COMMAND_MODE;
  }
  else
  {
    ret_code = Return::SUCCESS;
  }

  return ret_code;
}

FlightControlInterface::Return FlightControlInterface::get_snav_cached_data(SnavCachedData& data) const
{
  if (!initialized_)
  {
    print_error(__PRETTY_FUNCTION__, "object is not initialized");
    return Return::NOT_INITIALIZED;
  }

  if (!connected_)
  {
    print_error(__PRETTY_FUNCTION__, "object is not connected");
    return Return::NOT_CONNECTED;
  }

  data = scdts_.get();
  return Return::SUCCESS;
}

FlightControlInterface::Return FlightControlInterface::get_estimated_state(const SnavCachedData& snav_data,
    StateVector& state) const
{
  if (!initialized_)
  {
    print_error(__PRETTY_FUNCTION__, "object is not initialized");
    return Return::NOT_INITIALIZED;
  }

  StateVector state_wrt_e;
  state_wrt_e.position = Eigen::Vector3f(
        snav_data.pos_vel.position_estimated[0],
        snav_data.pos_vel.position_estimated[1],
        snav_data.pos_vel.position_estimated[2]);

  state_wrt_e.velocity = Eigen::Vector3f(
        snav_data.pos_vel.velocity_estimated[0],
        snav_data.pos_vel.velocity_estimated[1],
        snav_data.pos_vel.velocity_estimated[2]);

  state_wrt_e.yaw = snav_data.pos_vel.yaw_estimated;

  // Transform state from ESTIMATION frame into WAYPOINT frame
  state = tf_pw_.get_transform().inverse() * tf_ep_.get_transform().inverse()
    * state_wrt_e;

  return Return::SUCCESS;
}

FlightControlInterface::Return FlightControlInterface::get_desired_state(const SnavCachedData& snav_data,
    StateVector& state) const
{
  if (!initialized_)
  {
    print_error(__PRETTY_FUNCTION__, "object is not initialized");
    return Return::NOT_INITIALIZED;
  }

  StateVector state_wrt_e;
  state_wrt_e.position = Eigen::Vector3f(
        snav_data.pos_vel.position_desired[0],
        snav_data.pos_vel.position_desired[1],
        snav_data.pos_vel.position_desired[2]);

  state_wrt_e.velocity = Eigen::Vector3f(
        snav_data.pos_vel.velocity_desired[0],
        snav_data.pos_vel.velocity_desired[1],
        snav_data.pos_vel.velocity_desired[2]);

  state_wrt_e.yaw = snav_data.pos_vel.yaw_desired;

  // Transform state from ESTIMATION frame into WAYPOINT frame
  state = tf_pw_.get_transform().inverse() * tf_ep_.get_transform().inverse()
    * state_wrt_e;

  return Return::SUCCESS;
}

void FlightControlInterface::tx_loop()
{
  unsigned int sleep_time = 1.0 / tx_config_.tx_rate * 1e6;

  for (int ii = 0; ii < 10; ++ii)
  {
    send_tx_command();
    usleep(sleep_time);
  }

  while (FlightControlInterface::tx_ok())
  {
    update_waypoint_frame_parent_transform();
    send_tx_command();
    check_tx_conditions();
    usleep(sleep_time);
  }
}

void FlightControlInterface::update_waypoint_frame_parent_transform()
{
  if (tx_config_.waypoint_frame_parent == ReferenceFrame::LAUNCH)
  {
    SnavCachedData snav_data = scdts_.get();
    if (snav_data.pos_vel.launch_tf_is_valid)
    {
      Eigen::Quaternionf q_el(Eigen::AngleAxisf(snav_data.pos_vel.yaw_el,
            Eigen::Vector3f::UnitZ()));
      Eigen::Vector3f t_el(snav_data.pos_vel.t_el[0],
          snav_data.pos_vel.t_el[1],
          snav_data.pos_vel.t_el[2]);
      tf_ep_.set(q_el, t_el);
    }
  }
}

void FlightControlInterface::send_tx_command()
{
  TxCommand::Mode tx_cmd_mode = tcts_.get_tx_command_mode();
  if (tx_cmd_mode == TxCommand::Mode::RC)
  {
    RcCommand cmd;
    tcts_.get(cmd);
    sn_send_rc_command(cmd.type, cmd.options,
        cmd.commands[0],
        cmd.commands[1],
        cmd.commands[2],
        cmd.commands[3]);
  }

  else if (tx_cmd_mode == TxCommand::Mode::RPM)
  {
    RpmCommand cmd(tx_config_.num_motors);
    tcts_.get(cmd);
    static size_t fb_id = 0;
    sn_send_esc_rpm(cmd.rpms.data(), cmd.rpms.size(), fb_id);
    if (++fb_id >= tx_config_.num_motors)
    {
      fb_id = 0;
    }
  }

  else if (tx_cmd_mode == TxCommand::Mode::PWM)
  {
    PwmCommand cmd(tx_config_.num_motors);
    tcts_.get(cmd);
    static size_t fb_id = 0;
    sn_send_esc_pwm(cmd.pwms.data(), cmd.pwms.size(), fb_id);
    if (++fb_id >= tx_config_.num_motors)
    {
      fb_id = 0;
    }
  }

  else if (tx_cmd_mode == TxCommand::Mode::THRUST_ATT_ANG_VEL)
  {
    ThrustAttAngVelCommand cmd;
    tcts_.get(cmd);
    sn_send_thrust_att_ang_vel_command(cmd.thrust, cmd.quaternion[0],
        cmd.quaternion[1], cmd.quaternion[2], cmd.quaternion[3],
        cmd.angular_velocity[0], cmd.angular_velocity[1],
        cmd.angular_velocity[2]);
  }

  else if (tx_cmd_mode == TxCommand::Mode::TRAJECTORY)
  {
    TrajectoryCommand cmd;
    tcts_.get(cmd);

    SnavCachedData snav_data = scdts_.get();
    Eigen::Vector3f t_eb_wrt_e(
        snav_data.pos_vel.position_estimated[0],
        snav_data.pos_vel.position_estimated[1],
        snav_data.pos_vel.position_estimated[2]);

    Eigen::Vector3f t_vb_wrt_v(
        snav_data.vio_pos_vel.position_estimated[0],
        snav_data.vio_pos_vel.position_estimated[1],
        snav_data.vio_pos_vel.position_estimated[2]);
    Eigen::Vector3f t_vb_wrt_e = t_vb_wrt_v; // R_ev is identity

    Eigen::Vector3f t_ev_wrt_e = t_eb_wrt_e - t_vb_wrt_e;
    tf_ev_.set(t_ev_wrt_e);

    // Since this command type sends "absolute" position and yaw with respect
    // to the SNAV VIO Z up frame, need to initialize the command to current
    // desired position and yaw. This is necessary for takeoff, which ascends
    // in place relative to the takeoff location, otherwise it would be init'd
    // to some arbitrary value like 0
    if (snav_data.general_status.props_state != SN_PROPS_STATE_SPINNING)
    {
      desired_setpoint_wrt_e_.position = t_eb_wrt_e;
      desired_setpoint_wrt_e_.yaw = snav_data.pos_vel.yaw_estimated;
    }

    sn_send_trajectory_tracking_command(cmd.controller, cmd.options,
        cmd.position[0], cmd.position[1], cmd.position[2],
        cmd.velocity[0], cmd.velocity[1], cmd.velocity[2],
        cmd.acceleration[0], cmd.acceleration[1], cmd.acceleration[2],
        cmd.yaw, cmd.yaw_rate);
  }
}

void FlightControlInterface::rx_loop()
{
  unsigned int sleep_time = 1.0 / rx_config_.rx_rate * 1e6;
  while (FlightControlInterface::rx_ok())
  {
    if (scdts_.update() != 0)
    {
      print_error("error updating SnavCachedData; disconnecting rx");
      rx_ok_ = false;
    }

    usleep(sleep_time);
  }
}

void FlightControlInterface::check_tx_conditions()
{
  if (FlightControlInterface::rx_ok())
  {
    SnavCachedData snav_data = scdts_.get();

    static auto mode_ok_time_last = std::chrono::steady_clock::now();
    if (snav_data.general_status.current_mode != sits_.get().desired_mode)
    {
      std::chrono::duration<float> elapsed = std::chrono::steady_clock::now()
        - mode_ok_time_last;
      const float kModeErrorTimeSec = 0.1;
      if (elapsed.count() > kModeErrorTimeSec)
      {
        std::stringstream stream;
        stream << "current mode ("
          << sn_get_enum_string("SnMode", snav_data.general_status.current_mode)
          << ") does not match configured mode ("
          << sn_get_enum_string("SnMode", sits_.get().desired_mode)
          << "); disconnecting tx";
        print_error(stream.str());
        tx_ok_ = false;
      }
    }
    else
    {
      mode_ok_time_last = std::chrono::steady_clock::now();
    }

    static auto input_cmd_type_ok_time_last = std::chrono::steady_clock::now();
    if (snav_data.general_status.input_cmd_type != sits_.get().desired_input_cmd_type)
    {
      std::chrono::duration<float> elapsed = std::chrono::steady_clock::now()
        - input_cmd_type_ok_time_last;
      const float kInputCmdTypeErrorTimeSec = 0.1;
      if (elapsed.count() > kInputCmdTypeErrorTimeSec)
      {
        std::stringstream stream;
        stream << "unexpected input command type -- expected: "
          << sn_get_enum_string("SnInputCommandType", sits_.get().desired_input_cmd_type)
          << ", found: "
          << sn_get_enum_string("SnInputCommandType", snav_data.general_status.input_cmd_type)
          << "; disconnecting tx";
        print_error(stream.str());
        tx_ok_ = false;
      }
    }
    else
    {
      input_cmd_type_ok_time_last = std::chrono::steady_clock::now();
    }

    static auto rc_active_ok_time_last = std::chrono::steady_clock::now();
    if (snav_data.general_status.input_cmd_type == SN_INPUT_CMD_TYPE_RC)
    {
      if (snav_data.rc_active.source != SN_RC_CMD_API_INPUT)
      {
        std::chrono::duration<float> elapsed = std::chrono::steady_clock::now()
          - rc_active_ok_time_last;
        const float kRcActiveErrorTimeSec = 0.1;
        if (elapsed.count() > kRcActiveErrorTimeSec)
        {
          std::stringstream stream;
          stream << "unexpected rc command source -- expected: SN_RC_CMD_API_INPUT, found: "
            << sn_get_enum_string("SnRcCommandSource", snav_data.rc_active.source)
            << "; disconnecting tx";
          print_error(stream.str());
          tx_ok_ = false;
        }
      }
      else
      {
        input_cmd_type_ok_time_last = std::chrono::steady_clock::now();
      }
    }

    static auto props_ok_time_last = std::chrono::steady_clock::now();
    if (sits_.get().enforce_props_state
        && sits_.get().expected_props_state != snav_data.general_status.props_state)
    {
      std::chrono::duration<float> elapsed = std::chrono::steady_clock::now()
        - props_ok_time_last;
      const float kPropsErrorTimeSec = 0.1;
      if (elapsed.count() > kPropsErrorTimeSec)
      {
        std::stringstream stream;
        stream << "unexpected props state -- expected: "
          << sn_get_enum_string("SnPropsState", sits_.get().expected_props_state)
          << ", found: "
          << sn_get_enum_string("SnPropsState", snav_data.general_status.props_state)
          << "; disconnecting tx";
        print_error(stream.str());
        tx_ok_ = false;
      }
    }
    else
    {
      props_ok_time_last = std::chrono::steady_clock::now();
    }

    static auto pos_est_ok_time_last = std::chrono::steady_clock::now();
    if (sits_.get().strict_pos_est_type_checking
        && sits_.get().desired_pos_est_type
        != snav_data.pos_vel.position_estimate_type)
    {
      std::chrono::duration<float> elapsed = std::chrono::steady_clock::now()
        - pos_est_ok_time_last;
      const float kPosEstErrorTimeSec = 0.1;
      if (elapsed.count() > kPosEstErrorTimeSec)
      {
        std::stringstream stream;
        stream << "current position estimate type ("
          << sn_get_enum_string("SnPosEstType", snav_data.pos_vel.position_estimate_type)
          << ") does not match configured position estimate type ("
          << sn_get_enum_string("SnPosEstType", sits_.get().desired_pos_est_type)
          << "); disconnecting tx";
        print_error(stream.str());
        tx_ok_ = false;
      }
    }
    else
    {
      pos_est_ok_time_last = std::chrono::steady_clock::now();
    }

  }
  else
  {
    // rx is not ok
    tx_ok_ = false;
  }
}

int FlightControlInterface::infer_metadata_from_mode(const SnMode mode,
    const bool use_traj_tracking,
    TxCommand::Mode& tx_cmd_mode, SnInputCommandType& input_cmd_type,
    SnRcCommandType& rc_cmd_type)
{
  int ret_code = 0;

  if (use_traj_tracking && mode != SN_VIO_POS_HOLD_MODE)
  {
    print_error("use_traj_tracking is only supported in SN_VIO_POS_HOLD_MODE");
    return -1;
  }

  if (mode == SN_ESC_RPM_MODE)
  {
    tx_cmd_mode = TxCommand::Mode::RPM;
    input_cmd_type = SN_INPUT_CMD_TYPE_API_ESC;
  }
  else if (mode == SN_ESC_PWM_MODE)
  {
    tx_cmd_mode = TxCommand::Mode::PWM;
    input_cmd_type = SN_INPUT_CMD_TYPE_API_ESC;
  }
  else if (mode == SN_RATE_MODE)
  {
    tx_cmd_mode = TxCommand::Mode::RC;
    input_cmd_type = SN_INPUT_CMD_TYPE_RC;
    rc_cmd_type = SN_RC_RATES_CMD;
  }
  else if (mode == SN_THRUST_ANGLE_MODE)
  {
    tx_cmd_mode = TxCommand::Mode::RC;
    input_cmd_type = SN_INPUT_CMD_TYPE_RC;
    rc_cmd_type = SN_RC_THRUST_ANGLE_CMD;
  }
  else if (mode == SN_ALT_HOLD_MODE)
  {
    tx_cmd_mode = TxCommand::Mode::RC;
    input_cmd_type = SN_INPUT_CMD_TYPE_RC;
    rc_cmd_type = SN_RC_ALT_HOLD_CMD;
  }
  else if (mode == SN_THRUST_GPS_HOVER_MODE)
  {
    tx_cmd_mode = TxCommand::Mode::RC;
    input_cmd_type = SN_INPUT_CMD_TYPE_RC;
    rc_cmd_type = SN_RC_THRUST_ANGLE_GPS_HOVER_CMD;
  }
  else if (mode == SN_GPS_POS_HOLD_MODE)
  {
    tx_cmd_mode = TxCommand::Mode::RC;
    input_cmd_type = SN_INPUT_CMD_TYPE_RC;
    rc_cmd_type = SN_RC_GPS_POS_HOLD_CMD;
  }
  else if (mode == SN_OPTIC_FLOW_POS_HOLD_MODE)
  {
    tx_cmd_mode = TxCommand::Mode::RC;
    input_cmd_type = SN_INPUT_CMD_TYPE_RC;
    rc_cmd_type = SN_RC_OPTIC_FLOW_POS_HOLD_CMD;
  }
  else if (mode == SN_VIO_POS_HOLD_MODE)
  {
    if (use_traj_tracking)
    {
      tx_cmd_mode = TxCommand::Mode::TRAJECTORY;
      input_cmd_type = SN_INPUT_CMD_TYPE_API_TRAJECTORY_CONTROL;
    }
    else
    {
      tx_cmd_mode = TxCommand::Mode::RC;
      input_cmd_type = SN_INPUT_CMD_TYPE_RC;
      rc_cmd_type = SN_RC_VIO_POS_HOLD_CMD;
    }
  }
  else if (mode == SN_THRUST_ATT_ANG_VEL_MODE)
  {
    tx_cmd_mode = TxCommand::Mode::THRUST_ATT_ANG_VEL;
    input_cmd_type = SN_INPUT_CMD_TYPE_API_THRUST_ATT_ANG_VEL;
  }
  else if (mode == SN_ALT_HOLD_LOW_ANGLE_MODE)
  {
    tx_cmd_mode = TxCommand::Mode::RC;
    input_cmd_type = SN_INPUT_CMD_TYPE_RC;
    rc_cmd_type = SN_RC_ALT_HOLD_LOW_ANGLE_CMD;
  }
  else if (mode == SN_POS_HOLD_MODE)
  {
    tx_cmd_mode = TxCommand::Mode::RC;
    input_cmd_type = SN_INPUT_CMD_TYPE_RC;
    rc_cmd_type = SN_RC_POS_HOLD_CMD;
  }
  else
  {
    return -1;
  }


  return ret_code;
}

FlightControlInterface::Return FlightControlInterface::convert_velocity_to_rc_command(
    const std::array<float, 3> velocity, const float yaw_rate,
    RcCommand& rc_command)
{
  // velocity commands can only be set in closed-loop position control modes
  if (!has_closed_loop_position_control(rc_command.type))
  {
    return Return::NOT_SUPPORTED;
  }

  float yaw_est = scdts_.get().pos_vel.yaw_estimated;

  // Rotate desired velocity from ESTIMATION frame into BASE_LINK frame
  std::array<float, 3> des_vel_wrt_base_link;
  des_vel_wrt_base_link.fill(0);

  des_vel_wrt_base_link[0] = velocity[0]*cos(-yaw_est)
    - velocity[1]*sin(-yaw_est);
  des_vel_wrt_base_link[1] = velocity[0]*sin(-yaw_est)
    + velocity[1]*cos(-yaw_est);
  des_vel_wrt_base_link[2] = velocity[2];

  RcCommand rc;
  tcts_.get(rc);
  std::array<float, RcCommand::kNumRcCommands> min = {0, 0, 0, 0};
  std::array<float, RcCommand::kNumRcCommands> max = {0, 0, 0, 0};
  for (size_t ii = 0; ii < min.size(); ++ii)
  {
    min[ii] = sn_get_min_value(rc.type, ii);
    max[ii] = sn_get_max_value(rc.type, ii);
  }

  std::array<float, RcCommand::kNumRcCommands> input = {0, 0, 0, 0};
  input[0] = constrain_min_max(des_vel_wrt_base_link[0], min[0], max[0]);
  input[1] = constrain_min_max(des_vel_wrt_base_link[1], min[1], max[1]);
  input[2] = constrain_min_max(des_vel_wrt_base_link[2], min[2], max[2]);
  input[3] = constrain_min_max(yaw_rate, min[3], max[3]);

  float cmd0, cmd1, cmd2, cmd3;
  sn_apply_cmd_mapping(rc.type, rc.options,
      input[0], input[1], input[2], input[3], &cmd0, &cmd1, &cmd2, &cmd3);

  rc_command.commands[0] = cmd0;
  rc_command.commands[1] = cmd1;
  rc_command.commands[2] = cmd2;
  rc_command.commands[3] = cmd3;

  return Return::SUCCESS;
}

void FlightControlInterface::enforce_props_state_constraint(const SnPropsState props_state)
{
  StateInfo state_info = sits_.get();
  state_info.enforce_props_state = true;
  state_info.expected_props_state = props_state;
  sits_.set(state_info);
}

void FlightControlInterface::do_not_enforce_props_state_constraint()
{
  StateInfo state_info = sits_.get();
  state_info.enforce_props_state = false;
  sits_.set(state_info);
}

bool FlightControlInterface::has_closed_loop_position_control(SnRcCommandType
    rc_cmd_type)
{
  bool result = false;
  if (rc_cmd_type == SN_RC_POS_HOLD_CMD
      || rc_cmd_type == SN_RC_GPS_POS_HOLD_CMD
      || rc_cmd_type == SN_RC_OPTIC_FLOW_POS_HOLD_CMD
      || rc_cmd_type == SN_RC_VIO_POS_HOLD_CMD)
  {
    result = true;
  }
  return result;
}

void FlightControlInterface::print_error(const std::string& message)
{
  print_error("", message);
}

void FlightControlInterface::print_error(const std::string& origin,
    const std::string& message)
{
  if (verbose_)
  {
    std::stringstream stream;
    if (origin.size() > 0)
    {
      stream << origin << " -- " << message;
    }
    else
    {
      stream << message;
    }
    std::cerr << "\033[1;31m[ERROR] " << stream.str() << "\033[0m" << std::endl;
  }
}

void FlightControlInterface::print_info(const std::string& message)
{
  if (verbose_)
  {
    std::cout << "\033[1;32m[INFO] " << message << "\033[0m" << std::endl;
  }
}

FlightControlInterface::Return FlightControlInterface::basic_comm_checks(const std::string caller) const
{
  if (!initialized_)
  {
    print_error(caller, "object is not initialized");
    return Return::NOT_INITIALIZED;
  }

  if (permissions_ != Permissions::READ_WRITE)
  {
    print_error(caller, "object must have READ_WRITE access");
    return Return::NO_WRITE_ACCESS;
  }

  if (!connected_)
  {
    print_error(caller, "object is not connected");
    return Return::NOT_CONNECTED;
  }

  return Return::SUCCESS;
}

float FlightControlInterface::constrain_min_max(float input,
  const float min, const float max)
{
  if( input < min )
    return min;
  else if( input > max )
    return max;
  return input;
}

int FlightControlInterface::update_tx_command_pos_ctrl(const std::string& caller)
{
  int result = -1;

  TxCommand::Mode tx_mode = tcts_.get_tx_command_mode();
  if (tx_mode == TxCommand::Mode::RC)
  {
    RcCommand rc_cmd;
    tcts_.get(rc_cmd);
    std::array<float, 3> des_vel_wrt_e = {
        desired_setpoint_wrt_e_.velocity[0],
        desired_setpoint_wrt_e_.velocity[1],
        desired_setpoint_wrt_e_.velocity[2] };
    float des_yaw_rate = desired_setpoint_wrt_e_.yaw_rate;
    if (convert_velocity_to_rc_command(des_vel_wrt_e, des_yaw_rate,
          rc_cmd) == Return::SUCCESS)
    {
      tcts_.set(rc_cmd);
      result = 0;
    }
  }
  else if (tx_mode == TxCommand::Mode::TRAJECTORY)
  {
    StateVector desired_state_wrt_v
      = tf_ev_.get_transform().inverse() * desired_setpoint_wrt_e_;
    TrajectoryCommand traj_cmd(desired_state_wrt_v);
    tcts_.set(traj_cmd);
    result = 0;
  }

  if (result != 0)
  {
    std::stringstream stream;
    stream << "function is only supported for closed-loop position control modes, and "
      << sn_get_enum_string("SnMode", sits_.get().desired_mode)
      << " does not qualify";
    print_error(caller, stream.str());
  }

  return result;
}

} // namespace snav_fci

