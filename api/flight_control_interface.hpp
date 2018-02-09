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


#ifndef SNAV_FCI_FLIGHT_CONTROL_INTERFACE_HPP_
#define SNAV_FCI_FLIGHT_CONTROL_INTERFACE_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdbool>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Geometry>

#include "snav/snapdragon_navigator.h"

#include "api/waypoint.hpp"
#include "api/config/landing_config.hpp"
#include "api/config/rx_config.hpp"
#include "api/config/takeoff_config.hpp"
#include "api/config/tx_config.hpp"
#include "api/config/waypoint_config.hpp"
#include "imp/inc/snav_cached_data_thread_safe.hpp"
#include "imp/inc/state_info.hpp"
#include "imp/inc/planner.hpp"
#include "imp/inc/transform.hpp"
#include "imp/inc/tx_command_thread_safe.hpp"

/**
 * @brief Namespace used for the Snapdragon Navigator Flight Control Interface
 * project
 */
namespace snav_fci
{

/**
 *
 * @brief Interface to the Snapdragon Navigator Flight Controller
 *
 * FlightControlInterface is a thread-safe C++ wrapper around the Snapdragon
 * Navigator API that makes it easier to accomplish some high-level actions
 * like taking off and going to waypoints. It handles basic error checking so
 * that the developer does not have to implement as much logic to check aspects
 * of the flight controller's state during runtime, leading to safer API
 * programs.
 */
class FlightControlInterface
{
public:
  /**
   * @brief Defines the permissions of an instance of FlightControlInterface
   *
   * An instance with READ_WRITE permissions can both send commands to and
   * receive data from Snapdragon Navigator, whereas an instance with READ_ONLY
   * permissions can only receive data from Snapdragon Navigator.
   *
   * Only one instance of FlightControlInterface in a given process can obtain
   * READ_WRITE permissions. This is intended to be a safety mechanism, since
   * commands should only come from a single source.
   *
   * @warning This mechanism only works in a single process! A second instance
   * of FlightControlInterface instantiated in a separate process can obtain
   * READ_WRITE permissions.
   */
  enum class Permissions
  {
    NONE,       /**< No permissions; used for an uninitialized object */
    READ_WRITE, /**< Read and write permissions; capable of both sending
                  commands to and receiving information from Snapdragon
                  Navigator */
    READ_ONLY,  /**< Read-only permissions; capable of receiving
                  information from Snapdragon Navigator, but not capable of
                  sending any commands */
  };

  /**
   * @brief Return codes for FlightControlInterface
   *
   * These return codes define whether or not a function executed successfully
   * and, in the event of failure, why the failure occurred.
   */
  enum class Return
  {
    SUCCESS, /**< Action was completed successfully */
    FAILURE, /**< Action was not completed successfully */
    NOT_INITIALIZED, /**< Action was not completed successfully because object
                       is not initialized */
    ALREADY_INITIALIZED, /**< Action was not completed successfully because
                           object is already initialized */
    NOT_CONNECTED, /**< Action was not completed successfully because object
                     is not connected */
    ALREADY_CONNECTED, /**< Action was not completed successfully because
                         object is already connected */
    NO_WRITE_ACCESS, /**< Action was not completed successfully because object
                       does not have write access */
    WRITE_ACCESS_TAKEN, /**< Action was not completed successfully because
                          write access has already been taken by another
                          instance */
    INVALID_CONFIG, /**< Action was not completed successfully because given
                      configuration is not valid */
    NOT_IN_FLIGHT, /**< Action was not completed successfully because vehicle
                     is not in flight */
    IN_FLIGHT, /**< Action was not completed successfully because vehicle is
                 already in flight */
    CONNECTION_LOST, /**< Action was not completed successfully because the
                       connection to Snapdragon Navigator was lost */
    SNAV_CACHED_DATA_ERROR, /**< Action was not completed successfully because
                              the SnavCachedData structure could not be
                              properly initialized */
    NOT_SUPPORTED, /**< Action was not completed successfully because it is not
                     supported given the current configuration */
    PROP_START_FAILURE, /**< Action was not completed successfully because the
                          propellers failed to start */
    PROP_STOP_FAILURE, /**< Action was not completed successfully because the
                         propellers failed to stop */
    IMPROPER_TX_COMMAND_MODE, /**< Action was not completed successfully
                                because the configured tx command mode is
                                not correct for the requested action */
  };

  /**
   * @brief Constructs an instance of FlightControlInterface
   */
  FlightControlInterface();

  /**
   * @brief Destructor of FlightControlInterface
   *
   * Before object destruction, disconnect() is called if it was not called
   * previously.
   */
  ~FlightControlInterface();

  /**
   * @brief Initialize FlightControlInterface object
   *
   * @param[in] requested_permissions Specifies desired permissions of this
   * instance. Only one FlightControlInterface instance can be granted
   * write permissions.
   *
   * @return FlightControlInterface::Return
   *
   * @note Should be called immediately after object construction.
   */
  Return initialize(const Permissions requested_permissions);

  /**
   * @brief Specify configuration options for sending commands to Snapdragon
   * Navigator
   *
   * If this function is not called, default options are used.
   *
   * @param[in] tx_config Defines options used in sending commands
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   *   - If this function is called, it must be done prior to calling
   *     connect().
   */
  Return configure_tx(const TxConfig& tx_config);

  /**
   * @brief Specify configuration options for receiving data from Snapdragon
   * Navigator
   *
   * If this function is not called, default options are used.
   *
   * @param[in] rx_config Defines options used in receiving data
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   *   - If this function is called, it must be done prior to calling
   *     connect().
   */
  Return configure_rx(const RxConfig& rx_config);

  /**
   * @brief Block until FlightControlInterface has been configured
   *
   * This function can be used in a multithreaded architecture to synchronize
   * multiple FlightControlInterface objects before calling connect() to ensure
   * that the object with Permissions::READ_WRITE has an opportunity to set
   * the desired configuration. This is necessary because it would otherwise be
   * possible for an object with Permissions::READ_ONLY to spawn the rx thread
   * prior to the rx configuration being set from a separate thread.
   */
  void wait_for_configure();

  /**
   * @brief Establish connection to Snapdragon Navigator
   *
   * Opens a connection between the FlightControlInterface object and
   * Snapdragon Navigator.
   *
   * @return FlightControlInterface::Return
   *
   * @note Must call initialize() prior to calling this function.
   */
  Return connect();

  /**
   * @brief End connection to Snapdragon Navigator
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   *   - Must call connect() prior to calling this function.
   */
  Return disconnect();

  /**
   * @brief Blocking call to start the propellers
   *
   * In general, favor calling takeoff(), which automatically calls
   * start_props() at the appropriate time, instead of manually calling this
   * function.
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   *   - Object must have permissions set to Permissions::READ_WRITE.
   *   - Must call connect() prior to calling this function.
   */
  Return start_props();

  /**
   * @brief Blocking call to stop the propellers
   *
   * In general, favor calling land(), which automatically calls stop_props()
   * at the appropriate time, instead of manually calling this function.
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   *   - Object must have permissions set to Permissions::READ_WRITE.
   *   - Must call connect() prior to calling this function.
   */
  Return stop_props();

  /**
   * @brief Blocking call to takeoff
   *
   * This function attempts to spin the propellers via start_props() and to
   * take off to a specified height above the ground.
   *
   * @param[in] takeoff_config Specifies parameters for takeoff
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   *   - Object must have permissions set to Permissions::READ_WRITE.
   *   - Must call connect() prior to calling this function.
   *   - Propellers must not be spinning when calling this function.
   */
  Return takeoff(const TakeoffConfig& takeoff_config);

  /**
   * @overload
   */
  Return takeoff();

  /**
   * @brief Blocking call to land
   *
   * This function commands the vehicle to descend until reaching the
   * ground, at which point it attempts to stop the propellers via
   * stop_props().
   *
   * @param[in] landing_config Specifies parameters for landing
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   *   - Object must have permissions set to Permissions::READ_WRITE.
   *   - Must call connect() prior to calling this function.
   */
  Return land(const LandingConfig& landing_config);

  /**
   * @overload
   */
  Return land();

  /**
   * @brief Blocking call to go to specified waypoint
   *
   * @param[in] waypoint Specifies the destination waypoint represented with
   * respect to ReferenceFrame::WAYPOINT
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   *   - Object must have permissions set to Permissions::READ_WRITE.
   *   - Must call connect() prior to calling this function.
   */
  Return go_to_waypoint(const Waypoint& waypoint);

  /**
   * @brief Updates the transform between ReferenceFrame::WAYPOINT and its
   * parent
   *
   * FlightControlInterface expects waypoints to be given with respect to
   * ReferenceFrame::WAYPOINT. The transformation between
   * ReferenceFrame::WAYPOINT and its parent, defined by
   * TxConfig::waypoint_frame_parent, is initialized to identity rotation and
   * zero translation.
   *
   * @param[in] q_pw Quaternion representing the orientation of
   * ReferenceFrame::WAYPOINT with respect to its parent
   *
   * @param[in] t_pw Vector from the origin of the parent frame to the origin
   * of ReferenceFrame::WAYPOINT represented with respect to the parent frame
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   */
  Return set_waypoint_frame_tf(const Eigen::Quaternionf& q_pw,
      const Eigen::Vector3f& t_pw);

  /**
   * @brief Get the transform between ReferenceFrame::WAYPOINT and its parent
   *
   * @param[out] q_pq
   *   Quaternion representing the orientation of ReferenceFrame::WAYPOINT
   *   with respect to its parent
   *
   * @param[out] t_pw
   *   Vector from the origin of the parent frame to the origin of
   *   ReferenceFrame::WAYPOINT represented with respect to the parent frame
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   */
  Return get_waypoint_frame_tf(Eigen::Quaternionf& q_pw,
      Eigen::Vector3f& t_pw) const;

  /**
   * @brief Directly set the command being sent to the flight controller
   *
   * This function sets the command being sent to the flight controller in
   * the tx thread. Calling this function will only have an impact if the
   * tx thread is configured for sending this type of command.
   *
   * @param[in] command Command to be sent to the flight controller
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   *   - Object must have permissions set to Permissions::READ_WRITE.
   *   - Must call connect() prior to calling this function.
   *   - Tx thread must be configured for sending this type of command; refer
   *     to TxConfig for options.
   */
  Return set_tx_command(const TxCommand& command);

  /**
   * @brief Get a copy of the latest data from the cache.
   *
   * @param[out] data
   *   Structure to be filled with a copy of the latest cached data
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   *   - Must call connect() prior to calling this function.
   */
  Return get_snav_cached_data(SnavCachedData& data) const;

  /**
   * @brief Get the estimated state as a StateVector with respect to
   * ReferenceFrame::WAYPOINT from the SnavCachedData structure
   *
   * @param[in] snav_data
   *   Structure containing data from the cache
   * @param[out] state
   *   Estimated state vector represented with respect to
   *   ReferenceFrame::WAYPOINT
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   */
  Return get_estimated_state(const SnavCachedData& snav_data,
      StateVector& state) const;

  /**
   * @brief Get the desired state as a StateVector with respect to
   * ReferenceFrame::WAYPOINT from the SnavCachedData structure
   *
   * @param[in] snav_data
   *   Structure containing data from the cache
   * @param[out] state
   *   Desired state vector represented with respect to
   *   ReferenceFrame::WAYPOINT
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   */
  Return get_desired_state(const SnavCachedData& snav_data,
      StateVector& state) const;

  /**
   * @brief Set verbosity of FlightControlInterface
   *
   * By default, verbosity of FlightControlInterface is set to
   * true. When true, FlightControlInterface emits informational
   * messages to std::out and error messages to std::cerr.
   *
   * @param[in] verbose
   *   Desired verbosity of FlightControlInterface
   */
  inline static void set_verbosity(bool verbose)
  {
    verbose_ = verbose;
  }

  /**
   * @brief Whether or not write access to Snapdragon Navigator is ok
   */
  inline static bool tx_ok()
  {
    return tx_ok_;
  }

  /**
   * @brief Whether or not read access to Snapdragon Navigator is ok
   */
  inline static bool rx_ok()
  {
    return rx_ok_;
  }

  /**
   * @brief Whether or not read and write access to Snapdragon Navigator is ok
   */
  inline static bool ok()
  {
    return rx_ok_ && tx_ok_;
  }

  /**
   * @brief Convert metric velocity command into an RcCommand suitable for
   * sending to the flight controller
   *
   * @param[in] velocity
   *  Desired velocity in meters per second with respect to the estimation
   *  frame
   * @param[in] yaw_rate
   *  Desired yaw rate in radians per second with respect to the estimation
   *  frame
   * @param[out] rc_command
   *  Resulting rc command ready to be sent to the flight controller
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Must call initialize() prior to calling this function.
   */
  static Return convert_velocity_to_rc_command(const std::array<float, 3> velocity,
      const float yaw_rate, RcCommand& rc_command);

private:

  /**
   * Private static member functions
   */
  static void tx_loop();
  static void update_waypoint_frame_parent_transform();
  static void send_tx_command();
  static void rx_loop();
  static void check_tx_conditions();
  static float constrain_min_max(float input, const float min, const float max);
  static bool has_closed_loop_position_control(SnRcCommandType);
  static void print_error(const std::string& message);
  static void print_error(const std::string& origin, const std::string& message);
  static void print_info(const std::string& message);

  /**
   * Private non-static member functions
   */
  int infer_metadata_from_mode(const SnMode, const bool, TxCommand::Mode&, SnInputCommandType&,
      SnRcCommandType&);
  void enforce_props_state_constraint(const SnPropsState);
  void do_not_enforce_props_state_constraint();
  Return basic_comm_checks(const std::string) const;
  int update_tx_command_pos_ctrl(const std::string& caller);

  /**
   * Private static member variables
   */
  static std::mutex connection_mutex_;
  static std::mutex command_mutex_;
  static TxConfig tx_config_;
  static bool configured_tx_;
  static RxConfig rx_config_;
  static bool configured_rx_;
  static std::condition_variable configured_rx_cv_;
  static TxCommandThreadSafe tcts_;
  static TransformThreadSafe tf_ep_;  // ESTIMATION <-> (parent of WAYPOINT)
  static TransformThreadSafe tf_pw_;  // (parent of WAYPOINT) <-> WAYPOINT
  static TransformThreadSafe tf_ev_;  // ESTIMATION <-> snav VIO frame
  static StateVector desired_setpoint_wrt_e_; // Desired state w.r.t. ESTIMATION
  static StateVector desired_setpoint_wrt_w_; // Desired state w.r.t. WAYPOINT
  static SnavCachedDataThreadSafe scdts_;
  static StateInfoThreadSafe sits_;
  static std::thread tx_thread_;
  static std::atomic_bool tx_ok_;
  static std::thread rx_thread_;
  static std::atomic_bool rx_ok_;
  static bool write_access_taken_;
  static int num_connections_;
  static std::atomic_bool verbose_;

  /**
   * Private non-static member variables
   */
  std::atomic_bool initialized_;
  std::atomic_bool connected_;
  Permissions permissions_;
  Planner planner_; // TODO: make static?

};

} // namespace snav_fci

#endif // SNAV_FCI_FLIGHT_CONTROL_INTERFACE_HPP_

