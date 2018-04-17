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
 * @brief Namespace used for the Qualcomm Navigator Flight Control Interface
 * project
 */
namespace snav_fci
{

/**
 *
 * @brief Interface to the Qualcomm Navigator Flight Controller
 *
 * FlightControlInterface is a thread-safe C++ wrapper around the Qualcomm
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
   * receive data from Qualcomm Navigator, whereas an instance with READ_ONLY
   * permissions can only receive data from Qualcomm Navigator.
   *
   * Only one instance of FlightControlInterface in a given process can obtain
   * READ_WRITE permissions. This is intended to be a safety mechanism, since
   * commands should only come from a single source.
   *
   * @warning This mechanism only works in a single process! A second instance
   * of FlightControlInterface instantiated in a separate process can obtain
   * READ_WRITE permissions, so it is ultimately up to the client to ensure
   * that only one object sends commands. Multiple objects sending commands
   * simultaneously results in undefined behavior.
   */
  enum class Permissions
  {
    READ_WRITE, /**< Read and write permissions; capable of both sending
                  commands to and receiving information from Qualcomm
                  Navigator */
    READ_ONLY,  /**< Read-only permissions; capable of receiving
                  information from Qualcomm Navigator, but not capable of
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
    NOT_CONNECTED, /**< Action was not completed successfully because object
                     is not connected */
    ALREADY_CONNECTED, /**< Action was not completed successfully because
                         object is already connected */
    TRAJ_NOT_OPTIMIZED, /**< Action was not completed successfully because the
                     trajectory was not optimized */
    TRAJ_NOT_FEASIBLE, /**< Action was not completed successfully because a
                       feasible trajectory satisfying the required constraints
                       could not be generated */
    ACTION_PREEMPTED, /**< Action was not completed successfully because it was
                        preempted */
    NO_LOCK, /**< Action was not completed successfully because the necessary
               mutex lock could not be acquired */
  };

  /**
   * @brief Enum used for possible actions
   */
  enum class Action
  {
    NONE, /**< No action */
    TAKEOFF, /**< Vehicle is taking off */
    GO_TO_WAYPOINT, /**< Vehicle is going to a waypoint */
    EXECUTE_MISSION, /**< Vehicle is executing a mission */
    LAND, /**< Vehicle is landing */
  };

  /**
   * @brief Constructs an instance of FlightControlInterface
   *
   * @pre Flight controller is running
   *
   * @post FlightControlInterface object initialized (but not connected) with
   *   requested permissions
   *
   * @param[in] requested_permissions Specifies desired permissions of this
   *   instance. Only one FlightControlInterface instance can be granted
   *   write permissions.
   *
   * @throw std::runtime_error If unable to initialize SnavCachedData (usually
   *   means that flight controller is not responding)
   * @throw std::logic_error If requested Permissions::READ_WRITE but an
   *   object with write permissions has already been created
   */
  explicit FlightControlInterface(const Permissions& requested_access);

  /**
   * @brief Destructor of FlightControlInterface
   *
   * Before object destruction, disconnect() is called if it was not called
   * previously.
   */
  ~FlightControlInterface() noexcept;

  /**
   * @brief Specify configuration options for sending commands to Qualcomm
   * Navigator
   *
   * If this function is not called, default options are used.
   *
   * @pre Object has Permissions::READ_WRITE
   * @pre Object has not been connected via connect()
   *
   * @post Object is configured to use given tx_config
   *
   * @param[in] tx_config Defines options used in sending commands
   *
   * @return FlightControlInterface::Return
   *
   * @throw std::logic_error If any precondition is violated
   * @throw std::runtime_error If given config is impossible to achieve
   */
  Return configure_tx(const TxConfig& tx_config);

  /**
   * @brief Specify configuration options for receiving data from Qualcomm
   * Navigator
   *
   * If this function is not called, default options are used.
   *
   * @pre Object has not been connected via connect()
   *
   * @post Object is configured to use given rx_config
   *
   * @param[in] rx_config Defines options used in receiving data
   *
   * @return FlightControlInterface::Return
   *
   * @throw std::logic_error If any precondition is violated
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
   *
   * @pre None
   *
   * @post Rx thread has been configured
   */
  void wait_for_configure() const noexcept;

  /**
   * @brief Establish connection to Qualcomm Navigator
   *
   * Opens a persistent connection between the FlightControlInterface object
   * and Qualcomm Navigator.
   *
   * @pre None
   *
   * @post Function result is returned
   *
   * @return FlightControlInterface::Return
   */
  Return connect();

  /**
   * @brief End connection to Qualcomm Navigator
   *
   * @return FlightControlInterface::Return
   */
  Return disconnect() noexcept;

  /**
   * @brief Blocking call to start the propellers
   *
   * In general, favor calling takeoff(), which automatically calls
   * start_props() at the appropriate time, instead of manually calling this
   * function.
   *
   * @pre Object has Permissions::READ_WRITE
   * @pre Object has been connected via connect()
   *
   * @post Propellers are spinning according to flight controller
   *
   * @return FlightControlInterface::Return
   *
   * @throw std::logic_error If any precondition is violated
   * @throw std::runtime_error If tx/rx connection is lost
   * @throw std::runtime_error If unable to start props
   */
  Return start_props();

  /**
   * @brief Blocking call to stop the propellers
   *
   * In general, favor calling land(), which automatically calls stop_props()
   * at the appropriate time, instead of manually calling this function.
   *
   * @pre Object has Permissions::READ_WRITE
   * @pre Object has been connected via connect()
   *
   * @post Propellers are not spinning according to flight controller
   *
   * @return FlightControlInterface::Return
   *
   * @throw std::logic_error If any precondition is violated
   * @throw std::runtime_error If tx/rx connection is lost
   * @throw std::runtime_error If unable to stop props
   */
  Return stop_props();

  /**
   * @brief Blocking call to takeoff
   *
   * This function attempts to spin the propellers via start_props() and to
   * take off to a specified height above the ground.
   *
   * @pre Object has Permissions::READ_WRITE
   * @pre Object has been connected via connect()
   * @pre Propellers are not spinning
   * @pre No action is in progress
   *
   * @post Action::TAKEOFF is concluded due to either completion or preemption
   *
   * @param[in] takeoff_config Specifies parameters for takeoff
   *
   * @return FlightControlInterface::Return
   *
   * @throw std::logic_error If any precondition is violated
   * @throw std::runtime_error If tx/rx connection is lost
   *
   * @note
   *   - Can be safely preempted by calling preempt_current_action().
   */
  Return takeoff(const TakeoffConfig& takeoff_config);

  /**
   * @overload
   */
  Return takeoff();


  /**
   * @brief Non-blocking call to takeoff
   *
   * This is a non-blocking version of takeoff().
   *
   * @pre See takeoff()
   *
   * @post Action::TAKEOFF has been initiated
   *
   * @note
   *   - Can be safely preempted by calling preempt_current_action().
   */
  Return takeoff_nb(const TakeoffConfig& takeoff_config);

  /**
   * @overload
   */
  Return takeoff_nb();

  /**
   * @brief Blocking call to land
   *
   * This function commands the vehicle to descend until reaching the
   * ground, at which point it attempts to stop the propellers via
   * stop_props().
   *
   * @pre Object has Permissions::READ_WRITE
   * @pre Object has been connected via connect()
   * @pre Propellers are spinning
   * @pre No action is in progress
   *
   * @post Action::LAND is concluded due to either completion or preemption
   *
   * @param[in] landing_config Specifies parameters for landing
   *
   * @return FlightControlInterface::Return
   *
   * @throw std::logic_error If any precondition is violated
   * @throw std::runtime_error If tx/rx connection is lost
   *
   * @note
   *   - Can be safely preempted by calling preempt_current_action().
   */
  Return land(const LandingConfig& landing_config);

  /**
   * @overload
   */
  Return land();

  /**
   * @brief Non-blocking call to land
   *
   * This is a non-blocking version of land().
   *
   * @pre See land()
   *
   * @post Action::LAND has been initiated
   *
   * @note
   *   - Can be safely preempted by calling preempt_current_action().
   */
  Return land_nb(const LandingConfig& landing_config);

  /**
   * @overload
   */
  Return land_nb();

  /**
   * @brief Blocking call to go to specified waypoint
   *
   * @pre Object has Permissions::READ_WRITE
   * @pre Object has been connected via connect()
   * @pre Propellers are spinning
   * @pre No action is in progress
   *
   * @post Action::GO_TO_WAYPOINT is concluded due to either completion or preemption
   *
   * @param[in] waypoint Specifies the destination waypoint represented with
   * respect to ReferenceFrame::WAYPOINT
   *
   * @return FlightControlInterface::Return
   *
   * @throw std::logic_error If any precondition is violated
   * @throw std::runtime_error If tx/rx connection is lost
   *
   * @note
   *   - Can be safely preempted by calling preempt_current_action().
   */
  Return go_to_waypoint(const Waypoint& waypoint);

  /**
   * @brief Non-blocking call to go to specified waypoint
   *
   * This is a non-blocking version of go_to_waypoint().
   *
   * @pre See go_to_waypoint()
   *
   * @post Action::GO_TO_WAYPOINT has been initiated
   *
   * @note
   *   - Can be safely preempted by calling preempt_current_action().
   */
  Return go_to_waypoint_nb(const Waypoint& waypoint);

  /**
   * @brief Specify the configuration options for the planner
   *
   * If this function is not called, default options are used.
   *
   * @pre None
   *
   * @post Function result is returned
   *
   * @param[in] config Defines options used by the planner
   *
   * @return FlightControlInterface::Return
   */
  Return configure_planner(const PlannerConfig& config);

  /**
   * @brief Load a vector of waypoints
   *
   * @pre Object has Permissions::READ_WRITE
   * @pre Object has been connected via connect()
   * @pre Planner status is IDLE
   *
   * @post Waypoints have been added to planner
   *
   * @param[in] waypoints Waypoints to pass to planner.
   *
   * @return FlightControlInterface::Return
   *
   * @throw std::logic_error If any precondition is violated
   */
  Return preload_waypoints(const std::vector<Waypoint>& waypoints);

  /**
   * @brief Compute a trajectory through the given waypoints
   *
   * @pre Object has Permissions::READ_WRITE
   * @pre Object has been connected via connect()
   * @pre At least one waypoint has been added via preload_waypoints()
   *
   * @post Function result is returned
   *
   * @param[in] starting_state State represented with respect to
   *   ReferenceFrame::WAYPOINT used as first constraint of trajectory
   *
   * @return FlightControlInterface::Return
   *
   * @throw std::logic_error If any precondition is violated
   * @throw std::logic_error If waypoint timestamps are not monotonically
   *   increasing
   * @throw std::runtime_error If trajectory generation fails
   */
  Return compute_trajectory(StateVector starting_state);

  /**
   * @brief Execute the trajectory-following mission
   *
   * Causes the vehicle to execute the planned trajectory through the given
   * waypoints. If no trajectory has been planned, one will be generated and
   * consequently executed.
   *
   * @pre Object has Permissions::READ_WRITE
   * @pre Object has been connected via connect()
   * @pre Propellers are spinning
   * @pre No action is in progress
   * @pre At least one waypoint has been added via preload_waypoints()
   *
   * @post Action::EXECUTE_MISSION is concluded due to either completion or preemption
   *
   * @param[in] t_start Optional parameter defining the absolute start time
   *   for the mission in seconds according to the system-wide real time wall
   *   clock. Set t_start equal to zero for the mission to start immediately.
   *   If t_start is not zero, it must specify a moment in time in the future.
   *   t_start is used to establish time zero for "trajectory time," accessible
   *   via get_trajectory_time().
   *
   * @return FlightControlInterface::Return
   *
   * @throw std::logic_error If any precondition is violated
   * @throw std::logic_error If waypoint timestamps are not monotonically
   *   increasing
   * @throw std::runtime_error If trajectory generation fails
   * @throw std::runtime_error If tx/rx connection is lost
   * @throw std::invalid_argument If t_start is negative or corresponds to a
   *   time in the past
   *
   * @note
   *   - Can be safely preempted by calling preempt_current_action().
   */
  Return execute_mission(const double t_start = 0);

  /**
   * @brief Non-blocking call to execute the trajectory-following mission
   *
   * This is a non-blocking version of execute_mission().
   *
   * @pre See execute_mission()
   *
   * @post Action::EXECUTE_MISSION has been initiated
   *
   * @note
   *   - Can be safely preempted by calling preempt_current_action().
   */
  Return execute_mission_nb(const double t_start = 0);

  /**
   * @brief Wait for the currently-executing action to complete
   *
   * Blocks until the currently-executing action completes. The result of the
   * action can be obtained via get_last_action_result().
   *
   * @pre None
   *
   * @post Currently-executing action is complete
   */
  void wait_on_action() const noexcept;

  /**
   * @brief Return the permissions of this object
   *
   * @pre None
   *
   * @post Permissions are returned
   *
   * @return Permissions
   */
  Permissions get_permissions() const noexcept;

  /**
   * @brief Get the result of the most recently executed action
   *
   * @pre None
   *
   * @post Last action result is returned
   *
   * @return FlightControlInterface::Return
   */
  Return get_last_action_result() const noexcept;

  /**
   * @brief Get the most recently executed action
   *
   * @pre None
   *
   * @post Last action is returned
   *
   * @return FlightControlInterface::Action
   */
  Action get_last_action() const noexcept;

  /**
   * @brief Get the currently-executing action
   *
   * @pre None
   *
   * @post Current action is returned
   *
   * @return FlightControlInterface::Action
   */
  Action get_current_action() const noexcept;

  /**
   * @brief Get the current trajectory
   *
   * @pre None
   *
   * @post Function result is returned
   *
   * @param[out] traj The current trajectory, possibly composed of multiple
   *   segments each defined by a SnavTrajectory object, which has been
   *   optimized and satisfies the constraints enforced by the Planner. This is
   *   valid only if function returns Return::SUCCESS
   *
   * @return FlightControlInterface::Return
   */
  Return get_current_trajectory(std::vector<snav_traj_gen::SnavTrajectory>& traj) const;

  /**
   * @brief Get the last optimized trajectory
   *
   * The last optimized trajectory may be different from the current trajectory
   * if the last optimized trajectory failed to satisfy the constraints
   * enforced by the Planner.
   *
   * @pre None
   *
   * @post Function result is returned
   *
   * @param[out] traj The last optimized trajectory, possibly composed of
   *   multiple segments each defined by a SnavTrajectory object,  which has
   *   been optimized but does not necessarily satisfy the constraints enforced
   *   by the Planner. Only valid if function returns Return::SUCCESS.
   *
   * @return FlightControlInterface::Return
   */
  Return get_last_optimized_trajectory(std::vector<snav_traj_gen::SnavTrajectory>& traj) const;

  /**
   * @brief Get the input waypoints
   *
   * @pre None
   *
   * @post Waypoints are returned
   *
   * @param[out] waypoints Vector of input waypoints used by planner
   *
   * @return FlightControlInterface::Return
   *
   * @note
   *   - Use the preload_waypoints() function to add input waypoints
   */
  Return get_waypoints(std::vector<Waypoint>& waypoints) const;

  /**
   * @brief Get the optimized waypoints
   *
   * The optimized waypoints are different from the input waypoints in that
   * they are an augmented set of the input waypoints.  For example, the
   * optimized waypoints contain at least one extra waypoint prepended to the
   * start of the vector containing the current state. Further, they contain
   * the higher derivatives computed from the optimization.
   *
   * @pre None
   *
   * @post Optimized waypoints are returned
   *
   * @param[out] waypoints Vector of optimized waypoints used by planner
   *
   * @return FlightControlInterface::Return
   */
  Return get_optimized_waypoints(std::vector<Waypoint>& waypoints) const;

  /**
   * @brief Get the current status of the Planner
   *
   * @pre None
   *
   * @post Planner status is returned
   *
   * @return Planner::Status Current status of the planner
   */
  Planner::Status get_planner_status() const;

  /**
   * @brief Get the current trajectory time
   *
   * Trajectory time is relative to the current trajectory. The start of the
   * trajectory corresponds to time zero.
   *
   * @pre None
   *
   * @post Trajectory time is returned
   *
   * @return float Trajectory time in s
   */
  float get_trajectory_time() const;

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
   * @pre None
   *
   * @post Transform is updated
   *
   * @param[in] q_pw Quaternion representing the orientation of
   * ReferenceFrame::WAYPOINT with respect to its parent
   *
   * @param[in] t_pw Vector from the origin of the parent frame to the origin
   * of ReferenceFrame::WAYPOINT represented with respect to the parent frame
   *
   * @return FlightControlInterface::Return
   */
  Return set_waypoint_frame_tf(const Eigen::Quaternionf& q_pw,
      const Eigen::Vector3f& t_pw);

  /**
   * @brief Get the transform between ReferenceFrame::WAYPOINT and its parent
   *
   * @pre None
   *
   * @post ReferenceFrame::WAYPOINT transform is returned
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
   * @pre Object has Permissions::READ_WRITE
   * @pre Object has been connected via connect()
   * @pre Object has been configured for sending this type of command; refer
   *   to TxConfig for options
   *
   * @param[in] command Command to be sent to the flight controller
   *
   * @return FlightControlInterface::Return
   *
   * @throw std::logic_error If any precondition is violated
   */
  Return set_tx_command(const TxCommand& command);

  /**
   * @brief Get a copy of the latest data from the cache.
   *
   * @pre Object has been connected via connect()
   *
   * @post SnavCachedData is returned
   *
   * @return Latest cached data
   *
   * @throw std::logic_error If any precondition is violated
   */
  SnavCachedData get_snav_cached_data() const;

  /**
   * @brief Get the estimated state as a StateVector with respect to
   * ReferenceFrame::WAYPOINT from the SnavCachedData structure
   *
   * @pre None
   *
   * @post Estimated state is returned
   *
   * @param[in] snav_data
   *   Structure containing data from the cache
   *
   * @return Estimated state vector represented with respect to
   *   ReferenceFrame::WAYPOINT
   */
  StateVector get_estimated_state(const SnavCachedData& snav_data) const;

  /**
   * @brief Get the desired state as a StateVector with respect to
   * ReferenceFrame::WAYPOINT from the SnavCachedData structure
   *
   * @pre None
   *
   * @post Desired state is returned
   *
   * @param[in] snav_data
   *   Structure containing data from the cache
   *
   * @return Desired state vector represented with respect to
   *   ReferenceFrame::WAYPOINT
   */
  StateVector get_desired_state(const SnavCachedData& snav_data) const;

  /**
   * @brief Convert metric velocity command into an RcCommand suitable for
   * sending to the flight controller based on the configured mode
   *
   * This function only works for closed-loop position control modes.
   *
   * @pre Object has been connected via connect()
   * @pre Object is configured for a closed-loop position control mode
   *
   * @post rc_command is populated with appropriate commands
   *
   * @param[in] velocity
   *  Desired velocity in meters per second with respect to
   *  ReferenceFrame::ESTIMATION
   * @param[in] yaw_rate
   *  Desired yaw rate in radians per second with respect to
   *  ReferenceFrame::ESTIMATION
   * @param[out] rc_command
   *  Resulting rc command ready to be sent to the flight controller
   *
   * @return FlightControlInterface::Return
   *
   * @throw std::logic_error If any precondition is violated
   */
  Return convert_velocity_to_rc_command(const std::array<float, 3> velocity,
      const float yaw_rate, RcCommand& rc_command);

  /**
   * @brief Send preempt signal to currently-executing action
   */
  static void preempt_current_action();

  /**
   * @brief Set verbosity of FlightControlInterface
   *
   * By default, verbosity of FlightControlInterface is set to
   * true. When true, FlightControlInterface emits informational
   * messages to std::out and error messages to std::cerr.
   *
   * @pre None
   *
   * @post Verbosity is set to value of verbose
   *
   * @param[in] verbose
   *   Desired verbosity of FlightControlInterface
   */
  inline static void set_verbosity(bool verbose)
  {
    verbose_ = verbose;
  }

  /**
   * @brief Whether or not write access to Qualcomm Navigator is ok
   */
  inline static bool tx_ok()
  {
    return tx_ok_;
  }

  /**
   * @brief Whether or not read access to Qualcomm Navigator is ok
   */
  inline static bool rx_ok()
  {
    return rx_ok_;
  }

  /**
   * @brief Whether or not read and write access to Qualcomm Navigator is ok
   */
  inline static bool ok()
  {
    return rx_ok_ && tx_ok_;
  }

  /**
   * @brief Convert a Return code to a string
   *
   * @pre None
   *
   * @post String representation of ret_code is returned
   *
   * @param[in] ret_code Return code to convert to string
   *
   * @return std::string
   */
  static std::string get_return_as_string(Return ret_code);

private:

  // Disallow copy and assign
  FlightControlInterface(const FlightControlInterface&) = delete;
  FlightControlInterface& operator=(const FlightControlInterface&) = delete;

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
  static void print_warning(const std::string& message);
  static void print_warning(const std::string& origin, const std::string& message);
  static void print_info(const std::string& message);
  static void print_message(const std::string& prefix, const std::string& origin,
      const std::string& message);
  static void enter_action(Action action);
  static void exit_action(const Return& ret_code);

  /**
   * Private non-static member functions
   */
  void enforce_init_precondition() const;
  void enforce_connected_precondition() const;
  void enforce_write_precondition() const;
  void enforce_in_flight_precondition() const;
  void enforce_no_action_in_progress_precondition() const;
  void enforce_tx_connection_invariant() const;
  int infer_metadata_from_mode(const SnMode, const bool, TxCommand::Mode&, SnInputCommandType&,
      SnRcCommandType&);
  void enforce_props_state_constraint(const SnPropsState);
  void do_not_enforce_props_state_constraint();
  void update_tx_command_pos_ctrl();
  Return takeoff_impl(std::unique_lock<std::mutex>& lock, const TakeoffConfig& config);
  Return land_impl(std::unique_lock<std::mutex>& lock, const LandingConfig& config);
  Return go_to_waypoint_impl(std::unique_lock<std::mutex>& lock, const Waypoint& wp);
  Return execute_mission_impl(std::unique_lock<std::mutex>& lock, const double t_start);
  Return execute_planner(std::unique_lock<std::mutex>& lock, const double t_start = 0);

  // wrappers around functions to catch exceptions; safe for threads
  void takeoff_safe() noexcept;
  void takeoff_safe(const TakeoffConfig&) noexcept;
  void land_safe() noexcept;
  void land_safe(const LandingConfig&) noexcept;
  void go_to_waypoint_safe(const Waypoint&) noexcept;
  void execute_mission_safe(const double t_start) noexcept;

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
  // system_clock is used here instead of steady_clock so that an absolute
  // timestamp relative to the wall clock can be provided to the execute
  // mission function, which is useful for synchronizing multiple robots. Also,
  // system_clock may be continually adjusted to maintain synchronization among
  // a multi-robot system where clock drift may be significant over the course
  // of a mission. However, stepping the system_clock during a mission could
  // cause undefined behavior. Be careful! It may make sense to make this
  // configurable and only use system_clock if the absolute time sync is
  // required.
  static std::chrono::time_point<std::chrono::system_clock> t0_;
  static std::mutex time_mutex_;
  static Planner planner_;
  static std::thread action_thread_;
  static std::atomic<Action> current_action_;
  static std::atomic<Action> last_action_;
  static std::atomic<Return> last_action_result_;
  static std::atomic_bool kill_current_action_;
  static std::condition_variable action_cv_;
  static std::mutex action_mutex_;

  /**
   * Private non-static member variables
   */
  std::atomic_bool connected_;
  std::atomic<Permissions> permissions_;

};

} // namespace snav_fci

#endif // SNAV_FCI_FLIGHT_CONTROL_INTERFACE_HPP_

