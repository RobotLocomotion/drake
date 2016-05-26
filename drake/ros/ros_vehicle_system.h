#pragma once

#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#include "drake/systems/Simulation.h"
#include "drake/systems/System.h"
#include "drake/systems/cascade_system.h"

namespace drake {
namespace ros {

bool decode(const ackermann_msgs::AckermannDriveStamped& msg,
            drake::DrivingCommand<double>& x) {
  x.set_steering_angle(msg.drive.steering_angle);
  if (msg.drive.speed > 0) {
    x.set_throttle(msg.drive.speed);
    x.set_brake(0);
  } else {
    x.set_throttle(0);
    x.set_brake(-1 * msg.drive.speed);
  }
  return true;
}

/** \brief Adds support for ackermann_msgs/AckermannDriveStamped commands.
 *
 * Takes a car simulation system and prepends it with another sytsem that
 * subscribes to ackermann_msgs/AckermannDriveStamped messges. It saves these
 * commands in a vector and outputs this vector to the car simulation system.
 */

namespace internal {

template <template <typename> class Vector>
class ROSAckermannCommandReceiverSystem {
 private:
  static const int kSubscriberQueueSize = 100;

 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = Vector<ScalarType>;
  // static const bool has_lcm_input = true;

  explicit ROSAckermannCommandReceiverSystem() : spinner_(1) {
    ::ros::NodeHandle nh;

    // Instantiates a ROS topic subscriber that receives vehicle driving
    // commands.
    subscriber_ =
        nh.subscribe("ackermann_cmd", kSubscriberQueueSize,
                     &ROSAckermannCommandReceiverSystem::commandCallback, this);

    // Instantiates a child thread for receiving ROS messages.
    // See:
    // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Multi-threaded_Spinning
    // ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner_.start();
  }

  virtual ~ROSAckermannCommandReceiverSystem() {}

  void commandCallback(const ackermann_msgs::AckermannDriveStamped& msg) {
    data_mutex_.lock();
    decode(msg, data_);
    data_mutex_.unlock();
  }

  StateVector<double> dynamics(const double& t, const StateVector<double>& x,
                               const InputVector<double>& u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double& t, const StateVector<double>& x,
                              const InputVector<double>& u) const {
    data_mutex_.lock();
    OutputVector<double> y = data_;  // make a copy of the data
    data_mutex_.unlock();
    return y;
  }

 private:
  ::ros::Subscriber subscriber_;

  mutable std::mutex data_mutex_;

  OutputVector<double> data_;

  ::ros::AsyncSpinner spinner_;
};

}  // namespace internal

/**
 * Runs the vehicle simulation using a caller-specified initial state.
 * This method first adds another system to the beginning of the supplied system
 * that subscribes to a ROS topic containing
 * ackermann_msgs/AckermannDriveStamped messages. These messages are then packed
 * into a vector and passed to the supplied system as the driving command.
 *
 * This method will not return until the simulation is complete.
 *
 * @ingroup simulation
 *
 * @param[in] sys The system to simulate.
 * @param[in] t0 The initial simulation time.
 * @param[in] tf The final simulation time.
 * @param[in] x0 The initial state of the system being simulated.
 * @param[in] options The simulation options.
 */
template <typename System>
void run_ros_vehicle_sim(
    std::shared_ptr<System> sys, double t0, double tf,
    const typename System::template StateVector<double>& x0,
    const SimulationOptions& options = Drake::default_simulation_options) {
  auto ros_ackermann_input =
      std::make_shared<internal::ROSAckermannCommandReceiverSystem<
          System::template InputVector>>();

  auto ros_sys = cascade(ros_ackermann_input, sys);

  SimulationOptions sim_options = options;
  if (sim_options.realtime_factor < 0.0) sim_options.realtime_factor = 1.0;
  Drake::simulate(*ros_sys, t0, tf, x0, sim_options);
}

/**
 * Runs the vehicle simulation using an automatically-derived initial state.
 * It adds another system to the beginning of the supplied system that
 * subscribes to a ROS topic containing ackermann_msgs/AckermannDriveStamped
 * messages. These messages are then packed into a vector and passed to the
 * supplied system as the driving command.
 *
 * This method will not return until the simulation is complete.
 *
 * @ingroup simulation
 *
 * @param[in] sys The system to simulate.
 * @param[in] t0 The initial simulation time.
 * @param[in] tf The final simulation time.
 */
template <typename System>
void run_ros_vehicle_sim(const System& sys, double t0, double tf) {
  run_ros_vehicle_sim(sys, t0, tf, getInitialState(*sys));
}

}  // end namespace ros
}  // end namespace drake
