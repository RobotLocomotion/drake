#pragma once

#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#include "drake/common/drake_assert.h"
#include "drake/examples/Cars/gen/multi_driving_command.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/simulation_options.h"
#include "drake/systems/System.h"
#include "drake/systems/cascade_system.h"

namespace drake {
namespace ros {
namespace systems {

// Decodes @p msg into @p x.
bool decode(int model_instance_index,
            const ackermann_msgs::AckermannDriveStamped& msg,
            drake::MultiDrivingCommand<double>& x) {
  x.set_steering_angle(model_instance_index, msg.drive.steering_angle);
  if (msg.drive.speed > 0) {
    x.set_throttle(model_instance_index, msg.drive.speed);
    x.set_brake(model_instance_index, 0);
  } else {
    x.set_throttle(model_instance_index, 0);
    x.set_brake(model_instance_index, -1 * msg.drive.speed);
  }
  return true;
}

namespace internal {

template <template <typename> class Vector>
class RosMultiAckermannCommandReceiverSystem {
 private:
  static const int kSubscriberQueueSize = 100;

 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = Vector<ScalarType>;

  RosMultiAckermannCommandReceiverSystem(
        const std::map<int, std::string>& model_instance_name_table) :
        spinner_(1) {
    ::ros::NodeHandle nh;

    // TODO(liang.fok): Generalize this class to support a variable number of
    // vehicles.
    DRAKE_ABORT_UNLESS(model_instance_name_table.size() == 5);

    subscriber0_ =
        nh.subscribe(
            "Prius_1/ackermann_cmd",
            kSubscriberQueueSize,
            &RosMultiAckermannCommandReceiverSystem::commandCallback0, this);

    subscriber1_ =
        nh.subscribe(
            "Prius_2/ackermann_cmd",
            kSubscriberQueueSize,
            &RosMultiAckermannCommandReceiverSystem::commandCallback1, this);

    subscriber2_ =
        nh.subscribe(
            "Prius_3/ackermann_cmd",
            kSubscriberQueueSize,
            &RosMultiAckermannCommandReceiverSystem::commandCallback2, this);

    subscriber3_ =
        nh.subscribe(
            "Prius_4/ackermann_cmd",
            kSubscriberQueueSize,
            &RosMultiAckermannCommandReceiverSystem::commandCallback3, this);

    subscriber4_ =
        nh.subscribe(
            "Prius_5/ackermann_cmd",
            kSubscriberQueueSize,
            &RosMultiAckermannCommandReceiverSystem::commandCallback4, this);

    // Instantiates a child thread for receiving ROS messages.
    spinner_.start();
  }

  virtual ~RosMultiAckermannCommandReceiverSystem() {}

  void commandCallback0(const ackermann_msgs::AckermannDriveStamped& msg) {
    int model_instance_index = 0;
    data_mutex0_.lock();
    decode(model_instance_index, msg, data_);
    data_mutex0_.unlock();
  }

  void commandCallback1(const ackermann_msgs::AckermannDriveStamped& msg) {
    int model_instance_index = 1;
    data_mutex1_.lock();
    decode(model_instance_index, msg, data_);
    data_mutex1_.unlock();
  }

  void commandCallback2(const ackermann_msgs::AckermannDriveStamped& msg) {
    int model_instance_index = 2;
    data_mutex2_.lock();
    decode(model_instance_index, msg, data_);
    data_mutex2_.unlock();
  }

  void commandCallback3(const ackermann_msgs::AckermannDriveStamped& msg) {
    int model_instance_index = 3;
    data_mutex3_.lock();
    decode(model_instance_index, msg, data_);
    data_mutex3_.unlock();
  }

  void commandCallback4(const ackermann_msgs::AckermannDriveStamped& msg) {
    int model_instance_index = 4;
    data_mutex4_.lock();
    decode(model_instance_index, msg, data_);
    data_mutex4_.unlock();
  }

  StateVector<double> dynamics(const double& t, const StateVector<double>& x,
                               const InputVector<double>& u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double& t, const StateVector<double>& x,
                              const InputVector<double>& u) const {
    OutputVector<double> y;

    data_mutex0_.lock();
    data_mutex1_.lock();
    data_mutex2_.lock();
    data_mutex3_.lock();
    data_mutex4_.lock();
    y = data_;  // make a copy of the data
    data_mutex4_.unlock();
    data_mutex3_.unlock();
    data_mutex2_.unlock();
    data_mutex1_.unlock();
    data_mutex0_.unlock();

    return y;
  }

 private:
  ::ros::Subscriber
      subscriber0_,
      subscriber1_,
      subscriber2_,
      subscriber3_,
      subscriber4_;

  mutable std::mutex
      data_mutex0_,
      data_mutex1_,
      data_mutex2_,
      data_mutex3_,
      data_mutex4_;

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
    const std::map<int, std::string>& model_instance_name_table,
    const SimulationOptions& options = SimulationOptions()) {
  auto ros_ackermann_input =
      std::make_shared<internal::RosMultiAckermannCommandReceiverSystem<
          System::template InputVector>>(model_instance_name_table);

  auto ros_sys = cascade(ros_ackermann_input, sys);

  SimulationOptions sim_options = options;
  if (sim_options.realtime_factor < 0.0) sim_options.realtime_factor = 1.0;
  drake::simulate(*ros_sys, t0, tf, x0, sim_options);
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

}  // namespace systems
}  // namespace ros
}  // namespace drake
