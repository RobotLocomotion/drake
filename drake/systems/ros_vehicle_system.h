#pragma once

#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

// #include <lcm/lcm-cpp.hpp>
// #include "lcmtypes/drake/lcmt_drake_signal.hpp"

// #include "drake/drakeLCMSystem_export.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/System.h"
#include "drake/systems/cascade_system.h"

namespace Drake {

/** \brief Adds support for ackermann_msgs/AckermannDriveStamped commands.
 *
 * Takes a car simulation system and prepends it with another sytsem that
 * subscribes to ackermann_msgs/AckermannDriveStamped messges. It saves these
 * commands in a vector and outputs this vector to the car simulation system.
 */

// template <class Vector>
// bool encode(const double &t, const Vector &x, drake::lcmt_drake_signal &msg) {
//   msg.timestamp = static_cast<int64_t>(t * 1000);
//   msg.dim = size(x);
//   auto xvec = toEigen(x);
//   for (int i = 0; i < msg.dim; i++) {
//     msg.coord.push_back(getCoordinateName(x, i));
//     msg.val.push_back(xvec(i));
//   }
//   return true;
// }

// template <class Vector>
// bool decode(const drake::lcmt_drake_signal &msg, double &t, Vector &x) {
//   t = double(msg.timestamp) / 1000.0;
//   std::unordered_map<std::string, double> m;
//   for (int i = 0; i < msg.dim; i++) {
//     m[msg.coord[i]] = msg.val[i];
//   }
//   Eigen::Matrix<double, Vector::RowsAtCompileTime, 1> xvec(msg.dim);
//   for (int i = 0; i < msg.dim; i++) {
//     xvec(i) = m[getCoordinateName(x, i)];
//   }
//   x = xvec;
//   return true;
// }

namespace internal {
// template <template <typename> class Vector, typename Enable = void>
// class ROSAckermannSystem {
//  public:
//   template <typename ScalarType>
//   using StateVector = NullVector<ScalarType>;
//   template <typename ScalarType>
//   using InputVector = NullVector<ScalarType>;
//   template <typename ScalarType>
//   using OutputVector = Vector<ScalarType>;
//   static const bool has_lcm_input = false;

//   template <typename System>
//   ROSAckermannSystem(const System &wrapped_sys, std::shared_ptr<lcm::LCM> lcm)
//       : all_zeros(Eigen::VectorXd::Zero(getNumInputs(wrapped_sys))) {}

//   StateVector<double> dynamics(const double &t, const StateVector<double> &x,
//                                const InputVector<double> &u) const {
//     return StateVector<double>();
//   }

//   OutputVector<double> output(const double &t, const StateVector<double> &x,
//                               const InputVector<double> &u) const {
//     return all_zeros;
//   }

//  private:
//   OutputVector<double> all_zeros;
// };

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
    ros::NodeHandle nh;

    // Instantiates a ROS topic subscriber that receives vehicle driving
    // commands.
    subscriber_ = nh.subscribe("ackermann_cmd", kSubscriberQueueSize,
      &ROSAckermannCommandReceiverSystem::commandCallback, this);

    // Instantiates a child thread for receiving ROS messages.
    // See: http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Multi-threaded_Spinning
    // ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner_.start();
  }

  virtual ~ROSAckermannCommandReceiverSystem() {}

  void commandCallback(const ackermann_msgs::AckermannDriveStamped& msg) {
    std::cout << "RosAckermannCommandReceiverSystem: Received command:\n"
      << msg << std::endl;
    data_mutex_.lock();
    // data_(0) = msg.drive.steering_angle;
    // data_(1) = msg.drive.speed;
    data_mutex_.unlock();
  }

  StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                               const InputVector<double> &u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double &t, const StateVector<double> &x,
                              const InputVector<double> &u) const {
    data_mutex_.lock();
    OutputVector<double> y = data_;  // make a copy of the data
    data_mutex_.unlock();
    return y;
  }

 private:

  ros::Subscriber subscriber_;

  mutable std::mutex data_mutex_;

  // double timestamp;
  OutputVector<double> data_;

  ros::AsyncSpinner spinner_;
};

// template <template <typename> class Vector, typename Enable = void>
// class LCMOutputSystem {
//  public:
//   template <typename ScalarType>
//   using StateVector = NullVector<ScalarType>;
//   template <typename ScalarType>
//   using InputVector = Vector<ScalarType>;
//   template <typename ScalarType>
//   using OutputVector = NullVector<ScalarType>;

//   explicit LCMOutputSystem(std::shared_ptr<lcm::LCM> lcm) {}

//   StateVector<double> dynamics(const double &t, const StateVector<double> &x,
//                                const InputVector<double> &u) const {
//     return StateVector<double>();
//   }

//   OutputVector<double> output(const double &t, const StateVector<double> &x,
//                               const InputVector<double> &u) const {
//     return OutputVector<double>();
//   }
// };

// template <template <typename> class Vector>
// class LCMOutputSystem<
//     Vector, typename std::enable_if<!std::is_void<
//                 typename Vector<double>::LCMMessageType>::value>::type> {
//  public:
//   template <typename ScalarType>
//   using StateVector = NullVector<ScalarType>;
//   template <typename ScalarType>
//   using InputVector = Vector<ScalarType>;
//   template <typename ScalarType>
//   using OutputVector = NullVector<ScalarType>;

//   explicit LCMOutputSystem(std::shared_ptr<lcm::LCM> lcm) : lcm(lcm) {}

//   StateVector<double> dynamics(const double &t, const StateVector<double> &x,
//                                const InputVector<double> &u) const {
//     return StateVector<double>();
//   }

//   OutputVector<double> output(const double &t, const StateVector<double> &x,
//                               const InputVector<double> &u) const {
//     typename Vector<double>::LCMMessageType msg;
//     if (!encode(t, u, msg))
//       throw std::runtime_error(std::string("failed to encode") +
//                                msg.getTypeName());
//     lcm->publish(u.channel(), &msg);
//     return OutputVector<double>();
//   }

//  private:
//   const std::shared_ptr<lcm::LCM> lcm;
// };

// todo: template specialization for the CombinedVector case

// class DRAKELCMSYSTEM_EXPORT LCMLoop {
//  public:
//   bool stop;
//   lcm::LCM &lcm;

//   explicit LCMLoop(lcm::LCM &_lcm) : lcm(_lcm), stop(false) {}

//   void loopWithSelect();
// };

}  //  end namespace internal

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
void run_ros_vehicle_sim(std::shared_ptr<System> sys, double t0,
  double tf, const typename System::template StateVector<double> &x0,
  const SimulationOptions &options = default_simulation_options) {

  //    typename System::template OutputVector<double> x = 1;  // useful for
  //    debugging
  auto ros_ackermann_input =
      std::make_shared<internal::ROSAckermannCommandReceiverSystem<
        System::template InputVector>>();
  // auto lcm_output = std::make_shared<
  //     internal::LCMOutputSystem<System::template OutputVector> >(lcm);
  auto ros_sys = cascade(ros_ackermann_input, sys);

  // bool has_lcm_input =
  //     internal::LCMInputSystem<System::template InputVector>::has_lcm_input;

  // if (has_lcm_input && size(x0) == 0 && !sys->isTimeVarying()) {
  //   // then this is really a static function, not a dynamical system.
  //   // block on receiving lcm input and process the output exactly when a new
  //   // input message is received.
  //   // note: this will never return (unless there is an lcm error)

  //   //      std::cout << "LCM output will be triggered on receipt of an LCM
  //   //      Input" << std::endl;

  //   double t = 0.0;
  //   typename System::template StateVector<double> x;
  //   NullVector<double> u;

  //   while (1) {
  //     if (lcm->handle() != 0) {
  //       throw std::runtime_error("something went wrong in lcm.handle");
  //     }
  //     lcm_sys->output(t, x, u);
  //   }
  // } else {
  //   internal::LCMLoop lcm_loop(*lcm);
  //   std::thread lcm_thread;
  //   if (has_lcm_input) {
  //     // only start up the listener thread if I actually have inputs to listen
  //     // to
  //     lcm_thread = std::thread(&internal::LCMLoop::loopWithSelect, &lcm_loop);
  //   }

  SimulationOptions sim_options = options;
  if (sim_options.realtime_factor < 0.0) sim_options.realtime_factor = 1.0;
    simulate(*ros_sys, t0, tf, x0, sim_options);
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
void run_ros_vehicle_sim(const System &sys, double t0, double tf) {
  run_ros_vehicle_sim(sys, t0, tf, getInitialState(*sys));
}

}  // end namespace Drake
