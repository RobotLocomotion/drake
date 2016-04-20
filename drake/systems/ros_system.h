#ifndef DRAKE_SYSTEMS_ROS_SYSTEM_H_
#define DRAKE_SYSTEMS_ROS_SYSTEM_H_

#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include "ros/ros.h"

// #include <lcm/lcm-cpp.hpp>
// #include "lcmtypes/drake/lcmt_drake_signal.hpp"

#include "drake/rost_drake_signal.h"
#include "drake/drakeLCMSystem_export.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/System.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/Simulation.h"

using Drake::SimulationOptions;
using Drake::default_simulation_options;
using Drake::getCoordinateName;

namespace drake {
namespace systems {

/** @defgroup lcm_vector_concept LCMVector<ScalarType> Concept
 * @ingroup vector_concept
 * @brief A specialization of the Vector concept adding the ability to read and
 * publish LCM messages
 *
 * <table>
 * <tr><th colspan="2"> Valid Expressions (which must be implemented)
 * <tr><td> LCMMessageType
 *     <td> defined with a typedef
 * <tr><td> static std::string channel() const
 *     <td> return the name of the channel to subscribe to/ publish on
 * <tr><td><pre>
 * bool encode(const double& t,
 *             const Vector<double>& x,
 *             LCMMessageType& msg)</pre>
 *     <td> define the mapping from your LCM type to your Vector type
 * <tr><td><pre>
 * bool decode(const LCMMessageType& msg,
 *             double& t,
 *             Vector<double>& x)</pre>
 *     <td> define the mapping from your Vector type to your LCM type
 * </table>
//  */

template <class Vector>
bool encode(const double &t, const Vector &x, drake::rost_drake_signal &msg) {
  msg.timestamp = static_cast<int64_t>(t * 1000);
  auto xvec = toEigen(x);
  for (int i = 0; i < Drake::size(x); i++) {
    msg.coord.push_back(getCoordinateName(x, i));
    msg.val.push_back(xvec(i));
  }
  return true;
}

/**
 * Decodes a ROS message.
 *
 * @param msg A reference to the ROS message to decode.
 * @param timestamp A reference to the variable where the timestamp should be
 * saved.
 * @param x A reference to where the decoded signal is stored.
 * @return true if the decode operation is successful.
 */
template <class Vector>
bool decode(const drake::rost_drake_signal &msg, double &timestamp, Vector &x) {
  // Decodes the timestamp.
  timestamp = double(msg.timestamp) / 1000.0;

  // Stores the signal data in a map keyed by signal data name.
  std::unordered_map<std::string, double> m;
  for (int i = 0; i < msg.val.size(); i++) {
    m[msg.coord[i]] = msg.val[i];
  }

  // Saves the data in the order specifies by getCoordinateName().
  Eigen::Matrix<double, Vector::RowsAtCompileTime, 1> xvec(msg.val.size());
  for (int i = 0; i < msg.val.size(); i++) {
    xvec(i) = m[getCoordinateName(x, i)];
  }
  x = xvec;

  return true;
}

namespace internal {

template <template <typename> class Vector, typename Enable = void>
class ROSInputSystem {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = Vector<ScalarType>;
  static const bool has_ros_input = false;

  template <typename System>
  ROSInputSystem(const System &wrapped_sys)
      : all_zeros(Eigen::VectorXd::Zero(getNumInputs(wrapped_sys))) {}

  StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                               const InputVector<double> &u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double &t, const StateVector<double> &x,
                              const InputVector<double> &u) const {
    return all_zeros;
  }

 private:
  OutputVector<double> all_zeros;
};

template <template <typename> class Vector>
class ROSInputSystem<
    Vector,
    typename std::enable_if<
        !std::is_void<typename Vector<double>::LCMMessageType>::value>::type> {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = Vector<ScalarType>;
  static const bool has_ros_input = true;

  template <typename System>
  ROSInputSystem(const System &sys) {
    ros::NodeHandle nh;
    sub_ = nh.subscribe(Vector<double>::channel(), 1000,
                        &ROSInputSystem<Vector>::callback, this);
  }
  virtual ~ROSInputSystem() {}

  void callback(const drake::rost_drake_signal::ConstPtr &msg) {
    data_mutex.lock();
    decode(*msg, timestamp, data);
    data_mutex.unlock();
  }

  StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                               const InputVector<double> &u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double &t, const StateVector<double> &x,
                              const InputVector<double> &u) const {
    data_mutex.lock();
    OutputVector<double> y = data;  // make a copy of the data
    data_mutex.unlock();
    return y;
  }

 private:
  mutable std::mutex data_mutex;
  double timestamp;
  OutputVector<double> data;

  /**
   * The ROS topic subscriber.
   */
  ros::Subscriber sub_;
};

template <template <typename> class Vector, typename Enable = void>
class ROSOutputSystem {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = Vector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = NullVector<ScalarType>;

  ROSOutputSystem(std::shared_ptr<lcm::LCM> lcm) {}

  StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                               const InputVector<double> &u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double &t, const StateVector<double> &x,
                              const InputVector<double> &u) const {
    return OutputVector<double>();
  }
};

template <template <typename> class Vector>
class ROSOutputSystem<
    Vector,
    typename std::enable_if<
        !std::is_void<typename Vector<double>::LCMMessageType>::value>::type> {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = Vector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = NullVector<ScalarType>;

  ROSOutputSystem() {
    ros::NodeHandle nh;
    // TODO(liang): Modify to not hard code topic name. In LCMSystem, the topic
    // name was originally defined in method output() and used u.channel().
    publisher_ = nh.advertise<drake::rost_drake_signal>("output", 1000);
  }

  StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                               const InputVector<double> &u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double &t, const StateVector<double> &x,
                              const InputVector<double> &u) const {
    // Hard code message type to be drake::rost_drake_signal.
    drake::rost_drake_signal msg;

    if (!encode(t, u, msg))
      throw std::runtime_error(
          std::string("failed to encode meesage of type ") +
          "drake::rost_drake_signal");

    // Publishes the message.
    publisher_.publish(msg);
    return OutputVector<double>();
  }

 private:
  /*!
   * The ROS topic publisher for publishing drake::rost_drake_signal
   * messages.
   */
  ros::Publisher publisher_;
};

// todo: template specialization for the CombinedVector case

// class DRAKELCMSYSTEM_EXPORT LCMLoop {
//  public:
//   bool stop;
//   lcm::LCM &lcm;

//   LCMLoop(lcm::LCM &_lcm) : lcm(_lcm), stop(false) {}

//   void loopWithSelect();
// };

}  // end namespace internal

/**
 * @brief Simulates the system with the (exposed) inputs being read from ROS
 * topics and the output being published to ROS topics.
 *
 * @ingroup simulation
 *
 * The input and output vector types must overload a publishLCM namespace
 * method; the default for new vectors is to not publish anything.
 *
 * @param sys The rigid body system to simulate.
 * @param t0 The simulation's start time.
 * @param tf The simulation's final time.
 * @param x0 The initial state of the system.
 * @param options The simulation options.
 */
template <typename System>
void RunROS(std::shared_ptr<System> sys, double t0, double tf,
            const typename System::template StateVector<double> &x0,
            const SimulationOptions &options = default_simulation_options) {
  if (!ros::ok()) throw std::runtime_error("ROS is not OK!");

  //    typename System::template OutputVector<double> x = 1;  // useful for
  //    debugging
  auto ros_input =
      std::make_shared<internal::ROSInputSystem<System::template InputVector>>(
          *sys);
  auto ros_output = std::make_shared<
      internal::ROSOutputSystem<System::template OutputVector>>();
  auto ros_sys = cascade(ros_input, cascade(sys, ros_output));

  bool has_ros_input =
      internal::ROSInputSystem<System::template InputVector>::has_ros_input;

  if (has_ros_input && Drake::size(x0) == 0 && !sys->isTimeVarying()) {
    // then this is really a static function, not a dynamical system.
    // block on receiving lcm input and process the output exactly when a new
    // input message is received.
    // note: this will never return (unless there is an lcm error)

    //      std::cout << "LCM output will be triggered on receipt of an LCM
    //      Input" << std::endl;

    double t = 0.0;
    typename System::template StateVector<double> x;
    NullVector<double> u;

    while (1) {
      // if (lcm->handle() != 0) {
      //   throw std::runtime_error("something went wrong in lcm.handle");
      // }
      ros_sys->output(t, x, u);
    }
  } else {
    // internal::LCMLoop lcm_loop(*lcm);
    // std::thread lcm_thread;
    // if (has_ros_input) {
    //   // only start up the listener thread if I actually have inputs to
    //   listen
    //   // to
    //   lcm_thread = std::thread(&internal::LCMLoop::loopWithSelect,
    //   &lcm_loop);
    // }

    SimulationOptions ros_options = options;
    if (ros_options.realtime_factor < 0.0) ros_options.realtime_factor = 1.0;
    simulate(*ros_sys, t0, tf, x0, ros_options);

    // if (has_ros_input) {
    //   // shutdown the lcm thread
    //   lcm_loop.stop = true;
    //   lcm_thread.join();
    // }
  }
}

// template <typename System>
// void runLCM(const System &sys, std::shared_ptr<lcm::LCM> lcm, double t0,
//             double tf) {
//   runLCM(sys, lcm, t0, tf, getInitialState(*sys));
// }

}  // end namespace systems
}  // end namespace drake

#endif  // DRAKE_SYSTEMS_ROS_SYSTEM_H_
