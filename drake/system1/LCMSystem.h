#pragma once

#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_drake_signal.hpp"

#include "drake/common/drake_export.h"
#include "drake/system1/Simulation.h"
#include "drake/system1/System.h"
#include "drake/system1/cascade_system.h"

namespace drake {

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
 */

template <class Vector>
bool encode(const double &t, const Vector &x, drake::lcmt_drake_signal &msg) {
  msg.timestamp = static_cast<int64_t>(t * 1000);
  msg.dim = size(x);
  auto xvec = toEigen(x);
  for (int i = 0; i < msg.dim; i++) {
    msg.coord.push_back(getCoordinateName(x, i));
    msg.val.push_back(xvec(i));
  }
  return true;
}

template <class Vector>
bool decode(const drake::lcmt_drake_signal &msg, double &t, Vector &x) {
  // NOLINTNEXTLINE(readability/casting) This code will be deleted soon.
  t = double(msg.timestamp) / 1000.0;
  std::unordered_map<std::string, double> m;
  for (int i = 0; i < msg.dim; i++) {
    m[msg.coord[i]] = msg.val[i];
  }
  Eigen::Matrix<double, Vector::RowsAtCompileTime, 1> xvec(msg.dim);
  for (int i = 0; i < msg.dim; i++) {
    xvec(i) = m[getCoordinateName(x, i)];
  }
  x = xvec;
  return true;
}

namespace internal {
template <template <typename> class Vector, typename Enable = void>
class LCMInputSystem {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = Vector<ScalarType>;
  static const bool has_lcm_input = false;

  template <typename System>
  LCMInputSystem(const System &wrapped_sys, std::shared_ptr<lcm::LCM> lcm)
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
class LCMInputSystem<
    Vector, typename std::enable_if<!std::is_void<
                typename Vector<double>::LCMMessageType>::value>::type> {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = Vector<ScalarType>;
  static const bool has_lcm_input = true;

  template <typename System>
  LCMInputSystem(const System &sys, std::shared_ptr<lcm::LCM> lcm) {
    lcm::Subscription *sub =
        lcm->subscribe(Vector<double>::channel(),
                       &LCMInputSystem<Vector>::handleMessage, this);
    sub->setQueueCapacity(1);
  }
  virtual ~LCMInputSystem() {}

  void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                     const typename Vector<double>::LCMMessageType *msg) {
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
};

template <template <typename> class Vector, typename Enable = void>
class LCMOutputSystem {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = Vector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = NullVector<ScalarType>;

  explicit LCMOutputSystem(std::shared_ptr<lcm::LCM> lcm) {}

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
class LCMOutputSystem<
    Vector, typename std::enable_if<!std::is_void<
                typename Vector<double>::LCMMessageType>::value>::type> {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = Vector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = NullVector<ScalarType>;

  explicit LCMOutputSystem(std::shared_ptr<lcm::LCM> lcm) : lcm_(lcm) {}

  StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                               const InputVector<double> &u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double &t, const StateVector<double> &x,
                              const InputVector<double> &u) const {
    typename Vector<double>::LCMMessageType msg;
    if (!encode(t, u, msg))
      throw std::runtime_error(std::string("failed to encode") +
                               msg.getTypeName());
    lcm_->publish(u.channel(), &msg);
    return OutputVector<double>();
  }

 private:
  const std::shared_ptr<lcm::LCM> lcm_;
};

// todo: template specialization for the CombinedVector case

class DRAKE_EXPORT LCMLoop {
 public:
  bool stop;
  lcm::LCM &lcm;

  explicit LCMLoop(lcm::LCM &_lcm) : stop(false), lcm(_lcm) {}

  void loopWithSelect();
};

}  //  end namespace internal

/** runLCM
 * @brief Simulates the system with the (exposed) inputs being read from LCM
 * and the output being published to LCM.
 * @ingroup simulation
 *
 * The input and output vector types must overload a publishLCM namespace
 * method; the default for new vectors is to not publish anything.
 */

template <typename System>
void runLCM(std::shared_ptr<System> sys, std::shared_ptr<lcm::LCM> lcm,
            double t0, double tf,
            const typename System::template StateVector<double> &x0,
            const SimulationOptions &options = SimulationOptions()) {
  if (!lcm->good()) throw std::runtime_error("bad LCM reference");

  //    typename System::template OutputVector<double> x = 1;  // useful for
  //    debugging
  auto lcm_input =
      std::make_shared<internal::LCMInputSystem<System::template InputVector> >(
          *sys, lcm);
  auto lcm_output = std::make_shared<
      internal::LCMOutputSystem<System::template OutputVector> >(lcm);
  auto lcm_sys = cascade(lcm_input, cascade(sys, lcm_output));

  bool has_lcm_input =
      internal::LCMInputSystem<System::template InputVector>::has_lcm_input;

  if (has_lcm_input && size(x0) == 0 && !sys->isTimeVarying()) {
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
      if (lcm->handle() != 0) {
        throw std::runtime_error("something went wrong in lcm.handle");
      }
      lcm_sys->output(t, x, u);
    }
  } else {
    internal::LCMLoop lcm_loop(*lcm);
    std::thread lcm_thread;
    if (has_lcm_input) {
      // only start up the listener thread if I actually have inputs to listen
      // to
      lcm_thread = std::thread(&internal::LCMLoop::loopWithSelect, &lcm_loop);
    }

    SimulationOptions lcm_options = options;
    if (lcm_options.realtime_factor < 0.0) lcm_options.realtime_factor = 1.0;
    simulate(*lcm_sys, t0, tf, x0, lcm_options);

    if (has_lcm_input) {
      // shutdown the lcm thread
      lcm_loop.stop = true;
      lcm_thread.join();
    }
  }
}

template <typename System>
void runLCM(const System &sys, std::shared_ptr<lcm::LCM> lcm, double t0,
            double tf) {
  runLCM(sys, lcm, t0, tf, getInitialState(*sys));
}

}  // end namespace drake
