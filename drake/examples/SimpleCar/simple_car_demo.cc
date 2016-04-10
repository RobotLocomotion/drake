#include "simple_car.h"

#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "lcmtypes/drake/lcmt_driving_command_t.hpp"
#include "drake/systems/plants/BotVisualizer.h"
#include "floating_joint_state.h"

using namespace std;
using namespace Eigen;
using namespace Drake;
using Drake::getDrakePath;

// N.B.: LCMSystem uses this via c++ ADL magic. See also drake issue #1880.
bool decode(const drake::lcmt_driving_command_t &msg, double &t,
            DrivingCommand<double> &x) {
  t = double(msg.timestamp) / 1000.0;
  x.steering_angle = msg.steering_angle;
  x.throttle = msg.throttle;
  x.brake = msg.brake;
  return true;
}

// TODO rico: move this to somewhere common; it is useful.
/// LCMTap -- a system that wires input to output, and publishes input to LCM.
template <template <typename> class Vector>
class LCMTap {
 public:
  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = Vector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = Vector<ScalarType>;

  LCMTap(std::shared_ptr<lcm::LCM> lcm) : lcm(lcm) {}

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
    lcm->publish(u.channel(), &msg);
    return u;
  }

  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return true; }

 private:
  const std::shared_ptr<lcm::LCM> lcm;
};

int main(int argc, const char *argv[]) {
  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();

  auto car = make_shared<Drake::SimpleCar>();

  //
  // Make a linear system to map simple car state to the state vector of a
  // floating joint, allowing motion and steering in the x-y plane only.
  //
  Drake::FloatingJointState<double> y0;
  y0.yaw = kPi / 2;

  const int insize = Drake::SimpleCarState<double>::RowsAtCompileTime;
  const int outsize = Drake::FloatingJointState<double>::RowsAtCompileTime;
  Matrix<double, outsize, insize> D;
  D <<
      1, 0,  0, 0, 0,  // x
      0, 1,  0, 0, 0,  // y
      0, 0,  0, 0, 0,  // z
      0, 0,  0, 0, 0,  // roll
      0, 0,  0, 0, 0,  // pitch
      0, 0, -1, 0, 0;  // yaw
  auto adapter = make_shared<AffineSystem<NullVector, Drake::SimpleCarState,
                                          Drake::FloatingJointState> >(
      MatrixXd::Zero(0, 0), MatrixXd::Zero(0, insize), VectorXd::Zero(0),
      MatrixXd::Zero(outsize, 0), D, toEigen(y0));

  //
  // Load a simplistic rendering from accompanying URDF file.
  //
  auto tree = make_shared<RigidBodyTree>(getDrakePath() +
                                         "/examples/SimpleCar/boxcar.urdf");

  auto viz = make_shared<BotVisualizer<Drake::FloatingJointState> >(lcm, tree);

  // Make some taps to publish intermediate states to LCM.
  auto car_tap = make_shared<LCMTap<Drake::SimpleCarState> >(lcm);
  auto adapter_tap = make_shared<LCMTap<Drake::FloatingJointState> >(lcm);

  // Assemble car, adapter, and visualizer, with intervening taps.
  auto car_tapped = Drake::cascade(car, car_tap);
  auto adapter_tapped = Drake::cascade(adapter, adapter_tap);
  auto adapt_viz = Drake::cascade(adapter_tapped, viz);
  auto sys = Drake::cascade(car_tapped, adapt_viz);

  Drake::SimpleCarState<double> initial_state;
  runLCM(sys, lcm, 0, numeric_limits<double>::infinity(), initial_state);

  return 0;
}
