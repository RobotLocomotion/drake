#include "drake/examples/SimpleCar/simple_car.h"

#include <cmath>

#include "drake/Path.h"

#include "drake/examples/SimpleCar/euler_floating_joint_state.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "lcmtypes/drake/lcmt_driving_command_t.hpp"

// TODO(rpoyner-tri): move this to somewhere common; it is useful.
/// LCMTap -- a system that wires input to output, and publishes input to LCM.
template <template <typename> class Vector>
class LCMTap {
 public:
  template <typename ScalarType>
  using StateVector = Drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = Vector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = Vector<ScalarType>;

  explicit LCMTap(std::shared_ptr<lcm::LCM> lcm) : lcm(lcm) {}

  StateVector<double> dynamics(const double& t, const StateVector<double>& x,
                               const InputVector<double>& u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double& t, const StateVector<double>& x,
                              const InputVector<double>& u) const {
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

int main(int argc, const char* argv[]) {
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  auto car = std::make_shared<Drake::SimpleCar>();

  //
  // Make a linear system to map simple car state to the state vector of a
  // floating joint, allowing motion and steering in the x-y plane only.
  //
  Drake::EulerFloatingJointState<double> y0;
  y0.yaw = M_PI / 2;

  const int insize = Drake::SimpleCarState<double>::RowsAtCompileTime;
  const int outsize = Drake::EulerFloatingJointState<double>::RowsAtCompileTime;
  Eigen::Matrix<double, outsize, insize> D;
  D <<
      1, 0,  0, 0,  // x
      0, 1,  0, 0,  // y
      0, 0,  0, 0,  // z
      0, 0,  0, 0,  // roll
      0, 0,  0, 0,  // pitch
      0, 0, -1, 0;  // yaw
  auto adapter = std::make_shared<
      Drake::AffineSystem<Drake::NullVector, Drake::SimpleCarState,
                          Drake::EulerFloatingJointState> >(
      Eigen::MatrixXd::Zero(0, 0), Eigen::MatrixXd::Zero(0, insize),
      Eigen::VectorXd::Zero(0), Eigen::MatrixXd::Zero(outsize, 0), D,
      toEigen(y0));

  //
  // Load a simplistic rendering from accompanying URDF file.
  //
  auto tree = std::make_shared<RigidBodyTree>(
      Drake::getDrakePath() + "/examples/SimpleCar/boxcar.urdf");

  auto viz =
      std::make_shared<Drake::BotVisualizer<Drake::EulerFloatingJointState> >(
          lcm, tree);

  // Make some taps to publish intermediate states to LCM.
  auto car_tap = std::make_shared<LCMTap<Drake::SimpleCarState> >(lcm);
  auto adapter_tap =
      std::make_shared<LCMTap<Drake::EulerFloatingJointState> >(lcm);

  // Assemble car, adapter, and visualizer, with intervening taps.
  auto car_tapped = Drake::cascade(car, car_tap);
  auto adapter_tapped = Drake::cascade(adapter, adapter_tap);
  auto adapt_viz = Drake::cascade(adapter_tapped, viz);
  auto sys = Drake::cascade(car_tapped, adapt_viz);

  Drake::SimpleCarState<double> initial_state;
  runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(), initial_state);

  return 0;
}
