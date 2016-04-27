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

namespace Drake {
namespace {

int do_main(int argc, const char* argv[]) {
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  auto car = std::make_shared<SimpleCar>();

  //
  // Make a linear system to map simple car state to the state vector of a
  // floating joint, allowing motion and steering in the x-y plane only.
  //
  const int insize = SimpleCarState<double>().size();
  const int outsize = EulerFloatingJointState<double>().size();
  Eigen::MatrixXd D;
  D.setZero(outsize, insize);
  D(EulerFloatingJointStateIndices::kX, SimpleCarStateIndices::kX) = 1;
  D(EulerFloatingJointStateIndices::kY, SimpleCarStateIndices::kY) = 1;
  D(EulerFloatingJointStateIndices::kYaw, SimpleCarStateIndices::kHeading) = -1;
  EulerFloatingJointState<double> y0;
  y0.yaw() = M_PI / 2;
  auto adapter = std::make_shared<
      AffineSystem<
        NullVector,
        SimpleCarState,
        EulerFloatingJointState> >(
            Eigen::MatrixXd::Zero(0, 0),
            Eigen::MatrixXd::Zero(0, insize),
            Eigen::VectorXd::Zero(0),
            Eigen::MatrixXd::Zero(outsize, 0), D,
            toEigen(y0));

  //
  // Load a simplistic rendering from accompanying URDF file.
  //
  auto tree = std::make_shared<RigidBodyTree>(
      getDrakePath() + "/examples/SimpleCar/boxcar.urdf");

  auto viz =
      std::make_shared<BotVisualizer<EulerFloatingJointState> >(
          lcm, tree);

  // Make some taps to publish intermediate states to LCM.
  auto car_tap = std::make_shared<LCMTap<SimpleCarState> >(lcm);
  auto adapter_tap = std::make_shared<LCMTap<EulerFloatingJointState> >(lcm);

  // Assemble car, adapter, and visualizer, with intervening taps.
  auto car_tapped = cascade(car, car_tap);
  auto adapter_tapped = cascade(adapter, adapter_tap);
  auto adapt_viz = cascade(adapter_tapped, viz);
  auto sys = cascade(car_tapped, adapt_viz);

  SimpleCarState<double> initial_state;
  runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(), initial_state);

  return 0;
}

}  // namespace
}  // namespace Drake

int main(int argc, const char* argv[]) {
  return Drake::do_main(argc, argv);
}
