#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/system1/LinearSystem.h"
#include "drake/system1/Simulation.h"
#include "drake/system1/cascade_system.h"
#include "drake/system1/pd_control_system.h"
#include "drake/system1/robot_state_tap.h"
#include "drake/systems/plants/RigidBodySystem.h"

using drake::AffineSystem;
using Eigen::MatrixXd;
using drake::PDControlSystem;
using drake::RigidBodySystem;
using drake::RobotStateTap;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

// Test to verify behavior of the KUKA IIWA Arm under a PD joint position
// controller. Since the robot is present in an environment with gravity,
// the controller will not be able to stabilize the position precisely
// and oscillations about the set-point will always be present in the
// behaviour. The test verifies that the position reached is within some
// bound with respect to the initial condition.
GTEST_TEST(testIIWAArm, iiwaArmPDControl) {
  std::shared_ptr<RigidBodySystem> iiwa_system = CreateKukaIiwaSystem();

  const auto& iiwa_tree = iiwa_system->getRigidBodyTree();

  auto robot_state_tap =
      std::make_shared<RobotStateTap<RigidBodySystem::StateVector>>();

  int num_dof = iiwa_system->getNumInputs();

  // Large gains intentionally used for demo.
  const double kProportionalGainCommon = 500.0;
  const double kDerivativeGainCommon = 0.00;
  VectorXd Kpdiag = VectorXd::Constant(num_dof, kProportionalGainCommon);
  Kpdiag(1) = 800.0;
  Kpdiag(3) = 700.0;
  VectorXd Kddiag = VectorXd::Constant(num_dof, kDerivativeGainCommon);
  MatrixXd Kp = Kpdiag.asDiagonal();
  MatrixXd Kd = Kddiag.asDiagonal();

  // Obtains an initial state of the simulation.
  VectorXd x0 = VectorXd::Zero(iiwa_system->getNumStates());
  x0.head(num_dof) = iiwa_tree->getZeroConfiguration();

  Eigen::VectorXd arbitrary_initial_configuration(num_dof);
  arbitrary_initial_configuration << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;
  x0.head(num_dof) += arbitrary_initial_configuration;

  // Set point is the initial state.
  auto set_point = std::make_shared<
      AffineSystem<NullVector, NullVector, RigidBodySystem::StateVector>>(
      MatrixXd::Zero(0, 0), MatrixXd::Zero(0, 0), VectorXd::Zero(0),
      MatrixXd::Zero(num_dof, 0), MatrixXd::Zero(num_dof, 0), x0.head(num_dof));

  auto controlled_robot =
      std::allocate_shared<PDControlSystem<RigidBodySystem>>(
          Eigen::aligned_allocator<PDControlSystem<RigidBodySystem>>(),
          iiwa_system, Kp, Kd);

  auto sys = cascade(cascade(set_point, controlled_robot), robot_state_tap);

  drake::SimulationOptions options = SetupSimulation();

  // Specifies the start time of the simulation.
  const double kStartTime = 0;

  // Specifies the duration of the simulation.
  const double kDuration = 0.5;

  EXPECT_NO_THROW(
      drake::simulate(*sys.get(), kStartTime, kDuration, x0, options));

  // Obtains the final state of the IIWA robot at the end of the simulation.
  auto xf = robot_state_tap->get_input_vector();

  // Ensures joint position and velocity limits are not violated.
  EXPECT_NO_THROW(CheckLimitViolations(iiwa_system, xf));

  // Expects norm of the joint position difference to be below a maximum value.
  double kMaxPositionErrorNorm = 1e-3;
  EXPECT_LT((xf.head(num_dof) - x0.head(num_dof)).squaredNorm(),
              kMaxPositionErrorNorm);

  // Expects final joint velocity has a norm larger than a minimum value.
  // (since this controller won't stay at rest at the set-point).
  double kMinVelocityNorm = 1e-3;
  EXPECT_LT(kMinVelocityNorm, xf.tail(num_dof).squaredNorm());
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
