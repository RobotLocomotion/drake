#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/system1/LinearSystem.h"
#include "drake/system1/Simulation.h"
#include "drake/system1/cascade_system.h"
#include "drake/system1/gravity_compensated_system.h"
#include "drake/system1/robot_state_tap.h"

using drake::AffineSystem;
using drake::GravityCompensatedSystem;
using drake::RigidBodySystem;
using drake::RobotStateTap;
using drake::SimulationOptions;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const int kNumDof = 7;  // Number of DoF in the KUKA IIWA Arm.

VectorXd SimulateGravityCompensatedTorqueControlSystem(
    const VectorXd& input_torque_vector) {
  std::shared_ptr<RigidBodySystem> iiwa_system = CreateKukaIiwaSystem();

  auto controlled_robot =
      std::allocate_shared<GravityCompensatedSystem<RigidBodySystem>>(
          Eigen::aligned_allocator<GravityCompensatedSystem<RigidBodySystem>>(),
          iiwa_system);

  auto robot_state_tap =
      std::make_shared<RobotStateTap<RigidBodySystem::StateVector>>();

  auto input_torque = std::make_shared<
      AffineSystem<NullVector, NullVector, RigidBodySystem::StateVector>>(
      MatrixXd::Zero(0, 0), MatrixXd::Zero(0, 0), VectorXd::Zero(0),
      MatrixXd::Zero(kNumDof, 0), MatrixXd::Zero(kNumDof, 0),
      input_torque_vector);
  auto sys = cascade(cascade(input_torque, controlled_robot), robot_state_tap);

  drake::SimulationOptions options = SetupSimulation();

  // Specifies the start time of the simulation.
  const double kStartTime = 0;

  // Specifies the duration of the simulation.
  const double kDuration = 0.5;

  // Obtains an initial state of the simulation.
  VectorXd x0 = GenerateArbitraryIiwaInitialState();

  drake::simulate(*sys.get(), kStartTime, kDuration, x0, options);

  // Obtains the final state of the IIWA robot at the end of the simulation.
  VectorXd xf = robot_state_tap->get_input_vector();

  CheckLimitViolations(iiwa_system, xf);

  // Ensures joint position and velocity limits are not violated.
  return (xf);
}

// Verifies correct behavior of the KUKA IIWA Arm under a gravity compensation
// torque controller with 0 input torque. Since the robot is present in an
// environment with gravity, under no external forces, starting from rest, the
// robot should remain at rest. This test verifies that the arm remains at its
// initial position within small error bounds.
GTEST_TEST(testIIWAArm, iiwaArmGravityCompensatedTorqueControlZeroInput) {
  // Applies zero torque to the joints in the simulated IIWA robot.
  VectorXd input_torque_vector = VectorXd::Zero(kNumDof);

  // Obtains an initial state of the simulation.
  VectorXd x0 = GenerateArbitraryIiwaInitialState();

  VectorXd xf;
  EXPECT_NO_THROW(
      xf = SimulateGravityCompensatedTorqueControlSystem(input_torque_vector));

  // Expects norm of the joint position difference to be below a maximum value.
  double kMaxPositionErrorNorm = 1e-3;
  EXPECT_LT((xf.head(kNumDof) - x0.head(kNumDof)).squaredNorm(),
              kMaxPositionErrorNorm);

  // Expects final joint velocity has a norm smaller than a maximum value.
  // This should be true since this controller guarantees no motion under no
  // input and no external forces.
  double kMaxVelocityNorm = 1e-3;
  EXPECT_LT(xf.tail(kNumDof).squaredNorm(), kMaxVelocityNorm);
}

// Verifies the correct behavior of the KUKA IIWA Arm under a gravity
// compensation torque controller with a small torque input. Since the robot is
// present in an environment with gravity, when starting from rest, under a
// small torque input, the robot accelerates resulting from that torque input.
GTEST_TEST(testIIWAArm, iiwaArmGravityCompensatedTorqueControlSmallInput) {
  // Applying a small input torque on 5th joint.
  VectorXd input_torque_vector = VectorXd::Zero(kNumDof);
  input_torque_vector(4) = 1.75;

  // Obtains an initial state of the simulation.
  VectorXd x0 = GenerateArbitraryIiwaInitialState();

  VectorXd xf;
  EXPECT_NO_THROW(
      xf = SimulateGravityCompensatedTorqueControlSystem(input_torque_vector));

  // Expects norm of the joint position difference to be greater that a
  // minimum value.
  double kMinPositionErrorNorm = 1e-3;
  EXPECT_LT(kMinPositionErrorNorm,
            (xf.head(kNumDof) - x0.head(kNumDof)).squaredNorm());

  // Expects final joint velocity has a norm larger than a minimum value. This
  // should be true since the torque input results in continuous acceleration.
  double kMinVelocityNorm = 1e-3;
  EXPECT_LT(kMinVelocityNorm, xf.tail(kNumDof).squaredNorm());
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
