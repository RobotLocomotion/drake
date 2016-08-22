#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/gravity_compensated_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/robot_state_tap.h"

using drake::AffineSystem;
using drake::BotVisualizer;
using Eigen::MatrixXd;
using drake::RigidBodySystem;
using drake::GravityCompensatedSystem;
using drake::RobotStateTap;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

// Test to verify behavior of the KUKA IIWA Arm under a gravity
// compensation torque controller under 0 input torque. Since the robot is
// present in an environment with gravity, under no external forces,
// starting from rest, the robot is maintained at rest. The test verifies
// that the arm remains at the initial position within reasonable error bounds.
GTEST_TEST(testIIWAArm, iiwaArmGravityCompensatedTorqueControl0Input) {
  std::shared_ptr<RigidBodySystem> iiwa_system = CreateKukaIiwaSystem();

  const auto& iiwa_tree = iiwa_system->getRigidBodyTree();

  // Initializes LCM.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Instantiates additional systems and cascades them with the rigid body
  // system.
  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,
                                                                    iiwa_tree);

  auto robot_state_tap =
      std::make_shared<RobotStateTap<RigidBodySystem::StateVector>>();

  int num_dof = iiwa_tree->number_of_positions();

  // Obtains an initial state of the simulation.
  VectorXd x0 = VectorXd::Zero(iiwa_system->getNumStates());
  x0.head(num_dof) = iiwa_tree->getZeroConfiguration();

  Eigen::VectorXd arbitrary_initial_configuration(num_dof);
  arbitrary_initial_configuration << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;
  x0.head(num_dof) += arbitrary_initial_configuration;

  auto controlled_robot =
      std::allocate_shared<GravityCompensatedSystem<RigidBodySystem>>(
          Eigen::aligned_allocator<GravityCompensatedSystem<RigidBodySystem>>(),
          iiwa_system);

  auto sys = cascade(cascade(controlled_robot, visualizer), robot_state_tap);

  drake::SimulationOptions options = SetupSimulation();

  // Specifies the start time of the simulation.
  const double kStartTime = 0;

  // Specifies the duration of the simulation.
  const double kDuration = 0.75;

  EXPECT_NO_THROW(
      drake::simulate(*sys.get(), kStartTime, kDuration, x0, options));

  auto xf = robot_state_tap->get_input_vector();

  // Ensures joint position and velocity limits are not violated.
  EXPECT_NO_THROW(CheckLimitViolations(iiwa_system, xf));

  // Expects norm of the joint position difference to be below a maximum value.
  double kMaxPositionErrorNorm = 1e-3;
  EXPECT_TRUE((xf.head(num_dof) - x0.head(num_dof)).squaredNorm() <
              kMaxPositionErrorNorm);

  // Expects final joint velocity has a norm smaller than a maximum value.
  // (since this controller guarantees no motion under no input and no
  //  external forces).
  double kMaxVelocityNorm = 1e-3;
  EXPECT_TRUE(xf.tail(num_dof).squaredNorm() < kMaxVelocityNorm);
}

// Test to verify behavior of the KUKA IIWA Arm under a gravity
// compensation torque controller under a small torque input. Since the
// robot is present in an environment with gravity, when starting from rest,
// under a small torque input, the robot does not stay at rest but accelerates
// resulting from that torque input.
GTEST_TEST(testIIWAArm, iiwaArmGravityCompensatedTorqueControlSmallInput) {
  std::shared_ptr<RigidBodySystem> iiwa_system = CreateKukaIiwaSystem();

  const auto& iiwa_tree = iiwa_system->getRigidBodyTree();

  // Initializes LCM.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Instantiates additional systems and cascades them with the rigid body
  // system.
  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,
                                                                    iiwa_tree);

  auto robot_state_tap =
      std::make_shared<RobotStateTap<RigidBodySystem::StateVector>>();

  int num_dof = iiwa_tree->number_of_positions();

  // Obtains an initial state of the simulation.
  VectorXd x0 = VectorXd::Zero(iiwa_system->getNumStates());
  x0.head(num_dof) = iiwa_tree->getZeroConfiguration();

  Eigen::VectorXd arbitrary_initial_configuration(num_dof);
  arbitrary_initial_configuration << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;
  x0.head(num_dof) += arbitrary_initial_configuration;

  auto controlled_robot =
      std::allocate_shared<GravityCompensatedSystem<RigidBodySystem>>(
          Eigen::aligned_allocator<GravityCompensatedSystem<RigidBodySystem>>(),
          iiwa_system);

  // Applying a small input torque on 5th joint.
  VectorXd input_torque_vector = VectorXd::Zero(num_dof);
  input_torque_vector(4) = 1.75;

  auto input_torque = std::make_shared<
      AffineSystem<NullVector, NullVector, RigidBodySystem::StateVector>>(
      MatrixXd::Zero(0, 0), MatrixXd::Zero(0, 0), VectorXd::Zero(0),
      MatrixXd::Zero(num_dof, 0), MatrixXd::Zero(num_dof, 0),
      input_torque_vector);
  auto sys =
      cascade(cascade(cascade(input_torque, controlled_robot), visualizer),
              robot_state_tap);

  drake::SimulationOptions options = SetupSimulation();

  // Specifies the start time of the simulation.
  const double kStartTime = 0;

  // Specifies the duration of the simulation.
  const double kDuration = 0.75;

  EXPECT_NO_THROW(
      drake::simulate(*sys.get(), kStartTime, kDuration, x0, options));

  auto xf = robot_state_tap->get_input_vector();

  // Ensures joint position and velocity limits are not violated.
  EXPECT_NO_THROW(CheckLimitViolations(iiwa_system, xf));

  // Expects norm of the joint position difference to be greater that a
  // minimum value.
  double kMinPositionErrorNorm = 1e-3;
  EXPECT_TRUE((xf.head(num_dof) - x0.head(num_dof)).squaredNorm() >
              kMinPositionErrorNorm);

  // Expects final joint velocity has a norm larger than a minimum value (Since
  // the torque input results in continuous acceleration).
  double kMinVelocityNorm = 1e-3;
  EXPECT_TRUE(xf.tail(num_dof).squaredNorm() > kMinVelocityNorm);
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
