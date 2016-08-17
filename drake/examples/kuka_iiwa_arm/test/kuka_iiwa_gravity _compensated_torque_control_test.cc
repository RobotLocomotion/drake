#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/examples/kuka_iiwa_arm/robot_state_tap.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/gravity_compensated_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/util/eigen_matrix_compare.h"

using drake::AffineSystem;
using drake::BotVisualizer;
using Eigen::MatrixXd;
using drake::RigidBodySystem;
using drake::GravityCompensatedSystem;
using drake::RobotStateTap;
using Eigen::VectorXd;
using drake::util::MatrixCompareType;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

// Test to verify behavior of the KUKA IIWA Arm under a gravity
// compensation torque controller. Since the robot is present in an
// environment with gravity,
// the controller will not be able to stabilize the position precisely
// and oscillations about the set-point will always be present in the
// behaviour. The test verifies that the position reached is within some
// bound with respect to the initial condition.
GTEST_TEST(testIIWAArm, iiwaArmGravityCompensatedTorqueControl) {
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

  auto sys = cascade(cascade(controlled_robot, visualizer),
                     robot_state_tap);

  drake::SimulationOptions options = SetupSimulation();

  // Specifies the start time of the simulation.
  const double kStartTime = 0;

  // Specifies the duration of the simulation.
  const double kDuration = 1.0;

  EXPECT_NO_THROW(
      drake::simulate(*sys.get(), kStartTime, kDuration, x0, options));

  auto xf = robot_state_tap->get_input_vector();

  // Ensures joint position and velocity limits are not violated.
  EXPECT_NO_THROW(CheckLimitViolations(iiwa_system, xf));

  // Expects norm of the joint position difference to be below a maximum value.
  double kMaxPositionErrorNorm = 1e-3;
  EXPECT_TRUE((xf.head(num_dof) - x0.head(num_dof)).squaredNorm() <
              kMaxPositionErrorNorm);

  std::cout << " Final position : "<<xf.head(num_dof)<<", Errpr norm :"<<
    (xf.head(num_dof) - x0.head(num_dof)).squaredNorm()<<"\n";

  // Expects final joint velocity has a norm larger than a minimum value.
  // (since this controller won't stay at rest at the set-point).
  double kMinVelocityNorm = 1e-3;
  EXPECT_TRUE(xf.tail(num_dof).squaredNorm() < kMinVelocityNorm);
  std::cout << " Final velocity : "<<xf.tail(num_dof)<<", Errpr norm :"<<
    xf.tail(num_dof).squaredNorm()<<"\n";
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
