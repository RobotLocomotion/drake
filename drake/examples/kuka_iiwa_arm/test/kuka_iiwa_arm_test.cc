#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
<<<<<<< 7ce93bc0a3894986f0a816185dfe089f1f6def32
=======
#include "drake/examples/kuka_iiwa_arm/robot_state_tap.h"
>>>>>>> Working and tuned PID controller with setpoint
#include "drake/systems/LCMSystem.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/plants/BotVisualizer.h"
<<<<<<< 7ce93bc0a3894986f0a816185dfe089f1f6def32
#include "drake/util/eigen_matrix_compare.h"
=======
#include "drake/systems/plants/RigidBodySystem.h"
>>>>>>> Working and tuned PID controller with setpoint

using drake::RigidBodySystem;
using drake::BotVisualizer;
using Eigen::VectorXd;
using drake::RobotStateTap;
using drake::util::MatrixCompareType;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

// This test simulates the dynamic behavior of an uncontrolled IIWA arm
// under the presence of gravity. Without any control, the expected
// behavior is that the robot collapses to the ground. The test verifies
// that the initial and final position after a small duration of time are
// unequal.
GTEST_TEST(testIIWAArm, iiwaArmDynamics) {
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

  auto sys = cascade(cascade(iiwa_system, visualizer), robot_state_tap);

  int num_dof = iiwa_tree->number_of_positions();

  // Obtains an initial state of the simulation.
  VectorXd x0 = VectorXd::Zero(iiwa_system->getNumStates());
  x0.head(num_dof) = iiwa_tree->getZeroConfiguration();

<<<<<<< 7ce93bc0a3894986f0a816185dfe089f1f6def32
  Eigen::VectorXd arbitrary_initial_configuration(num_dof);
  arbitrary_initial_configuration << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;
  x0.head(num_dof) += arbitrary_initial_configuration;
=======
  Eigen::VectorXd random_initial_configuration(7);
  random_initial_configuration << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;
  x0.head(7) += random_initial_configuration;
>>>>>>> Working and tuned PID controller with setpoint

  drake::SimulationOptions options = SetupSimulation();

  // Specifies the start time of the simulation.
  const double kStartTime = 0;

  // Specifies the duration of the simulation.
  const double kDuration = 0.5;
  EXPECT_NO_THROW(
      drake::simulate(*sys.get(), kStartTime, kDuration, x0, options));

  auto xf = robot_state_tap->get_input_vector();

  // Ensures joint position and velocity limits are not violated.
  EXPECT_NO_THROW(CheckLimitViolations(iiwa_system, xf));

<<<<<<< 7ce93bc0a3894986f0a816185dfe089f1f6def32
  // Ensures the initial and final states are not the same (since there is no
  // control, the robot should "collapse" due to the presence of gravity).
  EXPECT_FALSE(CompareMatrices(x0, xf, 1e-3, MatrixCompareType::absolute));
=======
  drake::simulate(*sys.get(), kStartTime, kDuration, x0, options);

  // TODO(naveenoid) : Test for final state != initial state.
>>>>>>> Working and tuned PID controller with setpoint
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
