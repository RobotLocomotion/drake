#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/system1/Simulation.h"
#include "drake/system1/cascade_system.h"
#include "drake/system1/robot_state_tap.h"
#include "drake/systems/plants/RigidBodySystem.h"

using drake::CascadeSystem;
using drake::RigidBodySystem;
using Eigen::VectorXd;
using drake::RobotStateTap;
using drake::MatrixCompareType;

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

  auto robot_state_tap =
      std::make_shared<RobotStateTap<RigidBodySystem::StateVector>>();

  auto sys = cascade(iiwa_system, robot_state_tap);

  // Obtains an initial state of the simulation.
  VectorXd x0 = GenerateArbitraryIiwaInitialState();

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

  // Ensures the initial and final states are not the same (since there is no
  // control, the robot should "collapse" due to the presence of gravity).
  EXPECT_FALSE(CompareMatrices(x0, xf, 1e-3, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
