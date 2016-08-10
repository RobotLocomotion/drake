#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/examples/kuka_iiwa_arm/robot_state_tap.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/util/eigen_matrix_compare.h"

using drake::RigidBodySystem;
using drake::BotVisualizer;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::RobotStateTap;
using drake::PDControlSystem;
using drake::AffineSystem;
using drake::util::MatrixCompareType;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

// Test to verify behaviour of the KUKA IIWA Arm under a PD joint position
// controller. Since the robot is present in environment with gravity
// The controller will not be able to stabilise the position precisely
// and oscillations about the set-point will always be present in the
// behaviour. The test verifies that the position reached is within some
// bound with respect to the initial condition.
GTEST_TEST(testIIWAArm, iiwaArmPDControl) {
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

  // Large gains intentionally used for demo.
  const double Kp_common = 500.0;
  const double Kd_common = 0.00;
  VectorXd Kpdiag = VectorXd::Constant(num_dof, Kp_common);
  Kpdiag(1) = 800.0;
  Kpdiag(3) = 700.0;
  VectorXd Kddiag = VectorXd::Constant(num_dof, Kd_common);
  MatrixXd Kp = Kpdiag.asDiagonal();
  MatrixXd Kd = Kddiag.asDiagonal();

  // Obtains an initial state of the simulation.
  VectorXd x0 = VectorXd::Zero(iiwa_system->getNumStates());
  x0.head(num_dof) = iiwa_tree->getZeroConfiguration();

  Eigen::VectorXd random_initial_configuration(num_dof);
  random_initial_configuration << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;
  x0.head(num_dof) += random_initial_configuration;

  // Set point is the initial state.
  auto set_point = std::make_shared<
      AffineSystem<NullVector, NullVector, RigidBodySystem::StateVector>>(
      MatrixXd::Zero(0, 0), MatrixXd::Zero(0, 0), VectorXd::Zero(0),
      MatrixXd::Zero(num_dof, 0), MatrixXd::Zero(num_dof, 0),
      x0.head(num_dof));

  auto controlled_robot =
      std::allocate_shared<PDControlSystem<RigidBodySystem>>(
          Eigen::aligned_allocator<PDControlSystem<RigidBodySystem>>(),
          iiwa_system, Kp, Kd);

  auto sys = cascade(cascade(cascade(set_point, controlled_robot),
                             visualizer), robot_state_tap);

  drake::SimulationOptions options = SetupSimulation();

  // Specifies the start time of the simulation.
  const double kStartTime = 0;

  // Specifies the duration of the simulation.
  const double kDuration = 1.0;

  EXPECT_NO_THROW(drake::simulate(*sys.get(), kStartTime, kDuration, x0,
                                  options));

  auto xf = robot_state_tap->get_input_vector();

  // Ensure joint position and velocity limits are not violated.
  EXPECT_NO_THROW(CheckLimitViolations(iiwa_system, xf));

  // Expect normed joint position difference is below a maximum value.
  double max_position_norm = 1e-5;
  EXPECT_TRUE((xf.head(num_dof) - x0.head(num_dof)).squaredNorm()
                  > max_position_norm);

  // Expect final normed joint velocity is larger than a minimum value.
  // (since this controller wont stay at rest at the set-point).
  double min_velocity_norm = 1e-5;
  EXPECT_TRUE(xf.tail(num_dof).squaredNorm() > min_velocity_norm);
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
