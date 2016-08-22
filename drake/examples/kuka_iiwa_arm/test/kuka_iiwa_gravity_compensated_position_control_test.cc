#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/gravity_compensated_pd_position_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/robot_state_tap.h"
#include "drake/systems/Simulation.h"

using drake::AffineSystem;
using drake::BotVisualizer;
using lcm::LCM;
using Eigen::MatrixXd;
using drake::RobotStateTap;
using drake::RigidBodySystem;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

// Test to verify behavior of the KUKA IIWA Arm under a gravity
// compensated PD joint position controller. Even under lower gains
// (in contrast with a pure PD controller), accurate position control
// is possible. The test looks for 2 things : (i) with the set-point
// identical with the initial pose, the controller maintains the robot
// pose with negligible errors over a given duration; (ii) the controller
// maintains this pose with a negligible velocity error.
GTEST_TEST(testIIWAArm, iiwaArmGravityCompensationControl) {
  std::shared_ptr<RigidBodySystem> iiwa_system = CreateKukaIiwaSystem();

  const auto& iiwa_tree = iiwa_system->getRigidBodyTree();

  // Initializes LCM.
  std::shared_ptr<LCM> lcm = std::make_shared<LCM>();

  // Instantiates additional systems and cascades them with the rigid body
  // system.
  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,
                                                                    iiwa_tree);

  auto robot_state_tap =
      std::make_shared<RobotStateTap<RigidBodySystem::StateVector>>();

  int num_dof = iiwa_tree->number_of_positions();

  // Smaller gains intentionally used for demo.
  const double Kp_common = 10.0;  // Units : Nm/rad
  const double Kd_common = 0.30;  // Units : Nm/rad/sec
  VectorXd Kpdiag = VectorXd::Constant(num_dof, Kp_common);
  VectorXd Kddiag = VectorXd::Constant(num_dof, Kd_common);

  MatrixXd Kp = Kpdiag.asDiagonal();
  MatrixXd Kd = Kddiag.asDiagonal();

  // Obtains an initial state of the simulation.
  VectorXd x0 = VectorXd::Zero(iiwa_system->getNumStates());
  x0.head(iiwa_tree->number_of_positions()) = iiwa_tree->getZeroConfiguration();

  Eigen::VectorXd random_initial_configuration(num_dof);
  random_initial_configuration << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;
  x0.head(num_dof) += random_initial_configuration;

  // Set point is the initial configuration.
  VectorXd set_point_vector = x0.head(num_dof);
  auto set_point = std::make_shared<
      AffineSystem<NullVector, NullVector, RigidBodySystem::StateVector>>(
      MatrixXd::Zero(0, 0), MatrixXd::Zero(0, 0), VectorXd::Zero(0),
      MatrixXd::Zero(num_dof, 0), MatrixXd::Zero(num_dof, 0), set_point_vector);

  auto controlled_robot = std::allocate_shared<
      GravityCompensatedPDPositionControlSystem<RigidBodySystem>>(
      Eigen::aligned_allocator<
          GravityCompensatedPDPositionControlSystem<RigidBodySystem>>(),
      iiwa_system, Kp, Kd);

  auto sys = cascade(cascade(cascade(set_point, controlled_robot), visualizer),
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

  // Expects norm of the joint position error to be below a maximum value.
  double kMaxPositionErrorNorm = 1e-3;
  EXPECT_TRUE((xf.head(num_dof) - set_point_vector).squaredNorm() <
              kMaxPositionErrorNorm);

  // Expects final joint velocity has a norm to be smaller than a minimum value.
  double kMinVelocityNorm = 1e-3;
  EXPECT_TRUE(xf.tail(num_dof).squaredNorm() < kMinVelocityNorm);
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
