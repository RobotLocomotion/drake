#include "iiwa_simulation.h"

#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/Path.h"
#include "drake/examples/kuka_iiwa_arm/robot_state_tap.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"

using Drake::RigidBodySystem;
using Drake::BotVisualizer;
using Eigen::VectorXd;
using drake::RobotStateTap;
namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

GTEST_TEST(testIIWAArm, iiwaArmDynamics) {
  auto iiwa_system = CreateIIWAArmSystem();

  const auto& iiwa_tree = iiwa_system->getRigidBodyTree();
  SetupWorld(iiwa_tree);

  // Initializes LCM.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Instantiates additional systems and cascades them with the rigid body
  // system.
  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,
                                                                    iiwa_tree);

  auto robot_state_tap =
      std::make_shared<RobotStateTap<RigidBodySystem::StateVector>>();

  //  Build up PD controller in this form.
  //  Eigen::MatrixXd Kp = Eigen::VectorXd::Constant(7,100.0).asDiagonal();
  //  Eigen::MatrixXd Kd = Eigen::VectorXd::Constant(7,50.0).asDiagonal();
  //  auto robot_with_pd =
  //  std::allocate_shared<PDControlSystem<RigidBodySystem>>(
  //      Eigen::aligned_allocator<PDControlSystem<RigidBodySystem>>(),
  //          iiwa_system, Kp, Kd);
  //  auto iiwa_feedback_system = PDControlSystem<RigidBodySystem>()

  auto sys = cascade(cascade(iiwa_system, visualizer), robot_state_tap);

  // Obtains an initial state of the simulation.
  VectorXd x0 = VectorXd::Zero(iiwa_system->getNumStates());
  x0.head(iiwa_tree->number_of_positions()) = iiwa_tree->getZeroConfiguration();

  Eigen::VectorXd random_initial_configuration(7);
  random_initial_configuration << 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01;
  x0.head(7) += random_initial_configuration;

  Drake::SimulationOptions options = SetupSimulation();

  // Starts the simulation.
  const double kStartTime = 0;

  // Simulation duration in seconds.
  double duration = 0.5;

  Drake::simulate(*sys.get(), kStartTime, duration, x0, options);
  //
  //  ASSERT_NO_THROW(ValidateSimulation(robot_state_tap, iiwa_system));
}

}  // close namespace
}  // close namespace kuka_iiwa_arm
}  // close namespace examples
}  // close namespace drake
