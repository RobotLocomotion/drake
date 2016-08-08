#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/examples/kuka_iiwa_arm/robot_state_tap.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"

using drake::RigidBodySystem;
using drake::BotVisualizer;
using Eigen::VectorXd;
using drake::RobotStateTap;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

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

  // Obtains an initial state of the simulation.
  VectorXd x0 = VectorXd::Zero(iiwa_system->getNumStates());
  x0.head(iiwa_tree->number_of_positions()) = iiwa_tree->getZeroConfiguration();

  Eigen::VectorXd random_initial_configuration(7);
  random_initial_configuration << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;
  x0.head(7) += random_initial_configuration;

  drake::SimulationOptions options = SetupSimulation();

  // Starts the simulation.
  const double kStartTime = 0;

  // Simulation duration in seconds.
  const double kDuration = 0.5;

  drake::simulate(*sys.get(), kStartTime, kDuration, x0, options);

  // TODO(naveenoid) : Test for final state != initial state.
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
