#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/examples/kuka_iiwa_arm/robot_state_tap.h"
#include "drake/common/drake_path.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/Simulation.h"


using drake::RigidBodySystem;
using drake::BotVisualizer;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::RobotStateTap;
using drake::PDControlSystem;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

GTEST_TEST(testIIWAArm, iiwaArmPDControl) {
std::shared_ptr<RigidBodySystem> iiwa_system = CreateKukaIiwaSystem();

const auto& iiwa_tree = iiwa_system->getRigidBodyTree();

// Initializes LCM.
std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

// Instantiates additional systems and cascades them with the rigid body
// system.
auto visualizer =
    std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(
        lcm, iiwa_tree);

auto robot_state_tap =
    std::make_shared<RobotStateTap<RigidBodySystem::StateVector>>();

const double Kp_common = 100.0;
const double Kd_common = 25.0;
VectorXd Kpdiag = VectorXd::Constant(7, Kp_common);
VectorXd Kddiag = VectorXd::Constant(7, Kd_common);
MatrixXd Kp = Kpdiag.asDiagonal();
MatrixXd Kd = Kddiag.asDiagonal();


auto controlled_robot = std::allocate_shared<PDControlSystem<RigidBodySystem>>(
      Eigen::aligned_allocator<PDControlSystem<RigidBodySystem>>(),
      iiwa_system, Kp, Kd);
//
//auto controlled_robot =
//  std::make_shared<PDControlSystem<RigidBodySystem>>(iiwa_system, Kp, Kd);


auto sys = cascade(cascade(controlled_robot, visualizer), robot_state_tap);
//auto sys = cascade(cascade(iiwa_system, visualizer), robot_state_tap);

// Obtains an initial state of the simulation.
VectorXd x0 = VectorXd::Zero(iiwa_system->getNumStates());
x0.head(iiwa_tree->number_of_positions()) = iiwa_tree->getZeroConfiguration();

Eigen::VectorXd random_initial_configuration(7);
random_initial_configuration << 0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01;
x0.head(7) += random_initial_configuration;

drake::SimulationOptions options = SetupSimulation();

// Starts the simulation.
const double kStartTime = 0;

// Simulation duration in seconds.
const double kDuration = 5.0;

drake::simulate(*sys.get(), kStartTime, kDuration, x0, options);
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
