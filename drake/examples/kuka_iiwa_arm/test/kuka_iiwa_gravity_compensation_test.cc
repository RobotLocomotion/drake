#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/examples/kuka_iiwa_arm/robot_state_tap.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_controller.h"
#include "drake/common/drake_path.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/feedback_system.h"


using drake::RigidBodySystem;
using drake::BotVisualizer;
using Eigen::VectorXd;
using drake::RobotStateTap;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

GTEST_TEST(testIIWAArm, iiwaArmGravityCompensation) {
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

//auto robot_controller = gravityCompensationController(*iiwa_system,
//                                                      iiwa_system->getInitialState());
    //std::make_shared<KukaIIWAGravityCompensationController>(iiwa_system);
//auto tempSys = cascade(feedback(iiwa_system, robot_controller),
//                       visualizer);
auto sys = cascade(cascade(iiwa_system, visualizer), robot_state_tap);


}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
