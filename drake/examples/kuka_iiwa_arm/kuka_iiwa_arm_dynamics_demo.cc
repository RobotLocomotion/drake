#include "iiwa_simulation.h"

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

int do_main(int argc, const char *argv[]) {
  auto iiwa_system = CreateRigidBodySystem(argc, argv);

  const auto& iiwa_tree = iiwa_system->getRigidBodyTree();
  SetupWorld(iiwa_tree);

  // Initializes LCM.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Instantiates additional systems and cascades them with the rigid body
  // system.
  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, iiwa_tree);

  auto robot_state_tap =
      std::make_shared<RobotStateTap<RigidBodySystem::StateVector>>();

  //auto iiwa_feedback_system = PDControlSystem<RigidBodySystem>()
  auto sys = cascade(cascade(iiwa_system, visualizer), robot_state_tap);

  // Obtains an initial state of the simulation.
  VectorXd x0 = VectorXd::Zero(iiwa_system->getNumStates());
  x0.head(iiwa_tree->number_of_positions()) = iiwa_tree->getZeroConfiguration();

  // Instantates a variable that specifies the duration of the simulation.
  // The default value is 5 seconds.
  double duration = 5.0;

  Drake::SimulationOptions options = SetupSimulation(duration);

  for (int actuator_idx = 0; actuator_idx < iiwa_tree->actuators.size(); actuator_idx++) {
    const std::string &actuator_name = iiwa_tree->actuators[actuator_idx].name;
    std::cout << "Actuator : " << actuator_name << "\n";
  }

  Eigen::MatrixXd Kp = Eigen::MatrixXd::VectorXd(7,100.0).asDiagonal();
  Eigen::MatrixXd Kd = Eigen::MatrixXd::Constant(getNumInputs(*iiwa_system),
                                                 iiwa_tree->number_of_velocities());

  auto robot_with_pd = std::allocate_shared<PDControlSystem<RigidBodySystem>>(
      Eigen::aligned_allocator<PDControlSystem<RigidBodySystem>>(),
          iiwa_system, Kp, Kd);
  // Starts the simulation.
  const double kStartTime = 0;
  Drake::simulate(*sys.get(), kStartTime, duration, x0, options);


//  auto final_robot_state = robot_state_tap->get_input_vector();
//  int num_positions = iiwa_system->number_of_positions();
//  int num_velocities = iiwa_system->number_of_velocities();
//
//  // Ensures the size of the output is correct.
//  if (final_robot_state.size() != iiwa_system->getNumOutputs()) {
//    throw std::runtime_error(
//        "ERROR: Size of final robot state (" +
//            std::to_string(final_robot_state.size()) +
//            ") does not match size of rigid body system's output (" +
//            std::to_string(iiwa_system->getNumOutputs()) + ").");
//  }
//
//  // Ensures the number of position states equals the number of velocity states.
//  if (num_positions != num_velocities) {
//    throw std::runtime_error("ERROR: Number of positions (" +
//        std::to_string(num_positions) +
//        ") does not match the number of velocities (" +
//        std::to_string(num_velocities) + ").");
//  }
//
//  // Ensures the number of position and velocity states match the size of the
//  // final robot state.
//  if ((num_positions + num_velocities) != final_robot_state.size()) {
//    throw std::runtime_error(
//        "ERROR: Total number of positions and velocities (" +
//            std::to_string(num_positions + num_velocities) +
//            ") does not match size of robot state (" +
//            std::to_string(final_robot_state.size()) + ").");
//  }
//
//  // Ensures the robot's joints are within their position limits.
//  std::vector<std::unique_ptr<RigidBody>>& bodies = iiwa_tree->bodies;
//  for (int robot_state_index = 0, body_index = 0; body_index < bodies.size();
//       ++body_index) {
//    // Skips rigid bodies without a parent (this includes the world link).
//    if (!bodies[body_index]->hasParent()) continue;
//
//    const DrakeJoint& joint = bodies[body_index]->getJoint();
//    const Eigen::VectorXd& min_limit = joint.getJointLimitMin();
//    const Eigen::VectorXd& max_limit = joint.getJointLimitMax();
//
//    // Defines a joint limit tolerance. This is the amount in radians over which
//    // joint position limits can be violated and still be considered to be
//    // within the limits. Once we are able to model joint limits via
//    // constraints, we may be able to remove the need for this tolerance value.
//    const double kJointLimitTolerance = 0.0261799;  // 1.5 degrees.
//
//    for (int ii = 0; ii < joint.getNumPositions(); ++ii) {
//      double position = final_robot_state[robot_state_index++];
//      if (position < min_limit[ii] - kJointLimitTolerance) {
//        throw std::runtime_error("ERROR: Joint " + joint.getName() + " (DOF " +
//            joint.getPositionName(ii) +
//            ") violated minimum position limit (" +
//            std::to_string(position) + " < " +
//            std::to_string(min_limit[ii]) + ").");
//      }
//      if (position > max_limit[ii] + kJointLimitTolerance) {
//        throw std::runtime_error("ERROR: Joint " + joint.getName() + " (DOF " +
//            joint.getPositionName(ii) +
//            ") violated maximum position limit (" +
//            std::to_string(position) + " > " +
//            std::to_string(max_limit[ii]) + ").");
//      }
//    }
//  }

  return 0;
}

} // close namespace
} // close namespace kuka_iiwa_arm
} // close namespace examples
} // close namespace drake

int main(int argc, const char *argv[])
{
  return drake::examples::kuka_iiwa_arm::do_main(argc,argv);
}