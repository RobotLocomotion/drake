#include <iostream>

#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/robot_state_tap.h"
#include "drake/common/polynomial.h"
#include "drake/util/drakeAppUtil.h"

using drake::RigidBodySystem;
using drake::BotVisualizer;
using drake::cascade;
using drake::RobotStateTap;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

int DoMain(int argc, char* argv[]) {
  // Initializes LCM.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  std::shared_ptr<RigidBodySystem> rigid_body_sys = CreateKukaIiwaSystem();
  // Obtains a reference to the rigid body tree within the rigid body system.
  const auto& tree = rigid_body_sys->getRigidBodyTree();

  // Instantiates additional systems and cascades them with the rigid body
  // system.
  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto robot_state_tap =
      std::make_shared<RobotStateTap<RigidBodySystem::StateVector>>();

  auto sys = cascade(cascade(rigid_body_sys, visualizer), robot_state_tap);

  // Obtains an initial state of the simulation.
  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());
  x0.head(tree->number_of_positions()) = tree->getZeroConfiguration();

  // Specifies the simulation options.
  drake::SimulationOptions options;
  options.realtime_factor = 0;  // As fast as possible.
  options.initial_step_size = 0.002;

  // Prevents exception from being thrown when simulation runs slower than real
  // time, which it most likely will given the small step size.
  options.warn_real_time_violation = true;

  // Instantates a variable that specifies the duration of the simulation.
  // The default value is 5 seconds.
  double duration = 5.0;

  // Searches through the command line looking for a "--duration" flag followed
  // by a floating point number that specifies a custom duration.
  for (int ii = 1; ii < argc; ++ii) {
    if (std::string(argv[ii]) == "--duration") {
      if (++ii == argc) {
        throw std::runtime_error(
            "ERROR: Command line option \"--duration\" is not followed by a "
            "value!");
      }
      duration = atof(argv[ii]);
    }
  }

  // Starts the simulation.
  const double kStartTime = 0;
  drake::simulate(*sys.get(), kStartTime, duration, x0, options);

  auto final_robot_state = robot_state_tap->get_input_vector();
  int num_positions = rigid_body_sys->number_of_positions();
  int num_velocities = rigid_body_sys->number_of_velocities();

  // Ensures the size of the output is correct.
  if (final_robot_state.size() !=
      static_cast<int>(rigid_body_sys->getNumOutputs())) {
    throw std::runtime_error(
        "ERROR: Size of final robot state (" +
        std::to_string(final_robot_state.size()) +
        ") does not match size of rigid body system's output (" +
        std::to_string(rigid_body_sys->getNumOutputs()) + ").");
  }

  // Ensures the number of position states equals the number of velocity states.
  if (num_positions != num_velocities) {
    throw std::runtime_error("ERROR: Number of positions (" +
                             std::to_string(num_positions) +
                             ") does not match the number of velocities (" +
                             std::to_string(num_velocities) + ").");
  }

  // Ensures the number of position and velocity states match the size of the
  // final robot state.
  if ((num_positions + num_velocities) != final_robot_state.size()) {
    throw std::runtime_error(
        "ERROR: Total number of positions and velocities (" +
        std::to_string(num_positions + num_velocities) +
        ") does not match size of robot state (" +
        std::to_string(final_robot_state.size()) + ").");
  }

  // Ensures the robot's joints are within their position limits.
  std::vector<std::unique_ptr<RigidBody>>& bodies = tree->bodies;
  for (int robot_state_index = 0, body_index = 0;
       body_index < static_cast<int>(bodies.size());
       ++body_index) {
    // Skips rigid bodies without a mobilizer joint. This includes the RigidBody
    // that represents the world.
    if (!bodies[body_index]->has_mobilizer_joint()) continue;

    const DrakeJoint& joint = bodies[body_index]->getJoint();
    const Eigen::VectorXd& min_limit = joint.getJointLimitMin();
    const Eigen::VectorXd& max_limit = joint.getJointLimitMax();

    // Defines a joint limit tolerance. This is the amount in radians over which
    // joint position limits can be violated and still be considered to be
    // within the limits. Once we are able to model joint limits via
    // constraints, we may be able to remove the need for this tolerance value.
    const double kJointLimitTolerance = 0.0261799;  // 1.5 degrees.

    for (int ii = 0; ii < joint.getNumPositions(); ++ii) {
      double position = final_robot_state[robot_state_index++];
      if (position < min_limit[ii] - kJointLimitTolerance) {
        throw std::runtime_error("ERROR: Joint " + joint.getName() + " (DOF " +
                                 joint.getPositionName(ii) +
                                 ") violated minimum position limit (" +
                                 std::to_string(position) + " < " +
                                 std::to_string(min_limit[ii]) + ").");
      }
      if (position > max_limit[ii] + kJointLimitTolerance) {
        throw std::runtime_error("ERROR: Joint " + joint.getName() + " (DOF " +
                                 joint.getPositionName(ii) +
                                 ") violated maximum position limit (" +
                                 std::to_string(position) + " > " +
                                 std::to_string(max_limit[ii]) + ").");
      }
    }
  }

  // Unfortunately we cannot check the joint velocities since the velocity
  // limits are not being parsed.
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::kuka_iiwa_arm::DoMain(argc, argv);
}
