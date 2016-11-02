#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"

#include <vector>

#include "drake/common/drake_path.h"
#include "drake/systems/plants/joints/floating_base_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using lcm::LCM;

std::shared_ptr<RigidBodySystem> CreateKukaIiwaSystem(void) {
  // Instantiates a rigid body system and adds the robot arm to it.
  auto rigid_body_system = std::allocate_shared<RigidBodySystem>(
      Eigen::aligned_allocator<RigidBodySystem>());

  rigid_body_system->AddModelInstanceFromFile(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      drake::systems::plants::joints::kFixed);

  // Sets some simulation parameters.
  rigid_body_system->penetration_stiffness = 3000.0;
  rigid_body_system->penetration_damping = 0;

  // Adds the ground.
  double kBoxWidth = 3;
  double kBoxDepth = 0.2;
  DrakeShapes::Box geom(Eigen::Vector3d(kBoxWidth, kBoxWidth, kBoxDepth));
  Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
  T_element_to_link.translation() << 0, 0,
      -kBoxDepth / 2.0;  // top of the box is at z = 0

  const std::shared_ptr<RigidBodyTree>& tree =
      rigid_body_system->getRigidBodyTree();

  RigidBody& world = tree->world();
  Eigen::Vector4d color;
  color << 0.9297, 0.7930, 0.6758,
      1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
  world.AddVisualElement(
      DrakeShapes::VisualElement(geom, T_element_to_link, color));
  tree->addCollisionElement(
      DrakeCollision::Element(geom, T_element_to_link, &world), world,
      "terrain");
  tree->updateStaticCollisionElements();

  return rigid_body_system;
}

std::shared_ptr<BotVisualizer<RigidBodySystem::StateVector>>
    CreateKukaIiwaVisualizer(
    const std::shared_ptr<drake::RigidBodySystem> iiwa_system,
    const std::shared_ptr<lcm::LCM> lcm) {

  // Extracts the tree.
  const auto& iiwa_tree = iiwa_system->getRigidBodyTree();

  // Instantiates a BotVisualizer for @p iiwa_system
  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,
                                                                    iiwa_tree);
  return visualizer;
}

DRAKE_EXPORT
Eigen::VectorXd GenerateArbitraryIiwaInitialState() {
  const int kStateDimension = 14;  // Fixed for the IIWA Arm.
  const int kNumDof = 7;  // Fixed for the IIWA Arm.
  Eigen::VectorXd arbitrary_initial_state =
      Eigen::VectorXd::Zero(kStateDimension, 1);
  arbitrary_initial_state.head(kNumDof) << 0.01, -0.01, 0.01, 0.5,
      0.01, -0.01, 0.01;
  return(arbitrary_initial_state);
}

drake::SimulationOptions SetupSimulation(double initial_step_size) {
  // Specifies the simulation options.
  drake::SimulationOptions options;
  options.realtime_factor = 0;  // As fast as possible.
  options.initial_step_size = initial_step_size;

  // Prevents exception from being thrown when simulation runs slower than real
  // time, which it most likely will given the small step size.
  options.warn_real_time_violation = true;
  return (options);
}

void CheckLimitViolations(
    const std::shared_ptr<drake::RigidBodySystem> rigid_body_sys,
    const Eigen::VectorXd& final_robot_state) {
  const auto& tree = rigid_body_sys->getRigidBodyTree();
  int num_positions = rigid_body_sys->get_num_positions();
  int num_velocities = rigid_body_sys->get_num_velocities();

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
       body_index < static_cast<int>(bodies.size()); ++body_index) {
    // Skips rigid bodies without a mobilizer joint. This includes the RigidBody
    // that represents the world.
    if (!bodies[body_index]->has_parent_body()) continue;

    const DrakeJoint& joint = bodies[body_index]->getJoint();
    const Eigen::VectorXd& min_limit = joint.getJointLimitMin();
    const Eigen::VectorXd& max_limit = joint.getJointLimitMax();

    // Defines a joint limit tolerance. This is the amount in radians over which
    // joint position limits can be violated and still be considered within the
    // limits. Once we are able to model joint limits via
    // constraints, we may be able to remove the need for this tolerance value.
    const double kJointLimitTolerance = 0.0261799;  // 1.5 degrees.

    for (int i = 0; i < joint.get_num_positions(); ++i) {
      double position = final_robot_state[robot_state_index++];
      if (position < min_limit[i] - kJointLimitTolerance) {
        throw std::runtime_error("ERROR: Joint " + joint.get_name() + " (DOF " +
                                 joint.get_position_name(i) +
                                 ") violated minimum position limit (" +
                                 std::to_string(position) + " < " +
                                 std::to_string(min_limit[i]) + ").");
      }
      if (position > max_limit[i] + kJointLimitTolerance) {
        throw std::runtime_error("ERROR: Joint " + joint.get_name() + " (DOF " +
                                 joint.get_position_name(i) +
                                 ") violated maximum position limit (" +
                                 std::to_string(position) + " > " +
                                 std::to_string(max_limit[i]) + ").");
      }
    }
  }
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
