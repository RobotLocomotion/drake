/// @file
///
/// This demo sets up a simple passive dynamics simulation of the Kinova Jaco
/// arm. The robot is initialized with an (arbitrary) joint space pose, and is
/// simulated with zero torques at the joints.
///
/// This simulation uses a 6-degree of freedom Kinova Jaco arm with a three
/// finger gripper. Joints are numbered sequentually starting from the base
/// with the following joint index descriptions:
/// 0: shoulder roll
/// 1: shoulder fore/aft
/// 2: elbow fore/aft
/// 3: forearm roll
/// 4: wrist yaw
/// 5: wrist roll
/// 6: finger 1 bend/extend
/// 7: finger 2 bend/extend
/// 8: finger 3 bend/extend
///
/// Position units are in radians, velocity units are radians in per second

#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kinova_jaco_arm/jaco_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {

using systems::ConstantVectorSource;
using systems::Context;
using systems::ContinuousState;
using systems::RigidBodyPlant;
using systems::VectorBase;

namespace examples {
namespace kinova_jaco_arm {
namespace {

DEFINE_double(duration, 2, "Total duration of the simulation in seconds.");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  drake::lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  // Adds a plant.
  RigidBodyPlant<double>* plant = nullptr;
  const std::string kModelPath =
      "/manipulation/models/jaco_description/urdf/"
      "j2n6s300.urdf";
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::multibody::AddFlatTerrainToWorld(tree.get());
    CreateTreedFromFixedModelAtPose(kModelPath, tree.get());

    auto tree_sys =
        std::make_unique<RigidBodyPlant<double>>(std::move(tree));
    plant = builder.AddSystem<RigidBodyPlant<double>>(
        std::move(tree_sys));
    plant->set_name("plant");
  }

  // Verifies the tree.
  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  VerifyJacoTree(tree);

  // Creates and adds LCM publisher for visualization.
  auto visualizer =
      builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);

  // Feeds in constant command inputs of zero.
  VectorX<double> zero_values = VectorX<double>::Zero(plant->get_input_size());
  auto zero_source =
      builder.AddSystem<ConstantVectorSource<double>>(zero_values);
  zero_source->set_name("zero_source");
  builder.Connect(zero_source->get_output_port(), plant->get_input_port(0));

  // Connects the visualizer and builds the diagram.
  builder.Connect(plant->get_output_port(0), visualizer->get_input_port(0));
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  Context<double>* jaco_context = diagram->GetMutableSubsystemContext(
      simulator.get_mutable_context(), plant);

  // Sets (arbitrary) initial conditions.
  // See the @file docblock at the top of this file for joint index descriptions.
  VectorBase<double>* x0 = jaco_context->get_mutable_continuous_state_vector();
  x0->SetAtIndex(1, 0.5);

  simulator.Initialize();

  // Simulate for the desired duration.
  simulator.set_target_realtime_rate(1);
  simulator.StepTo(FLAGS_duration);

  // Ensures the simulation was successful.
  const Context<double>& context = simulator.get_context();
  const ContinuousState<double>* state = context.get_continuous_state();
  const VectorBase<double>& position_vector = state->get_generalized_position();
  const VectorBase<double>& velocity_vector = state->get_generalized_velocity();

  const int num_q = position_vector.size();
  const int num_v = velocity_vector.size();

  // Ensures the sizes of the position and velocity vectors are correct.
  if (num_q != plant->get_num_positions()) {
    throw std::runtime_error(
        "ERROR: Size of position vector (" + std::to_string(num_q) +
        ") does "
        "not match number of positions in RigidBodyTree (" +
        std::to_string(plant->get_num_positions()) + ").");
  }

  if (num_v != plant->get_num_velocities()) {
    throw std::runtime_error(
        "ERROR: Size of velocity vector (" + std::to_string(num_v) +
        ") does "
        "not match number of velocities in RigidBodyTree (" +
        std::to_string(plant->get_num_velocities()) + ").");
  }

  // Ensures the number of position states equals the number of velocity states.
  if (num_q != num_v) {
    throw std::runtime_error("ERROR: Number of positions (" +
                             std::to_string(num_q) +
                             ") does not "
                             "match the number of velocities (" +
                             std::to_string(num_v) + ").");
  }

  // Ensures the robot's joints are within their position limits.
  const std::vector<std::unique_ptr<RigidBody<double>>>& bodies =
      plant->get_rigid_body_tree().bodies;
  for (int state_index = 0, i = 0; i < static_cast<int>(bodies.size()); ++i) {
    // Skips rigid bodies without a parent. This includes the world.
    if (!bodies[i]->has_parent_body()) continue;

    const DrakeJoint& joint = bodies[i]->getJoint();
    const Eigen::VectorXd& min_limit = joint.getJointLimitMin();
    const Eigen::VectorXd& max_limit = joint.getJointLimitMax();

    // Defines a joint limit tolerance. This is the amount in radians over which
    // joint position limits can be violated and still be considered to be
    // within the limits. Once we are able to model joint limits via
    // constraints, we may be able to remove this tolerance value.
    const double kJointLimitTolerance = 0.0261799;  // 1.5 degrees.

    for (int j = 0; j < joint.get_num_positions(); ++j) {
      double position = position_vector.GetAtIndex(state_index++);
      if (position < min_limit[j] - kJointLimitTolerance) {
        throw std::runtime_error("ERROR: Joint " + joint.get_name() + " (DOF " +
                                 joint.get_position_name(j) +
                                 ") violated minimum position limit (" +
                                 std::to_string(position) + " < " +
                                 std::to_string(min_limit[j]) + ").");
      }
      if (position > max_limit[j] + kJointLimitTolerance) {
        throw std::runtime_error("ERROR: Joint " + joint.get_name() + " (DOF " +
                                 joint.get_position_name(j) +
                                 ") violated maximum position limit (" +
                                 std::to_string(position) + " > " +
                                 std::to_string(max_limit[j]) + ").");
      }
    }
  }

  return 0;
}

}  // namespace
}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::kinova_jaco_arm::DoMain(argc, argv);
}
