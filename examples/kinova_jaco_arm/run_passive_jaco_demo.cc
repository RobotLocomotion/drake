/// @file
///
/// This demo sets up a simple passive dynamics simulation of the Kinova Jaco
/// arm. The robot is initialized with an (arbitrary) joint space pose, and is
/// simulated with zero torques at the joints.

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
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

DEFINE_double(simulation_sec, 2, "Number of seconds to simulate");

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);

  drake::lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  // Adds a plant.
  RigidBodyPlant<double>* plant = nullptr;
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::multibody::AddFlatTerrainToWorld(tree.get());
    CreateTreeFromFixedModelAtPose(
        FindResourceOrThrow(
            "drake/manipulation/models/jaco_description/urdf/j2n6s300.urdf"),
        tree.get());

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

  Context<double>& jaco_context = diagram->GetMutableSubsystemContext(
      *plant, &simulator.get_mutable_context());

  // Sets (arbitrary) initial conditions.
  // See the @file docblock in jaco_common.h for joint index descriptions.
  VectorBase<double>& x0 = jaco_context.get_mutable_continuous_state_vector();
  x0.SetAtIndex(1, 0.5);  // shoulder fore/aft angle [rad]

  simulator.Initialize();

  // Simulate for the desired duration.
  simulator.set_target_realtime_rate(1);
  simulator.StepTo(FLAGS_simulation_sec);

  // Ensures the simulation was successful.
  const Context<double>& context = simulator.get_context();
  const ContinuousState<double>& state = context.get_continuous_state();
  const VectorBase<double>& position_vector = state.get_generalized_position();
  const VectorBase<double>& velocity_vector = state.get_generalized_velocity();

  const int num_q = position_vector.size();
  const int num_v = velocity_vector.size();

  // Ensures the sizes of the position and velocity vectors are correct.
  DRAKE_DEMAND(num_q == plant->get_num_positions());
  DRAKE_DEMAND(num_v == plant->get_num_velocities());
  DRAKE_DEMAND(num_q == num_v);

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
        std::cerr << "ERROR: Joint " + joint.get_name() + " (DOF " +
                                 joint.get_position_name(j) +
                                 ") violated minimum position limit (" +
                                 std::to_string(position) + " < " +
                                 std::to_string(min_limit[j]) + ").";
        return 1;
      }
      if (position > max_limit[j] + kJointLimitTolerance) {
        std::cerr << "ERROR: Joint " + joint.get_name() + " (DOF " +
                                 joint.get_position_name(j) +
                                 ") violated maximum position limit (" +
                                 std::to_string(position) + " > " +
                                 std::to_string(max_limit[j]) + ").";
        return 1;
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
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kinova_jaco_arm::DoMain();
}
