#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {

using parsers::ModelInstanceIdTable;
using parsers::urdf::AddModelInstanceFromUrdfFile;

using lcm::DrakeLcm;
using systems::ConstantVectorSource;
using systems::Context;
using systems::ContinuousState;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::VectorBase;
using multibody::joints::kFixed;

namespace examples {
namespace kuka_iiwa_arm {
namespace {

DEFINE_double(duration, 5, "Total duration of the simulation in seconds.");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;

  // Adds a plant
  RigidBodyPlant<double>* plant = nullptr;
  const std::string kModelPath =
      "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14.urdf";
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::multibody::AddFlatTerrainToWorld(tree.get());
    CreateTreedFromFixedModelAtPose(kModelPath, tree.get());
    plant = builder.AddPlant(std::move(tree));
  }
  // Creates and adds LCM publisher for visualization.
  builder.AddVisualizer(&lcm);

  // Feed in constant inputs of zero into the RigidBodyPlant.
  systems::DiagramBuilder<double>* base_builder = builder.get_mutable_builder();
  VectorX<double> zero_value = VectorX<double>::Zero(plant->get_input_size());
  auto zero_source =
      base_builder->template AddSystem<ConstantVectorSource<double>>(
          zero_value);
  base_builder->Connect(zero_source->get_output_port(),
                        plant->get_input_port(0));

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  Simulator<double> simulator(*diagram);

  simulator.Initialize();

  // Simulate for the desired duration.
  simulator.set_target_realtime_rate(1.0);
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
        "ERROR: Size of position vector (" + std::to_string(num_q) + ") does "
        "not match number of positions in RigidBodyTree (" +
        std::to_string(plant->get_num_positions()) + ").");
  }

  if (num_v != plant->get_num_velocities()) {
    throw std::runtime_error(
        "ERROR: Size of velocity vector (" + std::to_string(num_v) + ") does "
        "not match number of velocities in RigidBodyTree (" +
        std::to_string(plant->get_num_velocities()) + ").");
  }

  // Ensures the number of position states equals the number of velocity states.
  if (num_q != num_v) {
    throw std::runtime_error(
        "ERROR: Number of positions (" + std::to_string(num_q) + ") does not "
        "match the number of velocities (" + std::to_string(num_v) + ").");
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
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::kuka_iiwa_arm::main(argc, argv);
}
