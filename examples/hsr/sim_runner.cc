#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/hsr/hsr_world.h"
#include "drake/examples/hsr/parameters/parameters.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace hsr {
namespace {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using Eigen::Translation3d;
using Eigen::VectorXd;

int DoMain() {
  // Build a generic multibody plant.
  systems::DiagramBuilder<double> builder;
  auto hsr_world = builder.AddSystem<HsrWorld<double>>("");

  geometry::ConnectDrakeVisualizer(&builder,
                                   hsr_world->get_mutable_scene_graph(),
                                   hsr_world->GetOutputPort("pose_bundle"));

  multibody::ConnectContactResultsToDrakeVisualizer(
      &builder, hsr_world->get_mutable_multibody_plant(),
      hsr_world->GetOutputPort("contact_results"));

  const auto& hsr_plant = hsr_world->get_hsr_plant("hsr");
  // Add a constant source to set desired position and velocity. These values
  // are set arbitrarily.
  drake::VectorX<double> constant_pos_value = drake::VectorX<double>::Zero(
      hsr_plant.num_positions() + hsr_plant.num_velocities());
  // Set the w of the quaternion base to be 1.0 to make the state to be valid.
  constant_pos_value(0) = 1.0;

  auto desired_pos_constant_source =
      builder.template AddSystem<systems::ConstantVectorSource<double>>(
          constant_pos_value);
  desired_pos_constant_source->set_name("desired_pos_constant_source");
  builder.Connect(desired_pos_constant_source->get_output_port(),
                  hsr_world->GetInputPort("hsr_desired_state"));

  auto diagram = builder.Build();

  // Create a context for this diagram and plant.
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  auto& hsr_world_plant = hsr_world->get_mutable_multibody_plant();
  systems::Context<double>& plant_context = diagram->GetMutableSubsystemContext(
      hsr_world_plant, diagram_context.get());

  // Set the body frame P initial pose.
  const drake::multibody::Body<double>& body =
      hsr_world_plant.GetBodyByName("base_footprint");
  const Translation3d X_WP(0.0, 0.0, 0.0);
  hsr_world_plant.SetFreeBodyPoseInWorldFrame(&plant_context, body, X_WP);

  // Create and run the simulator.
  drake::systems::Simulator<double> simulator(*diagram,
                                              std::move(diagram_context));
  const auto& sim_parameters = hsr_sim_flags();
  simulator.set_target_realtime_rate(sim_parameters.target_realtime_rate);
  simulator.AdvanceTo(sim_parameters.simulation_time);

  return 0;
}

}  // namespace
}  // namespace hsr
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  return drake::examples::hsr::DoMain();
}
