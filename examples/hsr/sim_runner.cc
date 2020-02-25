#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/hsr/hsr_world.h"
#include "drake/examples/hsr/parameters.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/simulator.h"

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

  // const VectorXd tau = VectorXd::Zero(hsr_world_plant.num_actuators());
  // hsr_world->GetInputPort("hsr_actuation_input").FixValue(&plant_context,
  // tau);

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
