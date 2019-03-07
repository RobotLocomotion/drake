#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/benchmarks/inclined_plane/make_inclined_plane_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace multibody {
namespace inclined_plane {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(simulation_time, 1.0,
              "Desired duration of the simulation in seconds.");
DEFINE_double(time_step, 1.0e-3,
              "If zero, the plant is modeled as a continuous system. "
              "If positive, the period (in seconds) of the discrete updates "
              "for the plant modeled as a discrete system."
              "This parameter must be non-negative.");
DEFINE_double(integration_accuracy, 1e-6,
              "Integration accuracy when the plant is modeled as a continuous "
              "system. Not used if time_step > 0.");

DEFINE_double(static_friction, 0.5, "Static friction coefficient.");
DEFINE_double(dynamic_friction, 0.3, "Dynamic friction coefficient.");
DEFINE_double(stiction_tolerance, 1.0e-5, "Stribeck model stiction tolerance.");

using geometry::SceneGraph;
using geometry::SourceId;
using lcm::DrakeLcm;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::benchmarks::inclined_plane::AddInclinedPlaneToPlant;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;

int do_main() {
  systems::DiagramBuilder<double> builder;

  // Plant's parameters.
  const double radius = 0.05;   // m
  const double mass = 0.1;      // kg
  const double g = 9.81;        // m/s^2
  const double slope = 15.0 / 180 * M_PI;  // rad.
  const CoulombFriction<double> surface_friction(FLAGS_static_friction,
                                                 FLAGS_dynamic_friction);

  DRAKE_DEMAND(FLAGS_time_step >= 0);

  auto pair = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(FLAGS_time_step));
  MultibodyPlant<double>& plant = pair.plant;
  AddInclinedPlaneToPlant(
      radius, mass, slope, surface_friction, g, &plant);
  plant.Finalize();
  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(1.0e-5);
  plant.set_stiction_tolerance(FLAGS_stiction_tolerance);

  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  geometry::ConnectDrakeVisualizer(&builder, pair.scene_graph);
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // This will set a default initial condition with the sphere located at
  // p_WBcm = (0; 0; 0) and zero spatial velocity.
  plant.SetDefaultContext(&plant_context);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  systems::IntegratorBase<double>* integrator =
      simulator.get_mutable_integrator();
  integrator->set_target_accuracy(FLAGS_integration_accuracy);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace inclined_plane
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A rolling sphere down an inclined plane demo using Drake's "
      "MultibodyPlant with SceneGraph visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::multibody::inclined_plane::do_main();
}
