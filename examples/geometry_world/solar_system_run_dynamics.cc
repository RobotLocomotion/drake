#include "drake/examples/geometry_world/solar_system.h"
#include "drake/geometry/geometry_system.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace solar_system {
namespace {

using geometry::GeometrySystem;
using geometry::SourceId;

int do_main() {
  systems::DiagramBuilder<double> builder;

  auto geometry_system = builder.AddSystem<GeometrySystem<double>>();
  geometry_system->set_name("geometry_system");

  auto solar_system = builder.AddSystem<SolarSystem>(geometry_system);
  solar_system->set_name("SolarSystem");

  builder.Connect(
      solar_system->get_geometry_id_output_port(),
      geometry_system->get_source_frame_id_port(solar_system->source_id()));
  builder.Connect(
      solar_system->get_geometry_pose_output_port(),
      geometry_system->get_source_pose_port(solar_system->source_id()));

  // Last thing before building the diagram; configure the system for
  // visualization.
  geometry::ConfigureVisualization(*geometry_system, &builder);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.get_mutable_integrator()->set_maximum_step_size(0.002);
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.StepTo(13);

  return 0;
}

}  // namespace
}  // namespace solar_system
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::solar_system::do_main(); }
