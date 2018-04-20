#include "drake/examples/geometry_world/solar_system.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace solar_system {
namespace {

using geometry::SceneGraph;
using geometry::SourceId;
using lcm::DrakeLcm;

int do_main() {
  systems::DiagramBuilder<double> builder;

  auto scene_graph = builder.AddSystem<SceneGraph<double>>();
  scene_graph->set_name("scene_graph");

  auto solar_system = builder.AddSystem<SolarSystem>(scene_graph);
  solar_system->set_name("SolarSystem");

  builder.Connect(solar_system->get_geometry_pose_output_port(),
                  scene_graph->get_source_pose_port(solar_system->source_id()));

  // Last thing before building the diagram; configure the system for
  // visualization.
  DrakeLcm lcm;
  geometry::ConnectVisualization(*scene_graph, &builder, &lcm);
  geometry::DispatchLoadMessage(*scene_graph, &lcm);

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
