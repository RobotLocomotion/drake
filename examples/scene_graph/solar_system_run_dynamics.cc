#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/scene_graph/solar_system.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(simulation_time, 13.0,
              "Desired duration of the simulation in seconds.");

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

  // Note: we can't use AddDefaultVisualization() because we don't have a plant
  // in the Diagram.
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph);
  auto meshcat = std::make_shared<geometry::Meshcat>();
  geometry::MeshcatVisualizer<double>::AddToBuilder(&builder, *scene_graph,
                                                    meshcat);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.get_mutable_integrator().set_maximum_step_size(0.002);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace solar_system
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::solar_system::do_main();
}
