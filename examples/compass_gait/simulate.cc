#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/compass_gait/compass_gait.h"
#include "drake/examples/compass_gait/compass_gait_geometry.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace compass_gait {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

/// Simulates the compass gait from various initial velocities (accepted as
/// command-line arguments.  Run drake-visualizer to watch the results.
int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto compass_gait = builder.AddSystem<CompassGait>();
  compass_gait->set_name("compass_gait");

  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  CompassGaitGeometry::AddToBuilder(&builder,
      compass_gait->get_floating_base_state_output_port(), scene_graph);
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& cg_context = diagram->GetMutableSubsystemContext(
      *compass_gait, &simulator.get_mutable_context());
  CompassGaitContinuousState<double>& state =
      compass_gait->get_mutable_continuous_state(&cg_context);
  state.set_stance(0.0);
  state.set_swing(0.0);
  state.set_stancedot(0.4);
  state.set_swingdot(-2.0);
  compass_gait->get_input_port(0).FixValue(&cg_context, Vector1d(0.0));

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.get_mutable_context().SetAccuracy(1e-4);
  simulator.AdvanceTo(10);

  return 0;
}

}  // namespace
}  // namespace compass_gait
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::compass_gait::DoMain();
}
