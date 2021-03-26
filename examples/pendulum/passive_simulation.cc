#include <gflags/gflags.h>

#include "drake/examples/pendulum/pendulum_geometry.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace pendulum {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto source = builder.AddSystem<systems::ConstantVectorSource>(
      PendulumInput<double>{}.with_tau(0.0));
  source->set_name("source");
  auto pendulum = builder.AddSystem<PendulumPlant>();
  pendulum->set_name("pendulum");
  builder.Connect(*source, *pendulum);
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  PendulumGeometry::AddToBuilder(
      &builder, pendulum->get_state_output_port(), scene_graph);
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& pendulum_context =
      diagram->GetMutableSubsystemContext(*pendulum,
                                          &simulator.get_mutable_context());
  PendulumState<double>& state = pendulum->get_mutable_state(&pendulum_context);
  state.set_theta(1.);
  state.set_thetadot(0.);

  const double initial_energy = pendulum->CalcTotalEnergy(pendulum_context);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(10);

  const double final_energy = pendulum->CalcTotalEnergy(pendulum_context);

  // Adds a numerical sanity test on total energy.
  DRAKE_DEMAND(initial_energy > 2.0 * final_energy);
  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::pendulum::DoMain();
}
