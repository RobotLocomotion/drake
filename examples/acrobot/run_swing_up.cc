#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/acrobot/acrobot_geometry.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/gen/acrobot_state.h"
#include "drake/examples/acrobot/spong_controller.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/math/wrap_to.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

// Simple example which simulates the Acrobot, started near its stable fixed
// point, with a Spong swing-up controller designed to reach the unstable
// fixed point.  Run drake-visualizer to see the animated result.

DEFINE_double(simulation_sec, 10.0,
              "Number of seconds to simulate.");
DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int do_main() {
  systems::DiagramBuilder<double> builder;
  auto acrobot = builder.AddSystem<AcrobotPlant>();
  acrobot->set_name("acrobot");
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  AcrobotGeometry::AddToBuilder(
      &builder, acrobot->get_output_port(0), scene_graph);
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph);

  auto controller = builder.AddSystem<AcrobotSpongController>();
  builder.Connect(acrobot->get_output_port(0), controller->get_input_port(0));
  builder.Connect(controller->get_output_port(0), acrobot->get_input_port(0));

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& acrobot_context =
      diagram->GetMutableSubsystemContext(*acrobot,
                                          &simulator.get_mutable_context());

  // Sets an initial condition near the upright fixed point.
  AcrobotState<double>* state = dynamic_cast<AcrobotState<double>*>(
      &acrobot_context.get_mutable_continuous_state_vector());
  DRAKE_DEMAND(state != nullptr);
  state->set_theta1(0.1);
  state->set_theta2(-0.1);
  state->set_theta1dot(0.0);
  state->set_theta2dot(0.02);

  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.AdvanceTo(FLAGS_simulation_sec);

  DRAKE_DEMAND(std::abs(math::wrap_to(state->theta1(), 0., 2. * M_PI) - M_PI) <
               1e-2);
  DRAKE_DEMAND(std::abs(math::wrap_to(state->theta2(), -M_PI, M_PI)) < 1e-2);
  DRAKE_DEMAND(std::abs(state->theta1dot()) < 0.1);
  DRAKE_DEMAND(std::abs(state->theta2dot()) < 0.1);

  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::acrobot::do_main();
}
