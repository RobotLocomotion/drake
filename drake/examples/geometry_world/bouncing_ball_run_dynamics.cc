#include "drake/examples/geometry_world/bouncing_ball_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace bouncing_ball {
namespace {

using systems::InputPortDescriptor;

int do_main() {
  systems::DiagramBuilder<double> builder;
  auto bouncing_ball = builder.AddSystem<BouncingBallPlant>(
      Vector2<double>(0.25, 0.25));
  bouncing_ball->set_name("BouncingBall1");

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto init_ball = [&](BouncingBallPlant<double>* system,
                             double z, double zdot) {
    systems::Context<double>& ball_context =
        diagram->GetMutableSubsystemContext(*system,
            &simulator.get_mutable_context());
    system->set_z(&ball_context, z);
    system->set_zdot(&ball_context, zdot);
  };
  init_ball(bouncing_ball, 0.3, 0.);

  simulator.get_mutable_integrator()->set_maximum_step_size(0.002);
  simulator.Initialize();
  simulator.StepTo(10);

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::bouncing_ball::do_main();
}
