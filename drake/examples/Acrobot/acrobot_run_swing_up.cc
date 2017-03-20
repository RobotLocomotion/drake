#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/examples/Acrobot/acrobot_plant.h"
#include "drake/examples/Acrobot/acrobot_spong_controller.h"
#include "drake/examples/Acrobot/gen/acrobot_state_vector.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

// Simple example which simulates the Acrobot, started near its stable fixed
// point, with a Spong swing-up controller designed to reach the unstable
// fixed point.  Run drake-visualizer to see the animated result.

DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Acrobot/Acrobot.urdf",
      multibody::joints::kFixed, tree.get());

  systems::DiagramBuilder<double> builder;
  auto acrobot = builder.AddSystem<AcrobotPlant>();
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  builder.Connect(acrobot->get_output_port(0), publisher->get_input_port(0));

  auto controller = builder.AddSystem<AcrobotSpongController>();
  builder.Connect(acrobot->get_output_port(0), controller->get_input_port(0));
  builder.Connect(controller->get_output_port(0), acrobot->get_input_port(0));

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>* acrobot_context =
      diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                          acrobot);

  // Sets an initial condition near the upright fixed point.
  AcrobotStateVector<double>* x0 = dynamic_cast<AcrobotStateVector<double>*>(
      acrobot_context->get_mutable_continuous_state_vector());
  DRAKE_DEMAND(x0 != nullptr);
  x0->set_theta1(0.1);
  x0->set_theta2(-0.1);
  x0->set_theta1dot(0.0);
  x0->set_theta2dot(0.02);

  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.Initialize();
  simulator.StepTo(60);

  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::acrobot::do_main(argc, argv);
}
