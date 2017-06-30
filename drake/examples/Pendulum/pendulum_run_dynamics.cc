#include <memory>

#include "drake/common/find_resource.h"
#include "drake/examples/Pendulum/pendulum_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace pendulum {
namespace {

int do_main() {
  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/Pendulum/Pendulum.urdf"),
      multibody::joints::kFixed, tree.get());

  Eigen::VectorXd tau = Eigen::VectorXd::Zero(1);

  systems::DiagramBuilder<double> builder;
  auto source = builder.AddSystem<systems::ConstantVectorSource>(tau);
  source->set_name("tau");
  auto pendulum = builder.AddSystem<PendulumPlant>();
  pendulum->set_name("pendulum");
  auto publisher =
      builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");
  builder.Connect(source->get_output_port(), pendulum->get_tau_port());
  builder.Connect(pendulum->get_output_port(), publisher->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& pendulum_context =
      diagram->GetMutableSubsystemContext(
          *pendulum, simulator.get_mutable_context());
  pendulum->set_theta(&pendulum_context, 1.);
  pendulum->set_thetadot(&pendulum_context, 0.);

  simulator.Initialize();
  simulator.StepTo(10);
  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::pendulum::do_main();
}
