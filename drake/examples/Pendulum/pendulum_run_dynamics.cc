#include "drake/common/drake_path.h"
#include "drake/examples/Pendulum/pendulum_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace examples {
namespace pendulum {
namespace {

int do_main(int argc, char* argv[]) {
  lcm::DrakeLcm lcm;
  RigidBodyTree tree(GetDrakePath() + "/examples/Pendulum/Pendulum.urdf",
                     systems::plants::joints::kFixed);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(1);

  systems::DiagramBuilder<double> builder;
  auto source = builder.AddSystem<systems::ConstantVectorSource>(tau);
  auto pendulum = builder.AddSystem<PendulumPlant>();
  auto publisher =
      builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);
  builder.Connect(source->get_output_port(), pendulum->get_tau_port());
  builder.Connect(pendulum->get_output_port(), publisher->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>* pendulum_context =
      diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), pendulum);
  pendulum->set_theta(pendulum_context, 1.);
  pendulum->set_thetadot(pendulum_context, 0.);

  simulator.Initialize();
  simulator.StepTo(10);
  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::pendulum::do_main(argc, argv);
}
