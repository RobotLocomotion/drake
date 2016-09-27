
#include "drake/common/drake_path.h"
#include "drake/examples/Pendulum/pendulum_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_tree_lcm_publisher.h"
#include "drake/systems/plants/RigidBodyTree.h"

using namespace std;
using namespace drake;

using drake::examples::pendulum::PendulumSystem;

int main(int argc, char* argv[]) {
  lcm::LCM lcm;
  RigidBodyTree tree(GetDrakePath() + "/examples/Pendulum/Pendulum.urdf",
                     drake::systems::plants::joints::kFixed);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(1);

  systems::DiagramBuilder<double> builder;
  auto source = builder.AddSystem<systems::ConstantVectorSource>(tau);
  auto pendulum = builder.AddSystem<PendulumSystem>();
  auto publisher =
      builder.AddSystem<systems::RigidBodyTreeLcmPublisher>(tree, &lcm);
  builder.Connect(source->get_output_port(), pendulum->get_tau_port());
  builder.Connect(pendulum->get_output_port(), publisher->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  pendulum->set_theta(
      diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), pendulum), 1.);
  simulator.Initialize();
  simulator.StepTo(10);
}
