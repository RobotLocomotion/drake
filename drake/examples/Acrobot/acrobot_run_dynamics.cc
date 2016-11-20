#include "drake/common/drake_path.h"
#include "drake/examples/Acrobot/acrobot_plant.h"
#include "drake/examples/Acrobot/gen/acrobot_state_vector.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

int do_main(int argc, char* argv[]) {
  lcm::DrakeLcm lcm;
  RigidBodyTree<double> tree(GetDrakePath() + "/examples/Acrobot/Acrobot.urdf",
                             multibody::joints::kFixed);

  systems::DiagramBuilder<double> builder;
  auto acrobot = builder.AddSystem<AcrobotPlant>();
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);
  builder.Connect(acrobot->get_output_port(0), publisher->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>* acrobot_context =
      diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                          acrobot);

  double tau = 0;
  acrobot_context->FixInputPort(0, Eigen::Matrix<double, 1, 1>::Constant(tau));
  AcrobotStateVector<double>* x0 = dynamic_cast<AcrobotStateVector<double>*>(
      acrobot_context->get_mutable_continuous_state_vector());
  DRAKE_DEMAND(x0 != nullptr);
  x0->set_theta1(1.0);
  x0->set_theta2(1.0);
  x0->set_theta1dot(0.0);
  x0->set_theta2dot(0.0);

  simulator.Initialize();
  simulator.StepTo(10);
  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::acrobot::do_main(argc, argv);
}
