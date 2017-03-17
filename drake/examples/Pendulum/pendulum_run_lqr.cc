#include <cmath>
#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/examples/Pendulum/pendulum_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/input_port_value.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace pendulum {
namespace {

int do_main(int argc, char* argv[]) {
  lcm::DrakeLcm lcm;

  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Pendulum/Pendulum.urdf",
      multibody::joints::kFixed, tree.get());

  systems::DiagramBuilder<double> builder;
  auto pendulum = builder.AddSystem<PendulumPlant>();

  // Prepare to linearize around the vertical equilibrium point (with tau=0)
  auto pendulum_context = pendulum->CreateDefaultContext();
  pendulum->set_theta(pendulum_context.get(), M_PI);
  pendulum->set_thetadot(pendulum_context.get(), 0);
  pendulum_context->FixInputPort(0, Vector1d::Zero());

  // Set up cost function for LQR: integral of 10*theta^2 + thetadot^2 + tau^2.
  // The factor of 10 is heuristic, but roughly accounts for the unit conversion
  // between angles and angular velocity (using the time constant, \sqrt{g/l},
  // squared).
  Eigen::MatrixXd Q(2, 2);
  Q << 10, 0, 0, 1;
  Eigen::MatrixXd R(1, 1);
  R << 1;

  auto controller = builder.AddSystem(
      LinearQuadraticRegulator(*pendulum, *pendulum_context, Q, R));
  builder.Connect(pendulum->get_output_port(), controller->get_input_port());
  builder.Connect(controller->get_output_port(), pendulum->get_tau_port());

  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  builder.Connect(pendulum->get_output_port(), publisher->get_input_port(0));

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>* sim_pendulum_context =
      diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                          pendulum);
  pendulum->set_theta(sim_pendulum_context, M_PI + 0.1);
  pendulum->set_thetadot(sim_pendulum_context, 0.2);

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
