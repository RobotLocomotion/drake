#include <cmath>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/pendulum/pendulum_plant.h"
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

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int DoMain() {
  lcm::DrakeLcm lcm;

  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"),
      multibody::joints::kFixed, tree.get());

  systems::DiagramBuilder<double> builder;
  auto pendulum = builder.AddSystem<PendulumPlant>();
  pendulum->set_name("pendulum");

  // Prepare to linearize around the vertical equilibrium point (with tau=0)
  auto pendulum_context = pendulum->CreateDefaultContext();
  auto& desired_state = pendulum->get_mutable_state(pendulum_context.get());
  desired_state.set_theta(M_PI);
  desired_state.set_thetadot(0);
  auto input = std::make_unique<PendulumInput<double>>();
  input->set_tau(0.0);
  pendulum_context->FixInputPort(0, std::move(input));

  // Set up cost function for LQR: integral of 10*theta^2 + thetadot^2 + tau^2.
  // The factor of 10 is heuristic, but roughly accounts for the unit conversion
  // between angles and angular velocity (using the time constant, \sqrt{g/l},
  // squared).
  Eigen::MatrixXd Q(2, 2);
  Q << 10, 0, 0, 1;
  Eigen::MatrixXd R(1, 1);
  R << 1;

  auto controller =
      builder.AddSystem(systems::controllers::LinearQuadraticRegulator(
          *pendulum, *pendulum_context, Q, R));
  controller->set_name("controller");
  builder.Connect(pendulum->get_output_port(), controller->get_input_port());
  builder.Connect(controller->get_output_port(), pendulum->get_input_port());

  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");
  builder.Connect(pendulum->get_output_port(), publisher->get_input_port(0));

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& sim_pendulum_context =
      diagram->GetMutableSubsystemContext(*pendulum,
                                          &simulator.get_mutable_context());
  auto& state = pendulum->get_mutable_state(&sim_pendulum_context);
  state.set_theta(M_PI + 0.1);
  state.set_thetadot(0.2);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(10);

  // Adds a numerical test to make sure we're stabilizing the fixed point.
  DRAKE_DEMAND(is_approx_equal_abstol(state.get_value(),
                                      desired_state.get_value(), 1e-3));

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
