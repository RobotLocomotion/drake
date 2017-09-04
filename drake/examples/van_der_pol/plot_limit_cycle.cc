#include <memory>

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/common/proto/call_matlab.h"
#include "drake/examples/van_der_pol/van_der_pol.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace examples {
namespace van_der_pol {
namespace {

// TODO(russt): Move most of this boilerplate up into the framework.  This
// method should be just a few lines.
// TODO(russt): Move the computation of the cycle to a method in van_der_pol.cc
// (once the boilerplate and dependencies are reduced.
void do_main() {
  systems::DiagramBuilder<double> builder;

  auto vdp = builder.AddSystem<VanDerPolOscillator<double>>();
  auto logger = builder.AddSystem<systems::SignalLogger<double>>(2);
  builder.Connect(vdp->get_output_port(1), logger->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  // Initialize with pre-computed initial conditions on the limit cycle
  // for μ=1.
  Eigen::Vector2d initial_state(-0.1144, 2.0578);
  auto vdp_state =
      diagram->GetMutableSubsystemContext(*vdp, simulator.get_mutable_context())
          .get_mutable_continuous_state_vector();
  vdp_state->SetFromVector(initial_state);

  // Simulate for the known duration of the cycle for μ=1.
  simulator.StepTo(6.667);

  using common::CallMatlab;
  CallMatlab("plot", logger->data().row(0), logger->data().row(1), "k",
             "LineWidth", 2.0);
  CallMatlab("axis", Eigen::RowVector4d(-2.5, 2.5, -3, 3));
  CallMatlab("xlabel", "q");
  CallMatlab("ylabel", "qdot");
  CallMatlab("drawnow");

  // Check that the final state is very close to the initial state.
  DRAKE_DEMAND(
      is_approx_equal_abstol(vdp_state->CopyToVector(), initial_state, 1e-2));
}

}  // namespace
}  // namespace van_der_pol
}  // namespace examples
}  // namespace drake

int main() {
  drake::examples::van_der_pol::do_main();
  return 0;
}
