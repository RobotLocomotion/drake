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
// (once the boilerplate and dependencies are reduced).
void DoMain() {
  systems::DiagramBuilder<double> builder;

  auto vdp = builder.AddSystem<VanDerPolOscillator<double>>();
  auto logger = LogOutput(vdp->get_full_state_output_port(), &builder);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  // The following initial state was pre-computed (by running the simulation
  // with μ=1 long enough for it to effectively reach the steady-state limit
  // cycle).
  // TODO(russt): Use trajectory optimization to compute the cycle for arbitrary
  // μ.
  const Eigen::Vector2d initial_state(-0.1144, 2.0578);
  systems::VectorBase<double>& vdp_state =
      diagram
          ->GetMutableSubsystemContext(*vdp, &simulator.get_mutable_context())
          .get_mutable_continuous_state_vector();
  vdp_state.SetFromVector(initial_state);

  // Simulate for the (experimentally acquired) period of the cycle for μ=1.
  simulator.StepTo(6.667);

  using common::CallMatlab;
  CallMatlab("plot", logger->data().row(0), logger->data().row(1), "k",
             "LineWidth", 2.0);
  CallMatlab("axis", Eigen::RowVector4d(-2.5, 2.5, -3, 3));
  CallMatlab("xlabel", "q");
  CallMatlab("ylabel", "qdot");
  CallMatlab("drawnow");

  // Check that the final state is close to the initial state.
  // TODO(russt): Tighten this tolerance after I generalize the code to support
  // arbitrary μ.
  DRAKE_DEMAND(
      is_approx_equal_abstol(vdp_state.CopyToVector(), initial_state, 1e-2));
}

}  // namespace
}  // namespace van_der_pol
}  // namespace examples
}  // namespace drake

int main() {
  drake::examples::van_der_pol::DoMain();
  return 0;
}
