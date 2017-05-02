#include <iostream>

#include "drake/common/symbolic_expression.h"
#include "drake/examples/Pendulum/pendulum_plant.h"

// A simple example of extracting the symbolic dynamics of the pendulum system,
// and printing them to std::out.

namespace drake {
namespace examples {
namespace pendulum {
namespace {

int do_main() {
  PendulumPlant<symbolic::Expression> system;

  auto context = system.CreateDefaultContext();
  context->FixInputPort(
      0, Vector1<symbolic::Expression>::Constant(symbolic::Variable("tau")));
  context->get_mutable_continuous_state_vector()->SetAtIndex(
      0, symbolic::Variable("theta"));
  context->get_mutable_continuous_state_vector()->SetAtIndex(
      1, symbolic::Variable("thetadot"));

  auto derivatives = system.AllocateTimeDerivatives();
  system.CalcTimeDerivatives(*context, derivatives.get());

  std::cout << derivatives->CopyToVector() << "\n";

  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::pendulum::do_main();
}
