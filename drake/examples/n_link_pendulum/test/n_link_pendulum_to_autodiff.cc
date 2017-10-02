#include "drake/examples/n_link_pendulum/n_link_pendulum_plant.h"

#include <memory>

#include "drake/systems/primitives/linear_system.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace examples {
namespace n_link_pendulum {
namespace test {
namespace {

using systems::Context;

int do_main() {

  const double mass = 0.5;      // [Kgr], about a pound.
  const double length = 0.7;    // [m]
  const double radius = 0.015;  // [m]
  const double num_links = 20;
  
  auto pendulum =
      std::make_unique<NLinkPendulumPlant<double>>(mass, length, radius, num_links);
  pendulum->set_name("NLinkPendulum");

  std::unique_ptr<Context<double>> context = pendulum->CreateDefaultContext();
  //pendulum->SetStraightAtAnAngle(context.get(), M_PI / 3.0);

  std::unique_ptr<systems::LinearSystem<double>> linear_pendulum =
      systems::Linearize(*pendulum, *context);

  (void) linear_pendulum;

  return 0;
}

}  // namespace
}  // namespace test
}  // namespace n_link_pendulum
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::n_link_pendulum::test::do_main(); }
