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

using multibody::MultibodyTree;
using systems::Context;

int do_main() {

  const double mass = 0.5;      // [Kgr], about a pound.
  const double length = 0.7;    // [m]
  const double radius = 0.015;  // [m]
  const double num_links = 20;

  // TODO: place in Diagram to allow estension including GeometrySystem
  auto pendulum =
      std::make_unique<NLinkPendulumPlant<double>>(mass, length, radius, num_links);
  pendulum->set_name("NLinkPendulum");
  std::unique_ptr<Context<double>> context = pendulum->CreateDefaultContext();

  std::unique_ptr<NLinkPendulumPlant<AutoDiffXd>> pendulum_ad =
      systems::System<double>::ToAutoDiffXd(*pendulum);
  //ASSERT_NE(nullptr, pendulum_ad);

  // Construct a new autodiff context.
  std::unique_ptr<Context<AutoDiffXd>> context_ad =
      pendulum_ad->CreateDefaultContext();
  context_ad->SetTimeStateAndParametersFrom(*context);

  const MultibodyTree<AutoDiffXd>& model = pendulum_ad->get_multibody_model();

  // FIXME!! Linearize only works with systems that have a single vector output
  // port. Otherwise matrix C has zero size.
  std::unique_ptr<systems::LinearSystem<double>> linear_pendulum =
      systems::Linearize(*pendulum, *context);

  // C is the linearization of the output port value.
  // y = C * x
  const MatrixX<double>& C = linear_pendulum->C();

  // Partial of p_WEo with the first angle.
  const Vector3<double> Dq0_p_WEo = C.col(0);

  PRINT_VAR(Dq0_p_WEo.transpose());

  //const systems::OutputPort<AutoDiffXd>& end_pose_port =
  //    pendulum_ad->get_end_pose_output_port();
  //end_pose_port.Allocate()
  //end_pose_port.Eval(*context_ad);

  //std::unique_ptr<systems::LinearSystem<double>> linear_pendulum =
    //  systems::Linearize(*pendulum, *context);

  //(void) linear_pendulum;
  (void) context_ad;
  (void) pendulum_ad;
  (void) model;
  (void) linear_pendulum;

  return 0;
}

}  // namespace
}  // namespace test
}  // namespace n_link_pendulum
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::n_link_pendulum::test::do_main(); }
