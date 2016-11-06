
#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/Pendulum/pendulum_plant.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"

namespace drake {
namespace examples {
namespace pendulum {
namespace {

GTEST_TEST(urdfDynamicsTest, AllTests) {
  auto tree = std::make_unique<RigidBodyTree<double>>(
      GetDrakePath() + "/examples/Pendulum/Pendulum.urdf",
      systems::plants::joints::kFixed);
  systems::RigidBodyPlant<double> rbp(std::move(tree));
  PendulumPlant<double> p;

  auto context_rbp = rbp.CreateDefaultContext();
  auto context_p = p.CreateDefaultContext();

  auto input_rbp =
      std::make_unique<systems::FreestandingInputPort>(Vector1d::Zero());
  systems::FreestandingInputPort* rbp_u = input_rbp.get();
  context_rbp->SetInputPort(0, std::move(input_rbp));

  auto input_p =
      std::make_unique<systems::FreestandingInputPort>(Vector1d::Zero());
  systems::FreestandingInputPort* p_u = input_p.get();
  context_p->SetInputPort(0, std::move(input_p));

  Eigen::Vector2d x;
  Vector1d u;
  auto xdot_rbp = rbp.AllocateTimeDerivatives();
  auto xdot_p = p.AllocateTimeDerivatives();

  for (int i = 0; i < 1000; ++i) {
    x = Eigen::Vector2d::Random();
    u = Vector1d::Random();

    context_rbp->get_mutable_continuous_state_vector()->SetFromVector(x);
    context_p->get_mutable_continuous_state_vector()->SetFromVector(x);

    rbp_u->GetMutableVectorData<double>()->SetFromVector(u);
    p_u->GetMutableVectorData<double>()->SetFromVector(u);

    rbp.EvalTimeDerivatives(*context_rbp, xdot_rbp.get());
    p.EvalTimeDerivatives(*context_p, xdot_p.get());

    EXPECT_TRUE(CompareMatrices(xdot_rbp->CopyToVector(),
                                xdot_p->CopyToVector(), 1e-8,
                                MatrixCompareType::absolute));
  }
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake
