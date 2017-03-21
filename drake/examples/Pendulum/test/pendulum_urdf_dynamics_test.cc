#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/Pendulum/pendulum_plant.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

namespace drake {
namespace examples {
namespace pendulum {
namespace {

GTEST_TEST(UrdfDynamicsTest, AllTests) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Pendulum/Pendulum.urdf",
      multibody::joints::kFixed, tree.get());

  systems::RigidBodyPlant<double> rbp(std::move(tree));
  PendulumPlant<double> p;

  auto context_rbp = rbp.CreateDefaultContext();
  auto context_p = p.CreateDefaultContext();

  auto& u_rbp = context_rbp->FixInputPort(0, Vector1d::Zero());
  auto& u_p = context_p->FixInputPort(0, Vector1d::Zero());

  Eigen::Vector2d x;
  Vector1d u;
  auto xdot_rbp = rbp.AllocateTimeDerivatives();
  auto xdot_p = p.AllocateTimeDerivatives();

  for (int i = 0; i < 100; ++i) {
    x = Eigen::Vector2d::Random();
    u = Vector1d::Random();

    context_rbp->get_mutable_continuous_state_vector()->SetFromVector(x);
    context_p->get_mutable_continuous_state_vector()->SetFromVector(x);

    u_rbp.GetMutableVectorData<double>()->SetFromVector(u);
    u_p.GetMutableVectorData<double>()->SetFromVector(u);

    rbp.CalcTimeDerivatives(*context_rbp, xdot_rbp.get());
    p.CalcTimeDerivatives(*context_p, xdot_p.get());

    EXPECT_TRUE(CompareMatrices(xdot_rbp->CopyToVector(),
                                xdot_p->CopyToVector(), 1e-8,
                                MatrixCompareType::absolute));
  }
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake
