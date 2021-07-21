#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace pendulum {
namespace {

GTEST_TEST(UrdfDynamicsTest, AllTests) {
  multibody::MultibodyPlant<double> mbp(0.0);
  multibody::Parser(&mbp).AddModelFromFile(FindResourceOrThrow(
      "drake/examples/pendulum/Pendulum.urdf"));
  mbp.Finalize();
  PendulumPlant<double> p;

  auto context_mbp = mbp.CreateDefaultContext();
  auto context_p = p.CreateDefaultContext();

  auto& u_mbp = mbp.get_actuation_input_port().FixValue(context_mbp.get(), 0.);
  auto& u_p = p.get_input_port().FixValue(context_p.get(), 0.);

  Eigen::Vector2d x;
  Vector1d u;
  auto xdot_mbp = mbp.AllocateTimeDerivatives();
  auto xdot_p = p.AllocateTimeDerivatives();

  for (int i = 0; i < 100; ++i) {
    x = Eigen::Vector2d::Random();
    u = Vector1d::Random();

    context_mbp->get_mutable_continuous_state_vector().SetFromVector(x);
    context_p->get_mutable_continuous_state_vector().SetFromVector(x);

    u_mbp.GetMutableVectorData<double>()->SetFromVector(u);
    u_p.GetMutableVectorData<double>()->SetFromVector(u);

    mbp.CalcTimeDerivatives(*context_mbp, xdot_mbp.get());
    p.CalcTimeDerivatives(*context_p, xdot_p.get());

    EXPECT_TRUE(CompareMatrices(xdot_mbp->CopyToVector(),
                                xdot_p->CopyToVector(), 1e-8,
                                MatrixCompareType::absolute));
  }
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake
