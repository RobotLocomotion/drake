#include "drake/systems/estimators/luenberger_observer.h"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace {

// Test the estimator dynamics observing a linear system.
GTEST_TEST(TestLuenberger, ErrorDynamics) {
  Eigen::Matrix3d A;
  Eigen::Matrix<double, 3, 1> B;
  Eigen::Matrix<double, 2, 3> C;
  Eigen::Vector2d D;
  // clang-format off
  A << 1., 2., 3.,
       4., 5., 6.,
       7., 8., 9.;
  B << 10.,
       11.,
       12.;
  C << 16., 17., 18.,
       19., 20., 21.;
  D << 22.,
       23.;
  // clang-format on

  Eigen::Matrix<double, 3, 2> L;
  // clang-format off
  L << 24., 25., 26.,
       27., 28., 29.;
  // clang-format on

  auto plant = std::make_unique<systems::LinearSystem<double>>(A, B, C, D);
  auto plant_context = plant->CreateDefaultContext();
  auto observer =
      std::make_unique<systems::estimators::LuenbergerObserver<double>>(
          std::move(plant), std::move(plant_context), L);

  auto context = observer->CreateDefaultContext();
  auto derivatives = observer->AllocateTimeDerivatives();
  auto output = observer->AllocateOutput(*context);

  // The expected dynamics are:
  //  xhatdot = Axhat + Bu + L(y-yhat)
  //  y = xhat

  Eigen::Vector3d xhat(1.0, 2.0, 3.0);
  Vector1d u(4.0);

  Eigen::Vector2d y(5.0, 6.0);

  Eigen::Vector3d xhatdot = A * xhat + B * u + L * (y - C * xhat - D * u);

  context->FixInputPort(0, y);
  context->FixInputPort(1, u);
  context->get_mutable_continuous_state_vector()->SetFromVector(xhat);

  observer->CalcTimeDerivatives(*context, derivatives.get());
  observer->CalcOutput(*context, output.get());

  double tol = 1e-10;

  EXPECT_TRUE(CompareMatrices(xhatdot, derivatives->CopyToVector(), tol));
  EXPECT_TRUE(CompareMatrices(
      xhat, output->GetMutableVectorData(0)->CopyToVector(), tol));
}

}  // namespace
}  // namespace drake
