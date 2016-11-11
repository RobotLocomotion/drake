#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/rigid_body_system1/RigidBodySystem.h"

namespace drake {
namespace test {

using std::make_shared;

using Eigen::Matrix;

GTEST_TEST(testMassSpringDamper, AllTests) {
  auto sys = make_shared<RigidBodySystem>();
  sys->AddModelInstanceFromFile(
      GetDrakePath() +
      "/multibody/rigid_body_system1/test/MassSpringDamper.urdf",
      drake::multibody::joints::kFixed);

  double mass = 1.0, k = 10.0, b = 1.0;
  Matrix<double, 2, 1> xdot_desired;

  for (int i = 0; i < 1000; i++) {
    auto x0 = getInitialState(*sys);
    Matrix<double, 1, 1> u0 = Matrix<double, 1, 1>::Random();

    auto xdot = toEigen(sys->dynamics(0.0, x0, u0));
    xdot_desired << x0(1), (u0(0) - k * x0(0) - b * x0(1)) / mass;

    EXPECT_TRUE(
        CompareMatrices(xdot_desired, xdot, 1e-5, MatrixCompareType::absolute));
  }
}

}  // namespace test
}  // namespace drake
