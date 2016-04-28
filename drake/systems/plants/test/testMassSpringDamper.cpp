#include "gtest/gtest.h"

#include "drake/Path.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/eigen_matrix_compare.h"
#include "drake/util/testUtil.h"

using Eigen::Matrix;
using std::make_shared;
using Drake::getDrakePath;
using Drake::RigidBodySystem;
using drake::util::MatrixCompareType;
using Drake::toEigen;

namespace drake {
namespace test {

TEST(testMassSpringDamper, AllTests) {
  auto sys = make_shared<RigidBodySystem>();
  sys->addRobotFromFile(
      getDrakePath() + "/systems/plants/test/MassSpringDamper.urdf",
      DrakeJoint::FIXED);

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
