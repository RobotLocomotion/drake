#include "drake/examples/Pendulum/Pendulum.h"

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/plants/RigidBodySystem.h"

namespace drake {
namespace examples {
namespace pendulum {
namespace {

GTEST_TEST(urdfDynamicsTest, AllTests) {
  auto tree = shared_ptr<RigidBodyTree>(new RigidBodyTree(
      GetDrakePath() + "/examples/Pendulum/Pendulum.urdf",
      drake::systems::plants::joints::kFixed));
  auto rbsys = RigidBodySystem(tree);
  auto p = Pendulum();

  for (int i = 0; i < 1000; ++i) {
    auto x0 = getRandomVector<PendulumState>();
    auto u0 = getRandomVector<PendulumInput>();

    RigidBodySystem::StateVector<double> x0_rb = toEigen(x0);
    RigidBodySystem::InputVector<double> u0_rb = toEigen(u0);

    auto xdot = toEigen(p.dynamics(0.0, x0, u0));
    auto xdot_rb = rbsys.dynamics(0.0, x0_rb, u0_rb);
    EXPECT_TRUE(
        CompareMatrices(xdot_rb, xdot, 1e-8, MatrixCompareType::absolute));
  }
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake
