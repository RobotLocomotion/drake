#include "drake/examples/Quadrotor/Quadrotor.h"

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/joints/floating_base_types.h"

namespace drake {
namespace examples {
namespace quadrotor {
namespace {

GTEST_TEST(urdfDynamicsTest, AllTests) {
  auto rbsys = RigidBodySystem();
  rbsys.AddModelInstanceFromFile(
      GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
      drake::systems::plants::joints::kRollPitchYaw);

  auto p = Quadrotor();

  for (int i = 0; i < 1000; i++) {
    auto x0 = getRandomVector<QuadrotorState>();
    auto u0 = getRandomVector<QuadrotorInput>();

    RigidBodySystem::StateVector<double> x0_rb = toEigen(x0);
    RigidBodySystem::InputVector<double> u0_rb = toEigen(u0);

    auto xdot = toEigen(p.dynamics(0.0, x0, u0));
    auto xdot_rb = rbsys.dynamics(0.0, x0_rb, u0_rb);
    EXPECT_TRUE(
        CompareMatrices(xdot_rb, xdot, 1e-8, MatrixCompareType::absolute));
  }
}

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
