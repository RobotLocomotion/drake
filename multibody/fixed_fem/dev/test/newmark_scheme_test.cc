#include "drake/multibody/fixed_fem/dev/newmark_scheme.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/fixed_fem/dev/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace test {
namespace {
const double kDt = 1e-4;
const double kGamma = 0.2;
const double kBeta = 0.3;
const double kTol = std::numeric_limits<double>::epsilon();
class NewmarkSchemeTest : public ::testing::Test {
 protected:
  static Vector4<double> MakeQ() {
    return Vector4<double>(1.23, 2.34, 3.45, 4.56);
  }

  static Vector4<double> MakeQdot() {
    return Vector4<double>(5.67, 6.78, 7.89, 9.10);
  }

  static Vector4<double> MakeQddot() {
    return Vector4<double>(0.1011, 0.1112, 0.1213, 0.1314);
  }

  NewmarkScheme<FemState<DummyElement<2>>> newmark_scheme_{kDt, kGamma, kBeta};
};

TEST_F(NewmarkSchemeTest, Weights) {
  EXPECT_TRUE(CompareMatrices(
      newmark_scheme_.weights(),
      Vector3<double>(kBeta * kDt * kDt, kGamma * kDt, 1.0), 0));
}

TEST_F(NewmarkSchemeTest, UpdateStateFromChangeInUnknowns) {
  const Vector4<double> q = MakeQ();
  const Vector4<double> qdot = MakeQdot();
  const Vector4<double> qddot = MakeQddot();
  FemState<DummyElement<2>> state(q, qdot, qddot);
  const Vector4<double> dqddot(1.234, 4.567, 7.890, 0.123);
  newmark_scheme_.UpdateStateFromChangeInUnknowns(dqddot, &state);
  EXPECT_TRUE(CompareMatrices(state.qddot(), qddot + dqddot, 0));
  EXPECT_TRUE(
      CompareMatrices(state.qdot(), qdot + dqddot * kDt * kGamma, kTol));
  EXPECT_TRUE(CompareMatrices(state.q(), q + dqddot * kDt * kDt * kBeta, kTol));
}

/* Tests that Newmark scheme reproduces analytical solutions under qddot. */
TEST_F(NewmarkSchemeTest, AdvanceOneTimeStep) {
  const Vector4<double> q = MakeQ();
  const Vector4<double> qdot = MakeQdot();
  const Vector4<double> qddot = MakeQddot();
  const FemState<DummyElement<2>> state_0(q, qdot, qddot);
  FemState<DummyElement<2>> state_n(state_0);
  FemState<DummyElement<2>> state_np1(state_0);
  const int kTimeSteps = 10;
  for (int i = 0; i < kTimeSteps; ++i) {
    newmark_scheme_.AdvanceOneTimeStep(state_n, qddot, &state_np1);
    state_n = state_np1;
  }
  const double total_time = kDt * kTimeSteps;
  EXPECT_TRUE(CompareMatrices(state_n.qddot(), state_0.qddot()));
  EXPECT_TRUE(CompareMatrices(state_n.qdot(),
                              state_0.qdot() + total_time * state_0.qddot(),
                              kTimeSteps * kTol));
  EXPECT_TRUE(
      CompareMatrices(state_n.q(),
                      state_0.q() + total_time * state_0.qdot() +
                          0.5 * total_time * total_time * state_0.qddot(),
                      kTimeSteps * kTol));
}
}  // namespace
}  // namespace test
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
