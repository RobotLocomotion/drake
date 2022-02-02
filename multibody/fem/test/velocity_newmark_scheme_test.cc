#include "drake/multibody/fem/velocity_newmark_scheme.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/fem/acceleration_newmark_scheme.h"
#include "drake/multibody/fem/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::Vector4d;
using test::DummyElement;

const double kDt = 1e-3;
const double kGamma = 0.6;
const double kBeta = 0.3;
const double kTolerance = 8.0 * std::numeric_limits<double>::epsilon();

Vector4<double> MakeQ() { return Vector4<double>(1.23, 2.34, 3.45, 4.56); }
Vector4<double> MakeQdot() { return Vector4<double>(5.67, 6.78, 7.89, 9.10); }
Vector4<double> MakeQddot() {
  return Vector4<double>(0.1011, 0.1112, 0.1213, 0.1314);
}

/* Verify that the weights returned by the accessor match expectation. */
GTEST_TEST(VelocityNewmarkSchemeTest, Weights) {
  VelocityNewmarkScheme<double> scheme{kDt, kGamma, kBeta};
  EXPECT_TRUE(CompareMatrices(
      scheme.weights(),
      Vector3<double>(kBeta / kGamma * kDt, 1.0, 1.0 / (kGamma * kDt))));
}

/* Verify that the result of
 `VelocityNewmarkScheme::UpdateStateFromChangeInUnknowns()` is consistent with
 the weights. */
GTEST_TEST(VelocityNewmarkSchemeTest, UpdateStateFromChangeInUnknowns) {
  VelocityNewmarkScheme<double> scheme{kDt, kGamma, kBeta};
  FemStateImpl<DummyElement> state0(MakeQ(), MakeQdot(), MakeQddot());
  FemStateImpl<DummyElement> state(state0);
  const Vector4<double> dz(1.234, 4.567, 7.890, 0.123);
  const Vector3<double>& weights = scheme.weights();
  scheme.UpdateStateFromChangeInUnknowns(dz, &state);
  EXPECT_TRUE(CompareMatrices(state.GetPositions() - state0.GetPositions(),
                              weights(0) * dz, kTolerance));
  EXPECT_TRUE(CompareMatrices(state.GetVelocities() - state0.GetVelocities(),
                              weights(1) * dz, kTolerance));
  EXPECT_TRUE(
      CompareMatrices(state.GetAccelerations() - state0.GetAccelerations(),
                      weights(2) * dz, kTolerance / kDt));
}

/* Tests that VelocityNewmarkScheme reproduces analytical solutions with
 constant acceleration and linear velocity. */
GTEST_TEST(VelocityNewmarkSchemeTest, AdvanceOneTimeStep) {
  VelocityNewmarkScheme<double> scheme{kDt, kGamma, kBeta};
  const Vector4d q = MakeQ();
  Vector4d qdot = MakeQdot();
  const Vector4d qddot = MakeQddot();
  const FemStateImpl<DummyElement> state_0(q, qdot, qddot);
  FemStateImpl<DummyElement> state_n(state_0);
  FemStateImpl<DummyElement> state_np1(state_0);
  const int kTimeSteps = 10;
  for (int i = 0; i < kTimeSteps; ++i) {
    qdot += kDt * qddot;
    scheme.AdvanceOneTimeStep(state_n, qdot, &state_np1);
    state_n = state_np1;
  }
  const double total_time = kDt * kTimeSteps;
  EXPECT_TRUE(CompareMatrices(state_n.GetAccelerations(),
                              state_0.GetAccelerations(),
                              kTolerance * kTimeSteps / kDt));
  EXPECT_TRUE(CompareMatrices(
      state_n.GetVelocities(),
      state_0.GetVelocities() + total_time * state_0.GetAccelerations(),
      kTimeSteps * kTolerance));
  EXPECT_TRUE(CompareMatrices(
      state_n.GetPositions(),
      state_0.GetPositions() + total_time * state_0.GetVelocities() +
          0.5 * total_time * total_time * state_0.GetAccelerations(),
      kTimeSteps * kTolerance));
}

/* Tests that `VelocityNewmarkScheme` is equivalent with
 `AccelerationNewmarkScheme` by advancing one time step with each integration
 scheme using the same initial state and verifying that the resulting new states
 are the same. */
GTEST_TEST(VelocityNewmarkSchemeTest, EquivalenceWithAccelerationNewmark) {
  VelocityNewmarkScheme<double> velocity_scheme{kDt, kGamma, kBeta};
  FemStateImpl<DummyElement> state0(MakeQ(), MakeQdot(), MakeQddot());
  FemStateImpl<DummyElement> state_v(state0);
  velocity_scheme.AdvanceOneTimeStep(state0, MakeQdot(), &state_v);

  AccelerationNewmarkScheme<double> acceleration_scheme{kDt, kGamma, kBeta};
  FemStateImpl<DummyElement> state_a(state0);
  acceleration_scheme.AdvanceOneTimeStep(state0, state_v.GetAccelerations(),
                                         &state_a);
  EXPECT_TRUE(CompareMatrices(state_v.GetAccelerations(),
                              state_a.GetAccelerations(), kTolerance));
  EXPECT_TRUE(CompareMatrices(state_v.GetVelocities(), state_a.GetVelocities(),
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(state_v.GetPositions(), state_a.GetPositions(),
                              kTolerance));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
