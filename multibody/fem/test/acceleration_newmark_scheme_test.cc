#include "drake/multibody/fem/acceleration_newmark_scheme.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/fem/test/dummy_element.h"
#include "drake/multibody/fem/velocity_newmark_scheme.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

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
GTEST_TEST(AccelerationNewmarkSchemeTest, Weights) {
  AccelerationNewmarkScheme<double> scheme{kDt, kGamma, kBeta};
  EXPECT_TRUE(CompareMatrices(
      scheme.weights(), Vector3<double>(kBeta * kDt * kDt, kGamma * kDt, 1.0),
      0));
}

/* Verify that the result of
 `AccelerationNewmarkScheme::UpdateStateFromChangeInUnknowns()` is consistent
 with the weights. */
GTEST_TEST(AccelerationNewmarkSchemeTest, UpdateStateFromChangeInUnknowns) {
  AccelerationNewmarkScheme<double> scheme{kDt, kGamma, kBeta};
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
                      weights(2) * dz, kTolerance));
}

/* Tests that AccelerationNewmarkScheme reproduces analytical solutions with
 constant acceleration. */
GTEST_TEST(AccelerationNewmarkSchemeTest, AdvanceOneTimeStep) {
  AccelerationNewmarkScheme<double> scheme{kDt, kGamma, kBeta};
  const Vector4<double> q = MakeQ();
  const Vector4<double> qdot = MakeQdot();
  const Vector4<double> qddot = MakeQddot();
  const FemStateImpl<DummyElement> state_0(q, qdot, qddot);
  FemStateImpl<DummyElement> state_n(state_0);
  FemStateImpl<DummyElement> state_np1(state_0);
  const int kTimeSteps = 10;
  for (int i = 0; i < kTimeSteps; ++i) {
    scheme.AdvanceOneTimeStep(state_n, qddot, &state_np1);
    state_n = state_np1;
  }
  const double total_time = kDt * kTimeSteps;
  EXPECT_TRUE(
      CompareMatrices(state_n.GetAccelerations(), state_0.GetAccelerations()));
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

/* Tests that `AccelerationNewmarkScheme` is equivalent with
 `VelocityNewmarkScheme` by advancing one time step with each integration
 scheme using the same initial state and verifying that the resulting new states
 are the same. */
GTEST_TEST(AccelerationNewmarkSchemeTest, EquivalenceWithVelocityNewmark) {
  AccelerationNewmarkScheme<double> acceleration_scheme{kDt, kGamma, kBeta};
  FemStateImpl<DummyElement> state0(MakeQ(), MakeQdot(), MakeQddot());
  FemStateImpl<DummyElement> state_a(state0);
  acceleration_scheme.AdvanceOneTimeStep(state0, MakeQddot(), &state_a);

  VelocityNewmarkScheme<double> velocity_scheme{kDt, kGamma, kBeta};
  FemStateImpl<DummyElement> state_v(state0);
  velocity_scheme.AdvanceOneTimeStep(state0, state_a.GetVelocities(), &state_v);
  /* Set a larger error tolerance to accomodate the division by `dt` used in the
   VelocityNewmarkScheme. */
  EXPECT_TRUE(CompareMatrices(state_v.GetAccelerations(),
                              state_a.GetAccelerations(), kTolerance / kDt));
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
