#include "drake/multibody/fem/velocity_newmark_scheme.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/fem/acceleration_newmark_scheme.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

const double kDt = 1e-3;
const double kGamma = 0.6;
const double kBeta = 0.3;
const double kTolerance = 8.0 * std::numeric_limits<double>::epsilon();

/* Arbitrary initial states. */
Vector3<double> q() {
  return Vector3<double>(1.23, 2.34, 3.45);
}
Vector3<double> v() {
  return Vector3<double>(5.67, 6.78, 7.89);
}
Vector3<double> a() {
  return Vector3<double>(0.1011, 0.1112, 0.1213);
}

class VelocityNewmarkSchemeTest : public ::testing::Test {
 protected:
  void SetUp() {
    fem_state_system_ = std::make_unique<FemStateSystem<double>>(q(), v(), a());
  }

  const VelocityNewmarkScheme<double> scheme_{kDt, kGamma, kBeta};
  std::unique_ptr<FemStateSystem<double>> fem_state_system_;
};

/* Verify that the weights returned by the accessor match expectation. */
TEST_F(VelocityNewmarkSchemeTest, Weights) {
  EXPECT_TRUE(CompareMatrices(
      scheme_.GetWeights(),
      Vector3<double>(kBeta / kGamma * kDt, 1.0, 1.0 / (kGamma * kDt))));
}

/* Verify that the unknowns are the velocities. */
TEST_F(VelocityNewmarkSchemeTest, Unknowns) {
  const FemState<double> state(fem_state_system_.get());
  EXPECT_EQ(scheme_.GetUnknowns(state), v());
}

/* Verify that the result of
 `VelocityNewmarkScheme::UpdateStateFromChangeInUnknowns()` is consistent with
 the weights. */
TEST_F(VelocityNewmarkSchemeTest, UpdateStateFromChangeInUnknowns) {
  const FemState<double> state0(fem_state_system_.get());
  FemState<double> state(fem_state_system_.get());
  const Vector3<double> dz(1.234, 4.567, 7.890);
  const Vector3<double>& weights = scheme_.GetWeights();
  scheme_.UpdateStateFromChangeInUnknowns(dz, &state);
  const auto q_expected = state0.GetPositions() + weights(0) * dz;
  const auto v_expected = state0.GetVelocities() + weights(1) * dz;
  const auto a_expected = state0.GetAccelerations() + weights(2) * dz;

  EXPECT_TRUE(CompareMatrices(state.GetPositions(), q_expected, kTolerance));
  EXPECT_TRUE(CompareMatrices(state.GetVelocities(), v_expected, kTolerance));
  EXPECT_TRUE(
      CompareMatrices(state.GetAccelerations(), a_expected, kTolerance));
}

/* Tests that VelocityNewmarkScheme reproduces analytical solutions with
 constant acceleration and linear velocity. */
TEST_F(VelocityNewmarkSchemeTest, AdvanceOneTimeStep) {
  Vector3<double> qdot = v();
  const FemState<double> state_0(fem_state_system_.get());
  FemState<double> state_n(fem_state_system_.get());
  FemState<double> state_np1(fem_state_system_.get());
  const int kTimeSteps = 10;
  for (int i = 0; i < kTimeSteps; ++i) {
    qdot += kDt * a();
    scheme_.AdvanceOneTimeStep(state_n, qdot, &state_np1);
    // TODO(xuchenhan-tri): We need a FemState::SetFrom method or an assignment
    // operator.
    state_n.SetAccelerations(state_np1.GetAccelerations());
    state_n.SetVelocities(state_np1.GetVelocities());
    state_n.SetPositions(state_np1.GetPositions());
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

/* Tests that `VelocityNewmarkScheme` is equivalent to
 `AccelerationNewmarkScheme` by advancing one time step with each integration
 scheme using the same initial state and verifying that the resulting new states
 are the same. */
TEST_F(VelocityNewmarkSchemeTest, EquivalenceWithAccelerationNewmark) {
  const FemState<double> state0(fem_state_system_.get());
  FemState<double> state_v(fem_state_system_.get());
  scheme_.AdvanceOneTimeStep(state0, v(), &state_v);

  const AccelerationNewmarkScheme<double> acceleration_scheme{kDt, kGamma,
                                                              kBeta};
  FemState<double> state_a(fem_state_system_.get());
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
