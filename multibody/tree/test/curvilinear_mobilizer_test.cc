#include "drake/multibody/tree/curvilinear_mobilizer.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/trajectories/piecewise_constant_curvature_trajectory.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/curvilinear_joint.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using drake::trajectories::PiecewiseConstantCurvatureTrajectory;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;
using std::make_unique;
using std::sqrt;
using std::unique_ptr;
using systems::Context;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Fixture to setup a simple model containing a curvilinear mobilizer.
class CurvilinearMobilizerTest : public MobilizerTester {
 public:
  void SetUp() override {
    std::unique_ptr<CurvilinearJoint<double>> joint =
        std::make_unique<CurvilinearJoint<double>>(
            "joint1", tree().world_body().body_frame(), body_->body_frame(),
            trajectory_);
    mobilizer_ = &AddJointAndFinalize<CurvilinearJoint, CurvilinearMobilizer>(
        std::move(joint));
  }

 protected:
  const CurvilinearMobilizer<double>* mobilizer_{nullptr};
  // We specify a trajectory that draws a "stadium curve" (a rectangle with two
  // semicircle caps). The Rectangular section has length l_ and the circular
  // sections have radii 1/k_. This trajectory is periodic, of total length 2*l_
  // + 2*pi/k_.
  const double k_ = 1.0;  // Rectangular region length.
  const double l_ = 1.0;  // Radius of the circular end caps.
  const std::vector<double> breaks_{0, M_PI / k_, l_ + M_PI / k_,
                                    l_ + 2 * M_PI / k_, 2 * l_ + 2 * M_PI / k_};
  const std::vector<double> turning_rates_{k_, 0, k_, 0};
  const Vector3d tangent_axis_{1., -std::sqrt(2.), 1.};
  const Vector3d plane_axis_{1., std::sqrt(2.), 1.};
  const Vector3d initial_position_{1., 2., 3.};
  const bool is_periodic_{true};
  const PiecewiseConstantCurvatureTrajectory<double> trajectory_{
      breaks_,     turning_rates_,    tangent_axis_,
      plane_axis_, initial_position_, is_periodic_};
};

TEST_F(CurvilinearMobilizerTest, CanRotateAndTranslate) {
  EXPECT_TRUE(mobilizer_->can_rotate());
  EXPECT_TRUE(mobilizer_->can_translate());
}

TEST_F(CurvilinearMobilizerTest, StateAccess) {
  const double some_value1 = 1.5;
  const double some_value2 = std::sqrt(2);

  // Verify we can set position given the model's context.
  mobilizer_->SetDistance(context_.get(), some_value1);
  EXPECT_EQ(mobilizer_->get_distance(*context_), some_value1);
  mobilizer_->SetDistance(context_.get(), some_value2);
  EXPECT_EQ(mobilizer_->get_distance(*context_), some_value2);

  // Verify we can set tangential velocity given the model's context.
  mobilizer_->SetTangentialVelocity(context_.get(), some_value1);
  EXPECT_EQ(mobilizer_->get_tangential_velocity(*context_), some_value1);
  mobilizer_->SetTangentialVelocity(context_.get(), some_value2);
  EXPECT_EQ(mobilizer_->get_tangential_velocity(*context_), some_value2);
}

TEST_F(CurvilinearMobilizerTest, ZeroState) {
  const double some_value1 = 1.5;
  const double some_value2 = std::sqrt(2);

  // Set the state to some arbitrary non-zero value.
  mobilizer_->SetDistance(context_.get(), some_value1);
  EXPECT_EQ(mobilizer_->get_distance(*context_), some_value1);
  mobilizer_->SetTangentialVelocity(context_.get(), some_value2);
  EXPECT_EQ(mobilizer_->get_tangential_velocity(*context_), some_value2);

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // zero position and velocity.
  mobilizer_->SetZeroState(*context_, &context_->get_mutable_state());
  EXPECT_EQ(mobilizer_->get_distance(*context_), 0);
  EXPECT_EQ(mobilizer_->get_tangential_velocity(*context_), 0);
}

TEST_F(CurvilinearMobilizerTest, CalcAcrossMobilizerTransform) {
  const double kTol = 4 * std::numeric_limits<double>::epsilon();
  const double wrapped_distance = 0.5 * M_PI / k_;
  const double distance = wrapped_distance + trajectory_.end_time();

  mobilizer_->SetDistance(context_.get(), distance);
  RigidTransformd X_FM(mobilizer_->CalcAcrossMobilizerTransform(*context_));

  // Expect the mobilizer pose to be the trajectory pose, modulo curve length.
  const RigidTransformd X_FM_expected = trajectory_.CalcPose(wrapped_distance);
  EXPECT_TRUE(X_FM.IsNearlyEqualTo(X_FM_expected, kTol));

  // Now check the fast inline methods.
  RigidTransformd fast_X_FM = mobilizer_->calc_X_FM(&distance);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));
  const double new_distance = 1.5 * distance;
  mobilizer_->SetDistance(context_.get(), new_distance);
  X_FM = mobilizer_->CalcAcrossMobilizerTransform(*context_);
  mobilizer_->update_X_FM(&new_distance, &fast_X_FM);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));

  TestApplyR_FM(X_FM, *mobilizer_);
  TestPrePostMultiplyByX_FM(X_FM, *mobilizer_);
}

TEST_F(CurvilinearMobilizerTest, CalcAcrossMobilizerSpatialVelocity) {
  const double tangential_velocity = 1.5;
  const double wrapped_distance = 0.5 * M_PI / k_;
  const double distance = wrapped_distance + trajectory_.end_time();
  mobilizer_->SetDistance(context_.get(), distance);
  const SpatialVelocity<double> V_FM_F =
      mobilizer_->CalcAcrossMobilizerSpatialVelocity(
          *context_, Vector1d(tangential_velocity));

  // Expect the mobilizer spatial velocity to be the trajectory spatial
  // velocity, modulo curve length.
  const SpatialVelocity<double> V_FM_F_expected =
      trajectory_.CalcSpatialVelocity(wrapped_distance, tangential_velocity);
  EXPECT_TRUE(V_FM_F.IsApprox(V_FM_F_expected, kEpsilon));

  // The low-level function should give the same result.
  const SpatialVelocity<double> V_FM_F_inline =
      mobilizer_->calc_V_FM(&distance, &tangential_velocity);
  EXPECT_TRUE(V_FM_F_inline.IsApprox(V_FM_F_expected, kEpsilon));

  // The low-level M-frame function should match the trajectory M-frame result.
  const SpatialVelocity<double> V_FM_M_expected =
      trajectory_.CalcSpatialVelocityInM(wrapped_distance, tangential_velocity);
  const SpatialVelocity<double> V_FM_M_inline = mobilizer_->calc_V_FM_M(
      RigidTransformd() /*not used*/, &distance, &tangential_velocity);
  EXPECT_TRUE(V_FM_M_inline.IsApprox(V_FM_M_expected, kEpsilon));
}

TEST_F(CurvilinearMobilizerTest, CalcAcrossMobilizerSpatialAcceleration) {
  const double tangential_acceleration = 1.5;
  const double tangential_velocity = 1.5;
  const double wrapped_distance = 0.5 * M_PI / k_;
  const double distance = wrapped_distance + trajectory_.end_time();
  mobilizer_->SetDistance(context_.get(), distance);
  mobilizer_->SetTangentialVelocity(context_.get(), tangential_velocity);
  const SpatialAcceleration<double> A_FM_F =
      mobilizer_->CalcAcrossMobilizerSpatialAcceleration(
          *context_, Vector1d(tangential_acceleration));

  const SpatialAcceleration<double> A_FM_F_expected =
      trajectory_.CalcSpatialAcceleration(wrapped_distance, tangential_velocity,
                                          tangential_acceleration);

  // Expect the mobilizer spatial acceleration to be the trajectory spatial
  // acceleration, modulo curve length.
  EXPECT_TRUE(A_FM_F.IsApprox(A_FM_F_expected, kEpsilon));

  // The low-level function should give the same result.
  const SpatialAcceleration<double> A_FM_F_inline = mobilizer_->calc_A_FM(
      &distance, &tangential_velocity, &tangential_acceleration);
  EXPECT_TRUE(A_FM_F_inline.IsApprox(A_FM_F_expected, kEpsilon));

  // The low-level M-frame function should match the trajectory M-frame result.
  const SpatialAcceleration<double> A_FM_M_expected =
      trajectory_.CalcSpatialAccelerationInM(
          wrapped_distance, tangential_velocity, tangential_acceleration);
  const SpatialAcceleration<double> A_FM_M_inline =
      mobilizer_->calc_A_FM_M(RigidTransformd() /*not used*/, &distance,
                              &tangential_velocity, &tangential_acceleration);
  EXPECT_TRUE(A_FM_M_inline.IsApprox(A_FM_M_expected, kEpsilon));
}

TEST_F(CurvilinearMobilizerTest, ProjectSpatialForce) {
  const Vector3d torque_Mo_F(1.0, 2.0, 3.0);
  const Vector3d force_Mo_F(1.0, 2.0, 3.0);
  const SpatialForce<double> F_Mo_F(torque_Mo_F, force_Mo_F);
  Vector1d tau;
  const double distance = breaks_[1] / 2.3;
  // Set an arbitrary position.
  mobilizer_->SetDistance(context_.get(), distance);
  mobilizer_->ProjectSpatialForce(*context_, F_Mo_F, tau);

  // Only the torque along affects joint.
  // tau_rotational = (torque⋅(rotation axis))⋅turning rate.
  const double rotational_component_expected =
      k_ * torque_Mo_F.dot(plane_axis_.normalized());

  const RotationMatrixd R_FM = trajectory_.CalcPose(distance).rotation();
  // The force should be projected along the tangent (x) axis.
  const Vector3d tangent_axis = R_FM.col(0);
  const double translational_component_expected = force_Mo_F.dot(tangent_axis);
  const double tau_expected =
      rotational_component_expected + translational_component_expected;
  EXPECT_NEAR(tau(0), tau_expected, 4.0 * kEpsilon);

  // The low-level function should give the same result.
  double tau_from_F{};
  mobilizer_->calc_tau(&distance, F_Mo_F, &tau_from_F);
  EXPECT_NEAR(tau_from_F, tau_expected, 4.0 * kEpsilon);

  // Taking the force in the M frame shouldn't change the result.
  const SpatialForce<double> F_Mo_M = R_FM.transpose() * F_Mo_F;
  double tau_from_M{};
  mobilizer_->calc_tau_from_M(RigidTransformd() /*not used*/, &distance, F_Mo_M,
                              &tau_from_M);
  EXPECT_NEAR(tau_from_M, tau_expected, 4.0 * kEpsilon);
}

TEST_F(CurvilinearMobilizerTest, MapVelocityToQDotAndBack) {
  EXPECT_TRUE(mobilizer_->is_velocity_equal_to_qdot());

  Vector1d v(1.5);
  Vector1d qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);
  EXPECT_EQ(qdot(0), v(0));

  qdot(0) = -std::sqrt(2);
  mobilizer_->MapQDotToVelocity(*context_, qdot, &v);
  EXPECT_EQ(v(0), qdot(0));
}

TEST_F(CurvilinearMobilizerTest, KinematicMapping) {
  // For this joint, Nplus = 1 independently of the state. We therefore set the
  // state to NaN in order to verify this.
  tree()
      .GetMutablePositionsAndVelocities(context_.get())
      .setConstant(std::numeric_limits<double>::quiet_NaN());

  // Compute N.
  MatrixX<double> N(1, 1);
  mobilizer_->CalcNMatrix(*context_, &N);
  EXPECT_EQ(N(0, 0), 1.0);

  // Compute Nplus.
  MatrixX<double> Nplus(1, 1);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);
  EXPECT_EQ(Nplus(0, 0), 1.0);

  // Ensure Ṅ(q,q̇) = 1x1 zero matrix.
  MatrixX<double> NDot(1, 1);
  mobilizer_->CalcNDotMatrix(*context_, &NDot);
  EXPECT_EQ(NDot(0, 0), 0.0);

  // Ensure Ṅ⁺(q,q̇) = 1x1 zero matrix.
  MatrixX<double> NplusDot(1, 1);
  mobilizer_->CalcNplusDotMatrix(*context_, &NplusDot);
  EXPECT_EQ(NplusDot(0, 0), 0.0);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
