#include "drake/multibody/tree/space_xyz_floating_mobilizer.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

// Fixture to setup a simple model containing a space xyz floating mobilizer.
class SpaceXYZFloatingMobilizerTest : public MobilizerTester {
 public:
  // Creates a simple model consisting of a single body with a space xyz
  // floating mobilizer connecting it to the world.
  void SetUp() override {
    mobilizer_ = &AddMobilizerAndFinalize(
        std::make_unique<SpaceXYZFloatingMobilizer<double>>(
            tree().world_body().body_frame(), body_->body_frame()));
  }

  // Helper to set the this fixture's context to an arbitrary non-zero state
  // comprised of arbitrary_rpy() and arbitrary_translation().
  void SetArbitraryNonZeroState() {
    mobilizer_->set_angles(context_.get(), arbitrary_rpy().vector());
    mobilizer_->set_translation(context_.get(), arbitrary_translation());
  }

  RollPitchYawd arbitrary_rpy() const {
    return RollPitchYawd(M_PI / 3, -M_PI / 3, M_PI / 5);
  }
  Vector3d arbitrary_translation() const { return Vector3d(1.0, 2.0, 3.0); }

 protected:
  const SpaceXYZFloatingMobilizer<double>* mobilizer_{nullptr};
};

// Verifies methods to mutate and access the context.
TEST_F(SpaceXYZFloatingMobilizerTest, BasicIntrospection) {
  EXPECT_TRUE(mobilizer_->is_floating());
  EXPECT_FALSE(mobilizer_->has_quaternion_dofs());
}

// Verifies methods to mutate and access the context.
TEST_F(SpaceXYZFloatingMobilizerTest, ConfigurationAccessAndMutation) {
  SetArbitraryNonZeroState();
  EXPECT_EQ(mobilizer_->get_angles(*context_), arbitrary_rpy().vector());
  EXPECT_EQ(mobilizer_->get_translation(*context_), arbitrary_translation());
  Vector6<double> q;
  q << arbitrary_rpy().vector(), arbitrary_translation();
  EXPECT_EQ(mobilizer_->get_generalized_positions(*context_), q);
}

TEST_F(SpaceXYZFloatingMobilizerTest, SetFromRigidTransform) {
  SetArbitraryNonZeroState();
  const RigidTransformd X_WB(arbitrary_rpy(), arbitrary_translation());
  mobilizer_->SetFromRigidTransform(context_.get(), X_WB);
  EXPECT_TRUE(CompareMatrices(mobilizer_->get_angles(*context_),
                              arbitrary_rpy().vector(), kTolerance,
                              MatrixCompareType::relative));
  EXPECT_EQ(mobilizer_->get_translation(*context_), arbitrary_translation());
}

TEST_F(SpaceXYZFloatingMobilizerTest, VelocityAccessAndMutation) {
  const Vector3d w_FM(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->set_angular_velocity(context_.get(), w_FM);
  EXPECT_EQ(mobilizer_->get_angular_velocity(*context_), w_FM);

  const Vector3d v_FM(1.0, 2.0, 3.0);
  mobilizer_->set_translational_velocity(context_.get(), v_FM);
  EXPECT_EQ(mobilizer_->get_translational_velocity(*context_), v_FM);

  const auto v = (Vector6<double>() << w_FM, v_FM).finished();
  EXPECT_EQ(mobilizer_->get_generalized_velocities(*context_), v);
}

TEST_F(SpaceXYZFloatingMobilizerTest, ZeroState) {
  SetArbitraryNonZeroState();

  // The non-zero state is not the identity transform as expected.
  EXPECT_FALSE(
      mobilizer_->CalcAcrossMobilizerTransform(*context_).IsExactlyIdentity());

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // an identity rigid transform.
  mobilizer_->set_zero_state(*context_, &context_->get_mutable_state());
  EXPECT_TRUE(
      mobilizer_->CalcAcrossMobilizerTransform(*context_).IsExactlyIdentity());
}

// Verify the correctness of across-mobilizer quantities; X_FM, V_FM, A_FM.
TEST_F(SpaceXYZFloatingMobilizerTest, CalcAcrossMobilizer) {
  SetArbitraryNonZeroState();

  const RigidTransformd X_FM_expected(arbitrary_rpy(), arbitrary_translation());
  const RigidTransformd X_FM =
      mobilizer_->CalcAcrossMobilizerTransform(*context_);
  EXPECT_TRUE(CompareMatrices(X_FM.GetAsMatrix34(),
                              X_FM_expected.GetAsMatrix34(), kTolerance,
                              MatrixCompareType::relative));

  // Verify across mobilizer spatial velocity for an arbitrary non-zero state.
  Vector6<double> v;
  v << 1, 2, 3, 4, 5, 6;
  const SpatialVelocity<double> V_FM =
      mobilizer_->CalcAcrossMobilizerSpatialVelocity(*context_, v);
  EXPECT_TRUE(CompareMatrices(V_FM.get_coeffs(), v, kTolerance,
                              MatrixCompareType::relative));

  // Verify across mobilizer spatial velocity for an arbitrary non-zero state
  // and generalized accelrations.
  const SpatialAcceleration<double> A_FM =
      mobilizer_->CalcAcrossMobilizerSpatialAcceleration(*context_, v);
  EXPECT_TRUE(CompareMatrices(A_FM.get_coeffs(), v, kTolerance,
                              MatrixCompareType::relative));

  // Force projection across the mobilizer. Since the generalized velocities for
  // this mobilizer correspond to the spatial velocity across the mobilizer, the
  // generalized forces must equal the spatial force on the mobilizer frame due
  // to the principle of virtual work.
  SpatialForce<double> F(v);
  Vector6<double> tau;
  mobilizer_->ProjectSpatialForce(*context_, F, tau);
  EXPECT_TRUE(CompareMatrices(F.get_coeffs(), tau, kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(SpaceXYZFloatingMobilizerTest, MapVelocityToQdotAndBack) {
  SetArbitraryNonZeroState();
  const Vector6<double> v = (Vector6<double>() << 1, 2, 3, 4, 5, 6).finished();
  Vector6<double> qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);
  Vector6<double> v_back;
  mobilizer_->MapQDotToVelocity(*context_, qdot, &v_back);
  EXPECT_TRUE(
      CompareMatrices(v, v_back, kTolerance, MatrixCompareType::relative));
}

// For an arbitrary state verify that the computed Nplus(q) matrix is the
// inverse of N(q).
TEST_F(SpaceXYZFloatingMobilizerTest, KinematicMapping) {
  RollPitchYawd rpy(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->set_angles(context_.get(), rpy.vector());

  ASSERT_EQ(mobilizer_->num_positions(), 6);
  ASSERT_EQ(mobilizer_->num_velocities(), 6);

  // Compute N.
  Matrix6<double> N;
  mobilizer_->CalcNMatrix(*context_, &N);

  // Compute Nplus.
  Matrix6<double> Nplus;
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);

  // Verify that Nplus is the inverse of N.
  MatrixX<double> N_x_Nplus = N * Nplus;
  MatrixX<double> Nplus_x_N = Nplus * N;

  EXPECT_TRUE(CompareMatrices(N_x_Nplus, Matrix6<double>::Identity(),
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(Nplus_x_N, Matrix6<double>::Identity(),
                              kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
