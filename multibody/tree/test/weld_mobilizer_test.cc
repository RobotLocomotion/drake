#include "drake/multibody/tree/weld_mobilizer.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;
using std::make_unique;
using std::unique_ptr;
using systems::Context;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

// Fixture to setup a simple MBT model containing a weld mobilizer.
class WeldMobilizerTest :  public MobilizerTester {
 public:
  void SetUp() override {
    weld_body_to_world_ = &AddMobilizerAndFinalize(
        std::make_unique<WeldMobilizer<double>>(
            tree().world_body().body_frame(), body_->body_frame(), X_WB_));
  }

 protected:
  const WeldMobilizer<double>* weld_body_to_world_{nullptr};
  // Pose of body B in the world frame W.
  math::RigidTransformd X_WB_;
};

TEST_F(WeldMobilizerTest, ZeroSizedState) {
  EXPECT_EQ(tree().num_positions(), 0);
  EXPECT_EQ(tree().num_velocities(), 0);
}

TEST_F(WeldMobilizerTest, CalcAcrossMobilizerTransform) {
  const math::RigidTransformd X_FM(
      weld_body_to_world_->CalcAcrossMobilizerTransform(*context_));
  EXPECT_TRUE(CompareMatrices(X_FM.GetAsMatrix34(),
                              X_WB_.GetAsMatrix34(),
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(WeldMobilizerTest, CalcAcrossMobilizerSpatialVeloctiy) {
  const VectorXd zero_sized_vector(0);
  const SpatialVelocity<double> V_FM =
      weld_body_to_world_->CalcAcrossMobilizerSpatialVelocity(
          *context_, zero_sized_vector);
  EXPECT_EQ(V_FM.get_coeffs(), Vector6<double>::Zero());
}

TEST_F(WeldMobilizerTest, CalcAcrossMobilizerSpatialAcceleration) {
  const VectorXd zero_sized_vector(0);
  const SpatialAcceleration<double> A_FM =
      weld_body_to_world_->CalcAcrossMobilizerSpatialAcceleration(
          *context_, zero_sized_vector);
  EXPECT_EQ(A_FM.get_coeffs(), Vector6<double>::Zero());
}

TEST_F(WeldMobilizerTest, ProjectSpatialForce) {
  VectorXd zero_sized_vector(0);
  const SpatialForce<double> F_Mo_F;  // value not important for this test.
  // no-op, just tests we can call it with a zero sized vector.
  weld_body_to_world_->ProjectSpatialForce(
      *context_, F_Mo_F, zero_sized_vector);
}

TEST_F(WeldMobilizerTest, MapVelocityToQDotAndBack) {
  VectorXd zero_sized_vector(0);
  // These methods are no-ops, just test we can call them with zero sized
  // vectors.
  weld_body_to_world_->MapVelocityToQDot(*context_,
                                         zero_sized_vector, &zero_sized_vector);
  weld_body_to_world_->MapQDotToVelocity(*context_,
                                         zero_sized_vector, &zero_sized_vector);
}

TEST_F(WeldMobilizerTest, KinematicMapping) {
  // These methods are no-ops, just test we can call them with zero sized
  // matrices.
  MatrixX<double> N(0, 0);
  weld_body_to_world_->CalcNMatrix(*context_, &N);
  weld_body_to_world_->CalcNplusMatrix(*context_, &N);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
