#include "drake/multibody/tree/quaternion_floating_mobilizer.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Quaterniond;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using std::make_unique;
using std::unique_ptr;
using systems::Context;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

// Fixture to setup a simple MBT model containing a quaternion mobilizer.
class QuaternionFloatingMobilizerTest : public MobilizerTester {
 public:
  void SetUp() override {
    mobilizer_ = &AddMobilizerAndFinalize(
        std::make_unique<QuaternionFloatingMobilizer<double>>(
            tree().world_body().body_frame(), body_->body_frame()));
  }

 protected:
  const QuaternionFloatingMobilizer<double>* mobilizer_{nullptr};
};

// Verifies methods to mutate and access the context.
TEST_F(QuaternionFloatingMobilizerTest, StateAccess) {
  const Quaterniond quaternion_value(
      RollPitchYawd(M_PI / 3, -M_PI / 3, M_PI / 5).ToQuaternion());
  mobilizer_->set_quaternion(context_.get(), quaternion_value);
  EXPECT_EQ(mobilizer_->get_quaternion(*context_).coeffs(),
            quaternion_value.coeffs());

  const Vector3d position_value(1.0, 2.0, 3.0);
  mobilizer_->set_position(context_.get(), position_value);
  EXPECT_EQ(mobilizer_->get_position(*context_), position_value);

  const RotationMatrixd R_WB(RollPitchYawd(M_PI / 5, -M_PI / 7, M_PI / 3));
  const Quaterniond Q_WB = R_WB.ToQuaternion();
  mobilizer_->SetFromRotationMatrix(context_.get(), R_WB.matrix());
  EXPECT_TRUE(CompareMatrices(mobilizer_->get_quaternion(*context_).coeffs(),
                              Q_WB.coeffs(),
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(QuaternionFloatingMobilizerTest, ZeroState) {
  // Set an arbitrary "non-zero" state.
  const Quaterniond quaternion_value(
      RollPitchYawd(M_PI / 3, -M_PI / 3, M_PI / 5).ToQuaternion());
  mobilizer_->set_quaternion(context_.get(), quaternion_value);
  EXPECT_EQ(mobilizer_->get_quaternion(*context_).coeffs(),
            quaternion_value.coeffs());

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // an identity rigid transform.
  mobilizer_->set_zero_state(*context_, &context_->get_mutable_state());
  const RigidTransformd X_WB(
      mobilizer_->CalcAcrossMobilizerTransform(*mbt_context_));
  EXPECT_TRUE(X_WB.IsExactlyIdentity());
}

// For an arbitrary state verify that the computed Nplus(q) matrix is the
// left pseudoinverse of N(q).
TEST_F(QuaternionFloatingMobilizerTest, KinematicMapping) {
  const Quaterniond Q_WB(
      RollPitchYawd(M_PI / 3, -M_PI / 3, M_PI / 5).ToQuaternion());
  mobilizer_->set_quaternion(context_.get(), Q_WB);

  const Vector3d p_WB(1.0, 2.0, 3.0);
  mobilizer_->set_position(context_.get(), p_WB);

  ASSERT_EQ(mobilizer_->num_positions(), 7);
  ASSERT_EQ(mobilizer_->num_velocities(), 6);

  // Compute N.
  MatrixX<double> N(7, 6);
  mobilizer_->CalcNMatrix(*mbt_context_, &N);

  // Compute Nplus.
  MatrixX<double> Nplus(6, 7);
  mobilizer_->CalcNplusMatrix(*mbt_context_, &Nplus);

  // Verify that Nplus is the left pseudoinverse of N.
  MatrixX<double> Nplus_x_N = Nplus * N;

  EXPECT_TRUE(CompareMatrices(
      Nplus_x_N, MatrixX<double>::Identity(6, 6),
      kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
