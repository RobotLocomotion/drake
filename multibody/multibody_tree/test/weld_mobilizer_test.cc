#include "drake/multibody/multibody_tree/weld_mobilizer.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace {

using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::make_unique;
using std::unique_ptr;
using systems::Context;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

// Fixture to setup a simple MBT model containing a weld mobilizer.
class WeldMobilizerTest : public ::testing::Test {
 public:
  // Creates a simple model consisting of a single body with a weld mobilizer
  // connecting it to the world, with the sole purpose of verifying the
  // WeldMobilizer methods.
  void SetUp() override {
    // Spatial inertia for adding a body. The actual value is not important for
    // these tests since they are all kinematic.
    const SpatialInertia<double> M_B;

    // Add a body so we can add a mobilizer to it.
    body_ = &model_.AddBody<RigidBody>(M_B);

    X_WB_ = Translation3d(1.0, 2.0, 3.0);

    // Add a weld mobilizer between the world and the body:
    weld_body_to_world_ = &model_.AddMobilizer<WeldMobilizer>(
        model_.world_body().body_frame(), body_->body_frame(), X_WB_);

    // We are done adding modeling elements. Finalize the model:
    model_.Finalize();

    // Create a context to store the state for this model:
    context_ = model_.CreateDefaultContext();
    // Performance critical queries take a MultibodyTreeContext to avoid dynamic
    // casting.
    mbt_context_ = dynamic_cast<MultibodyTreeContext<double>*>(context_.get());
    ASSERT_NE(mbt_context_, nullptr);
  }

 protected:
  MultibodyTree<double> model_;
  const RigidBody<double>* body_{nullptr};
  const WeldMobilizer<double>* weld_body_to_world_{nullptr};
  std::unique_ptr<Context<double>> context_;
  MultibodyTreeContext<double>* mbt_context_{nullptr};
  // Pose of body B in the world frame W.
  Isometry3d X_WB_;
};

TEST_F(WeldMobilizerTest, ZeroSizedState) {
  EXPECT_EQ(model_.num_positions(), 0);
  EXPECT_EQ(model_.num_velocities(), 0);
}

TEST_F(WeldMobilizerTest, CalcAcrossMobilizerTransform) {
  const Isometry3d X_FM =
      weld_body_to_world_->CalcAcrossMobilizerTransform(*mbt_context_);
  EXPECT_TRUE(CompareMatrices(X_FM.matrix(), X_WB_.matrix(),
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(WeldMobilizerTest, CalcAcrossMobilizerSpatialVeloctiy) {
  const VectorXd zero_sized_vector(0);
  const SpatialVelocity<double> V_FM =
      weld_body_to_world_->CalcAcrossMobilizerSpatialVelocity(
          *mbt_context_, zero_sized_vector);
  EXPECT_EQ(V_FM.get_coeffs(), Vector6<double>::Zero());
}

TEST_F(WeldMobilizerTest, CalcAcrossMobilizerSpatialAcceleration) {
  const VectorXd zero_sized_vector(0);
  const SpatialAcceleration<double> A_FM =
      weld_body_to_world_->CalcAcrossMobilizerSpatialAcceleration(
          *mbt_context_, zero_sized_vector);
  EXPECT_EQ(A_FM.get_coeffs(), Vector6<double>::Zero());
}

TEST_F(WeldMobilizerTest, ProjectSpatialForce) {
  VectorXd zero_sized_vector(0);
  const SpatialForce<double> F_Mo_F;  // value not important for this test.
  // no-op, just tests we can call it with a zero sized vector.
  weld_body_to_world_->ProjectSpatialForce(
      *mbt_context_, F_Mo_F, zero_sized_vector);
}

TEST_F(WeldMobilizerTest, MapVelocityToQDotAndBack) {
  VectorXd zero_sized_vector(0);
  // These methods are no-ops, just test we can call them with zero sized
  // vectors.
  weld_body_to_world_->MapVelocityToQDot(*mbt_context_,
                                         zero_sized_vector, &zero_sized_vector);
  weld_body_to_world_->MapQDotToVelocity(*mbt_context_,
                                         zero_sized_vector, &zero_sized_vector);
}

}  // namespace
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
