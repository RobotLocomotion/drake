#include "drake/multibody/multibody_tree/prismatic_mobilizer.h"

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
using std::make_unique;
using std::unique_ptr;
using systems::Context;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

// Fixture to setup a simple MBT model containing a prismatic mobilizer.
class PrismaticMobilizerTest : public ::testing::Test {
 public:
  // Creates a simple model consisting of a single body with a prismatic joint
  // connecting it to the world, with the sole purpose of verifying the
  // PrismaticJoint methods.
  void SetUp() override {
    // Spatial inertia for adding a body. The actual value is not important for
    // these tests since they are all kinematic.
    const SpatialInertia<double> M_B;

    // Add a body so we can add a mobilizer to it.
    body_ = &model_.AddBody<RigidBody>(M_B);

    // Add a prismatic mobilizer between the world and the body:
    slider_ = &model_.AddMobilizer<PrismaticMobilizer>(
        model_.world_body().body_frame(), body_->body_frame(), axis_F_);

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
  const PrismaticMobilizer<double>* slider_{nullptr};
  std::unique_ptr<Context<double>> context_;
  MultibodyTreeContext<double>* mbt_context_{nullptr};
  // Prismatic mobilizer axis, expressed in the inboard frame F.
  // It's intentionally left non-normalized to verify the mobilizer properly
  // normalizes it at construction.
  const Vector3d axis_F_{1.0, 2.0, 3.0};
};

// Verify that PrismaticMobilizer normalizes its axis on construction.
TEST_F(PrismaticMobilizerTest, AxisIsNormalizedAtConstruction) {
  EXPECT_TRUE(CompareMatrices(
      slider_->translation_axis(), axis_F_.normalized(),
      kTolerance, MatrixCompareType::relative));
}

// Verifies method to mutate and access the context.
TEST_F(PrismaticMobilizerTest, StateAccess) {
  const double some_value1 = 1.5;
  const double some_value2 = std::sqrt(2);
  // Verify we can set a prismatic mobilizer position given the model's context.
  slider_->set_translation(context_.get(), some_value1);
  EXPECT_EQ(slider_->get_translation(*context_), some_value1);
  slider_->set_translation(context_.get(), some_value2);
  EXPECT_EQ(slider_->get_translation(*context_), some_value2);

  // Verify we can set a prismatic mobilizer position rate given the model's
  // context.
  slider_->set_translation_rate(context_.get(), some_value1);
  EXPECT_EQ(slider_->get_translation_rate(*context_), some_value1);
  slider_->set_translation_rate(context_.get(), some_value2);
  EXPECT_EQ(slider_->get_translation_rate(*context_), some_value2);
}

TEST_F(PrismaticMobilizerTest, ZeroState) {
  const double some_value1 = 1.5;
  const double some_value2 = std::sqrt(2);
  // Set the state to some arbitrary non-zero value.
  slider_->set_translation(context_.get(), some_value1);
  EXPECT_EQ(slider_->get_translation(*context_), some_value1);
  slider_->set_translation_rate(context_.get(), some_value2);
  EXPECT_EQ(slider_->get_translation_rate(*context_), some_value2);

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // zero position and velocity.
  slider_->set_zero_state(*context_, &context_->get_mutable_state());
  EXPECT_EQ(slider_->get_translation(*context_), 0);
  EXPECT_EQ(slider_->get_translation_rate(*context_), 0);
}

TEST_F(PrismaticMobilizerTest, CalcAcrossMobilizerTransform) {
  const double translation = 1.5;
  slider_->set_translation(context_.get(), translation);
  const Isometry3d X_FM = slider_->CalcAcrossMobilizerTransform(*mbt_context_);

  const Isometry3d X_FM_expected(
      Translation3d(axis_F_.normalized() * translation));

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no rotations at all.
  EXPECT_EQ(X_FM.linear(), Matrix3d::Identity());
  EXPECT_TRUE(CompareMatrices(X_FM.matrix(), X_FM_expected.matrix(),
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(PrismaticMobilizerTest, CalcAcrossMobilizerSpatialVeloctiy) {
  const double translation_rate = 1.5;
  const SpatialVelocity<double> V_FM =
      slider_->CalcAcrossMobilizerSpatialVelocity(
          *mbt_context_, Vector1d(translation_rate));

  const SpatialVelocity<double> V_FM_expected(
      Vector3d::Zero(), axis_F_.normalized() * translation_rate);

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no rotations at all.
  EXPECT_EQ(V_FM.rotational(), Vector3d::Zero());
  EXPECT_TRUE(V_FM.IsApprox(V_FM_expected, kTolerance));
}

TEST_F(PrismaticMobilizerTest, CalcAcrossMobilizerSpatialAcceleration) {
  const double translational_acceleration = 1.5;
  const SpatialAcceleration<double> A_FM =
      slider_->CalcAcrossMobilizerSpatialAcceleration(
          *mbt_context_, Vector1d(translational_acceleration));

  const SpatialAcceleration<double> A_FM_expected(
      Vector3d::Zero(), axis_F_.normalized() * translational_acceleration);

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no rotations at all.
  EXPECT_EQ(A_FM.rotational(), Vector3d::Zero());
  EXPECT_TRUE(A_FM.IsApprox(A_FM_expected, kTolerance));
}

TEST_F(PrismaticMobilizerTest, ProjectSpatialForce) {
  const Vector3d torque_Mo_F(1.0, 2.0, 3.0);
  const Vector3d force_Mo_F(1.0, 2.0, 3.0);
  const SpatialForce<double> F_Mo_F(torque_Mo_F, force_Mo_F);
  Vector1d tau;
  slider_->ProjectSpatialForce(*mbt_context_, F_Mo_F, tau);

  // Only the force along axis_F does work.
  const double tau_expected = force_Mo_F.dot(axis_F_.normalized());
  EXPECT_NEAR(tau(0), tau_expected, kTolerance);
}

TEST_F(PrismaticMobilizerTest, MapVelocityToQDotAndBack) {
  Vector1d v(1.5);
  Vector1d qdot;
  slider_->MapVelocityToQDot(*mbt_context_, v, &qdot);
  EXPECT_NEAR(qdot(0), v(0), kTolerance);

  qdot(0) = -std::sqrt(2);
  slider_->MapQDotToVelocity(*mbt_context_, qdot, &v);
  EXPECT_NEAR(v(0), qdot(0), kTolerance);
}

}  // namespace
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
