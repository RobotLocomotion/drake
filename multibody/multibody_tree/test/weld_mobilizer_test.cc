#include "drake/multibody/multibody_tree/weld_mobilizer.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/multibody_tree_system.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace {

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

    // Create an empty model.
    auto model = std::make_unique<MultibodyTree<double>>();

    // Add a body so we can add a mobilizer to it.
    auto& body = model->AddBody<RigidBody>(M_B);

    X_WB_ = math::RigidTransformd(Vector3d(1.0, 2.0, 3.0));

    // Add a weld mobilizer between the world and the body:
    weld_body_to_world_ = &model->AddMobilizer<WeldMobilizer>(
        model->world_body().body_frame(), body.body_frame(),
        X_WB_.GetAsIsometry3());

    // We are done adding modeling elements. Transfer tree to system and get
    // a Context.
    system_ = std::make_unique<MultibodyTreeSystem<double>>(std::move(model));
    context_ = system_->CreateDefaultContext();

    // Performance critical queries take a MultibodyTreeContext to avoid dynamic
    // casting.
    mbt_context_ = dynamic_cast<MultibodyTreeContext<double>*>(context_.get());
    ASSERT_NE(mbt_context_, nullptr);
  }

  const MultibodyTree<double>& tree() const { return system_->tree(); }

 protected:
  std::unique_ptr<MultibodyTreeSystem<double>> system_;
  std::unique_ptr<Context<double>> context_;
  MultibodyTreeContext<double>* mbt_context_{nullptr};

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
      weld_body_to_world_->CalcAcrossMobilizerTransform(*mbt_context_));
  EXPECT_TRUE(CompareMatrices(X_FM.GetAsMatrix34(),
                              X_WB_.GetAsMatrix34(),
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
