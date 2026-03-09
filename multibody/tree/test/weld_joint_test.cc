#include "drake/multibody/tree/weld_joint.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Translation3d;
using Eigen::Vector3d;
using math::RigidTransformd;
using systems::Context;

class WeldJointTest : public ::testing::Test {
 public:
  // Creates a simple model consisting of a single body with a weld joint
  // with the sole purpose of testing the WeldJoint user facing API.
  void SetUp() override {
    // Spatial inertia for adding body. The actual value is not important for
    // these tests and therefore we do not initialize it.
    const auto M_B = SpatialInertia<double>::NaN();

    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    // Add bodies so we can add forward and reverse weld joints.
    body_ = &model->AddRigidBody("body", M_B);
    rbody_ = &model->AddRigidBody("rbody", M_B);
    collide_body_ = &model->AddRigidBody("collide_body", M_B);

    joint_ = &model->AddJoint<WeldJoint>("Welder", model->world_body(), X_PJp_,
                                         *body_, X_CJc_, X_JpJc_);
    // This is reversed since we're using the outboard rbody as the parent.
    rjoint_ = &model->AddJoint<WeldJoint>("RWelder", *rbody_, X_PJp_, *body_,
                                          X_CJc_, X_JpJc_);

    // Add a frame with a name intentionally the same as the first one the next
    // joint will try to make, to test name collision resolution. Joint frame
    // names for Jp and Jc (when none are explicitly provided) use the formula
    // jointname_framename where framename is "parent" or "child". Weld optimal
    // "F" frames are offsets from the parent frame named with the formula
    // jointname_parentframename_F. So here the joint is named "collide",
    // its parent frame is "collide_parent" and the made-up F frame is
    // "collide_collide_parent_F".
    model->AddFrame<FixedOffsetFrame>("collide_collide_parent_F",
                                      *collide_body_,
                                      RigidTransformd(Vector3d(1.0, 2.0, 3.0)));
    collide_joint_ =
        &model->AddJoint<WeldJoint>("collide", model->world_body(), X_PJp_,
                                    *collide_body_, X_CJc_, X_JpJc_);

    // We are done adding modeling elements. Transfer tree to system for
    // computation. This finalizes the MultibodyTree.
    system_ = std::make_unique<internal::MultibodyTreeSystem<double>>(
        std::move(model), true /* is_discrete */);
  }

  const internal::MultibodyTree<double>& tree() const {
    return internal::GetInternalTree(*system_);
  }

 protected:
  std::unique_ptr<internal::MultibodyTreeSystem<double>> system_;

  const RigidBody<double>* body_{nullptr};
  const WeldJoint<double>* joint_{nullptr};
  const RigidBody<double>* rbody_{nullptr};
  const WeldJoint<double>* rjoint_{nullptr};
  const RigidBody<double>* collide_body_{nullptr};
  const WeldJoint<double>* collide_joint_{nullptr};
  const Translation3d X_JpJc_{0, 0.5, 0};
  const Translation3d X_PJp_{0.5, 0, 0};
  const Translation3d X_CJc_{0, 0, 0.5};
};

TEST_F(WeldJointTest, CanRotateOrTranslate) {
  EXPECT_FALSE(joint_->can_rotate());
  EXPECT_FALSE(joint_->can_translate());
}

TEST_F(WeldJointTest, Type) {
  const Joint<double>& base = *joint_;
  EXPECT_EQ(base.type_name(), WeldJoint<double>::kTypeName);
}

// Verify the expected number of dofs.
TEST_F(WeldJointTest, NumDOFs) {
  EXPECT_EQ(tree().num_positions(), 0);
  EXPECT_EQ(tree().num_velocities(), 0);
  EXPECT_EQ(joint_->num_positions(), 0);
  EXPECT_EQ(joint_->num_velocities(), 0);
  // We just verify we can call these methods. However their return value is
  // irrelevant since joints of type WeldJoint have no state.
  DRAKE_EXPECT_NO_THROW(joint_->position_start());
  DRAKE_EXPECT_NO_THROW(joint_->velocity_start());
}

// Verify we can retrieve the frame poses and that the implementing mobilizer
// uses the best inboard (F) and outboard (M) frames. Check both forward and
// reverse cases with non-identity X_JpJc.
TEST_F(WeldJointTest, CheckFramesForward) {
  EXPECT_TRUE(joint_->X_FM().IsExactlyEqualTo(X_JpJc_));
  const auto& Jp = joint_->frame_on_parent();
  const auto& Jc = joint_->frame_on_child();
  const math::RigidTransform<double>& X_PJp = Jp.GetFixedPoseInBodyFrame();
  const math::RigidTransform<double>& X_CJc = Jc.GetFixedPoseInBodyFrame();
  EXPECT_TRUE(X_PJp.IsExactlyEqualTo(X_PJp_));
  EXPECT_TRUE(X_CJc.IsExactlyEqualTo(X_CJc_));

  const auto& mobilizer = joint_->GetMobilizerInUse();
  const auto& F = mobilizer.inboard_frame();
  const auto& M = mobilizer.outboard_frame();

  // F must always be inboard so must be on World, with M on body.
  EXPECT_EQ(F.body().index(), BodyIndex(0));
  EXPECT_EQ(M.body().index(), body_->index());

  EXPECT_EQ(&M, &Jc);  // We don't move the M frame.
  EXPECT_NE(&F, &Jp);  // But should have moved the F frame.
  EXPECT_FALSE(M.is_ephemeral());
  EXPECT_TRUE(F.is_ephemeral());

  const math::RigidTransform<double>& X_PF = F.GetFixedPoseInBodyFrame();
  EXPECT_TRUE(X_PF.IsNearlyEqualTo(X_PJp * X_JpJc_, 1e-14));
}

TEST_F(WeldJointTest, CheckFramesReverse) {
  EXPECT_TRUE(rjoint_->X_FM().IsExactlyEqualTo(X_JpJc_));
  const auto& Jp = rjoint_->frame_on_parent();
  const auto& Jc = rjoint_->frame_on_child();
  const math::RigidTransform<double>& X_PJp = Jp.GetFixedPoseInBodyFrame();
  const math::RigidTransform<double>& X_CJc = Jc.GetFixedPoseInBodyFrame();
  EXPECT_TRUE(X_PJp.IsExactlyEqualTo(X_PJp_));
  EXPECT_TRUE(X_CJc.IsExactlyEqualTo(X_CJc_));

  const auto& mobilizer = rjoint_->GetMobilizerInUse();
  const auto& F = mobilizer.inboard_frame();
  const auto& M = mobilizer.outboard_frame();

  // F must always be inboard so must now be on body, with M on rbody.
  EXPECT_EQ(F.body().index(), body_->index());
  EXPECT_EQ(M.body().index(), rbody_->index());

  EXPECT_EQ(&M, &Jp);  // Switch bodies, but don't move the M frame.
  EXPECT_NE(&F, &Jc);  // F needs to be moved to be coincident with M.
  EXPECT_FALSE(M.is_ephemeral());
  EXPECT_TRUE(F.is_ephemeral());

  const math::RigidTransform<double>& X_CF = F.GetFixedPoseInBodyFrame();
  EXPECT_TRUE(X_CF.IsNearlyEqualTo(X_CJc * X_JpJc_.inverse(), 1e-14));
}

// Check that we were able to create the needed optimal F frame even though
// there was already a frame with the name we'd like to use for it. (See
// notes in WeldJointTest class above.)
TEST_F(WeldJointTest, EphemeralFrameNameCollision) {
  const auto& mobilizer = collide_joint_->GetMobilizerInUse();
  const auto& F = mobilizer.inboard_frame();
  const auto& M = mobilizer.outboard_frame();

  // We had to disambiguate with a leading "_".
  EXPECT_EQ(F.name(), "_collide_collide_parent_F");
  // This is just the original child frame.
  EXPECT_EQ(M.name(), "collide_child");
}

TEST_F(WeldJointTest, GetJointLimits) {
  EXPECT_EQ(joint_->position_lower_limits().size(), 0);
  EXPECT_EQ(joint_->position_upper_limits().size(), 0);
  EXPECT_EQ(joint_->velocity_lower_limits().size(), 0);
  EXPECT_EQ(joint_->velocity_upper_limits().size(), 0);
  EXPECT_EQ(joint_->acceleration_lower_limits().size(), 0);
  EXPECT_EQ(joint_->acceleration_upper_limits().size(), 0);
}

TEST_F(WeldJointTest, Damping) {
  EXPECT_EQ(joint_->default_damping_vector().size(), 0);
}

TEST_F(WeldJointTest, Clone) {
  auto model_clone = tree().CloneToScalar<AutoDiffXd>();
  const auto& joint_clone1 = dynamic_cast<const WeldJoint<AutoDiffXd>&>(
      model_clone->get_variant(*joint_));

  const std::unique_ptr<Joint<AutoDiffXd>> shallow =
      joint_clone1.ShallowClone();
  const auto& joint_clone2 =
      dynamic_cast<const WeldJoint<AutoDiffXd>&>(*shallow);

  for (const auto* clone : {&joint_clone1, &joint_clone2}) {
    EXPECT_EQ(clone->name(), joint_->name());
    EXPECT_EQ(clone->frame_on_parent().index(),
              joint_->frame_on_parent().index());
    EXPECT_EQ(clone->frame_on_child().index(),
              joint_->frame_on_child().index());
    EXPECT_TRUE(clone->X_FM().IsExactlyEqualTo(joint_->X_FM()));
  }
}

TEST_F(WeldJointTest, JointLocking) {
  // Joint locking on a weld joint does nothing; still, invoking it should not
  // be an error.
  auto context = system_->CreateDefaultContext();
  DRAKE_EXPECT_NO_THROW(joint_->Lock(context.get()));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
