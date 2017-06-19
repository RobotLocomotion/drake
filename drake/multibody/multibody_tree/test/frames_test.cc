#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>
#include <set>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using std::unique_ptr;
using systems::Context;

// This unit test fixture sets up a simple MultibodyTree model containing a
// number frames to verify the correctness of the frame methods.
// The model consists of a body B (with its corresponding BodyFrame) connected
// to the world via a revolute mobilizer, a frame P at pose X_BP on B, and a
// frame Q at pose X_PQ attached to P.
class FrameTests : public ::testing::Test {
 public:
  void SetUp() override {
    // Creates a NaN SpatialInertia to instantiate a body for the test.
    // Using a NaN spatial inertia is ok in this unit test since all
    // computations only relate to frame kinematics (and therefore mass
    // properties do not play any role.)
    SpatialInertia<double> M_Bo_B;

    model_ = std::make_unique<MultibodyTree<double>>();

    bodyB_ = &model_->AddBody<RigidBody>(M_Bo_B);
    frameB_ = &bodyB_->get_body_frame();

    // Mobilizer connecting bodyB to the world.
    // The mobilizer is only needed because it is a requirement of MultibodyTree
    // that all bodies in the model must have an inboard mobilizer.
    model_->AddMobilizer<RevoluteMobilizer>(
        model_->get_world_frame(), bodyB_->get_body_frame(),
        Vector3d::UnitZ() /*revolute axis*/);

    // Some arbitrary pose of frame P in the body frame B.
    X_BP_ = AngleAxisd(M_PI / 6.0, Vector3d::UnitZ()) *
            AngleAxisd(M_PI / 5.0, Vector3d::UnitX()) *
            Translation3d(0.0, -1.0, 0.0);
    // Frame P is rigidly attached to B with pose X_BP.
    frameP_ =
        &model_->AddFrame<FixedOffsetFrame>(bodyB_->get_body_frame(), X_BP_);

    // Some arbitrary pose of frame Q in frame P.
    X_PQ_ = AngleAxisd(-M_PI / 3.0, Vector3d::UnitZ()) *
            AngleAxisd(M_PI / 7.0, Vector3d::UnitX()) *
            Translation3d(0.5, 1.0, -2.0);
    // Frame Q is rigidly attached to P with pose X_PQ.
    frameQ_ =
        &model_->AddFrame<FixedOffsetFrame>(*frameP_, X_PQ_);

    model_->Finalize();
    context_ = model_->CreateDefaultContext();

    // An arbitrary pose of an arbitrary frame G in an arbitrary frame F.
    X_FG_ = AngleAxisd(M_PI / 6.0, Vector3d::UnitY()) *
            AngleAxisd(M_PI / 8.0, Vector3d::UnitZ()) *
            AngleAxisd(M_PI / 5.0, Vector3d::UnitX()) *
            Translation3d(2.0, -1.0, 0.5);

    // An arbitrary pose of an arbitrary frame F in an arbitrary frame Q.
    X_QF_ = X_FG_;

    // An arbitrary pose of an arbitrary frame G in an arbitrary frame Q.
    X_QG_ = X_FG_;
  }

 protected:
  std::unique_ptr<MultibodyTree<double>> model_;
  std::unique_ptr<Context<double>> context_;
  // Bodies:
  const RigidBody<double>* bodyB_;
  // Frames:
  const Frame<double>* frameB_{};
  const Frame<double>* frameP_{};
  const Frame<double>* frameQ_{};
  // Poses:
  Isometry3d X_BP_;
  Isometry3d X_PQ_;
  Isometry3d X_FG_;
  Isometry3d X_QF_;
  Isometry3d X_QG_;
};

// Verifies the BodyFrame methods to compute poses in different frames.
TEST_F(FrameTests, BodyFrameCalcPoseMethods) {
  // Verify this method computes the pose X_FB of the body this frame
  // attaches to measured in this frame F. Since in this case frame F IS the
  // body frame B, X_FB = Id and this method should return the identity
  // transformation.
  EXPECT_TRUE(frameB_->CalcBodyPoseInThisFrame(*context_).
      isApprox(Isometry3d::Identity()));

  // Verify this method computes the pose of a frame G measured in this
  // frame F given the pose of frame G in this frame F as: X_BG = X_BF * X_FG.
  // Since in this case frame F IS the body frame B, X_BF = Id and this method
  // simply returns X_FG.
  EXPECT_TRUE(frameB_->CalcOffsetPoseInBody(*context_, X_FG_).isApprox(X_FG_));

  // Verify this method computes the pose X_QB of the body B this frame
  // F attaches to given the pose X_QF of this frame F measured in a third
  // frame Q. Therefore X_QB = X_QF * X_FB and since in this case frame F IS the
  // body frame B, X_BF = Id and this method simply returns X_QF.
  EXPECT_TRUE(frameB_->CalcBodyPoseInOtherFrame(*context_, X_QF_).
      isApprox(X_QF_));
}

// Verifies the FixedOffsetFrame methods to compute poses in different frames.
// In these tests frame P is attached to body frame B with a fixed posed X_BP.
// Schematically:
//
//         X_BP
//     B -------> P
TEST_F(FrameTests, FixedOffsetFrameCalcPoseMethods) {
  // Verify this method computes the pose of body B in frame P as:
  // X_PB = X_BP.inverse()
  EXPECT_TRUE(
      frameP_->CalcBodyPoseInThisFrame(*context_).isApprox(X_BP_.inverse()));

  // Verify this method computes the pose X_BQ of a third frame Q measured in
  // the body frame B given we know the pose X_PQ of frame G in our frame P as:
  // X_BQ = X_BP * X_PQ
  EXPECT_TRUE(frameP_->CalcOffsetPoseInBody(*context_, X_PQ_).
      isApprox(X_BP_ * X_PQ_));

  // Verifies this method computes the pose X_QB of the body B frame measured in
  // a third frame Q given we know the pose X_QP of frame P in this third frame
  // Q as:
  // X_QB = X_QP * X_PB = X_PQ.inverse() * X_BP.inverse()
  EXPECT_TRUE(frameP_->CalcBodyPoseInOtherFrame(*context_, X_PQ_.inverse()).
      isApprox(X_PQ_.inverse() * X_BP_.inverse()));
}

// Verifies FixedOffsetFrame methods to compute poses in different frames when
// several FixedOffsetFrame objects are chained in sequence.
// In these tests frame P is attached to body B with fixed offset X_BP while
// frame Q attaches to frame P with fixed offset X_PQ. Schematically:
//
//         X_BP       X_PQ
//     B -------> P -------> Q
TEST_F(FrameTests, ChainedFixedOffsetFrames) {
  // Verify this method computes the pose of body B in frame Q as:
  // X_QB = X_QP * X_PB = X_PQ.inverse() * X_BP.inverse()
  EXPECT_TRUE(frameQ_->CalcBodyPoseInThisFrame(*context_).
      isApprox(X_PQ_.inverse() * X_BP_.inverse()));

  // Verify this method computes the pose X_BG of a fourth frame G measured in
  // the body frame B given we know the pose X_QG of frame G in our frame Q as:
  // X_BG = X_BP * X_PQ * X_QG
  EXPECT_TRUE(frameQ_->CalcOffsetPoseInBody(*context_, X_QG_).
      isApprox(X_BP_ * X_PQ_ * X_QG_));

  // Verifies this method computes the pose X_FB of the body B attached to frame
  // Q given we know the pose X_FQ of our frame Q in frame F as:
  // X_FB = X_FQ * X_QP * X_PB =
  //        X_QF.inverse() * X_PQ.inverse() * X_BP.inverse()
  EXPECT_TRUE(frameQ_->CalcBodyPoseInOtherFrame(*context_, X_QF_.inverse()).
      isApprox(X_QF_.inverse() * X_PQ_.inverse() * X_BP_.inverse()));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
