// clang-format: off
#include "drake/multibody/multibody_tree/multibody_tree.h"
// clang-format: on

#include <functional>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using std::make_unique;
using std::unique_ptr;

// Set of MultibodyTree tests for a double pendulum model.
// This double pendulum is similar to the acrobot model described in Section 3.1
// of the Underactuated Robotics notes available online at
// http://underactuated.csail.mit.edu/underactuated.html?chapter=3.
// The only difference is that this model has no actuation.
// This double pendulum is defined in the x-y plane with gravity acting in the
// negative y-axis direction.
// In this model the two links of the pendulum have the same length and their
// body frames are located at the links' centroids.
class PendulumTests : public ::testing::Test {
 public:
  // Creates an "empty" MultibodyTree that only contains the "world" body and
  // world body frame.
  void SetUp() override {
    owned_model_ = std::make_unique<MultibodyTree<double>>();
    model_ = owned_model_.get();

    // Retrieves the world body.
    world_body_ = &model_->get_world_body();
  }

  // Sets up the MultibodyTree model for a double pendulum. See this unit test's
  // class description for details.
  void CreatePendulumModel() {
    // Creates a NaN SpatialInertia to instantiate the two RigidBody links of
    // the pendulum. Using a NaN spatial inertia is ok so far since we are still
    // not performing any numerical computations. This is only to test API.
    // M_Bo_B is the spatial inertia about the body frame's origin Bo and
    // expressed in the body frame B.
    SpatialInertia<double> M_Bo_B;

    // Adds the upper and lower links of the pendulum.
    // Using: const BodyType& AddBody(std::unique_ptr<BodyType> body).
    upper_link_ =
        &model_->AddBody(make_unique<RigidBody<double>>(M_Bo_B));
    // Using: const BodyType<T>& AddBody(Args&&... args)
    lower_link_ = &model_->AddBody<RigidBody>(M_Bo_B);

    // The shoulder is the mobilizer that connects the world to the upper link.
    // Its inboard frame, Si, is the world frame. Its outboard frame, So, a
    // fixed offset frame on the upper link.
    shoulder_inboard_frame_ = &world_body_->get_body_frame();

    // The body frame of the upper link is U, and that of the lower link is L.
    // We will add a frame for the pendulum's shoulder. This will be the
    // shoulder's outboard frame So.
    // X_USo specifies the pose of the shoulder outboard frame So in the body
    // frame U of the upper link.
    // In this case the frame is created explicitly from the body frame of
    // upper_link.
    shoulder_outboard_frame_ =
        &model_->AddFrame<FixedOffsetFrame>(
            upper_link_->get_body_frame(), X_USo_);

    // The elbow is the mobilizer that connects upper and lower links.
    // Below we will create inboard and outboard frames associated with the
    // pendulum's elbow.
    // An inboard frame Ei is rigidly attached to the upper link. It is located
    // at y = -half_link_length in the frame of the upper link body.
    // An outboard frame Eo is rigidly attached to the lower link. It is located
    // at y = +half_link_length in the frame of the lower link body.
    // X_UEi specifies the pose of the elbow inboard frame Ei in the body
    // frame U of the upper link.
    // X_LEo specifies the pose of the elbow outboard frame Eo in the body
    // frame L of the lower link.
    // In this case we create a frame using the FixedOffsetFrame::Create()
    // method taking a Body, i.e., creating a frame with a fixed offset from the
    // upper link body frame.
    elbow_inboard_frame_ =
        &model_->AddFrame<FixedOffsetFrame>(*upper_link_, X_UEi_);
    elbow_outboard_frame_ =
        &model_->AddFrame<FixedOffsetFrame>(*lower_link_, X_LEo_);


    // Adds the shoulder and elbow mobilizers of the pendulum.
    // Using:
    //  const Mobilizer& AddMobilizer(std::unique_ptr<MobilizerType> mobilizer).
    shoulder_mobilizer_ =
        &model_->AddMobilizer(
            make_unique<Mobilizer<double>>(
                *shoulder_inboard_frame_, *shoulder_outboard_frame_));
    // Using: const MobilizerType<T>& AddMobilizer(Args&&... args)
    elbow_mobilizer_ = &model_->AddMobilizer<Mobilizer>(
        *elbow_inboard_frame_, *elbow_outboard_frame_);
  }

 protected:
  std::unique_ptr<MultibodyTree<double>> owned_model_;
  MultibodyTree<double>* model_;
  const Body<double>* world_body_;
  // Bodies:
  const RigidBody<double>* upper_link_;
  const RigidBody<double>* lower_link_;
  // Frames:
  const BodyFrame<double>* shoulder_inboard_frame_;
  const FixedOffsetFrame<double>* shoulder_outboard_frame_;
  const FixedOffsetFrame<double>* elbow_inboard_frame_;
  const FixedOffsetFrame<double>* elbow_outboard_frame_;
  // Mobilizers:
  const Mobilizer<double>* shoulder_mobilizer_;
  const Mobilizer<double>* elbow_mobilizer_;
  // Pendulum parameters:
  const double link_length = 1.0;
  const double half_link_length = link_length / 2;
  // Poses:
  // Desired pose of the lower link frame L in the world frame W.
  const Isometry3d X_WL_{Translation3d(0.0, -half_link_length, 0.0)};
  // Pose of the shoulder outboard frame So in the upper link frame U.
  const Isometry3d X_USo_{Translation3d(0.0, half_link_length, 0.0)};
  // Pose of the elbow inboard frame Ei in the upper link frame U.
  const Isometry3d X_UEi_{Translation3d(0.0, -half_link_length, 0.0)};
  // Pose of the elbow outboard frame Eo in the lower link frame L.
  const Isometry3d X_LEo_{Translation3d(0.0, half_link_length, 0.0)};
};

TEST_F(PendulumTests, CreateModelBasics) {
  // Initially there is only one body, the world.
  EXPECT_EQ(model_->get_num_bodies(), 1);
  // And there is only one frame, the world frame.
  EXPECT_EQ(model_->get_num_frames(), 1);

  CreatePendulumModel();

  // Verifies the number of multibody elements is correct.
  EXPECT_EQ(model_->get_num_bodies(), 3);
  EXPECT_EQ(model_->get_num_frames(), 6);
  EXPECT_EQ(model_->get_num_mobilizers(), 2);

  // Check that frames are associated with the correct bodies.
  EXPECT_EQ(
      shoulder_inboard_frame_->get_body().get_index(),
      world_body_->get_index());
  EXPECT_EQ(
      shoulder_outboard_frame_->get_body().get_index(),
      upper_link_->get_index());
  EXPECT_EQ(
      elbow_inboard_frame_->get_body().get_index(), upper_link_->get_index());
  EXPECT_EQ(
      elbow_outboard_frame_->get_body().get_index(), lower_link_->get_index());

  // Checks that mobilizers connect the right frames.
  EXPECT_EQ(shoulder_mobilizer_->get_inboard_frame().get_index(),
            world_body_->get_body_frame().get_index());
  EXPECT_EQ(shoulder_mobilizer_->get_outboard_frame().get_index(),
            shoulder_outboard_frame_->get_index());
  EXPECT_EQ(elbow_mobilizer_->get_inboard_frame().get_index(),
            elbow_inboard_frame_->get_index());
  EXPECT_EQ(elbow_mobilizer_->get_outboard_frame().get_index(),
            elbow_outboard_frame_->get_index());

  // Checks that mobilizers connect the right bodies.
  EXPECT_EQ(shoulder_mobilizer_->get_inboard_body().get_index(),
            world_body_->get_index());
  EXPECT_EQ(shoulder_mobilizer_->get_outboard_body().get_index(),
            upper_link_->get_index());
  EXPECT_EQ(elbow_mobilizer_->get_inboard_body().get_index(),
            upper_link_->get_index());
  EXPECT_EQ(elbow_mobilizer_->get_outboard_body().get_index(),
            lower_link_->get_index());

  // Checks we can retrieve the body associated with a frame.
  EXPECT_EQ(&shoulder_inboard_frame_->get_body(), world_body_);
  EXPECT_EQ(&shoulder_outboard_frame_->get_body(), upper_link_);
  EXPECT_EQ(&elbow_inboard_frame_->get_body(), upper_link_);
  EXPECT_EQ(&elbow_outboard_frame_->get_body(), lower_link_);

  // Checks we can request inboard/outboard bodies to a mobilizer.
  EXPECT_EQ(&shoulder_mobilizer_->get_inboard_body(), world_body_);
  EXPECT_EQ(&shoulder_mobilizer_->get_outboard_body(), upper_link_);
  EXPECT_EQ(&elbow_mobilizer_->get_inboard_body(), upper_link_);
  EXPECT_EQ(&elbow_mobilizer_->get_outboard_body(), lower_link_);
}

// Frame indexes are assigned by MultibodyTree. The number of frames
// equals the number of body frames (one per body) plus the number of
// additional frames added to the system (like FixedOffsetFrame objects).
// Frames are indexed in the order they are added to the MultibodyTree model.
// The order of the frames and their indexes is an implementation detail that
// users do not need to know about. Therefore this unit test would need to
// change in the future if we decide to change the "internal detail" on how we
// assign these indexes.
TEST_F(PendulumTests, Indexes) {
  CreatePendulumModel();
  EXPECT_EQ(shoulder_inboard_frame_->get_index(), FrameIndex(0));
  EXPECT_EQ(upper_link_->get_body_frame().get_index(), FrameIndex(1));
  EXPECT_EQ(lower_link_->get_body_frame().get_index(), FrameIndex(2));
  EXPECT_EQ(shoulder_outboard_frame_->get_index(), FrameIndex(3));
  EXPECT_EQ(elbow_inboard_frame_->get_index(), FrameIndex(4));
  EXPECT_EQ(elbow_outboard_frame_->get_index(), FrameIndex(5));
}

// Asserts that the Finalize() stage is successful and that re-finalization is
// not allowed.
TEST_F(PendulumTests, Finalize) {
  CreatePendulumModel();
  // Finalize() stage.
  EXPECT_FALSE(model_->topology_is_valid());  // Not valid before Finalize().
  EXPECT_NO_THROW(model_->Finalize());
  EXPECT_TRUE(model_->topology_is_valid());  // Valid after Finalize().

  // Asserts that no more multibody elements can be added after finalize.
  SpatialInertia<double> M_Bo_B;
  EXPECT_THROW(model_->AddBody<RigidBody>(M_Bo_B), std::logic_error);
  EXPECT_THROW(model_->AddFrame<FixedOffsetFrame>(*lower_link_, X_LEo_),
               std::logic_error);
  EXPECT_THROW(model_->AddMobilizer<Mobilizer>(
      *shoulder_inboard_frame_, *shoulder_outboard_frame_), std::logic_error);

  // Asserts re-finalization is not allowed.
  EXPECT_THROW(model_->Finalize(), std::logic_error);
}

// This is an experiment with std::reference_wrapper to show that we can save
// bodies in an array of references.
TEST_F(PendulumTests, StdReferenceWrapperExperiment) {
  // Initially there is only one body, the world.
  EXPECT_EQ(model_->get_num_bodies(), 1);
  // And there is only one frame, the world frame.
  EXPECT_EQ(model_->get_num_frames(), 1);
  CreatePendulumModel();

  // Vector of references.
  std::vector<std::reference_wrapper<const Body<double>>> bodies;
  bodies.push_back(*world_body_);
  bodies.push_back(*upper_link_);
  bodies.push_back(*lower_link_);

  // Verify that vector "bodies" effectively holds valid references to the
  // actual body elements in the tree.
  // In addition, since these tests compare actual memory addresses, they
  // ensure that bodies were not copied instead.
  // Unfortunately we need the ugly get() method since operator.() is not
  // overloaded.
  EXPECT_EQ(&bodies[world_body_->get_index()].get(), world_body_);
  EXPECT_EQ(&bodies[upper_link_->get_index()].get(), upper_link_);
  EXPECT_EQ(&bodies[lower_link_->get_index()].get(), lower_link_);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
