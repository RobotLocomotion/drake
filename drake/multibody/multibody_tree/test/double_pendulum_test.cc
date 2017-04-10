#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using std::make_unique;
using std::unique_ptr;

// Tests the logic to create a multibody tree model for a double pendulum.
// This double pendulum is similar to the acrobot model described in Section 3.1
// of the Underactuated Robotics notes available online at
// http://underactuated.csail.mit.edu/underactuated.html?chapter=3.
// The only difference is that this model has no actuation.
// This double pendulum is defined in the x-y plane with gravity acting in the
// negative y-axis direction.
// In this model the two links of the pendulum have the same length and their
// body frames are located at the links' centroids.
GTEST_TEST(MultibodyTree, CreateModel) {
  const double link_length = 1.0;
  const double half_link_length = link_length / 2;

  auto owned_model = std::make_unique<MultibodyTree<double>>();
  MultibodyTree<double>* model = owned_model.get();

  // Initially there is only one body, the world.
  EXPECT_EQ(model->get_num_bodies(), 1);
  // And there is only one frame, the world frame.
  EXPECT_EQ(model->get_num_frames(), 1);

  // Retrieves the world body.
  const Body<double>& world_body = model->get_world_body();

  // Adds the upper and lower links of the pendulum.
  // Using: const BodyType& AddBody(std::unique_ptr<BodyType> body).
  const RigidBody<double>& upper_link =
      model->AddBody(make_unique<RigidBody<double>>());
  // Using: const BodyType<T>& AddBody(Args&&... args)
  const RigidBody<double>& lower_link = model->AddBody<RigidBody>();

  // Verifies the number of multibody elements is correct.
  EXPECT_EQ(model->get_num_bodies(), 3);
  EXPECT_EQ(model->get_num_frames(), 3);

  // Shoulder's inboard frame Si in this model IS the world frame. We will place
  // a mobilizer between the shoulder inboard and outboard frames.
  const BodyFrame<double>& shoulder_inboard_frame = world_body.get_body_frame();

  // Add a frame for the pendulum's shoulder. This will be the shoulder's
  // outboard frame So.
  // X_UlSo specifies the pose of the shoulder outboard frame So in the body
  // frame Ul of the upper link.
  Isometry3d X_UlSo(Translation3d(0.0, half_link_length, 0.0));
  // In this case the frame is created explicitly from the body frame of
  // upper_link. Another option is to use the FixedOffsetFrame::Create() method
  // directly taking a Body, as shown below, which creates a frame with a fixed
  // offset from the body frame moving with the body.
  const auto& shoulder_outboard_frame =
      model->AddFrame<FixedOffsetFrame>(upper_link.get_body_frame(), X_UlSo);

  // Create frames associated with the pendulum's elbow.
  // An inboard frame Ei is rigidly attached to the upper link. It is located at
  // y = -half_link_length in the frame of the upper link body.
  // An outboard frame Eo is rigidly attached to the lower link. It is located
  // at y = +half_link_length in the frame of the lower link body.
  // X_UlEi specifies the pose of the elbow inboard frame Ei in the body
  // frame Ul of the upper link.
  Isometry3d X_UlEi(Translation3d(0.0, -half_link_length, 0.0));
  // X_LlEo specifies the pose of the elbow outboard frame Eo in the body
  // frame Ll of the lower link.
  // In this case we create a frame using the FixedOffsetFrame::Create() method
  // taking a Body, i.e. creating a frame with a fixed offset from the upper
  // link body frame.
  const auto& elbow_inboard_frame =
      model->AddFrame<FixedOffsetFrame>(upper_link, X_UlEi);
  Isometry3d X_LlEo(Translation3d(0.0, +half_link_length, 0.0));
  const auto& elbow_outboard_frame =
      model->AddFrame<FixedOffsetFrame>(lower_link, X_LlEo);

  // Verify the new number of frames.
  EXPECT_EQ(model->get_num_frames(), 6);

  // Compile() stage.
  EXPECT_FALSE(model->topology_is_valid());  // Not valid before Compile().
  EXPECT_NO_THROW(model->Compile());
  EXPECT_TRUE(model->topology_is_valid());  // Valid after Compile().

  // Asserts that no more bodies can be added after compilation.
  EXPECT_THROW(model->AddBody<RigidBody>(), std::logic_error);
  EXPECT_THROW(model->AddFrame<FixedOffsetFrame>(lower_link, X_LlEo),
               std::logic_error);

  // Asserts re-compilation is not allowed.
  EXPECT_THROW(model->Compile(), std::logic_error);

  // Frame indexes are assigned by MultibodyTree. The number of physical frames
  // equals the number of body frames (one per body) plus the number of
  // additional frames added to the system (like FixedOffsetFrame objects).
  // The first MultibodyTree::get_num_bodies() frame indexes (starting at zero)
  // correspond to the body frames. All other physical frames have indexes from
  // MultibodyTree::get_num_bodies() to
  // MultibodyTree::get_num_frames() - 1.
  // The order of the frames and their indexes is an implementation detail that
  // users do not need to know about. Therefore this unit tests would need to
  // change in the future if we decide to change th "internal detail" on how we
  // assign these indexes.
  EXPECT_EQ(shoulder_inboard_frame.get_index(), FrameIndex(0));
  EXPECT_EQ(upper_link.get_body_frame().get_index(), FrameIndex(1));
  EXPECT_EQ(lower_link.get_body_frame().get_index(), FrameIndex(2));
  EXPECT_EQ(shoulder_outboard_frame.get_index(), FrameIndex(3));
  EXPECT_EQ(elbow_inboard_frame.get_index(), FrameIndex(4));
  EXPECT_EQ(elbow_outboard_frame.get_index(), FrameIndex(5));

  // Check that frames are associated with the correct bodies.
  EXPECT_EQ(
      shoulder_inboard_frame.get_body().get_index(), world_body.get_index());
  EXPECT_EQ(
      shoulder_outboard_frame.get_body().get_index(), upper_link.get_index());
  EXPECT_EQ(
      elbow_inboard_frame.get_body().get_index(), upper_link.get_index());
  EXPECT_EQ(
      elbow_outboard_frame.get_body().get_index(), lower_link.get_index());

  // Checks we can retrieve the body associated with a frame.
  EXPECT_EQ(&shoulder_inboard_frame.get_body(), &world_body);
  EXPECT_EQ(&shoulder_outboard_frame.get_body(), &upper_link);
  EXPECT_EQ(&elbow_inboard_frame.get_body(), &upper_link);
  EXPECT_EQ(&elbow_outboard_frame.get_body(), &lower_link);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
