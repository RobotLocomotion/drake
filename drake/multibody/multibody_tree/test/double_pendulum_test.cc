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

  // Creates a NaN SpatialInertia to instantiate the two RigidBody links of
  // the pendulum. Using a NaN spatial inertia is ok so far since we are still
  // not performing any numerical computations. This is only to test API.
  // M_Bo_B is the spatial inertia about the body frame's origin Bo and
  // expressed in the body frame B.
  SpatialInertia<double> M_Bo_B;

  // Adds the upper and lower links of the pendulum.
  // Using: const BodyType& AddBody(std::unique_ptr<BodyType> body).
  const RigidBody<double>& upper_link =
      model->AddBody(make_unique<RigidBody<double>>(M_Bo_B));
  // Using: const BodyType<T>& AddBody(Args&&... args)
  const RigidBody<double>& lower_link = model->AddBody<RigidBody>(M_Bo_B);

  // This is an experiment with std::reference_wrapper to show that we can save
  // bodies in an array of references.
  std::vector<std::reference_wrapper<const Body<double>>> bodies;
  bodies.push_back(world_body);
  bodies.push_back(upper_link);
  bodies.push_back(lower_link);

  // Verify that vector "bodies" effectively holds valid references to the
  // actual body elements in the tree.
  // In addition, since these tests compare actual memory addresses, they
  // ensure that bodies were not copied instead.
  // Unfortunately we need the ugly get() method since operator.() is not
  // overloaded.
  EXPECT_EQ(&bodies[world_body.get_index()].get(), &world_body);
  EXPECT_EQ(&bodies[upper_link.get_index()].get(), &upper_link);
  EXPECT_EQ(&bodies[lower_link.get_index()].get(), &lower_link);

  // Verifies the number of multibody elements is correct.
  EXPECT_EQ(model->get_num_bodies(), 3);
  EXPECT_EQ(model->get_num_frames(), 3);

  // The shoulder is the mobilizer that connects the world to the upper link.
  // Its inboard frame, Si, is the world frame. Its outboard frame, So, a fixed
  // offset frame on the upper link.
  const BodyFrame<double>& shoulder_inboard_frame = world_body.get_body_frame();

  // The body frame of the upper link is U, and that of the lower link is L.
  // We will add a frame for the pendulum's shoulder. This will be the
  // shoulder's outboard frame So.
  // X_USo specifies the pose of the shoulder outboard frame So in the body
  // frame U of the upper link.
  Isometry3d X_USo(Translation3d(0.0, half_link_length, 0.0));
  // In this case the frame is created explicitly from the body frame of
  // upper_link.
  const auto& shoulder_outboard_frame =
      model->AddFrame<FixedOffsetFrame>(upper_link.get_body_frame(), X_USo);

  // The elbow is the mobilizer that connects upper and lower links.
  // Below we will create inboard and outboard frames associated with the
  // pendulum's elbow.
  // An inboard frame Ei is rigidly attached to the upper link. It is located at
  // y = -half_link_length in the frame of the upper link body.
  // An outboard frame Eo is rigidly attached to the lower link. It is located
  // at y = +half_link_length in the frame of the lower link body.
  // X_UEi specifies the pose of the elbow inboard frame Ei in the body
  // frame U of the upper link.
  Isometry3d X_UEi(Translation3d(0.0, -half_link_length, 0.0));
  // X_LEo specifies the pose of the elbow outboard frame Eo in the body
  // frame L of the lower link.
  // In this case we create a frame using the FixedOffsetFrame::Create() method
  // taking a Body, i.e., creating a frame with a fixed offset from the upper
  // link body frame.
  const auto& elbow_inboard_frame =
      model->AddFrame<FixedOffsetFrame>(upper_link, X_UEi);
  Isometry3d X_LEo(Translation3d(0.0, +half_link_length, 0.0));
  const auto& elbow_outboard_frame =
      model->AddFrame<FixedOffsetFrame>(lower_link, X_LEo);

  // Verify the new number of frames.
  EXPECT_EQ(model->get_num_frames(), 6);

  // Finalize() stage.
  EXPECT_FALSE(model->topology_is_valid());  // Not valid before Finalize().
  EXPECT_NO_THROW(model->Finalize());
  EXPECT_TRUE(model->topology_is_valid());  // Valid after Finalize().

  // Asserts that no more bodies can be added after finalize.
  EXPECT_THROW(model->AddBody<RigidBody>(M_Bo_B), std::logic_error);
  EXPECT_THROW(model->AddFrame<FixedOffsetFrame>(lower_link, X_LEo),
               std::logic_error);

  // Asserts re-finalization is not allowed.
  EXPECT_THROW(model->Finalize(), std::logic_error);

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
