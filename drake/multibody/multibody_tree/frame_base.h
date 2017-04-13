#pragma once

#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

/// %FrameBase is an abstract representation of the concept of a frame in
/// multibody dynamics. Frames are a mathematical object that can be thought of
/// as a set of three orthogonal axes forming a right-handed orthogonal basis
/// located at a point called the frame's origin.
/// It is important, when talking about frames, to make a distinction between
/// the frame itself, and the description of the frame's state. A frame is a
/// mathematical object that can move through space, just like a body. At any
/// given moment the _state_ of a frame can be described by its _pose_ as
/// measured in another frame. These two concepts, the _state_ of the frame and
/// the _pose_ of the frame, are somewhat orthogonal, and yet this abstraction
/// needs to account for both.
/// The pose of a frame depends on the state of the system to which it belongs
/// (where here the term "system" should not be confused with systems::System).
/// For example, if a frame is rigidly affixed to a body, then the frame moves
/// to follow the body's movement. But knowing the system's state is only part
/// of the pose definition. The pose of a frame cannot be expressed in absolute
/// terms, it can only be described relative to another frame. In the previous
/// example, if the pose of the frame is described relative to some fixed
/// "world" frame, the frame's state will appear to change as it moves through
/// space. However, if the pose of the frame is described relative to the body
/// to which it is attached, the values of the pose will never change. And yet,
/// the frame is still moving.
/// Summarizing, this class serves as an abstraction for computing the pose of a
/// frame. It doesn't store any values itself. It provides an interface through
/// which a frame can report its position relative to another frame, given the
/// current state of the system to which it belongs.
///
/// Sub-classes of %FrameBase provide semantic details of how the pose of a
/// frame, as measured in another frame, relates to the state of the system to
/// which this frame (and the measured-in frame) belongs.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class FrameBase : public MultibodyTreeElement<FrameBase<T>, FrameIndex> {
  // TODO(amcastro-tri): Provide a method with the signature:
  // const Isometry3<T>& get_pose_in_world_frame(
  //     const Context<T>& context) const;
  // returning the pose X_WF of this frame F measured and expressed in the world
  // frame W. The pose will be stored in the cache and will be the product of
  // the position kinematics update.
};

}  // namespace multibody
}  // namespace drake
