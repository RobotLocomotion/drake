#pragma once

#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

/// %FrameBase is an abstract base class representation of the concept of a
/// frame in multibody dynamics. Frames can be thought of as a set of three
/// orthogonal axes forming a right-handed orthogonal basis located at a point
/// called the frame's origin.
/// It is important at this point to make the distinction between a _frame_,
/// and its _pose_ as measured and expressed in another frame. Even though
/// related, _frame_ and _pose_ are two different concepts. Given two frames E
/// and F, the pose `X_EF` of frame F measured and expressed in frame E defines
/// the geometrical relationship between these two frames.
/// This class does not store the pose `X_EF` of a frame but it only provides an
/// abstraction to represent the frame itself. Notice that storing any pose
/// `X_EF` within this class does not make sense since it would be imposible to
/// specify the pose of F without knowledge of a specific frame E in which its
/// pose is measured.
/// Frames are defined within a given physical or geometrical system (where here
/// the term "system" should not be confused with systems::System). For
/// instance, MultibodyTree allows to define frames which can then be attached
/// to bodies. Within MultibodyTree the pose of the frame depends on the state
/// of the multibody tree as a whole, which is given by its systems::Context.
/// In this regard, a frame sub-class will map a state or Context to a pose in
/// SE(3). This pose will only be well defined when it is measured with respect
/// to another frame.
///
/// Sub-classes of %FrameBase are responsible for communicating useful semantics
/// about that frame's dependencies and providing implementations for defining
/// the pose based on those semantics. For instace, the pose of a frame could
/// be determined with the knowledge that it is attached to a body with a fixed
/// pose in the body frame. Another example is a frame that is attached to
/// a particular material point on a soft body. In that case the pose of the
/// frame will generally depend on the state of deformation of the body.
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
