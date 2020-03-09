#pragma once

#include "drake/multibody/tree/multibody_element.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

/// %FrameBase is an abstract representation of the concept of a _frame_ in
/// multibody dynamics. A frame F is a mathematical object consisting of
/// a set of three orthogonal unit vector axes Fx,Fy,Fz forming a right-handed
/// orthogonal basis located at a point Fo called the frame's origin. If
/// the frame origin Fo is a material point of a body, then F is a _material
/// frame_ (also called a _physical frame_) and can be used to apply forces and
/// torques to its body. A material frame can serve as an attachment point for
/// force-producing elements such as joints and constraints. Otherwise, we call
/// the frame a _free-floating_ or _computed_ frame and it is still suited for
/// observation, visualization, and measurement but cannot be used to apply
/// forces. Because material frames are by far the most common and useful frames
/// encountered in multibody systems, the derived class with the simple name
/// Frame is used to represent them.
///
/// Given numerical values in a Context for the associated multibody system,
/// _every_ frame has a location and orientation (collectively, _pose_) in space
/// that can be obtained through this base class. Most frames will also move
/// based on the multibody system's configuration, or on general runtime
/// computations, so may have meaningful spatial velocity and acceleration --
/// that will always be the case for material frames during a dynamic
/// simulation. Such kinematic quantities must always be measured with respect
/// to some other specified frame. The only frame we can be sure exists is the
/// World frame W, so pose and motion with respect to W are always available.
/// Utilities are provided for calculating frame motion with respect to other
/// frames. Derived frame objects will have additional properties. For example,
/// material frames have an associated Body.
///
/// Summarizing, %FrameBase serves as an abstraction for a general frame object;
/// it doesn't store any values itself. As always in Drake, runtime values are
/// obtained from a Context object. %FrameBase provides an interface through
/// which the pose of a frame may be obtained from a given Context.
/// Classes derived from %FrameBase are used to represent more specific types
/// of frames, most importantly whether a frame is associated with a material
/// point of a body.
///
/// @tparam_default_scalar
template <typename T>
class FrameBase : public MultibodyElement<FrameBase, T, FrameIndex> {
  // TODO(amcastro-tri): Provide a method with the signature:
  // const math::RigidTransform<T>& get_pose_in_world_frame(
  //     const Context<T>& context) const;
  // returning the pose X_WF of this frame F measured and expressed in the world
  // frame W. The pose will be stored in the cache and will be the product of
  // the position kinematics update.
  // TODO(amcastro-tri): Consider to provide a method with signature:
  // math::RigidTransform<T> CalcPoseAsMeasuredIn(
  //     const Context<T>& context,
  //     const FrameBase<T> measured_in_frame) const;
  // That computes the pose of `this` frame as measured in the
  // `measured_in_frame` frame.
 protected:
  explicit FrameBase(ModelInstanceIndex model_instance)
      : MultibodyElement<FrameBase, T, FrameIndex>(model_instance) {}
};

}  // namespace multibody
}  // namespace drake
