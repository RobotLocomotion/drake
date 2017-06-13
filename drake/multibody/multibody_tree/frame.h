#pragma once

#include "drake/multibody/multibody_tree/frame_base.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class Body;

/// %Frame is an abstract class representing a _material frame_ (also called a
/// _physical frame_), meaning that it is associated with a material point of a
/// Body. A material frame can be used to apply forces and torques to a
/// multibody system, and can be used as an attachment point for force-producing
/// elements like joints, actuators, and constraints. Despite its name, %Frame
/// is not the most general frame representation in Drake; see FrameBase for a
/// more-general discussion.
///
/// The pose and motion of a %Frame object is always calculated relative to the
/// BodyFrame of the body with which it is associated, and every %Frame object
/// can report which Body object that is. Concrete classes derived from %Frame
/// differ only in how those kinematic properties are calculated. For soft
/// bodies that calculation may depend on the body's deformation state
/// variables. A %Frame on a rigid body will usually have a fixed offset from
/// its BodyFrame, but that is not required -- a frame that moves with respect
/// to its BodyFrame can still be a material frame on that rigid body.
///
/// As always in Drake, runtime numerical quantities are stored in a Context.
/// A %Frame object does not store runtime values, but provides methods for
/// extracting frame-associated values (such as the %Frame object's kinematics)
/// from a given Context.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class Frame : public FrameBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Frame)

  /// Returns a const reference to the body associated to this %Frame.
  const Body<T>& get_body() const {
    return body_;
  }

  /// Computes the pose `X_FB` of the body B associated with `this` frame F,
  /// measured in `this` frame F.
  /// In particular, if `this` **is**` the body frame B, i.e. `X_BF` is the
  /// identity transformation, this method directly returns the identity
  /// transformation.
  /// @sa CalcBodyPoseInOtherFrame()
  virtual Isometry3<T> CalcBodyPoseInThisFrame(
      const systems::Context<T>& context) const = 0;

  /// Given the offset pose `X_FQ` of a frame Q measured in this frame F,
  /// this method computes the pose `X_BQ` of frame Q measured and expressed in
  /// the frame B of the body to which this frame is attached.
  /// In other words, if the pose of `this` frame F in the body frame B is
  /// `X_BF`, this method computes the pose `X_BQ` of frame Q in the body frame
  /// B as `X_BQ = X_BF * X_FQ`.
  /// In particular, if `this` **is**` the body frame B, i.e. `X_BF` is the
  /// identity transformation, this method directly returns `X_FQ`.
  /// Specific frame subclasses can override this method to provide faster
  /// implementations if needed.
  virtual Isometry3<T> CalcOffsetPoseInBody(
      const systems::Context<T>& context,
      const Isometry3<T>& X_FQ) const {
    return CalcBodyPoseInThisFrame(context).inverse() * X_FQ;
  }

  /// Computes the pose `X_QB` of the body B associated with this frame F
  /// measured in a frame Q, given the pose `X_QF` of this frame F measured
  /// in Q.
  /// In other words, if `X_FB` is the pose of body frame B in `this` frame F,
  /// this method computes the pose `X_QB` of body frame B in frame Q as
  /// `X_QB = X_QF * X_FB`.
  /// In particular, if `this` **is**` the body frame B, i.e. `X_FB` is the
  /// identity transformation, this method directly returns `X_QF`.
  /// @sa CalcBodyPoseInThisFrame() to compute the pose of the body associated
  /// with this frame as measured in this frame.
  /// Specific frame subclasses can override this method to provide faster
  /// implementations if needed.
  virtual Isometry3<T> CalcBodyPoseInOtherFrame(
      const systems::Context<T>& context,
      const Isometry3<T>& X_QF) const {
    return X_QF * CalcBodyPoseInThisFrame(context);
  }

 protected:
  // Only derived classes can use this constructor.
  explicit Frame(const Body<T>& body) : body_(body) {}

 private:
  // Implementation for MultibodyTreeElement::DoSetTopology().
  void DoSetTopology(const MultibodyTreeTopology& tree_topology) final {
    topology_ = tree_topology.get_frame(this->get_index());
    DRAKE_ASSERT(topology_.index == this->get_index());
  }

  // The body associated with this frame.
  const Body<T>& body_;

  // The internal bookkeeping topology struct used by MultibodyTree.
  FrameTopology topology_;
};

}  // namespace multibody
}  // namespace drake
