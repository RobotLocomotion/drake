#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/common/autodiff.h"
#include "drake/common/nice_type_name.h"
#include "drake/multibody/math/spatial_velocity.h"
#include "drake/multibody/tree/frame_base.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

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
/// @tparam_default_scalar
template <typename T>
class Frame : public FrameBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Frame)

  /// Returns a const reference to the body associated to this %Frame.
  const Body<T>& body() const {
    return body_;
  }

  /// Returns the name of this frame. It may be empty if unnamed.
  const std::string& name() const {
    return name_;
  }

  /// Returns the pose `X_BF` of `this` frame F in the body frame B associated
  /// with this frame.
  /// In particular, if `this` **is** the body frame B, this method directly
  /// returns the identity transformation.
  virtual math::RigidTransform<T> CalcPoseInBodyFrame(
      const systems::Context<T>& context) const = 0;

  /// Returns the rotation matrix `R_BF` that relates body frame B to `this`
  /// frame F (B is the body frame to which `this` frame F is attached).
  /// @note If `this` is B, this method returns the identity RotationMatrix.
  virtual math::RotationMatrix<T> CalcRotationMatrixInBodyFrame(
      const systems::Context<T>& context) const = 0;

  /// Variant of CalcPoseInBodyFrame() that returns the fixed pose `X_BF` of
  /// `this` frame F in the body frame B associated with this frame.
  /// @throws std::logic_error if called on a %Frame that does not have a
  /// fixed offset in the body frame.
  // %Frame sub-classes that can represent the fixed pose of `this` frame F in
  // a body frame B, must override this method.
  // An example of a frame sub-class not implementing this method would be that
  // of a frame on a soft body, for which its pose in the body frame depends
  // on the state of deformation of the body.
  virtual math::RigidTransform<T> GetFixedPoseInBodyFrame() const {
    throw std::logic_error(
        "Attempting to retrieve a fixed pose from a frame of type '" +
            drake::NiceTypeName::Get(*this) +
            "', which does not support this operation.");
  }

  /// Returns the rotation matrix `R_BF` that relates body frame B to `this`
  /// frame F (B is the body frame to which `this` frame F is attached).
  /// @throws std::logic_error if `this` frame F is a %Frame that does not have
  /// a fixed offset in the body frame B (i.e., `R_BF` is not constant).
  /// %Frame sub-classes that have a constant `R_BF` must override this method.
  /// An example of a frame sub-class not implementing this method would be that
  /// of a frame on a soft body, for which its pose in the body frame depends
  /// on the state of deformation of the body.
  virtual math::RotationMatrix<T> GetFixedRotationMatrixInBodyFrame() const {
    throw std::logic_error(
        "Unable to retrieve a fixed rotation matrix from a frame of type '" +
            drake::NiceTypeName::Get(*this) +
            "', which does not support this method.");
  }

  /// Given the offset pose `X_FQ` of a frame Q in `this` frame F, this method
  /// computes the pose `X_BQ` of frame Q in the body frame B to which this
  /// frame is attached.
  /// In other words, if the pose of `this` frame F in the body frame B is
  /// `X_BF`, this method computes the pose `X_BQ` of frame Q in the body frame
  /// B as `X_BQ = X_BF * X_FQ`.
  /// In particular, if `this` **is**` the body frame B, i.e. `X_BF` is the
  /// identity transformation, this method directly returns `X_FQ`.
  /// Specific frame subclasses can override this method to provide faster
  /// implementations if needed.
  virtual math::RigidTransform<T> CalcOffsetPoseInBody(
      const systems::Context<T>& context,
      const math::RigidTransform<T>& X_FQ) const {
    return CalcPoseInBodyFrame(context) * X_FQ;
  }

  /// Calculates and returns the rotation matrix `R_BQ` that relates body frame
  /// B to frame Q via `this` intermediate frame F, i.e., `R_BQ = R_BF * R_FQ`
  /// (B is the body frame to which `this` frame F is attached).
  /// @param[in] R_FQ rotation matrix that relates frame F to frame Q.
  virtual math::RotationMatrix<T> CalcOffsetRotationMatrixInBody(
      const systems::Context<T>& context,
      const math::RotationMatrix<T>& R_FQ) const {
    return CalcRotationMatrixInBodyFrame(context) * R_FQ;
  }

  /// Variant of CalcOffsetPoseInBody() that given the offset pose `X_FQ` of a
  /// frame Q in `this` frame F, returns the pose `X_BQ` of frame Q in the body
  /// frame B to which this frame is attached.
  /// @throws std::logic_error if called on a %Frame that does not have a
  /// fixed offset in the body frame.
  virtual math::RigidTransform<T> GetFixedOffsetPoseInBody(
      const math::RigidTransform<T>& X_FQ) const {
    return GetFixedPoseInBodyFrame() * X_FQ;
  }

  /// Calculates and returns the rotation matrix `R_BQ` that relates body frame
  /// B to frame Q via `this` intermediate frame F, i.e., `R_BQ = R_BF * R_FQ`
  /// (B is the body frame to which `this` frame F is attached).
  /// @param[in] R_FQ rotation matrix that relates frame F to frame Q.
  /// @throws std::logic_error if `this` frame F is a %Frame that does not have
  /// a fixed offset in the body frame B (i.e., `R_BF` is not constant).
  virtual math::RotationMatrix<T> GetFixedRotationMatrixInBody(
      const math::RotationMatrix<T>& R_FQ) const {
    return GetFixedRotationMatrixInBodyFrame() * R_FQ;
  }

  /// Computes and returns the pose `X_WF` of `this` frame F in the world
  /// frame W as a function of the state of the model stored in `context`.
  /// @note Body::EvalPoseInWorld() provides a more efficient way to obtain
  /// the pose for a body frame.
  math::RigidTransform<T> CalcPoseInWorld(
      const systems::Context<T>& context) const {
    return this->get_parent_tree().CalcRelativeTransform(
        context, this->get_parent_tree().world_frame(), *this);
  }

  /// Computes and returns the pose `X_MF` of `this` frame F in measured in
  /// `frame_M` as a function of the state of the model stored in `context`.
  /// @see CalcPoseInWorld().
  math::RigidTransform<T> CalcPose(
      const systems::Context<T>& context, const Frame<T>& frame_M) const {
    return this->get_parent_tree().CalcRelativeTransform(
        context, frame_M, *this);
  }

  /// Calculates and returns the rotation matrix `R_MF` that relates `frame_M`
  /// and `this` frame F as a function of the state stored in `context`.
  math::RotationMatrix<T> CalcRotationMatrix(
      const systems::Context<T>& context, const Frame<T>& frame_M) const {
    return this->get_parent_tree().CalcRelativeRotationMatrix(
        context, frame_M, *this);
  }

  /// Calculates and returns the rotation matrix `R_WF` that relates the world
  /// frame W and `this` frame F as a function of the state stored in `context`.
  math::RotationMatrix<T> CalcRotationMatrixInWorld(
      const systems::Context<T>& context) const {
    return this->get_parent_tree().CalcRelativeRotationMatrix(
        context, this->get_parent_tree().world_frame(), *this);
  }

  /// Computes and returns the spatial velocity `V_WF` of `this` frame F in the
  /// world frame W as a function of the state of the model stored in `context`.
  /// @note Body::EvalSpatialVelocityInWorld() provides a more efficient way to
  /// obtain the spatial velocity for a body frame.
  SpatialVelocity<T> CalcSpatialVelocityInWorld(
      const systems::Context<T>& context) const {
    const math::RotationMatrix<T>& R_WB =
        body().EvalPoseInWorld(context).rotation();
    const Vector3<T> p_BF_B = CalcPoseInBodyFrame(context).translation();
    const Vector3<T> p_BF_W = R_WB * p_BF_B;
    const SpatialVelocity<T>& V_WB = body().EvalSpatialVelocityInWorld(context);
    const SpatialVelocity<T> V_WF = V_WB.Shift(p_BF_W);
    return V_WF;
  }

  /// Computes and returns the spatial velocity `V_MF_E` of `this` frame F
  /// measured in `frame_M` and expressed in `frame_E` as a function of the
  /// state of the model stored in `context`.
  /// @see CalcSpatialVelocityInWorld().
  SpatialVelocity<T> CalcSpatialVelocity(
      const systems::Context<T>& context,
      const Frame<T>& frame_M, const Frame<T>& frame_E) const {
    const math::RotationMatrix<T> R_WM =
        frame_M.CalcRotationMatrixInWorld(context);
    const Vector3<T> p_MF_M = this->CalcPose(context, frame_M).translation();
    const Vector3<T> p_MF_W = R_WM * p_MF_M;
    const SpatialVelocity<T> V_WM = frame_M.CalcSpatialVelocityInWorld(context);
    const SpatialVelocity<T> V_WF = this->CalcSpatialVelocityInWorld(context);
    // We obtain V_MF from the composition of spatial velocities:
    const SpatialVelocity<T> V_MF_W = V_WF - V_WM.Shift(p_MF_W);
    if (frame_E.index() == FrameIndex(0)) return V_MF_W;
    // If expressed-in frame_E is not the world, perform additional
    // transformation.
    const math::RotationMatrix<T> R_WE =
        frame_E.CalcRotationMatrixInWorld(context);
    return R_WE.inverse() * V_MF_W;
  }

  /// (Advanced) NVI to DoCloneToScalar() templated on the scalar type of the
  /// new clone to be created. This method is mostly intended to be called by
  /// MultibodyTree::CloneToScalar(). Most users should not call this clone
  /// method directly but rather clone the entire parent MultibodyTree if
  /// needed.
  /// @sa MultibodyTree::CloneToScalar()
  template <typename ToScalar>
  std::unique_ptr<Frame<ToScalar>> CloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const {
    return DoCloneToScalar(tree_clone);
  }

 protected:
  /// Only derived classes can use this constructor. It creates a %Frame
  /// object attached to `body` and puts the frame in the body's model
  /// instance.
  explicit Frame(
      const std::string& name, const Body<T>& body,
      std::optional<ModelInstanceIndex> model_instance = {})
      : FrameBase<T>(model_instance.value_or(body.model_instance())),
        name_(name), body_(body) {}

  /// Overload to permit constructing an unnamed frame.
  explicit Frame(const Body<T>& body)
      : Frame("", body) {}

  /// @name Methods to make a clone templated on different scalar types.
  ///
  /// These methods are meant to be called by MultibodyTree::CloneToScalar()
  /// when making a clone of the entire tree or a new instance templated on a
  /// different scalar type. The only const argument to these methods is the
  /// new MultibodyTree clone under construction. Specific %Frame subclasses
  /// might specify a number of prerequisites on the cloned tree and therefore
  /// require it to be at a given state of cloning. See
  /// MultibodyTree::CloneToScalar() for a list of prerequisites that are
  /// guaranteed to be satisfied during the cloning process.
  /// @{

  /// Clones this %Frame (templated on T) to a frame templated on `double`.
  virtual std::unique_ptr<Frame<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const = 0;

  /// Clones this %Frame (templated on T) to a frame templated on AutoDiffXd.
  virtual std::unique_ptr<Frame<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const = 0;

  virtual std::unique_ptr<Frame<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const = 0;
  /// @}

 private:
  // Implementation for MultibodyElement::DoSetTopology().
  void DoSetTopology(const internal::MultibodyTreeTopology& tree_topology)
  final {
    topology_ = tree_topology.get_frame(this->index());
    DRAKE_ASSERT(topology_.index == this->index());
  }

  std::string name_;

  // The body associated with this frame.
  const Body<T>& body_;

  // The internal bookkeeping topology struct used by MultibodyTree.
  internal::FrameTopology topology_;
};

}  // namespace multibody
}  // namespace drake
