#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/common/autodiff.h"
#include "drake/common/nice_type_name.h"
#include "drake/multibody/tree/frame_body_pose_cache.h"
#include "drake/multibody/tree/multibody_element.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/scoped_name.h"

namespace drake {
namespace multibody {

namespace internal {
std::string DeprecateWhenEmptyName(std::string name, std::string_view type);
}  // namespace internal

// Forward declarations.
template <typename T>
class RigidBody;

template <typename T>
using Link = RigidBody<T>;

/// %Frame is an abstract class representing a _material frame_ (also called a
/// _physical frame_) of its underlying RigidBody (Link). The %Frame's origin
/// is a material point of its RigidBody, and its axes have fixed directions
/// in that body. A %Frame's pose (position and orientation) with respect to its
/// RigidBodyFrame (LinkFrame) may be parameterized, but is fixed (not time or
/// state dependent) once parameters have been set.
///
/// An important characteristic of a %Frame is that forces or torques applied to
/// a %Frame are applied to the %Frame's underlying body. Force-producing
/// elements like joints, actuators, and constraints usually employ two %Frames,
/// with one %Frame connected to one body and the other connected to a different
/// body. Every %Frame F can report the Link (RigidBody) L to which it is
/// attached and its pose X_LF with respect to L's LinkFrame (RigidBodyFrame).
///
/// A %Frame's pose in World (or relative to other frames) is always calculated
/// starting with its pose relative to its underlying LinkFrame.
/// Subclasses derived from %Frame differ in how kinematic calculations are
/// performed. For example, the angular velocity of a FixedOffsetFrame or
/// LinkFrame (RigidBodyFrame) is identical to the angular velocity of its
/// underlying body,
/// whereas the translational velocity of a FixedOffsetFrame differs from that
/// of a LinkFrame.
///
/// %Frame provides methods for obtaining its current orientation, position,
/// motion, etc. from a Context passed to those methods.
///
/// @note For historical reasons, many of the method names here use "Body" to
/// mean "Link". The distinction matters when we form composite bodies, which
/// consist of multiple links welded together. Those composites form a single
/// _rigid body_ in the physics sense. Frames only know about their Links, not
/// how they may have been combined into a composite body.
///
/// @tparam_default_scalar
template <typename T>
class Frame : public MultibodyElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Frame);

  ~Frame() override;

  /// Returns this element's unique index.
  FrameIndex index() const { return this->template index_impl<FrameIndex>(); }

  /// Returns a const reference to the RigidBody (Link) to which this %Frame is
  /// attached (synonym for link()).
  const RigidBody<T>& body() const { return link(); }

  /// Returns a const reference to the Link (RigidBody) to which this %Frame is
  /// attached (synonym for body()).
  const Link<T>& link() const { return link_; }

  /// Returns true if `this` is the world frame.
  bool is_world_frame() const { return this->index() == world_frame_index(); }

  /// Returns true if `this` is the RigidBodyFrame (LinkFrame) of the
  /// associated RigidBody (Link). This is a synonym for is_link_frame().
  bool is_body_frame() const { return is_link_frame(); }

  /// Returns true if `this` is the LinkFrame (RigidBodyFrame) of the
  /// associated Link (RigidBody). This is a synonym for is_body_frame().
  bool is_link_frame() const {
    return this->index() == link().link_frame().index();
  }

  /// Returns the name of this frame. The name will never be empty.
  const std::string& name() const { return name_; }

  /// Returns scoped name of this frame. Neither of the two pieces of the name
  /// will be empty (the scope name and the element name).
  /// @throws std::exception if this element is not associated with a
  /// MultibodyPlant.
  ScopedName scoped_name() const;

  /// Returns a reference to the link-relative pose X_LF giving the pose of this
  /// %Frame with respect to its link's LinkFrame (RigidBodyFrame). This may
  /// depend on parameters in the Context but not on time or state. The first
  /// time this is called after a parameter change will precalculate offset
  /// poses for all %Frames into the Context's cache; subsequent calls on any
  /// %Frame are very fast.
  const math::RigidTransform<T>& EvalPoseInBodyFrame(
      const systems::Context<T>& context) const {
    const internal::FrameBodyPoseCache<T>& frame_body_poses =
        this->GetParentTreeSystem().EvalFrameBodyPoses(context);
    return get_X_LF(frame_body_poses);
  }

  /// Returns the pose `X_LF` of `this` frame F in the LinkFrame
  /// (RigidBodyFrame) L of this %Frame's Link (RigidBody). In particular, if
  /// `this` **is** the link frame L, this method directly returns the
  /// identity transformation. Note that this ONLY depends on the Parameters
  /// in the context; it does not depend on time, input, state, etc.
  math::RigidTransform<T> CalcPoseInBodyFrame(
      const systems::Context<T>& context) const {
    return DoCalcPoseInBodyFrame(context.get_parameters());
  }

  /// Returns the rotation matrix `R_LF` that relates link frame L to `this`
  /// frame F (L is the LinkFrame of the Link (RigidBody) to which `this`
  /// frame F is  attached).
  /// @note If `this` is L, this method returns the identity RotationMatrix.
  /// Note that this ONLY depends on the Parameters in the context; it does
  /// not depend on time, input, state, etc.
  math::RotationMatrix<T> CalcRotationMatrixInBodyFrame(
      const systems::Context<T>& context) const {
    return DoCalcRotationMatrixInBodyFrame(context.get_parameters());
  }

  /// Variant of CalcPoseInBodyFrame() that returns the fixed pose `X_LF` of
  /// `this` frame F in the link frame L associated with this frame.
  /// @throws std::exception if called on a %Frame that does not have a
  /// fixed offset in the body frame.
  // Frame sub-classes that can represent the fixed pose of `this` frame F in
  // a link frame L, must override this method.
  virtual math::RigidTransform<T> GetFixedPoseInBodyFrame() const {
    throw std::logic_error(
        "Attempting to retrieve a fixed pose from a frame of type '" +
        drake::NiceTypeName::Get(*this) +
        "', which does not support this operation.");
  }

  /// Returns the rotation matrix `R_LF` that relates link frame L to `this`
  /// frame F (L is the LinkFrame of the Link (RigidBody) to which `this`
  /// frame F is attached).
  /// @throws std::exception if `this` frame F is a %Frame that does not have
  /// a fixed offset in the link frame L (i.e., `R_LF` is not constant).
  /// %Frame sub-classes that have a constant `R_LF` must override this method.
  virtual math::RotationMatrix<T> GetFixedRotationMatrixInBodyFrame() const {
    throw std::logic_error(
        "Unable to retrieve a fixed rotation matrix from a frame of type '" +
        drake::NiceTypeName::Get(*this) +
        "', which does not support this method.");
  }

  // TODO(jwnimmer-tri) These next four functions only exist so that
  //  RigidBodyFrame can override their NVI body to be a simple copy instead of
  //  ComposeXX or ComposeRR against an identity. It is not at all clear to me
  //  that this implementation complexity is buying us any measurable speedup at
  //  runtime.

  /// Given the offset pose `X_FQ` of a frame Q in `this` frame F, this method
  /// computes the pose `X_LQ` of frame Q in the link frame L of the Link
  /// (RigidBody) to which this frame is attached. In other words, if the
  /// pose of `this` frame F in the link frame L is `X_LF`, this method
  /// computes the pose `X_LQ` of frame Q in the link frame L as
  /// `X_LQ = X_LF * X_FQ`. In particular, if `this` **is** the link frame L,
  /// i.e. `X_LF` is identically the identity transform, this method directly
  /// returns `X_FQ`. Specific frame subclasses can override this method to
  /// provide faster implementations if needed.
  math::RigidTransform<T> CalcOffsetPoseInBody(
      const systems::Context<T>& context,
      const math::RigidTransform<T>& X_FQ) const {
    return DoCalcOffsetPoseInBody(context.get_parameters(), X_FQ);
  }

#ifndef DRAKE_DOXYGEN_CXX
  // (Internal use only) Overload on Parameters instead of Context.
  math::RigidTransform<T> CalcOffsetPoseInBody(
      const systems::Parameters<T>& parameters,
      const math::RigidTransform<T>& X_FQ) const {
    return DoCalcOffsetPoseInBody(parameters, X_FQ);
  }
#endif

  /// Calculates and returns the rotation matrix `R_LQ` that relates link frame
  /// L to frame Q via `this` intermediate frame F, i.e., `R_LQ = R_LF * R_FQ`
  /// (L is the link frame of the Link (RigidBody) to which `this` frame F is
  /// attached).
  /// @param[in] R_FQ rotation matrix that relates frame F to frame Q.
  math::RotationMatrix<T> CalcOffsetRotationMatrixInBody(
      const systems::Context<T>& context,
      const math::RotationMatrix<T>& R_FQ) const {
    return DoCalcOffsetRotationMatrixInBody(context.get_parameters(), R_FQ);
  }

#ifndef DRAKE_DOXYGEN_CXX
  // (Internal use only) Overload on Parameters instead of Context.
  math::RotationMatrix<T> CalcOffsetRotationMatrixInBody(
      const systems::Parameters<T>& parameters,
      const math::RotationMatrix<T>& R_FQ) const {
    return DoCalcOffsetRotationMatrixInBody(parameters, R_FQ);
  }
#endif

  /// Variant of CalcOffsetPoseInBody() that given the offset pose `X_FQ` of a
  /// frame Q in `this` frame F, returns the pose `X_LQ` of frame Q in the link
  /// frame L to which this frame is attached.
  /// @throws std::exception if called on a %Frame that does not have a
  /// fixed offset in the body frame.
  virtual math::RigidTransform<T> GetFixedOffsetPoseInBody(
      const math::RigidTransform<T>& X_FQ) const {
    return GetFixedPoseInBodyFrame() * X_FQ;
  }

  /// Calculates and returns the rotation matrix `R_LQ` that relates link frame
  /// L to frame Q via `this` intermediate frame F, i.e., `R_LQ = R_LF * R_FQ`
  /// (L is the link frame of the Link (RigidBody) to which `this` frame F is
  /// attached).
  /// @param[in] R_FQ rotation matrix that relates frame F to frame Q.
  /// @throws std::exception if `this` frame F is a %Frame that does not have
  /// a fixed offset in the link frame L (i.e., `R_LF` is not constant).
  virtual math::RotationMatrix<T> GetFixedRotationMatrixInBody(
      const math::RotationMatrix<T>& R_FQ) const {
    return GetFixedRotationMatrixInBodyFrame() * R_FQ;
  }

  /// Computes and returns the pose `X_WF` of `this` frame F in the world
  /// frame W as a function of the state of the model stored in `context`.
  /// @note RigidBody::EvalPoseInWorld() provides a more efficient way to obtain
  /// the pose for a RigidBodyFrame (LinkFrame).
  math::RigidTransform<T> CalcPoseInWorld(
      const systems::Context<T>& context) const {
    DRAKE_THROW_UNLESS(this->has_parent_tree());
    const internal::MultibodyTree<T>& tree = this->get_parent_tree();
    return tree.CalcRelativeTransform(context, tree.world_frame(), *this);
  }

  /// Computes and returns the pose `X_MF` of `this` frame F in measured in
  /// `frame_M` as a function of the state of the model stored in `context`.
  /// @see CalcPoseInWorld().
  math::RigidTransform<T> CalcPose(const systems::Context<T>& context,
                                   const Frame<T>& frame_M) const {
    DRAKE_THROW_UNLESS(this->has_parent_tree());
    return this->get_parent_tree().CalcRelativeTransform(context, frame_M,
                                                         *this);
  }

  /// Calculates and returns the rotation matrix `R_MF` that relates `frame_M`
  /// and `this` frame F as a function of the state stored in `context`.
  math::RotationMatrix<T> CalcRotationMatrix(const systems::Context<T>& context,
                                             const Frame<T>& frame_M) const {
    DRAKE_THROW_UNLESS(this->has_parent_tree());
    return this->get_parent_tree().CalcRelativeRotationMatrix(context, frame_M,
                                                              *this);
  }

  /// Calculates and returns the rotation matrix `R_WF` that relates the world
  /// frame W and `this` frame F as a function of the state stored in `context`.
  math::RotationMatrix<T> CalcRotationMatrixInWorld(
      const systems::Context<T>& context) const {
    DRAKE_THROW_UNLESS(this->has_parent_tree());
    const internal::MultibodyTree<T>& tree = this->get_parent_tree();
    return tree.CalcRelativeRotationMatrix(context, tree.world_frame(), *this);
  }

  /// Evaluates `this` frame F's angular velocity measured and expressed in the
  /// world frame W.
  /// @param[in] context contains the state of the multibody system.
  /// @return ω_WF_W (frame F's angular velocity ω measured and expressed in
  /// the world frame W).
  /// @see CalcAngularVelocity() to calculate ω_MF_E (`this` frame F's angular
  /// velocity ω measured in a frame M and expressed in a frame E).
  const Vector3<T>& EvalAngularVelocityInWorld(
      const systems::Context<T>& context) const {
    const SpatialVelocity<T>& V_WL = link().EvalSpatialVelocityInWorld(context);
    const Vector3<T>& w_WF_W = V_WL.rotational();
    return w_WF_W;
  }

  /// Calculates `this` frame F's angular velocity measured in a frame M,
  /// expressed in a frame E.
  /// @param[in] context contains the state of the multibody system.
  /// @param[in] measured_in_frame which is frame M (the frame in which `this`
  /// angular velocity is to be measured).
  /// @param[in] expressed_in_frame which is frame E (the frame in which the
  /// returned angular velocity is to be expressed).
  /// @return ω_MF_E, `this` frame F's angular velocity ω measured in frame M,
  /// expressed in frame E.
  /// @see EvalAngularVelocityInWorld() to evaluate ω_WF_W (`this` frame F's
  /// angular velocity ω measured and expressed in the world frame W).
  Vector3<T> CalcAngularVelocity(const systems::Context<T>& context,
                                 const Frame<T>& measured_in_frame,
                                 const Frame<T>& expressed_in_frame) const;

  /// Calculates `this` frame F's spatial velocity measured and expressed in
  /// the world frame W.
  /// @param[in] context contains the state of the multibody system.
  /// @return V_WF_W, `this` frame F's spatial velocity measured and expressed
  /// in the world frame W. The rotational part of the returned quantity is
  /// ω_WF_W (frame F's angular velocity ω measured and expressed in the world
  /// frame W). The translational part is v_WFo_W (translational velocity v of
  /// frame F's origin point Fo, measured and expressed in the world frame W).
  /// @note RigidBody::EvalSpatialVelocityInWorld() provides a more efficient
  /// way to obtain a RigidBodyFrame (LinkFrame) spatial velocity measured in
  /// the world  frame.
  /// @see CalcSpatialVelocity(), CalcRelativeSpatialVelocityInWorld(), and
  /// CalcSpatialAccelerationInWorld().
  SpatialVelocity<T> CalcSpatialVelocityInWorld(
      const systems::Context<T>& context) const;

  /// Calculates `this` frame F's spatial velocity measured in a frame M,
  /// expressed in a frame E.
  /// @param[in] context contains the state of the multibody system.
  /// @param[in] frame_M which is the measured_in_frame.
  /// @param[in] frame_E which is the expressed_in_frame.
  /// @return V_MF_E, `this` frame F's spatial velocity measured in frame M,
  /// expressed in frame E. The rotational part of the returned quantity is
  /// ω_MF_E (frame F's angular velocity ω measured in frame M, expressed in
  /// frame E). The translational part is v_MFo_E (translational velocity v of
  /// frame F's origin point Fo, measured in frame M, expressed in frame E).
  /// @see CalcSpatialVelocityInWorld(), CalcRelativeSpatialVelocity(), and
  /// CalcSpatialAcceleration().
  SpatialVelocity<T> CalcSpatialVelocity(const systems::Context<T>& context,
                                         const Frame<T>& frame_M,
                                         const Frame<T>& frame_E) const;

  /// Calculates `this` frame F's spatial velocity relative to another frame B,
  /// measured and expressed in the world frame W.
  /// @param[in] context contains the state of the multibody system.
  /// @param[in] other_frame which is frame B.
  /// @return V_W_BF_W = V_WF_W - V_WB_W, frame F's spatial velocity relative to
  /// frame B, measured and expressed in the world frame W. The rotational part
  /// of the returned quantity is ω_BF_W (F's angular velocity measured in B and
  /// expressed in W). The translational part is v_W_BoFo_W (Fo's translational
  /// velocity relative to Bo, measured and expressed in world frame W). <pre>
  ///     ω_BF_W  = ω_WF_W - ω_WB_W
  ///  v_W_BoFo_W = v_WFo_W - v_WBo_W = DtW(p_BoFo)
  /// </pre>
  /// where DtW(p_BoFo) is the time-derivative in frame W of p_BoFo (position
  /// vector from Bo to Fo), and this vector is expressed in frame W.
  /// @note The method CalcSpatialVelocityInWorld() is more efficient and
  /// coherent if any of `this`, other_frame, or the world frame W are the same.
  /// @see CalcSpatialVelocityInWorld() and CalcRelativeSpatialVelocity().
  SpatialVelocity<T> CalcRelativeSpatialVelocityInWorld(
      const systems::Context<T>& context, const Frame<T>& other_frame) const {
    const Frame<T>& frame_B = other_frame;
    const SpatialVelocity<T> V_WB_W =
        frame_B.CalcSpatialVelocityInWorld(context);
    const SpatialVelocity<T> V_WF_W = CalcSpatialVelocityInWorld(context);
    return V_WF_W - V_WB_W;
  }

  /// Calculates `this` frame F's spatial velocity relative to another frame B,
  /// measured in a frame M, expressed in a frame E.
  /// @param[in] context contains the state of the multibody system.
  /// @param[in] other_frame which is frame B.
  /// @param[in] measured_in_frame which is frame M.
  /// @param[in] expressed_in_frame which is frame E.
  /// @return V_M_BF_E = V_MF_E - V_MB_E, frame F's spatial velocity relative to
  /// frame B, measured in frame M, expressed in frame E. The rotational part
  /// of the returned quantity is ω_BF_E (F's angular velocity measured in B and
  /// expressed in E). The translational part is v_M_BoFo_E (Fo's translational
  /// velocity relative to Bo, measured in M, and expressed in E). <pre>
  ///  ω_BF_E = ω_MF_E - ω_MB_E
  ///  v_M_BoFo_E = v_MFo_E - v_MBo_E = DtM(p_BoFo)
  /// </pre>
  /// where DtM(p_BoFo) is the time-derivative in frame M of p_BoFo (position
  /// vector from Bo to Fo), and this vector is expressed in frame E.
  /// @note The method CalcSpatialVelocity() is more efficient and coherent
  /// if any of `this`, other_frame, or measured_in_frame are the same.
  /// Also, the value of V_M_BoFo does not depend on the measured_in_frame if
  /// Bo and Fo are coincident (i.e., p_BoFo = 0), in which case consider the
  /// more efficient method CalcRelativeSpatialVelocityInWorld().
  /// Lastly, the calculation of elongation between Bo and Fo can be done with
  /// relative translational velocity, but elongation does not depend on the
  /// measured-in-frame (hence consider CalcRelativeSpatialVelocityInWorld()).
  /// @see CalcSpatialVelocityInWorld(), CalcSpatialVelocity(), and
  /// CalcRelativeSpatialVelocityInWorld().
  SpatialVelocity<T> CalcRelativeSpatialVelocity(
      const systems::Context<T>& context, const Frame<T>& other_frame,
      const Frame<T>& measured_in_frame,
      const Frame<T>& expressed_in_frame) const {
    const Frame<T>& frame_B = other_frame;
    const Frame<T>& frame_M = measured_in_frame;
    const Frame<T>& frame_E = expressed_in_frame;
    const SpatialVelocity<T> V_MB_E =
        frame_B.CalcSpatialVelocity(context, frame_M, frame_E);
    const SpatialVelocity<T> V_MF_E =
        CalcSpatialVelocity(context, frame_M, frame_E);
    return V_MF_E - V_MB_E;
  }

  /// Calculates `this` frame F's spatial acceleration measured and expressed in
  /// the world frame W.
  /// @param[in] context contains the state of the multibody system.
  /// @return A_WF_W, `this` frame F's spatial acceleration measured and
  /// expressed in the world frame W. The rotational part of the returned
  /// quantity is α_WF_E (frame F's angular acceleration α measured and
  /// expressed in the world frame W).  The translational part is a_WFo_W
  /// (translational acceleration of frame F's origin point Fo, measured and
  /// expressed in the world frame W).
  /// @note RigidBody::EvalSpatialAccelerationInWorld() provides a more
  /// efficient way to obtain a body frame's spatial acceleration measured in
  /// the world frame.
  /// @note When cached values are out of sync with the state stored in context,
  /// this method performs an expensive forward dynamics computation, whereas
  /// once evaluated, successive calls to this method are inexpensive.
  /// @see CalcSpatialAcceleration() and CalcSpatialVelocityInWorld().
  SpatialAcceleration<T> CalcSpatialAccelerationInWorld(
      const systems::Context<T>& context) const;

  /// Calculates `this` frame F's spatial acceleration measured in a frame M,
  /// expressed in a frame E.
  /// @param[in] context contains the state of the multibody system.
  /// @param[in] measured_in_frame which is frame M.
  /// @param[in] expressed_in_frame which is frame E.
  /// @return A_MF_E, `this` frame F's spatial acceleration measured in frame M,
  /// expressed in frame E. The rotational part of the returned quantity is
  /// α_MF_E (frame F's angular acceleration α measured in frame M, expressed in
  /// frame E). The translational part is a_MFo_E (translational acceleration of
  /// frame F's origin point Fo, measured in frame M, expressed in frame E).
  /// Although α_MF is defined below in terms of DtM(ω_MF), the time-derivative
  /// in frame M of ω_MF, the actual calculation of α_MF avoids differentiation.
  /// Similarly for the definition vs. calculation for a_MFo. <pre>
  ///  α_MF = DtM(ω_MF)           ω_MF is frame F's angular velocity in frame M.
  ///  a_MFo = DtM(v_MFo)    v_MF is Fo's translational acceleration in frame M.
  /// </pre>
  /// @see CalcSpatialAccelerationInWorld() and CalcSpatialVelocity().
  SpatialAcceleration<T> CalcSpatialAcceleration(
      const systems::Context<T>& context, const Frame<T>& measured_in_frame,
      const Frame<T>& expressed_in_frame) const;

  /// Calculates `this` frame F's spatial acceleration relative to another
  /// frame B, measured and expressed in the world frame W.
  /// @param[in] context contains the state of the multibody system.
  /// @param[in] other_frame which is frame B.
  /// @return A_W_BF_W = A_WF_W - A_WB_W, frame F's spatial acceleration
  /// relative to frame B, measured and expressed in the world frame W.
  ///
  /// In general, A_W_BF = DtW(V_W_BF), the time-derivative in the world frame W
  /// of frame F's spatial velocity relative to frame B. The rotational part of
  /// the returned quantity is α_WF_W - α_WB_W = DtW(ω_BF)_W. For 3D analysis,
  /// DtW(ω_BF) ≠ α_BF. The translational part of the returned quantity is
  /// a_W_BoFo_W (Fo's translational acceleration relative to Bo, measured and
  /// expressed in world frame W). <pre>
  ///  α_WF_W - α_WB_W = DtW(ω_WF)_W - DtW(ω_WB)_W = DtW(ω_BF)_W
  ///  a_W_BoFo_W = a_WFo_W - a_WBo_W = DtW(v_WFo) - DtW(v_WBo) = Dt²W(p_BoFo)_W
  /// </pre>
  /// where Dt²W(p_BoFo)_W is the 2ⁿᵈ time-derivative in frame W of p_BoFo (the
  /// position vector from Bo to Fo), and this result is expressed in frame W.
  /// @note The method CalcSpatialAccelerationInWorld() is more efficient and
  /// coherent if any of `this`, other_frame, or the world frame W are the same.
  /// @see CalcSpatialAccelerationInWorld(), CalcRelativeSpatialAcceleration().
  SpatialAcceleration<T> CalcRelativeSpatialAccelerationInWorld(
      const systems::Context<T>& context, const Frame<T>& other_frame) const {
    const Frame<T>& frame_B = other_frame;
    const SpatialAcceleration<T> A_WB_W =
        frame_B.CalcSpatialAccelerationInWorld(context);
    const SpatialAcceleration<T> A_WF_W =
        CalcSpatialAccelerationInWorld(context);
    return A_WF_W - A_WB_W;
  }

  /// Calculates `this` frame F's spatial acceleration relative to another
  /// frame B, measured in a frame M, expressed in a frame E.
  /// @param[in] context contains the state of the multibody system.
  /// @param[in] other_frame which is frame B.
  /// @param[in] measured_in_frame which is frame M.
  /// @param[in] expressed_in_frame which is frame E.
  /// @return A_M_BF_E = A_MF_E - A_MB_E, frame F's spatial acceleration
  /// relative to frame B, measured in frame M, expressed in frame E.
  ///
  /// In general, A_M_BF = DtW(V_M_BF), the time-derivative in frame M of
  /// frame F's spatial velocity relative to frame B. The rotational part of the
  /// returned quantity is α_MF_E - α_MB_E = DtM(ω_BF)_E. Note: For 3D analysis,
  /// DtM(ω_BF) ≠ α_BF. The translational part of the returned quantity is
  /// a_M_BoFo_E (Fo's translational acceleration relative to Bo, measured in
  /// frame M, expressed in frame E). <pre>
  ///  α_MF_E - α_MB_E = DtM(ω_MF)_E - DtM(ω_MB)_E = DtM(ω_BF)_E
  ///  a_M_BoFo_E = a_MFo_E - a_MBo_E = DtM(v_MFo) - DtM(v_MBo) = Dt²M(p_BoFo)_E
  /// </pre>
  /// where Dt²M(p_BoFo)_E is the 2ⁿᵈ time-derivative in frame M of p_BoFo (the
  /// position vector from Bo to Fo), and this result is expressed in frame E.
  /// @note The calculation of the 2ⁿᵈ time-derivative of the distance between
  /// Bo and Fo can be done with relative translational acceleration, but this
  /// calculation does not depend on the measured-in-frame, hence in this case,
  /// consider CalcRelativeSpatialAccelerationInWorld() since it is faster.
  /// @see CalcSpatialAccelerationInWorld(), CalcSpatialAcceleration(), and
  /// CalcRelativeSpatialAccelerationInWorld().
  SpatialAcceleration<T> CalcRelativeSpatialAcceleration(
      const systems::Context<T>& context, const Frame<T>& other_frame,
      const Frame<T>& measured_in_frame,
      const Frame<T>& expressed_in_frame) const {
    const Frame<T>& frame_B = other_frame;
    const Frame<T>& frame_M = measured_in_frame;
    const Frame<T>& frame_E = expressed_in_frame;
    const SpatialAcceleration<T> A_MB_E =
        frame_B.CalcSpatialAcceleration(context, frame_M, frame_E);
    const SpatialAcceleration<T> A_MF_E =
        CalcSpatialAcceleration(context, frame_M, frame_E);
    return A_MF_E - A_MB_E;
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

  /// (Internal use only) Returns a shallow clone (i.e., dependent elements such
  /// as bodies are aliased, not copied) that is not associated with any MbT (so
  /// the assigned index, if any, is discarded).
  std::unique_ptr<Frame<T>> ShallowClone() const;

  /// @name Internal use only
  /// These functions work directly with the frame body pose cache entry.
  //@{
  /// (Internal use only) A %Frame's pose-in-parent-frame X_PF can be
  /// parameterized, the parent frame's pose may also be parameterized, and so
  /// on. Thus the calculation of this frame's pose in its link frame (X_LF) can
  /// be expensive. There is a cache entry that holds the calculated X_LF,
  /// evaluated whenever parameters change. This allows us to grab X_LF as a
  /// const reference rather than having to extract and reformat parameters, and
  /// compose with parent and ancestor poses at runtime.
  ///
  /// When we are optimizing assemblies using composite Mobods, the pose of
  /// a frame on its Mobod (X_BF) can differ from its pose on its Link (X_LF).
  /// Most multibody computations need X_BF, but X_LF is available also.

  /// (Internal use only) Given an already up-to-date frame body pose cache,
  /// extract X_LF for this %Frame from it.
  /// @note Be sure you have called MultibodyTreeSystem::EvalFrameBodyPoses()
  ///       since the last parameter change; we can't check here.
  /// @retval X_LF pose of this frame in its Link's frame
  const math::RigidTransform<T>& get_X_LF(
      const internal::FrameBodyPoseCache<T>& frame_body_poses) const {
    return frame_body_poses.get_X_LF(index());
  }

  /// (Internal use only) Given an already up-to-date frame body pose cache,
  /// extract X_BF for this %Frame from it. Note that X_BF is F's pose on its
  /// mobilized body B which might not be the same as its link L.
  /// @note Be sure you have called MultibodyTreeSystem::EvalFrameBodyPoses()
  ///       since the last parameter change; we can't check here.
  /// @retval X_BF pose of this frame in its Mobod's frame
  const math::RigidTransform<T>& get_X_BF(
      const internal::FrameBodyPoseCache<T>& frame_body_poses) const {
    return frame_body_poses.get_X_BF(index());
  }

  /// (Internal use only) Given an already up-to-date frame body pose cache,
  /// extract X_FB (=X_BF⁻¹) for this %Frame from it.
  /// @note Be sure you have called MultibodyTreeSystem::EvalFrameBodyPoses()
  ///       since the last parameter change; we can't check here.
  /// @retval X_FB inverse of this frame's pose in its Mobod's frame
  const math::RigidTransform<T>& get_X_FB(
      const internal::FrameBodyPoseCache<T>& frame_body_poses) const {
    return frame_body_poses.get_X_FB(index());
  }

  /// (Internal use only) Given an already up-to-date frame body pose cache,
  /// returns whether X_BF (and thus X_FB) is exactly identity. This is
  /// precomputed in the cache so is very fast to check.
  /// @note Be sure you have called MultibodyTreeSystem::EvalFrameBodyPoses()
  ///       since the last parameter change; we can't check here.
  /// @see get_X_BF(), get_X_FB()
  bool is_X_BF_identity(
      const internal::FrameBodyPoseCache<T>& frame_body_poses) const {
    return frame_body_poses.is_X_BF_identity(index());
  }
  //@}

 protected:
  /// Only derived classes can use this constructor. It creates a %Frame
  /// object attached to `link` and puts the frame in the link's model
  /// instance.
  explicit Frame(const std::string& name, const Link<T>& link,
                 std::optional<ModelInstanceIndex> model_instance = {})
      : MultibodyElement<T>(model_instance.value_or(link.model_instance())),
        name_(internal::DeprecateWhenEmptyName(name, "Frame")),
        link_(link) {}

  /// Called by DoDeclareParameters(). Derived classes may choose to override
  /// to declare their sub-class specific parameters.
  virtual void DoDeclareFrameParameters(internal::MultibodyTreeSystem<T>*) {}

  /// Called by DoSetDefaultParameters(). Derived classes may choose to override
  /// to set their sub-class specific parameters.
  virtual void DoSetDefaultFrameParameters(systems::Parameters<T>*) const {}

  /// @name Methods to make a clone, optionally templated on different scalar
  /// types.
  ///
  /// The first three are meant to be called by MultibodyTree::CloneToScalar()
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

  /// NVI for ShallowClone().
  virtual std::unique_ptr<Frame<T>> DoShallowClone() const;
  /// @}

  // NVI for CalcPoseInBodyFrame.
  virtual math::RigidTransform<T> DoCalcPoseInBodyFrame(
      const systems::Parameters<T>& parameters) const = 0;

  // NVI for CalcRotationMatrixInBodyFrame.
  virtual math::RotationMatrix<T> DoCalcRotationMatrixInBodyFrame(
      const systems::Parameters<T>& parameters) const = 0;

  // NVI for CalcOffsetPoseInBody.
  virtual math::RigidTransform<T> DoCalcOffsetPoseInBody(
      const systems::Parameters<T>& parameters,
      const math::RigidTransform<T>& X_FQ) const {
    return DoCalcPoseInBodyFrame(parameters) * X_FQ;
  }

  // NVI for CalcOffsetRotationMatrixInBody.
  virtual math::RotationMatrix<T> DoCalcOffsetRotationMatrixInBody(
      const systems::Parameters<T>& parameters,
      const math::RotationMatrix<T>& R_FQ) const {
    return DoCalcRotationMatrixInBodyFrame(parameters) * R_FQ;
  }

 private:
  // Implementation for MultibodyElement::DoSetTopology().
  void DoSetTopology() final {
    // Frame gets everything it needs at construction.
  }

  // Implementation for MultibodyElement::DoDeclareParameters().
  void DoDeclareParameters(
      internal::MultibodyTreeSystem<T>* tree_system) final {
    DoDeclareFrameParameters(tree_system);
  }

  // Implementation for MultibodyElement::DoSetDefaultParameters().
  void DoSetDefaultParameters(systems::Parameters<T>* parameters) const final {
    DoSetDefaultFrameParameters(parameters);
  }

  const std::string name_;

  // The link to which this frame is attached.
  const Link<T>& link_;
};

}  // namespace multibody
}  // namespace drake
