#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/common/autodiff.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/nice_type_name.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/tree/frame_base.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

namespace internal {
std::string DeprecateWhenEmptyName(std::string name, std::string_view type);
}  // namespace internal

// Forward declarations.
template<typename T> class Body;

/// %Frame is an abstract class representing a _material frame_ (also called a
/// _physical frame_), meaning that the %Frame's origin is a material point of
/// a Body.
///
/// An important characteristic of a %Frame is that forces or torques applied
/// to a %Frame are applied to the %Frame's underlying Body. Force-producing
/// elements like joints, actuators, and constraints usually employ two %Frames,
/// with one %Frame connected to one body and the other connected to a different
/// Body. Every %Frame object can report the Body to which it is attached.
/// Despite its name, %Frame is not the most general frame in Drake
/// (see FrameBase for more information).
///
/// A %Frame's pose in World (or relative to other frames) is always calculated
/// starting with its pose relative to its underlying %Body's BodyFrame.
/// Subclasses derived from %Frame differ in how kinematic calculations are
/// performed. For example, the angular velocity of a FixedOffsetFrame or
/// BodyFrame is identical to the angular velocity of its underlying body,
/// whereas the translational velocity of a FixedOffsetFrame differs from that
/// of a BodyFrame. If a %Frame is associated with a soft body, kinematic
/// calculations can depend on the soft body's deformation state variables.
///
/// A %Frame object does _not_ store a Context (where Context means state that
/// contains the %Frame's current orientation, position, motion, etc.).
/// Instead, %Frame provides methods for calculating these %Frame-properties
/// from a Context passed to %Frame methods.
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

  /// Returns true if `this` is the world frame.
  bool is_world_frame() const {
    return this->index() == world_frame_index();
  }

  /// Returns true if `this` is the body frame.
  bool is_body_frame() const {
    return this->index() == body_.body_frame().index();
  }

  /// Returns the name of this frame. The name will never be empty.
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
  /// @throws std::exception if called on a %Frame that does not have a
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
  /// @throws std::exception if `this` frame F is a %Frame that does not have
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
  /// @throws std::exception if called on a %Frame that does not have a
  /// fixed offset in the body frame.
  virtual math::RigidTransform<T> GetFixedOffsetPoseInBody(
      const math::RigidTransform<T>& X_FQ) const {
    return GetFixedPoseInBodyFrame() * X_FQ;
  }

  /// Calculates and returns the rotation matrix `R_BQ` that relates body frame
  /// B to frame Q via `this` intermediate frame F, i.e., `R_BQ = R_BF * R_FQ`
  /// (B is the body frame to which `this` frame F is attached).
  /// @param[in] R_FQ rotation matrix that relates frame F to frame Q.
  /// @throws std::exception if `this` frame F is a %Frame that does not have
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

  /// Evaluates `this` frame F's angular velocity measured and expressed in the
  /// world frame W.
  /// @param[in] context contains the state of the multibody system.
  /// @return ω_WF_W (frame F's angular velocity ω measured and expressed in
  /// the world frame W).
  /// @see CalcAngularVelocity() to calculate ω_MF_E (`this` frame F's angular
  /// velocity ω measured in a frame M and expressed in a frame E).
  const Vector3<T>& EvalAngularVelocityInWorld(
      const systems::Context<T>& context) const {
    // TODO(Mitiguy) The calculation below assumes "this" frame is attached to a
    //  rigid body (not a soft body). Modify if soft bodies are possible.
    const SpatialVelocity<T>& V_WB = body().EvalSpatialVelocityInWorld(context);
    const Vector3<T>& w_WF_W = V_WB.rotational();
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
  Vector3<T> CalcAngularVelocity(
    // TODO(Mitiguy) The calculation below assumes "this" frame is attached to a
    //  rigid body (not a soft body). Modify if soft bodies are possible.
      const systems::Context<T>& context,
      const Frame<T>& measured_in_frame,
      const Frame<T>& expressed_in_frame) const {
    const Frame<T>& frame_M = measured_in_frame;
    const Frame<T>& frame_E = expressed_in_frame;
    const Vector3<T>& w_WF_W = EvalAngularVelocityInWorld(context);
    const Vector3<T>& w_WM_W = frame_M.EvalAngularVelocityInWorld(context);
    const Vector3<T> w_MF_W = w_WF_W - w_WM_W;

    // If the expressed-in frame E is the world, no need to re-express results.
    if (frame_E.is_world_frame()) return w_MF_W;

    const math::RotationMatrix<T> R_WE =
        frame_E.CalcRotationMatrixInWorld(context);
    const math::RotationMatrix<T> R_EW = R_WE.inverse();
    const Vector3<T> w_MF_E = R_EW * w_MF_W;
    return w_MF_E;
  }

  /// Calculates `this` frame F's spatial velocity measured and expressed in
  /// the world frame W.
  /// @param[in] context contains the state of the multibody system.
  /// @return V_WF_W, `this` frame F's spatial velocity measured and expressed
  /// in the world frame W. The rotational part of the returned quantity is
  /// ω_WF_W (frame F's angular velocity ω measured and expressed in the world
  /// frame W). The translational part is v_WFo_W (translational velocity v of
  /// frame F's origin point Fo, measured and expressed in the world frame W).
  /// @note Body::EvalSpatialVelocityInWorld() provides a more efficient way to
  /// obtain a body frame's spatial velocity measured in the world frame.
  /// @see CalcSpatialVelocity(), CalcRelativeSpatialVelocityInWorld(), and
  /// CalcSpatialAccelerationInWorld().
  SpatialVelocity<T> CalcSpatialVelocityInWorld(
      const systems::Context<T>& context) const {
    // TODO(Mitiguy) The calculation below assumes "this" frame is attached to a
    //  rigid body (not a soft body). Modify if soft bodies are possible.
    const math::RotationMatrix<T>& R_WB =
        body().EvalPoseInWorld(context).rotation();
    const Vector3<T> p_BF_B = GetFixedPoseInBodyFrame().translation();
    const Vector3<T> p_BF_W = R_WB * p_BF_B;
    const SpatialVelocity<T>& V_WB = body().EvalSpatialVelocityInWorld(context);
    const SpatialVelocity<T> V_WF = V_WB.Shift(p_BF_W);
    return V_WF;
  }

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
  SpatialVelocity<T> CalcSpatialVelocity(
      const systems::Context<T>& context,
      const Frame<T>& frame_M,
      const Frame<T>& frame_E) const {
    const math::RotationMatrix<T> R_WM =
        frame_M.CalcRotationMatrixInWorld(context);
    const Vector3<T> p_MF_M = this->CalcPose(context, frame_M).translation();
    const Vector3<T> p_MF_W = R_WM * p_MF_M;
    const SpatialVelocity<T> V_WM_W =
        frame_M.CalcSpatialVelocityInWorld(context);
    const SpatialVelocity<T> V_WF_W = this->CalcSpatialVelocityInWorld(context);
    // V_MF is calculated from a rearranged "composition" of spatial velocities.
    // The angular velocity addition theorem  ω_WF = ω_WM + ω_MF
    // rearranges to                          ω_MF = ω_WF - ω_WM
    // The translational velocity formula for a point Fo moving on frame M is
    // v_WFo = v_WMo + ω_WM x p_MoFo + v_MFo   which rearranges to
    // v_MFo = V_WFo - (v_WMo + ω_WM x p_MoFo).
    const SpatialVelocity<T> V_MF_W = V_WF_W - V_WM_W.Shift(p_MF_W);

    // If the expressed-in frame E is the world, no need to re-express results.
    if (frame_E.is_world_frame()) return V_MF_W;

    // Otherwise re-express results from world frame W to frame E.
    const math::RotationMatrix<T> R_WE =
        frame_E.CalcRotationMatrixInWorld(context);
    return R_WE.inverse() * V_MF_W;
  }

  /// Calculates `this` frame C's spatial velocity relative to another frame B,
  /// measured and expressed in the world frame W.
  /// @param[in] context contains the state of the multibody system.
  /// @param[in] other_frame which is frame B.
  /// @return V_W_BC_W = V_WC_W - V_WB_W, frame C's spatial velocity relative to
  /// frame B, measured and expressed in the world frame W. The rotational part
  /// of the returned quantity is ω_BC_W (C's angular velocity measured in B and
  /// expressed in W). The translational part is v_W_BoCo_W (Co's translational
  /// velocity relative to Bo, measured and expressed in world frame W). <pre>
  ///     ω_BC_W  = ω_WC_W - ω_WB_W
  ///  v_W_BoCo_W = v_WCo_W - v_WBo_W = DtW(p_BoCo)
  /// </pre>
  /// where DtW(p_BoCo) is the time-derivative in frame W of p_BoCo (position
  /// vector from Bo to Co), and this vector is expressed in frame W.
  /// @note The method CalcSpatialVelocityInWorld() is more efficient and
  /// coherent if any of `this`, other_frame, or the world frame W are the same.
  /// @see CalcSpatialVelocityInWorld() and CalcRelativeSpatialVelocity().
  SpatialVelocity<T> CalcRelativeSpatialVelocityInWorld(
      const systems::Context<T>& context,
      const Frame<T>& other_frame) const {
    const Frame<T>& frame_B = other_frame;
    const SpatialVelocity<T> V_WB_W =
        frame_B.CalcSpatialVelocityInWorld(context);
    const SpatialVelocity<T> V_WC_W = CalcSpatialVelocityInWorld(context);
    return V_WC_W - V_WB_W;
  }

  /// Calculates `this` frame C's spatial velocity relative to another frame B,
  /// measured in a frame M, expressed in a frame E.
  /// @param[in] context contains the state of the multibody system.
  /// @param[in] other_frame which is frame B.
  /// @param[in] measured_in_frame which is frame M.
  /// @param[in] expressed_in_frame which is frame E.
  /// @return V_M_BC_E = V_MC_E - V_MB_E, frame C's spatial velocity relative to
  /// frame B, measured in frame M, expressed in frame E. The rotational part
  /// of the returned quantity is ω_BC_E (C's angular velocity measured in B and
  /// expressed in E). The translational part is v_M_BoCo_E (Co's translational
  /// velocity relative to Bo, measured in M, and expressed in E). <pre>
  ///  ω_BC_E = ω_MC_E - ω_MB_E
  ///  v_M_BoCo_E = v_MCo_E - v_MBo_E = DtM(p_BoCo)
  /// </pre>
  /// where DtM(p_BoCo) is the time-derivative in frame M of p_BoCo (position
  /// vector from Bo to Co), and this vector is expressed in frame E.
  /// @note The method CalcSpatialVelocity() is more efficient and coherent
  /// if any of `this`, other_frame, or measured_in_frame are the same.
  /// Also, the value of V_M_BoCo does not depend on the measured_in_frame if
  /// Bo and Co are coincident (i.e., p_BoCo = 0), in which case consider the
  /// more efficient method CalcRelativeSpatialVelocityInWorld().
  /// Lastly, the calculation of elongation between Bo and Co can be done with
  /// relative translational velocity, but elongation does not depend on the
  /// measured-in-frame (hence consider CalcRelativeSpatialVelocityInWorld()).
  /// @see CalcSpatialVelocityInWorld(), CalcSpatialVelocity(), and
  /// CalcRelativeSpatialVelocityInWorld().
  SpatialVelocity<T> CalcRelativeSpatialVelocity(
      const systems::Context<T>& context,
      const Frame<T>& other_frame,
      const Frame<T>& measured_in_frame,
      const Frame<T>& expressed_in_frame) const {
    const Frame<T>& frame_B = other_frame;
    const Frame<T>& frame_M = measured_in_frame;
    const Frame<T>& frame_E = expressed_in_frame;
    const SpatialVelocity<T> V_MB_E =
        frame_B.CalcSpatialVelocity(context, frame_M, frame_E);
    const SpatialVelocity<T> V_MC_E =
        CalcSpatialVelocity(context, frame_M, frame_E);
    return V_MC_E - V_MB_E;
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
  /// @note Body::EvalSpatialAccelerationInWorld() provides a more efficient way
  /// to obtain a body frame's spatial acceleration measured in the world frame.
  /// @note When cached values are out of sync with the state stored in context,
  /// this method performs an expensive forward dynamics computation, whereas
  /// once evaluated, successive calls to this method are inexpensive.
  /// @see CalcSpatialAcceleration() and CalcSpatialVelocityInWorld().
  SpatialAcceleration<T> CalcSpatialAccelerationInWorld(
      const systems::Context<T>& context) const {
    // TODO(Mitiguy) The calculation below assumes "this" frame is attached to a
    //  rigid body (not a soft body). Modify if soft bodies are possible.

    // `this` frame_F is fixed to a body B.  Calculate A_WB_W, body B's spatial
    // acceleration in the world frame W, expressed in W.
    const SpatialAcceleration<T>& A_WB_W =
        body().EvalSpatialAccelerationInWorld(context);

    // Optimize for the common case that `this` is body B's frame.
    if (is_body_frame()) return A_WB_W;

    // Shift spatial acceleration A_WB_W from Bo to Fp.
    const math::RotationMatrix<T>& R_WB =
        body().EvalPoseInWorld(context).rotation();
    const Vector3<T> p_BoFo_B = GetFixedPoseInBodyFrame().translation();
    const Vector3<T> p_BoFo_W = R_WB * p_BoFo_B;
    const Vector3<T>& w_WB_W = EvalAngularVelocityInWorld(context);
    const SpatialAcceleration<T> A_WF_W = A_WB_W.Shift(p_BoFo_W, w_WB_W);
    return A_WF_W;
  }

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
      const systems::Context<T>& context,
      const Frame<T>& measured_in_frame,
      const Frame<T>& expressed_in_frame) const {
    const Frame<T> &frame_M = measured_in_frame;
    const Frame<T> &frame_E = expressed_in_frame;

    // A_MF is calculated from a rearranged composition of spatial acceleration.
    // Angular acceleration addition theorem: α_WF = α_WM + α_MF + ω_WM x ω_MF
    // rearranges to                          α_MF = α_WF - α_WM - ω_WM x ω_MF
    // The translational acceleration formula for point Fo moving on frame M is
    // a_WFo = a_WCoincidentPoint + 2 ω_WM x v_MFo + a_MFo   which rearranges to
    // a_MFo = a_WFo - a_WCoincidentPoint - 2 ω_WM x v_MFo,  where
    // a_WCoincidentPoint = a_WMo + α_WM x p_MoFo + ω_WM x (ω_WM x p_MoFo).

    // The first term in the calculated quantity A_MF_W is always A_WF_W (frame
    // F's spatial acceleration measured and expressed in the world frame W).
    const SpatialAcceleration<T> A_WF_W =
        this->CalcSpatialAccelerationInWorld(context);

    // Helper function to calculate A_MF_W when frame M ≠ frame W.
    auto calc_A_MF_W = [this, &context, &frame_M, &A_WF_W]() {
      // Form additional terms for the rotational part of A_MF_W.
      const SpatialAcceleration<T> A_WM_W =
          frame_M.CalcSpatialAccelerationInWorld(context);
      const Vector3<T>& alpha_WM_W = A_WM_W.rotational();
      const Vector3<T>& w_WM_W = frame_M.EvalAngularVelocityInWorld(context);
      const Frame<T>& frame_W = this->get_parent_tree().world_frame();
      const SpatialVelocity<T> V_MF_W =
          CalcSpatialVelocity(context, frame_M, frame_W);
      const Vector3<T>& w_MF_W = V_MF_W.rotational();
      const Vector3<T> alpha_MF_W =  // α_MF = α_WF - α_WM - ω_WM x ω_MF
          A_WF_W.rotational() - alpha_WM_W - w_WM_W.cross(w_MF_W);

      // Form additional terms for the translational part of A_MF_W.
      const math::RotationMatrix<T> R_WM =
          frame_M.CalcRotationMatrixInWorld(context);
      const Vector3<T> p_MoFo_M = CalcPose(context, frame_M).translation();
      const Vector3<T> p_MoFo_W = R_WM * p_MoFo_M;
      const Vector3<T> a_WcoincidentPoint_W =
          A_WM_W.Shift(p_MoFo_W, w_WM_W).translational();
      const Vector3<T>& v_MFo_W = V_MF_W.translational();
      const Vector3<T> coriolis_W = 2 * w_WM_W.cross(v_MFo_W);
      const Vector3<T> a_MFo =  // a_WFo - a_WCoincidentPoint - 2 ω_WM x v_MFo
          A_WF_W.translational() - a_WcoincidentPoint_W - coriolis_W;

      // Form A_MF_W (frame F's spatial acceleration in frame M, measured in W).
      return SpatialAcceleration<T>(alpha_MF_W, a_MFo);
    };

    // Avoid inefficient unnecessary calculations if frame M is the world frame.
    const SpatialAcceleration<T> A_MF_W =
        frame_M.is_world_frame() ? A_WF_W : calc_A_MF_W();

    // If expressed-in frame E is the world, no need to re-express results.
    if (frame_E.is_world_frame()) return A_MF_W;

    // Otherwise re-express results from world frame W to frame E.
    const math::RotationMatrix<T> R_WE =
        frame_E.CalcRotationMatrixInWorld(context);
    return R_WE.inverse() * A_MF_W;  // returns A_MF_E.
  }

  /// Calculates `this` frame C's spatial acceleration relative to another
  /// frame B, measured and expressed in the world frame W.
  /// @param[in] context contains the state of the multibody system.
  /// @param[in] other_frame which is frame B.
  /// @return A_W_BC_W = A_WC_W - A_WB_W, frame C's spatial acceleration
  /// relative to frame B, measured and expressed in the world frame W.
  ///
  /// In general, A_W_BC = DtW(V_W_BC), the time-derivative in the world frame W
  /// of frame C's spatial velocity relative to frame B. The rotational part of
  /// the returned quantity is α_WC_W - α_WB_W = DtW(ω_BC)_W. For 3D analysis,
  /// DtW(ω_BC) ≠ α_BC. The translational part of the returned quantity is
  /// a_W_BoCo_W (Co's translational acceleration relative to Bo, measured and
  /// expressed in world frame W). <pre>
  ///  α_WC_W - α_WB_W = DtW(ω_WC)_W - DtW(ω_WB)_W = DtW(ω_BC)_W
  ///  a_W_BoCo_W = a_WCo_W - a_WBo_W = DtW(v_WCo) - DtW(v_WBo) = Dt²W(p_BoCo)_W
  /// </pre>
  /// where Dt²W(p_BoCo)_W is the 2ⁿᵈ time-derivative in frame W of p_BoCo (the
  /// position vector from Bo to Co), and this result is expressed in frame W.
  /// @note The method CalcSpatialAccelerationInWorld() is more efficient and
  /// coherent if any of `this`, other_frame, or the world frame W are the same.
  /// @see CalcSpatialAccelerationInWorld(), CalcRelativeSpatialAcceleration().
  SpatialAcceleration<T> CalcRelativeSpatialAccelerationInWorld(
      const systems::Context<T>& context,
      const Frame<T>& other_frame) const {
    const Frame<T>& frame_B = other_frame;
    const SpatialAcceleration<T> A_WB_W =
        frame_B.CalcSpatialAccelerationInWorld(context);
    const SpatialAcceleration<T> A_WC_W =
        CalcSpatialAccelerationInWorld(context);
    return A_WC_W - A_WB_W;
  }

  /// Calculates `this` frame C's spatial acceleration relative to another
  /// frame B, measured in a frame M, expressed in a frame E.
  /// @param[in] context contains the state of the multibody system.
  /// @param[in] other_frame which is frame B.
  /// @param[in] measured_in_frame which is frame M.
  /// @param[in] expressed_in_frame which is frame E.
  /// @return A_M_BC_E = A_MC_E - A_MB_E, frame C's spatial acceleration
  /// relative to frame B, measured in frame M, expressed in frame E.
  ///
  /// In general, A_M_BC = DtW(V_M_BC), the time-derivative in frame M of
  /// frame C's spatial velocity relative to frame B. The rotational part of the
  /// returned quantity is α_MC_E - α_MB_E = DtM(ω_BC)_E. Note: For 3D analysis,
  /// DtM(ω_BC) ≠ α_BC. The translational part of the returned quantity is
  /// a_M_BoCo_E (Co's translational acceleration relative to Bo, measured in
  /// frame M, expressed in frame E). <pre>
  ///  α_MC_E - α_MB_E = DtM(ω_MC)_E - DtM(ω_MB)_E = DtM(ω_BC)_E
  ///  a_M_BoCo_E = a_MCo_E - a_MBo_E = DtM(v_MCo) - DtM(v_MBo) = Dt²M(p_BoCo)_E
  /// </pre>
  /// where Dt²M(p_BoCo)_E is the 2ⁿᵈ time-derivative in frame M of p_BoCo (the
  /// position vector from Bo to Co), and this result is expressed in frame E.
  /// @note The calculation of the 2ⁿᵈ time-derivative of the distance between
  /// Bo and Co can be done with relative translational acceleration, but this
  /// calculation does not depend on the measured-in-frame, hence in this case,
  /// consider CalcRelativeSpatialAccelerationInWorld() since it is faster.
  /// @see CalcSpatialAccelerationInWorld(), CalcSpatialAcceleration(), and
  /// CalcRelativeSpatialAccelerationInWorld().
  SpatialAcceleration<T> CalcRelativeSpatialAcceleration(
      const systems::Context<T>& context,
      const Frame<T>& other_frame,
      const Frame<T>& measured_in_frame,
      const Frame<T>& expressed_in_frame) const {
    const Frame<T>& frame_B = other_frame;
    const Frame<T>& frame_M = measured_in_frame;
    const Frame<T>& frame_E = expressed_in_frame;
    const SpatialAcceleration<T> A_MB_E =
        frame_B.CalcSpatialAcceleration(context, frame_M, frame_E);
    const SpatialAcceleration<T> A_MC_E =
        CalcSpatialAcceleration(context, frame_M, frame_E);
    return A_MC_E - A_MB_E;
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
        name_(internal::DeprecateWhenEmptyName(name, "Frame")), body_(body) {}

  DRAKE_DEPRECATED("2022-12-01",
      "The name parameter to the Frame constructor is now required.")
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

  const std::string name_;

  // The body associated with this frame.
  const Body<T>& body_;

  // The internal bookkeeping topology struct used by MultibodyTree.
  internal::FrameTopology topology_;
};

}  // namespace multibody
}  // namespace drake
