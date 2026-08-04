#include "drake/multibody/tree/frame.h"

#include "drake/common/identifier.h"
#include "drake/common/nice_type_name.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {
namespace multibody {
namespace internal {

std::string DeprecateWhenEmptyName(std::string name, std::string_view type) {
  if (name.empty()) {
    throw std::runtime_error(fmt::format(
        "The name parameter to the {} constructor is required.", type));
  }
  return name;
}

}  // namespace internal

template <typename T>
Frame<T>::~Frame() = default;

template <typename T>
ScopedName Frame<T>::scoped_name() const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  return ScopedName(
      this->get_parent_tree().GetModelInstanceName(this->model_instance()),
      name_);
}

// TODO(Mitiguy) The calculation below assumes "this" frame is attached to a
//  rigid body (not a soft body). Modify if soft bodies are possible.
template <typename T>
Vector3<T> Frame<T>::CalcAngularVelocity(
    const systems::Context<T>& context, const Frame<T>& measured_in_frame,
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

// TODO(Mitiguy) The calculation below assumes "this" frame is attached to a
//  rigid body (not a soft body). Modify if soft bodies are possible.
template <typename T>
SpatialVelocity<T> Frame<T>::CalcSpatialVelocityInWorld(
    const systems::Context<T>& context) const {
  const math::RotationMatrix<T>& R_WB =
      body().EvalPoseInWorld(context).rotation();
  const Vector3<T> p_BF_B = GetFixedPoseInBodyFrame().translation();
  const Vector3<T> p_BF_W = R_WB * p_BF_B;
  const SpatialVelocity<T>& V_WB = body().EvalSpatialVelocityInWorld(context);
  const SpatialVelocity<T> V_WF = V_WB.Shift(p_BF_W);
  return V_WF;
}

template <typename T>
SpatialVelocity<T> Frame<T>::CalcSpatialVelocity(
    const systems::Context<T>& context, const Frame<T>& frame_M,
    const Frame<T>& frame_E) const {
  const math::RotationMatrix<T> R_WM =
      frame_M.CalcRotationMatrixInWorld(context);
  const Vector3<T> p_MF_M = this->CalcPose(context, frame_M).translation();
  const Vector3<T> p_MF_W = R_WM * p_MF_M;
  const SpatialVelocity<T> V_WM_W = frame_M.CalcSpatialVelocityInWorld(context);
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

// TODO(Mitiguy) The calculation below assumes "this" frame is attached to a
//  rigid body (not a soft body). Modify if soft bodies are possible.
template <typename T>
SpatialAcceleration<T> Frame<T>::CalcSpatialAccelerationInWorld(
    const systems::Context<T>& context) const {
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

template <typename T>
SpatialAcceleration<T> Frame<T>::CalcSpatialAcceleration(
    const systems::Context<T>& context, const Frame<T>& measured_in_frame,
    const Frame<T>& expressed_in_frame) const {
  const Frame<T>& frame_M = measured_in_frame;
  const Frame<T>& frame_E = expressed_in_frame;

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
    DRAKE_ASSERT(this->has_parent_tree());
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

template <typename T>
std::unique_ptr<Frame<T>> Frame<T>::ShallowClone() const {
  std::unique_ptr<Frame<T>> result = DoShallowClone();
  DRAKE_THROW_UNLESS(result != nullptr);
  return result;
}

template <typename T>
std::unique_ptr<Frame<T>> Frame<T>::DoShallowClone() const {
  throw std::logic_error(fmt::format("{} failed to override DoShallowClone()",
                                     NiceTypeName::Get(*this)));
}

}  // namespace multibody
}  // namespace drake

// Ideally, we'd be instantiating the entire class here, instead of just one
// member function. However, the MultibodyTree physical design is so contrary to
// GSG best practices that trying to do the entire class here doesn't work.
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&drake::multibody::Frame<T>::scoped_name,
     &drake::multibody::Frame<T>::CalcAngularVelocity,
     &drake::multibody::Frame<T>::CalcSpatialVelocityInWorld,
     &drake::multibody::Frame<T>::CalcSpatialVelocity,
     &drake::multibody::Frame<T>::CalcSpatialAccelerationInWorld,
     &drake::multibody::Frame<T>::CalcSpatialAcceleration));

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::Frame);
