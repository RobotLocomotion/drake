#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

using math::RigidTransform;
using math::RotationMatrix;

template <typename T>
LinearBushingRollPitchYaw<T>::LinearBushingRollPitchYaw(
    const Frame<T>& frameA, const Frame<T>& frameC,
    const Vector3<double>& torque_stiffness_constants,
    const Vector3<double>& torque_damping_constants,
    const Vector3<double>& force_stiffness_constants,
    const Vector3<double>& force_damping_constants)
    : LinearBushingRollPitchYaw(frameC.model_instance(),
                                frameA.index(), frameC.index(),
                                torque_stiffness_constants,
                                torque_damping_constants,
                                force_stiffness_constants,
                                force_damping_constants) {}

template <typename T>
LinearBushingRollPitchYaw<T>::LinearBushingRollPitchYaw(
    ModelInstanceIndex model_instance,
    FrameIndex frameA_index, FrameIndex frameC_index,
    const Vector3<double>& torque_stiffness_constants,
    const Vector3<double>& torque_damping_constants,
    const Vector3<double>& force_stiffness_constants,
    const Vector3<double>& force_damping_constants)
    : ForceElement<T>(model_instance),
      frameA_index_(frameA_index),
      frameC_index_(frameC_index),
      torque_stiffness_constants_(torque_stiffness_constants),
      torque_damping_constants_(torque_damping_constants),
      force_stiffness_constants_(force_stiffness_constants),
      force_damping_constants_(force_damping_constants) {
  DRAKE_THROW_UNLESS(torque_stiffness_constants.minCoeff() >= 0);
  DRAKE_THROW_UNLESS(torque_damping_constants.minCoeff() >= 0);
  DRAKE_THROW_UNLESS(force_stiffness_constants.minCoeff() >= 0);
  DRAKE_THROW_UNLESS(force_damping_constants.minCoeff() >= 0);
}

template <typename T>
SpatialForce<T> LinearBushingRollPitchYaw<T>::CalcBushingSpatialForceOnFrameA(
    const systems::Context<T>& context) const {
  // Reminder: The set of all forces applied by the bushing to frame A are
  // replaced by the set's resultant force −𝐟 applied to point Ap of frame A
  // together with a torque −𝐭 equal to the moment of the set about point Ap.

  // Calculate the bushing torque −𝐭 on frame A, expressed in frame A.
  const Vector3<T> t_Ap_A = -CalcBushingTorqueOnCExpressedInA(context);

  // Calculate the bushing force −𝐟 on point Ap of A, expressed in frame A.
  const math::RotationMatrix<T> R_AB = CalcR_AB(context);
  const Vector3<T> f_Ap_B = -CalcBushingNetForceOnCExpressedInB(context);
  const Vector3<T> f_Ap_A = R_AB * f_Ap_B;

  // Form the spatial force for point Ap of A expressed in A.
  const SpatialForce<T> F_Ap_A(t_Ap_A, f_Ap_A);

  // Shift the spatial force from point Ap of A to point Ao of A.
  // Note: Point Ap is the point of frame A that is coincident with both Bo
  // and Cp and is located midway between Ao (A's origin) and Co (C's origin).
  const Vector3<T> p_ApAo_B = -0.5 * Calcp_AoCo_B(context);
  const Vector3<T> p_ApAo_A = R_AB * p_ApAo_B;
  const SpatialForce<T> F_Ao_A = F_Ap_A.Shift(p_ApAo_A);
  return F_Ao_A;
}

template <typename T>
SpatialForce<T> LinearBushingRollPitchYaw<T>::CalcBushingSpatialForceOnFrameC(
    const systems::Context<T>& context) const {
  // Reminder: The set of forces on C from the bushing can be replaced by a
  // force 𝐟 at point CAo (the point of C coincident with Ao) together with a
  // torque t_CAo equal to the moment of all bushing forces on C about CAo.
  // Force 𝐟 and torque t_CAo are negative of the bushing's force/torque on A.
  const SpatialForce<T> F_CAo_A = -CalcBushingSpatialForceOnFrameA(context);
  const Vector3<T> p_AoCo_A = Calcp_AoCo_A(context);

  // Shift the spatial force from point CAo of C to point Co of C.
  const SpatialForce<T> F_Co_A = F_CAo_A.Shift(p_AoCo_A);

  // Form and return F_Co_C by expressing the spatial force F_Co_A in frame C.
  const math::RotationMatrix<T> R_CA = CalcR_AC(context).inverse();
  return R_CA * F_Co_A;
}

template <typename T>
void LinearBushingRollPitchYaw<T>::DoCalcAndAddForceContribution(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */,
    MultibodyForces<T>* forces) const {

  // Form F_Ao_A, the spatial force at point Ao of frame A due to the bushing.
  const SpatialForce<T> F_Ao_A = CalcBushingSpatialForceOnFrameA(context);

  // Form F_Ao_W by expressing F_Ao_A in the world frame W.
  const RotationMatrix<T> R_WA = frameA().CalcRotationMatrixInWorld(context);
  const SpatialForce<T> F_Ao_W = R_WA * F_Ao_A;

  // The next calculation needs the position from Aₒ (frame A's origin) to L0ₒ
  // (the origin of link L0), expressed in the world frame W.
  const RigidTransform<T>& X_WL0 = link0().EvalPoseInWorld(context);
  const RotationMatrix<T>& R_WL0 = X_WL0.rotation();
  const Vector3<T> p_L0Ao_L0 =
      frameA().CalcPoseInBodyFrame(context).translation();
  const Vector3<T> p_AoL0_W = -(R_WL0 * p_L0Ao_L0);

  // Form the spatial force F_L0_W by shifting the spatial force F_Ao_W from
  // Aₒ (A's origin) to L0ₒ (link L0's origin).
  const SpatialForce<T> F_L0_W = F_Ao_W.Shift(p_AoL0_W);

  // The next calculation needs the position from L0 to L1 expressed in world.
  const Vector3<T>& p_WoL0_W = X_WL0.translation();
  const Vector3<T>& p_WoL1_W = link1().EvalPoseInWorld(context).translation();
  const Vector3<T> p_L0L1_W = p_WoL1_W - p_WoL0_W;

  // Form the spatial force F_L1_W by shifting the spatial force F_L0_W from L0ₒ
  // (link L0's origin) to L1ₒ (link L1's origin) and negating the result.
  const SpatialForce<T> F_L1_W = -(F_L0_W.Shift(p_L0L1_W));

  // Alias to the array of spatial forces applied to each link (body).
  std::vector<SpatialForce<T>>& F_BodyOrigin_W_array =
      forces->mutable_body_forces();

  // Apply a torque to link L0 and apply the force −𝐟 to L0ₒ.
  // Apply a torque to link L1 and apply the force +𝐟 to L1ₒ.
  F_BodyOrigin_W_array[link0().node_index()] += F_L0_W;
  F_BodyOrigin_W_array[link1().node_index()] += F_L1_W;
}

template <typename T>
math::RotationMatrix<T> LinearBushingRollPitchYaw<T>::CalcR_AB(
    math::RotationMatrix<T> R_AC) {
  const Eigen::Quaternion<T> q_AC = R_AC.ToQuaternion();
  const T q0 = q_AC.w(), q1 = q_AC.x(), q2 = q_AC.y(), q3 = q_AC.z();
  // ----------------------------------------------------------------------
  // The algorithm below is usually more efficient than calculating the `θ λ`
  // AngleAxis from R_AC and then forming R_AB from the AngleAxis `θ/2 λ`.
  // Conversion from a rotation matrix to AngleAxis usually first converts the
  // rotation matrix to a quaternion and then uses that quaternion with a sqrt
  // and atan2 to convert to AngleAxis.  The algorithm below converts the
  // rotation matrix to a quaternion and then uses a sqrt (not needing atan2).
  // So conversion to AngleAxis has a sqrt and atan2 whereas the algorithm
  // below only has a sqrt.  One may wonder about the cost of the atan2.
  // With typical libraries on common modern hardware, there are estimates
  // that atan2 typically requires ≈4 times the cycles of a sqrt.
  // ----------------------------------------------------------------------
  // The derivation below employs double-angle trigonometric formulas.
  // The quaternion q_AC = [q0 q1 q2 q3] has an associated angle-axis with an
  // angle θ and axis [λx λy λz] which relate to [q0 q1 q2 q3] as follows.
  // q0 = cos(θ/2) = cos(θ/4 + θ/4) = 2*cos²(θ/4) - 1
  //      which can be rearranged to  =>  cos(θ/4) = √(0.5*(q0 + 1)).
  // q1 = λx sin(θ/2) = λx sin(θ/4 + θ/4) = 2 λx sin(θ/4) cos(θ/4)
  //      which can be rearranged to  =>  λx = q1 / (2 sin(θ/4) cos(θ/4) ).
  // q2 = λy sin(θ/2) leads to        =>  λy = q2 / (2 sin(θ/4) cos(θ/4) ).
  // q3 = λz sin(θ/2) leads to        =>  λz = q3 / (2 sin(θ/4) cos(θ/4) ).
  // ----------------------------------------------------------------------
  // Frame B's unit vectors 𝐁𝐱, 𝐁𝐲, 𝐁𝐳 are "halfway" (in an angle-axis sense)
  // between the unit vectors 𝐀𝐱, 𝐀𝐲, 𝐀𝐳 of frame A and 𝐂𝐱, 𝐂𝐲, 𝐂𝐳 of frame C.
  // The quaternion q_AB = [e0 e1 e2 e3] is associated with an angle-axis with
  // angle θ/2 and the same axis [λx λy λz], which relate to [e0 e1 e2 e3] as
  // e0 = cos(θ/4) = √(0.5*(q0 + 1)).
  // e1 = λx sin(θ/4) = q1 / (2 cos(θ/4) ).
  // e2 = λy sin(θ/4) = q2 / (2 cos(θ/4) ).
  // e3 = λz sin(θ/4) = q3 / (2 cos(θ/4) ).
  // ----------------------------------------------------------------------
  using std::sqrt;
  const T e0 = sqrt(0.5 *(q0 + 1));
  // If q0 = −1 the e0 = 0 and the next line has a divide-by-zero error.
  // However, R_AC.ToQuaternion() guarantees q0 >= 0, so sqrt(0.5) <= e0 <= 1
  // which means the angle θₑ in e0 = cos(θₑ/2) has range  0 <= θₑ <= π/2.
  DRAKE_ASSERT(q0 >= 0);
  const T oneOver2e0 = T(1) / (2 * e0);
  const T e1 = q1 * oneOver2e0;
  const T e2 = q2 * oneOver2e0;
  const T e3 = q3 * oneOver2e0;
  const Eigen::Quaternion<T> q_AB(e0, e1, e2, e3);
  const math::RotationMatrix<T> R_AB(q_AB);

  // The next test is useful to verify the algorithm above because a generic
  // unit test is insufficient to test the significant number of variations
  // this algorithm may encounter (although there are also unit tests).
  DRAKE_ASSERT_VOID(ThrowIfInvalidHalfAngleAxis(R_AC, R_AB));

  return R_AB;
}

template <typename T>
void LinearBushingRollPitchYaw<T>::ThrowIfInvalidHalfAngleAxis(
    const math::RotationMatrix<T>& R_AC, const math::RotationMatrix<T>& R_AB) {
  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  const Eigen::AngleAxis<T> angleAxis_AC = R_AC.ToAngleAxis();
  const T half_theta = 0.5 * angleAxis_AC.angle();
  const Eigen::AngleAxis<T> angleAxis_AB(half_theta, angleAxis_AC.axis());
  const math::RotationMatrix<T> R_AB_expected(angleAxis_AB);
  if (!R_AB.IsNearlyEqualTo(R_AB_expected, 64 * kEpsilon)) {
    throw std::runtime_error(
        "Error: Calculation of R_AB from quaternion differs from the "
        "R_AB_expected formed via a half-angle axis calculation.");
  }
}

template <typename T>
Vector3<T> LinearBushingRollPitchYaw<T>::CalcBushing_xyzDt(
    const systems::Context<T>& context) const {
  // Calculate V_AC_A, frame C's spatial velocity in frame A, expressed in A.
  const SpatialVelocity<T> V_AC_A = frameC().CalcSpatialVelocity(context,
      frameA(), frameA());
  const Vector3<T>& w_AC_A = V_AC_A.rotational();
  const Vector3<T>& v_ACo_A = V_AC_A.translational();
  const Vector3<T> w_AB_A = 0.5 * w_AC_A;
  const Vector3<T> p_AoCo_A = Calcp_AoCo_A(context);

  // Calculate the time-derivative in frame B of p_AoCo (derivation below).
  // The results of this calculation is a vector expressed in frame A.
  // v_ACo = DtA_p_AoCo                  (definition)
  //       = DtB_p_AoCo + w_AB x p_AoCo  (Golden rule for vector derivatives)
  // DtB_p_AoCo = v_ACo − w_AB x p_AoCo  (rearrange previous line).
  const Vector3<T> DtB_p_AoCo_A = v_ACo_A - w_AB_A.cross(p_AoCo_A);
  // The previous line is the time-derivative in frame B of p_AoCo, where
  // the resulting vector happens to be expressed in frame A.

  // Form the time-derivative in frame B of p_AoCo, expressed in frame B.
  const math::RotationMatrix<T> R_BA = CalcR_AB(context).inverse();
  const Vector3<T> DtB_p_AoCo_B = R_BA * DtB_p_AoCo_A;
  return DtB_p_AoCo_B;  // This vector derivative happens to be [ẋ, ẏ, ż]ʙ.
}

template <typename T>
Vector3<T> LinearBushingRollPitchYaw<T>::CalcBushingTorqueOnCExpressedInA(
    const systems::Context<T>& context) const {
  const Vector3<T> tau = CalcBushingTorqueTau(context);
  // The set of forces on frame C from the bushing is equivalent to a
  // torque 𝐭 on frame C and a force 𝐟 applied to a point Cp of C.
  // The set of forces on frame A from the bushing is equivalent to a
  // torque −𝐭 on frame A and a force −𝐟 applied to a point Ap of A.
  // Points Ap and Cp are coincident and located halfway between Aₒ and Cₒ.
  // ------------------------------------------------------------------------
  // This method calculates the torque `𝐭 = t_Cp_A = tx 𝐀𝐱 + ty 𝐀𝐲 + tz 𝐀𝐳`
  // that the bushing applies to frame C.  In monogram notation, 𝐭 is computed
  // as t_Cp_A = Nᵀ τ where the N matrix arises from q̇ = N w_AC_A, whereas in
  // matrix form, this relationship is
  // ⌈ tx ⌉       ⌈ τ₀ ⌉                       ⌈ q̇₀ ⌉     ⌈ ωx ⌉
  // | ty |  = Nᵀ | τ₁ |  where N arises from  | q̇₁ | = N | ωy |
  // ⌊ tz ⌋ᴀ      ⌊ τ₂ ⌋                       ⌊ q̇₂ ⌋     ⌊ ωz ⌋ᴀ
  // ------------------------------------------------------------------------
  // The expressions for tx, ty, tz in terms of τ₀, τ₁, τ₂ is derived below by
  // equating the power `𝐭 ⋅ w_AC_A = tx ωx + ty ωy + tz ωz` of torque 𝐭 to
  // the power `τ₀ q̇₀ + τ₁ q̇₁ + τ₂ q̇₂` of the three spring-damper "gimbal"
  // torques τ₂ 𝐀𝐳, τ₁ 𝐏𝐲, τ₀ 𝐂𝐱 (each of 𝐀𝐳, 𝐏𝐲, 𝐂𝐱 are associated with
  // a frame in the yaw-pitch-roll rotation sequence, where 𝐏𝐲 denotes a
  // unit vector of the "pitch" intermediate frame).
  // ------------------------------------------------------------------------
  // Power = [τ₀ τ₁ τ₂]⌈ q̇₀ ⌉ = [τ₀ τ₁ τ₂] N ⌈ ωx ⌉ =  [tx ty tz] ⌈ ωx ⌉
  //                   | q̇₁ |                | ωy |               | ωy |
  //                   ⌊ q̇₂ ⌋                ⌊ ωz ⌋               ⌊ ωz ⌋
  // which is true in view of the transpose of `[tx ty tz] = [τ₀ τ₁ τ₂] N`.
  // ------------------------------------------------------------------------

  // Calculate the matrix N that relates q̇₀, q̇₁, q̇₂ to ωx, ωy, ωz, where frame
  // C's angular velocity in A is expressed `w_AC_A = ωx 𝐀𝐱 + ωy 𝐀𝐲 + ωz 𝐀𝐳`.
  // The calculation of N is documented in the class math::RollPitchYaw.
  const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
  const Matrix3<T> N = rpy.CalcMatrixRelatingRpyDtToAngularVelocityInParent();

  // Form `𝐭 = t_Cp_A = [tx ty tz]ᴀ` which is the torque required when the
  // bushing forces on C have their resultant force 𝐟 applied at Cp (not Co).
  const Vector3<T> t_Cp_A = N.transpose() * tau;
  return t_Cp_A;  // [tx ty tz]ᴀ
}

template <typename T>
void LinearBushingRollPitchYaw<T>::ThrowPitchAngleViolatesGimbalLockTolerance(
    const T& pitch_angle, const char* function_name) {
    const double pitch_radians = ExtractDoubleOrThrow(pitch_angle);
    const double pitch_tolerance =
        math::RollPitchYaw<double>::GimbalLockPitchAngleTolerance();
    std::string message = fmt::format("LinearBushingRollPitchYaw::{}():"
        " Pitch angle p = {:G} degrees is within {:G} degrees of gimbal-lock"
        " which means p ≈ (n*π ± π/2) where n = 0, 1, 2, ..."
        " There is a divide-by-zero error (singularity) at gimbal-lock due to"
        " this bushing's mathematical dependence on roll-pitch-yaw angles."
        " A pitch angle near gimbal-lock cause numerical inaccuracies.  To"
        " avoid this pitch angle problem, use a reasonable default alignment of"
        " the frames associated with this bushing and/or choose stiffness and"
        " damping properties that help avoid pitch angles near gimbal lock.",
        function_name, pitch_radians * 180 / M_PI, pitch_tolerance);
    throw std::runtime_error(message);
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcPotentialEnergy(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& /* pc */) const {
  // TODO(Mitiguy) Per issues #12982 and #12752, implement this method.
  //  Currently this method has not been implemented and throws an exception.
  throw std::runtime_error(
      "Error: LinearBushingRollPitchYaw::CalcPotentialEnergy() "
      "has not been implemented.  Related: Issues #12982 and #12752.");
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcConservativePower(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */) const {
  // TODO(Mitiguy) Per issues #12982 and #12752, implement the following method.
  //  Currently this method has not been implemented and throws an exception.
  throw std::runtime_error(
      "Error: LinearBushingRollPitchYaw::CalcConservativePower() "
      "has not been implemented.  Related: Issues #12982 and #12752.");
}

template <typename T>
T LinearBushingRollPitchYaw<T>::CalcNonConservativePower(
    const systems::Context<T>&,
    const internal::PositionKinematicsCache<T>& /* pc */,
    const internal::VelocityKinematicsCache<T>& /* vc */) const {
  // TODO(Mitiguy) Per issues #12982 and #12752, implement the following method.
  //  Currently this method has not been implemented and throws an exception.
  throw std::runtime_error(
      "Error: LinearBushingRollPitchYaw::CalcNonConservativePower() "
      "has not been implemented.  Related: Issues #12982 and #12752.");
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<ForceElement<ToScalar>>
LinearBushingRollPitchYaw<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>&) const {
  const Vector3<double>& k012 = torque_stiffness_constants();
  const Vector3<double>& d012 = torque_damping_constants();
  const Vector3<double>& kxyz = force_stiffness_constants();
  const Vector3<double>& dxyz = force_damping_constants();

  // The declaration <typename U> friend class LinearBushingRollPitchYaw
  // is needed to facilitate the _private_ use of constructor below.
  std::unique_ptr<LinearBushingRollPitchYaw<ToScalar>> bushing_clone(
      new LinearBushingRollPitchYaw<ToScalar>(this->model_instance(),
          frameA_index_, frameC_index_, k012, d012, kxyz, dxyz));

  return bushing_clone;
}

template <typename T>
std::unique_ptr<ForceElement<double>>
LinearBushingRollPitchYaw<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>>
LinearBushingRollPitchYaw<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<ForceElement<symbolic::Expression>>
LinearBushingRollPitchYaw<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::LinearBushingRollPitchYaw)
