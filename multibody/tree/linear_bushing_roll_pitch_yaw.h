#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/force_element.h"

namespace drake {
namespace multibody {

template <typename T> class Body;

/// This %ForceElement models a massless bushing B that connects a frame A of a
/// link L0 to a frame C of a link L1.  The bushing can apply a torque and force
/// due to stiffness (spring) and dissipation (damper) properties.
/// Frame A is regarded as welded to link (body) L0.
/// Frame C is regarded as welded to link (body) L1.
/// Frame B is the bushing frame whose origin Bo is halfway between Ao (A's
/// origin) and Co (C's origin) and whose unit vectors Bx, By, Bz are "halfway"
/// (in an angle-axis sense) between the unit vectors of frame A and frame C.
///
/// The set of forces on frame C from the bushing is equivalent to a
/// torque T on frame C and a force f applied to a point Cp of C.
/// The set of forces on frame A from the bushing is equivalent to a
/// torque −T on frame A and a force −f applied to a point Ap of A.
/// Points Ap and Cp are coincident with Bo (frame B's origin) which is located
/// halfway between point Ao (frame A's origin) and point Co (frame C's origin).
///
/// This "symmetric" bushing force/torque model was developed at Toyota Research
/// Institute and is advantageous compared to traditional bushing models because
/// it employs a bushing-centered "symmetric" frame B and it ensures the moment
/// of −f on A about Ao is equal to the moment of f on C about Co.  Traditional
/// models differ, e.g., not using a "symmetric" frame B and applying −f at Ao,
/// which means the  moment of −f on A about Ao is always zero.
///
/// The torque model depends on spring-damper "gimbal" torques T₀, T₁, T₂ which
/// themselves depend on roll-pitch-yaw angles q₀, q₁, q₂ and rates q̇₀, q̇₁, q̇₂
/// via diagonal torque-stiffness and torque-damping matrices as <pre>
/// ⌈ T₀ ⌉     ⌈k₀    0    0⌉ ⌈ q₀ ⌉     ⌈b₀    0    0⌉ ⌈ q̇₀ ⌉
/// | T₁ | = − | 0   k₁    0| | q₁ |  −  | 0   b₁    0| | q̇₁ |
/// ⌊ T₂ ⌋     ⌊ 0    0   k₂⌋ ⌊ q₂ ⌋     ⌊ 0    0   b₂⌋ ⌊ q̇₂ ⌋
/// </pre>
/// where k₀, k₁, k₂ and b₀, b₁, b₂ are torque stiffness and damping constants.
/// Note: As discussed in the Advanced section below, the aforementioned torque
/// T is not the array of gimbal torques, i.e., `T ≠ ⌈T₀ T₁ T₂⌉ᵀ`.
///
/// The symmetric bushing model for f depends on x, y, z, which are defined so
/// the position from Ao to Co is `p_AoCo_B = x Bx + y By + z Bz`.  The model
/// for f uses a diagonal force-stiffness matrix, a diagonal force-damping
/// matrix, and defines fx, fy, fz such that `f = fx Bx + fy By + fz Bz`. <pre>
/// ⌈ fx ⌉       ⌈kx    0    0⌉ ⌈ x ⌉     ⌈bx    0    0⌉ ⌈ ẋ ⌉
/// | fy |  =  − | 0   ky    0| | y |  −  | 0   by    0| | ẏ |
/// ⌊ fz ⌋ʙ      ⌊ 0    0   kz⌋ ⌊ z ⌋     ⌊ 0    0   bz⌋ ⌊ ż ⌋
/// </pre>
/// where kx, ky, kz and bx, by, bz are force stiffness and damping constants.
///
/// <b>Advanced:</b> The torque model uses spring-damper "gimbal" torques
/// `T₀ Cx`, `T₁ Py`, `T₂ Az`, where each of Cx, Py, Az are associated with a
/// frame in the roll-pitch-yaw rotation sequence and `Py` denotes a unit vector
/// of the "pitch" intermediate frame.  As shown in code documentation, the
/// torque `T = Tx Ax + Ty Ay + Tz Az` differs from the gimbal torques as <pre>
/// ⌈ Tx ⌉       ⌈ T₀ ⌉             ⌈ cos(q₂)/cos(q₁)  sin(q₂)/cos(q₁)   0 ⌉
/// | Ty |  = Nᵀ | T₁ |   where N = |   −sin(q2)            cos(q2)      0 |
/// ⌊ Tz ⌋ᴀ      ⌊ T₂ ⌋             ⌊ cos(q₂)*tan(q₁)   sin(q₂)*tan(q₁)  1 ⌋
/// </pre>
/// Angles q₀, q₁, q₂ are calculated from frame C's orientation relative to
/// frame A, with [`−π < q₀ <= π`, `−π/2 <= q₁ <= π/2`, `−π < q₂ <= π`].
/// Torque T can be discontinuous if one of q₀, q₁, q₂ or q̇₀, q̇₁, q̇₂ is
/// discontinuous and its associated torque spring/damper constant is nonzero.
/// For example, this can occur if `k₂ ≠ 0` and the bushing has a large rotation
/// such that q₂ jumps from `≈ −π to π` or if `b₀ ≠ 0` and q̇₀ is undefined
/// (which occurs when `q₁ = π/2`).
///
/// @note The complete theory for this bushing is documented in the source code.
/// Please look there if you want more information.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// @see math::RollPitchYaw for definitions of roll, pitch, yaw `[q₀, q₁, q₂]`.
template <typename T>
class LinearBushingRollPitchYaw final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearBushingRollPitchYaw)

  /// Construct a LinearBushingRollPitchYaw B that connects frames A and C,
  /// where frame A is welded to a link L0 and frame C is welded to a link L1.
  /// @param[in] frameA frame A of link L0 that connects to bushing B.
  /// @param[in] frameC frame C of link L1 that connects to bushing B.
  /// @param[in] torque_stiffness_constants `[k₀, k₁, k₂]` multiply the
  /// roll-pitch-yaw angles `[q₀, q₁, q₂]` to produce the spring portion of the
  /// "gimbal" torques T₀, T₁, T₂. The SI units of `k₀, k₁, k₂` are N*m/rad.
  /// @param[in] torque_damping_constants `[b₀, b₁, b₂]` multiply the
  /// roll-pitch-yaw rates `[q̇₀, q̇₁, q̇₂]` to produce the damper portion of the
  /// "gimbal" torques T₀, T₁, T₂.  The SI units of `b₀, b₁, b₂` are N*m/rad.
  /// @param[in] force_stiffness_constants `[kx, ky, kz]` multiply the
  /// bushing displacements `[x, y, z]` to form the spring portion of the
  /// force measures [fx fy fz].  The SI units of `kx, ky, kz` are N/m.
  /// @param[in] force_damping_constants `[bx, by, bz]` multiply the
  /// bushing displacement rates `[ẋ, ẏ, ż]` to form the damper portion of the
  /// force measures [fx fy fz].  The SI units of `bx, by, bz` are N*s/m.
  /// @note The LinearBushingRollPitchYaw class documentation describes the
  /// stiffness and damping constants.
  /// @note The net moment on A about Ao is affected by both the gimbal torque
  /// and the moment of f about Ao. Similarly, for the net moment on C about Co.
  /// @note math::RollPitchYaw describes the roll pitch yaw angles q₀, q₁, q₂.
  /// @note The position from Ao to Co is `p_AoCo_B = x Bx + y By + z Bz`.
  /// @pre All the stiffness and damping constants must be non-negative.
  LinearBushingRollPitchYaw(const Frame<T>& frameA,
                            const Frame<T>& frameC,
                            const Vector3<double>& torque_stiffness_constants,
                            const Vector3<double>& torque_damping_constants,
                            const Vector3<double>& force_stiffness_constants,
                            const Vector3<double>& force_damping_constants);

  /// Returns link L0, which is the body attached to frame A.
  const Body<T>& link0() const { return frameA_.body(); }

  /// Returns link L1, which is the body attached to frame C.
  const Body<T>& link1() const { return frameC_.body(); }

  /// Returns frame A, which is the frame welded to link L0 and attached to the
  /// bushing.
  const Frame<T>& frameA() const { return frameA_; }

  /// Returns frame C, which is the frame welded to link L1 and attached to the
  /// bushing.
  const Frame<T>& frameC() const { return frameC_; }

  /// Returns the torque stiffness constants [k₀, k₁, k₂].
  const Vector3<double>& torque_stiffness_constants() const {
    return torque_stiffness_constants_;
  }

  /// Returns the torque damping constants [b₀, b₁, b₂].
  const Vector3<double>& torque_damping_constants() const {
    return torque_damping_constants_;
  }

  /// Returns the force stiffness constants [kx, ky, kz].
  const Vector3<double>& force_stiffness_constants() const {
    return force_stiffness_constants_;
  }

  /// Returns the force damping constants [bx, by, bz].
  const Vector3<double>& force_damping_constants() const {
    return force_damping_constants_;
  }

  /// Calculate F_A_A, the bushing's spatial force on frame A expressed in A.
  /// F_A_A contains two vectors: the moment of all bushing forces on A about Ao
  /// (−T + p_AoAp × −f) and the net bushing force on A (−f expressed in A).
  /// @param[in] context The state of the multibody system.
  /// @see CalcBushingSpatialForceOnFrameC().
  SpatialForce<T> CalcBushingSpatialForceOnFrameA(
      const systems::Context<T>& context) const {
    // TODO(Mitiguy) move most of the code in this .h file to its .cc file.
    // Reminder: The set of all forces applied by the bushing to frame A are
    // replaced by the set's resultant force applied to point Ap of frame A
    // together with a torque equal to the moment of the set about point Ap.

    // Calculate the bushing torque on frame A, expressed in frame A.
    const Vector3<T> t_A_A = -CalcBushingTorqueOnCExpressedInA(context);

    // Calculate the net bushing force on point Ap of A, expressed in frame A.
    const math::RotationMatrix<T> R_AB = CalcR_AB(context);
    const Vector3<T> f_Ap_B = -CalcBushingNetForceOnCExpressedInB(context);
    const Vector3<T> f_Ap_A = R_AB * f_Ap_B;

    // Form the spatial force for point Ap of A expressed in A,
    const SpatialForce<T> F_Ap_A(t_A_A, f_Ap_A);

    // Shift the spatial force from point Ap of A to point Ao of A.
    // Reminder: Point Ap is the point of frame A that is coincident with Bo and
    // Cp and is located halfway between Ao (A's origin) and Co (C's origin).
    const Vector3<T> p_ApAo_B = -0.5 * Calcp_AoCo_B(context);
    const Vector3<T> p_ApAo_A = R_AB * p_ApAo_B;
    const SpatialForce<T> F_Ao_A = F_Ap_A.Shift(p_ApAo_A);
    return F_Ao_A;
  }

  /// Calculate F_C_C, the bushing's spatial force on frame C expressed in C.
  /// F_C_C contains two vectors: the moment of all bushing forces on C about Co
  /// (T + p_CoCp × f) and the resultant bushing force on C (f expressed in C).
  /// @param[in] context The state of the multibody system.
  /// @see CalcBushingSpatialForceOnFrameA().
  SpatialForce<T> CalcBushingSpatialForceOnFrameC(
      const systems::Context<T>& context) const {
    // Reminder: The set of forces on C from the bushing can be replaced by a
    // force f at point C̅ (the point of C coincident with Ao) together with a
    // torque T̅ equal to the moment of all forces from the bushing on C about C̅.
    // Force f and torque T̅ are the negative of the bushing's force/torque on A.
    const SpatialForce<T> F_Cbar_A = -CalcBushingSpatialForceOnFrameA(context);
    const Vector3<T> p_CbarCo_A = Calcp_AoCo_A(context);

    // Shift the spatial force from point C̅ of C to point Co of C.
    const SpatialForce<T> F_Co_A = F_Cbar_A.Shift(p_CbarCo_A);

    // Form and return F_Co_C by expressing the spatial force F_Co_A in frame C.
    const math::RotationMatrix<T> R_CA = CalcR_AC(context).inverse();
    return R_CA * F_Co_A;
  }

 private:
  T CalcPotentialEnergy(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc) const override;

  T CalcConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const override;

  T CalcNonConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const override;

  void DoCalcAndAddForceContribution(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc,
      MultibodyForces<T>* forces) const override;

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<ForceElement<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override;

  // Calculate R_AC, the rotation matrix that relates frames A and C.
  // @param[in] context The state of the multibody system.
  math::RotationMatrix<T> CalcR_AC(const systems::Context<T>& context) const {
    return frameC().CalcRotationMatrix(context, frameA());
  }

  // Calculate R_AB, the rotation matrix that relates frames A and B.
  // @param[in] context The state of the multibody system.
  math::RotationMatrix<T> CalcR_AB(const systems::Context<T>& context) const {
    const math::RotationMatrix<T> R_AC = CalcR_AC(context);
    const Eigen::Quaternion<T> q_AC = R_AC.ToQuaternion();
    const T q0 = q_AC.w(), q1 = q_AC.x(), q2 = q_AC.y(), q3 = q_AC.z();
    //-----------------------------------------------------------------------
    // The derivation below employs double-angle trigonometric formulas.
    // The q_AC quaternion [q0 q1 q2 q3] has an associated angle-axis with an
    // angle θ and axis [λx λy λz] which relate to [q0 q1 q2 q3] as follows.
    // q0 = cos(θ/2) = cos(θ/4 + θ/4) = 2*cos²(θ/4) - 1
    //      which can be rearranged to  =>  cos(θ/4) = √(0.5*(q0 + 1)).
    // q1 = λx sin(θ/2) = λx sin(θ/4 + θ/4) = 2 λx sin(θ/4) cos(θ/4)
    //      which can be rearranged to  =>  λx = q1 / (2 sin(θ/4) cos(θ/4) ).
    // q2 = λy sin(θ/2) leads to        =>  λy = q2 / (2 sin(θ/4) cos(θ/4) ).
    // q3 = λz sin(θ/2) leads to        =>  λz = q3 / (2 sin(θ/4) cos(θ/4) ).
    //-----------------------------------------------------------------------
    // Frame B's unit vectors Bx, By, Bz are "halfway" (in an angle-axis sense)
    // between the unit vectors Ax, Ay, Az of frame A and Cx, Cy, Cz of frame C.
    // The q_AB quaternion [e0 e1 e2 e3] is associated with an angle-axis with
    // angle θ/2 and the same axis [λx λy λz], which relate to [e0 e1 e2 e3] as
    // e0 = cos(θ/4) = √(0.5*(q0 + 1)).
    // e1 = λx sin(θ/4) = q1 / (2 cos(θ/4) ).
    // e2 = λy sin(θ/4) = q2 / (2 cos(θ/4) ).
    // e3 = λz sin(θ/4) = q3 / (2 cos(θ/4) ).
    //-----------------------------------------------------------------------
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
    return math::RotationMatrix<T>(q_AB);
  }

  // Uses the rotation matrix R_AC that relates frames A and C to calculate the
  // RollPitchYaw angles `[roll, pitch, yaw] = [q₀, q₁, q₂]`, with
  // [`−π < q₀ <= π`, `−π/2 <= q₁ <= π/2`, `−π < q₂ <= π`].
  // @param[in] context The state of the multibody system.
  math::RollPitchYaw<T> CalcBushingRollPitchYawAngles(
      const systems::Context<T>& context) const {
    return math::RollPitchYaw<T>(CalcR_AC(context));
  }

  // Calculate the time-derivative of the roll, pitch, yaw angles associated
  // with the orientation between frames A and C.
  // @param[in] context The state of the multibody system.
  // @retval `[ṙoll, ṗitch, ẏaw] = [q̇₀, q̇₁, q̇₂]`
  Vector3<T> CalcBushingRollPitchYawAngleRates(
      const systems::Context<T>& context) const {
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    return CalcBushingRollPitchYawAngleRates(context, rpy);
  }

  // Calculate time-derivatives of the roll, pitch, yaw angles associated with
  // the orientation of frames A and C.
  // @param[in] context The state of the multibody system.
  // @param[in] rpy RollPitchYaw angles for the orientation of frames A and C.
  // @retval `[ṙoll, ṗitch, ẏaw] = [q̇₀, q̇₁, q̇₂]`
  Vector3<T> CalcBushingRollPitchYawAngleRates(
      const systems::Context<T>& context,
      const math::RollPitchYaw<T>& rpy) const {
    const Vector3<T> w_AC_A = Calcw_AC_A(context);
    return rpy.CalcRpyDtFromAngularVelocityInParent(w_AC_A);
  }

  // Calculate p_AoCo_A, the position vector from Ao to Co expressed in A.
  // @param[in] context The state of the multibody system.
  Vector3<T> Calcp_AoCo_A(const systems::Context<T>& context) const {
    return CalcX_AC(context).translation();
  }

  // Returns X_AC, the rigid transform that relates frames A and C.
  // @param[in] context The state of the multibody system.
  math::RigidTransform<T> CalcX_AC(const systems::Context<T>& context) const {
    return frameC().CalcPose(context, frameA());
  }

  // Calculate p_AoCo_B, the position vector from Ao to Co expressed in B.
  // @param[in] context The state of the multibody system.
  // @see CalcBushing_xyz() returns the same result since `p_AoCo_B = [x, y, z]`
  Vector3<T> Calcp_AoCo_B(const systems::Context<T>& context) const {
    const Vector3<T> p_AoCo_A = Calcp_AoCo_A(context);
    const math::RotationMatrix<T> R_BA = CalcR_AB(context).inverse();
    return R_BA * p_AoCo_A;
  }

  // Calculate the bushing's displacement `[x, y, z]`.
  // @param[in] context The state of the multibody system.
  // @see Calcp_AoCo_B() returns this same result since `p_AoCo_B = [x, y, z]`.
  Vector3<T> CalcBushing_xyz(const systems::Context<T>& context) const {
    return Calcp_AoCo_B(context);
  }

  // Calculate the time derivative of the bushing's displacement `[ẋ ẏ ż]`,
  // which is equal to the time-derivative in B of p_AoCo (expressed in B),
  // @param[in] context The state of the multibody system.
  Vector3<T> CalcBushing_xyzDt(const systems::Context<T>& context) const {
    // FYI: It happens that [ẋ ẏ ż]ʙ is equal to both
    //  `2 * v_BCo_B`  (2 * Co's velocity in B, expressed in B) and
    // `−2 * v_BAo_B` (−2 * Ao's velocity in B, expressed in B).
    // Proof: Denoting DtB as the time-derivative in frame B, one knows:
    //        DtB( p_AoCo ) = DtB( [x y z]ʙ ) = [ẋ ẏ ż]ʙ
    //        The definition of Co's velocity in B leads to
    //        v_BCo = DtB( p_BoCo )
    //              = DtB( p_AoCo / 2 )   (since Bo is halfway to Co)
    //              = 0.5 * [ẋ ẏ ż]ʙ
    //        Similarly, the definition of Ao's velocity in B leads to
    //        v_BAo = DtB( p_BoAo )
    //              = DtB( -p_AoCo / 2 )
    //              = -0.5 * [ẋ ẏ ż]ʙ
    // End of FYI - which is not used for the calculation below.
    // -------------------------------------------------------------------
    const SpatialVelocity<T> V_AC_A = CalcV_AC_A(context);
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
    return DtB_p_AoCo_B;  // This vector derivative happens to be [ẋ, ẏ, ż].
  }

  // Calculate w_AC_A, frame C's angular velocity in frame A, expressed in A.
  // @param[in] context The state of the multibody system.
  // @note `w_AC_A ≠ [q̇₀, q̇₁, q̇₂]`
  // @see CalcBushingRollPitchYawAngleRates() for `[q̇₀, q̇₁, q̇₂]`.
  // TODO(Mitiguy) improve efficiency by implementing a frame method such as
  //  frameC().CalcAngularVelocity(context, frameA(), frameA()) to mimic
  //  frameC().CalcSpatialVelocity(context, frameA(), frameA());
  Vector3<T> Calcw_AC_A(const systems::Context<T>& context) const {
    const SpatialVelocity<T> V_ACo_A = CalcV_AC_A(context);
    return V_ACo_A.rotational();
  }

  // Calculate V_AC_A, frame C's spatial velocity in frame A, expressed in A.
  // @param[in] context The state of the multibody system.
  // @see CalcBushingRollPitchYawAngleRates() for `[q̇₀, q̇₁, q̇₂]`.
  // @see CalcBushing_xyzDt() for `[ẋ, ẏ, ż]`.
  SpatialVelocity<T> CalcV_AC_A(const systems::Context<T>& context) const {
    return frameC().CalcSpatialVelocity(context, frameA(), frameA());
  }

  // Returns [k₀q₀, k₁q₁, k₂q₂], element-wise multiplication of the torque
  // stiffness constants [k₀, k₁, k₂] and roll-pitch-yaw angles [q₀, q₁, q₂].
  // @param[in] context The state of the multibody system.
  Vector3<T> TorqueStiffnessConstantsTimesAngles(
      const systems::Context<T>& context) const {
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    return torque_stiffness_constants().cwiseProduct(rpy.vector());
  }

  // Returns [b₀q̇₀, b₁q̇₁, b₂q̇₂], element-wise multiplication of the torque
  // damping constants [b₀, b₁, b₂] and roll-pitch-yaw rates [q̇₀, q̇₁, q̇₂].
  // @param[in] context The state of the multibody system.
  Vector3<T> TorqueDampingConstantsTimesAngleRates(
      const systems::Context<T>& context) const {
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    const Vector3<T> rpyDt = CalcBushingRollPitchYawAngleRates(context, rpy);
    return torque_damping_constants().cwiseProduct(rpyDt);
  }

  // Calculate the 3x1 array (not really a vector) containing [T₀ T₁ T₂].
  // @param[in] context The state of the multibody system.
  Vector3<T> CalcBushingTorque012(const systems::Context<T>& context) const {
    // T₀ = −(k₀ q₀ + b₀ q̇₀)
    // T₁ = −(k₁ q₁ + b₁ q̇₁)
    // T₂ = −(k₂ q₂ + b₂ q̇₂)
    const Vector3<T> t012 = -(TorqueStiffnessConstantsTimesAngles(context) +
                              TorqueDampingConstantsTimesAngleRates(context));
    return t012;
  }

  // Calculate the bushing torque on frame C, expressed in frame A.
  // @param[in] context The state of the multibody system.
  // @see CalcBushingSpatialForceOnFrameA(),
  //      CalcBushingSpatialForceOnFrameC().
  Vector3<T> CalcBushingTorqueOnCExpressedInA(
      const systems::Context<T>& context) const {
    const Vector3<T> t012 = CalcBushingTorque012(context);
    // The set of forces on frame C from the bushing is equivalent to a
    // torque T on frame C and a force f applied to a point Cp of C.
    // The set of forces on frame A from the bushing is equivalent to a
    // torque −T on frame A and a force −f applied to a point Ap of A.
    // Points Ap and Cp are coincident and located halfway between Aₒ and Cₒ.
    // ------------------------------------------------------------------------
    // This method calculates the torque `T = Tx Ax + Ty Ay + Tz Az` that
    // the bushing applies to C as
    // ⌈ Tx ⌉       ⌈ T₀ ⌉                       ⌈ q̇₀ ⌉     ⌈ ωx ⌉
    // | Ty |  = Nᵀ | T₁ |  where N arises from  | q̇₁ | = N | ωy |
    // ⌊ Tz ⌋ᴀ      ⌊ T₂ ⌋                       ⌊ q̇₂ ⌋     ⌊ ωz ⌋ᴀ
    // ------------------------------------------------------------------------
    // The expressions for Tx, Ty, Tz in terms of T₀, T₁, T₂ is derived below by
    // equating the power `T ⋅ w_CA_A = Tx ωx + Ty ωy + Tz ωz` of torque T to
    // the power `T₀ q̇₀ + T₁ q̇₁ + T₂ q̇₂` of the three spring-damper "gimbal"
    // torques `T₀ Cx`, `T₁ Py`, `T₂ Az` (each of Cx, Py, Az are associated with
    // a frame in the roll-pitch-yaw rotation sequence, where `Py` denotes a
    // unit vector of the "pitch" intermediate frame).
    // ------------------------------------------------------------------------
    // Power = [T₀ T₁ T₂]⌈ q̇₀ ⌉ = [T₀ T₁ T₂] N ⌈ ωx ⌉ =  [Tx Ty Tz]ᴀ ⌈ ωx ⌉
    //                   | q̇₁ |                | ωy |                | ωy |
    //                   ⌊ q̇₂ ⌋                ⌊ ωz ⌋                ⌊ ωz ⌋
    // which is true in view of the tranpose of `[Tx Ty Tz]ᴀ = [T₀ T₁ T₂] N`.
    // ------------------------------------------------------------------------

    // Calculate the matrix N that relates q̇₀, q̇₁, q̇₂ to ωx, ωy, ωz, where frame
    // C's angular velocity in A is expressed `w_AC_A = ωx Ax + ωy Ay + ωz Az`.
    // The calculation of N is documented in the class math::RollPitchYaw.
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    const Matrix3<T> N = rpy.CalcMatrixRelatingRpyDtToAngularVelocityInParent(
        __func__, __FILE__, __LINE__);

    // Form `Txyz = Tx Ax + Ty Ay + Tz Az` which is the torque required when the
    // bushing forces on C have their resultant force f applied at Cp (not Co).
    const Vector3<T> Txyz = N.transpose() * t012;
    return Txyz;
  }

  // Returns [kx x, ky y, kz z], element-wise multiplication of the force
  // stiffness constants [kx, ky, kz] and displacements [x, y, z].
  // @param[in] context The state of the multibody system.
  Vector3<T> ForceStiffnessConstantsTimesDisplacement(
      const systems::Context<T>& context) const {
    const Vector3<T> xyz = CalcBushing_xyz(context);
    return force_stiffness_constants().cwiseProduct(xyz);
  }

  // Returns [bx ẋ, by ẏ, bz ż], element-wise multiplication of the force
  // damping constants [bx, by, bz] and displacement rates [ẋ, ẏ, ż].
  // @param[in] context The state of the multibody system.
  Vector3<T> ForceDampingConstantsTimesDisplacementRate(
      const systems::Context<T>& context) const {
    const Vector3<T> xyzDt = CalcBushing_xyzDt(context);
    return force_damping_constants().cwiseProduct(xyzDt);
  }

  // Return f_C_B, the resultant bushing force on frame C expressed in frame B.
  // @param[in] context The state of the multibody system.
  Vector3<T> CalcBushingNetForceOnCExpressedInB(
      const systems::Context<T>& context) const {
    // Calculate force `f = fx Bx + fy By + fz Bz`.
    // fx = −(kx x + bx ẋ)
    // fy = −(ky y + by ẏ)
    // fz = −(kz z + bz ż)
    return -(ForceStiffnessConstantsTimesDisplacement(context) +
             ForceDampingConstantsTimesDisplacementRate(context));
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  const Frame<T>& frameA_;
  const Frame<T>& frameC_;

  const Vector3<double> torque_stiffness_constants_;
  const Vector3<double> torque_damping_constants_;
  const Vector3<double> force_stiffness_constants_;
  const Vector3<double> force_damping_constants_;
};


}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::LinearBushingRollPitchYaw)
