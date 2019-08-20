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
/// Frame B is the bushing frame whose origin Bo is halfway between Ao and Co
/// (the origins of frame A and C) and whose unit vectors Bx, By, Bz are halfway
/// (in an angle-axis sense) between the unit vectors of frame A and frame C.
///
/// The set of forces on frame C from the bushing is equivalent to a
/// torque T on frame C and a force f applied to a point Cp of C.
/// The set of forces on frame A from the bushing is equivalent to a
/// torque -T on frame A and a force -f applied to a point Ap of A.
/// Points Ap and Cp are coincident with Bo (frame B's origin) which is located
/// halfway between point Ao (frame A's origin) and point Co (frame C's origin).
///
/// The torque T on frame C and force f on point Cp are expressed in the bushing
/// frame B as shown below.  Together, they form the spatial force F_Cp_B. <pre>
/// T = Tx Bx + Ty By + Tz Bz
/// f = fx Bx + fy By + fz Bz
/// </pre>
///
/// This "symmetric" bushing force/torque model was developed at Toyota Research
/// Institute and is similar to traditional bushing models in commercial and
/// open-source dynamics codes.  The symmetric bushing is advantageous because
/// it ensures the moment of -f on A about Ao is equal to the moment of f on C
/// about Co.  Other models differ, e.g., they apply f at Ao, which means the
/// moment of -f on A about Ao is always zero whereas the moment of f on C about
/// Co is nonzero. Also, unlike traditional models, the symmetric model employs
/// a frame B with unit vectors Bx, By, Bz "halfway" between the unit vectors
///  Ax, Ay, Az and Cx, Cy, Cz of frames A and C.
///
/// Torque T depends on roll-pitch-yaw angles q₀, q₁, q₂ which are
/// determined from frame C's orientation relative to frame A, with
/// [`-π < q₀ <= π`, `-π/2 <= q₁ <= π/2`, `-π < q₂ <= π`].
/// Force f depends on the position vector from Ao to Co, which is expressed as
/// `p_AoCo_B = x Bx + y By + z Bz`.
///
/// Specifically, the model of the force f and torque T is written in terms
/// force stiffness/damping constants kx, ky, kz and bx, by, bz and
/// torque stiffness/damping constants k₀, k₁, k₂ and b₀, b₁, b₂ as <pre>
/// fx = -(kx x + bx ẋ)
/// fy = -(ky y + by ẏ)
/// fz = -(kz z + bz ż)
/// T₀ = -(k₀ q₀ + b₀ q̇₀)
/// T₁ = -(k₁ q₁ + b₁ q̇₁)
/// T₂ = -(k₂ q₂ + b₂ q̇₂)
/// ⌈ Tx ⌉ = ⌈ cos(q₂)/cos(q₁)  -sin(q2)   cos(q₂)*tan(q₁) ⌉ ⌈ T₀ ⌉
/// | Ty | = | sin(q₂)/cos(q₁)   cos(q2)   sin(q₂)*tan(q₁) | | T₁ |
/// ⌊ Tz ⌋ = ⌊        0             0             1        ⌋ ⌊ T₂ ⌋
/// </pre>
///
/// The expressions for Tx, Ty, Tz in terms of T₀, T₁, T₂ were found by equating
/// the generalized forces produced by torque `T = Tx Ax + Ty Ay + Tz Az` to
/// the generalized forces produced by the three spring-damper torques
///  `T₀ Bx`,  `T₁ Py`,  `T₂ Az`  (each of these torques are associated with
/// a frame in the roll-pitch-yaw successive rotation sequence, where `Py`
/// denotes a unit vector of the "pitch" intermediate frame).
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// @see math::RollPitchYaw for definitions of roll, pitch, yaw `[q₀, q₁, q₂]`.
template <typename T>
class LinearBushingRollPitchYaw final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearBushingRollPitchYaw)

  /// Constructor for a RollPitchYaw bushing B that connects frames A and C,
  /// where frame A is welded to a link L0 and frame C is welded to a link L1.
  /// @param[in] frameA frame A of link L0 that connects to bushing B.
  /// @param[in] frameC frame C of link L1 that connects to bushing B.
  /// @param[in] torque_stiffness_constants `[k₀, k₁, k₂]` multiply
  /// roll, pitch, yaw angles `[q₀, q₁, q₂]` to form part of the torque measures
  /// T₀, T₁, T₂ (defined above).  The SI units of `k₀, k₁, k₂` are N*m/rad.
  /// @param[in] torque_damping_constants `[b₀, b₁, b₂]` multiply
  /// roll, pitch, yaw angle rates `[q̇₀, q̇₁, q̇₂]` to form part of the torque
  /// measures T₀, T₁, T₂.  The SI units of `k₀, k₁, k₂` are N*m/rad.
  /// @param[in] force_stiffness_constants `[kx, ky, kz]` multiply
  /// bushing displacements `[x, y, z]` to form part of the force measures
  /// fx, fy, fz (defined above).  The SI units of `kx, ky, kz` are N/m.
  /// @param[in] force_damping_constants `[bx, by, bz]` multiply
  /// bushing displacement rates `[ẋ, ẏ, ż]` to form part of the force measures
  /// fx, fy, fz.  The SI units of `bx, by, bz` are N*s/m.
  /// @note math::RollPitchYaw describes the roll pitch yaw angles q₀, q₁, q₂.
  /// @note The position from Ao to Co is `p_AoCo_B = x Bx + y By + z Bz`.
  /// @note The stiffness constants `[k₀, k₁, k₂]`, `[kx, ky, kz]` and the
  /// damping constants `[b₀, b₁, b₂]`, `[bx, by, bz]` are usually non-negative.
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

  /// Calculate F_A_A, the bushing's spatial force on frame A, expressed in A.
  /// @param[in] context The state of the multibody system.
  /// @related CalcBushingSpatialForceOnFrameC().
  SpatialForce<T> CalcBushingSpatialForceOnFrameA(
      const systems::Context<T>& context) const {
    // The set of all forces applied by the bushing to frame A are replaced by
    // the set's resultant force f applied to a point Ap of A together with a
    // torque T equal to the moment of the set about point Ap.
    //--------------------------------------------------------
    // Calculate bushing torque T on frame A, expressed in frame A.
    const Vector3<T> t_A_A = -CalcBushingTorqueOnCExpressedInA(context);

    // Calculate bushing force f on point Ap of frame A, expressed in frame A.
    const math::RotationMatrix<T> R_AB = CalcR_AB(context);
    const Vector3<T> f_Ap_B = -CalcBushingNetForceOnCExpressedInB(context);
    const Vector3<T> f_Ap_A = R_AB * f_Ap_B;

    // Form spatial force for point Ap of A (expressed in A), then shift to Ao.
    // Reminder: Point Ap is the point of frame A that is coincident with Bo and
    // Cp, and is located halfway between Ao (A's origin) and Co (C's origin).
    const SpatialForce<T> F_Ap_A(t_A_A, f_Ap_A);
    const Vector3<T> p_ApAo_B = -0.5 * Calcp_AoCo_B(context);
    const Vector3<T> p_ApAo_A = R_AB * p_ApAo_B;
    const SpatialForce<T> F_Ao_A = F_Ap_A.Shift(p_ApAo_A);
    return F_Ao_A;
  }

  /// Calculate F_C_C, the bushing's spatial force on frame C, expressed in C.
  /// @param[in] context The state of the multibody system.
  /// @related CalcBushingSpatialForceOnFrameA().
  SpatialForce<T> CalcBushingSpatialForceOnFrameC(
      const systems::Context<T>& context) const {
    // The set of forces on C from the bushing can be replaced by a force f at
    // point C̅ (the point of C coincident with Ao) together with a torque T̅
    // equal to the moment of all forces from the bushing on C about C̅.
    // Force f and torque T̅ are the negative of the bushing's force/torque on A.
    const SpatialForce<T> F_Cbar_A = -CalcBushingSpatialForceOnFrameA(context);
    const Vector3<T> p_CbarCo_A = Calcp_AoCo_A(context);
    const SpatialForce<T> F_Co_A = F_Cbar_A.Shift(p_CbarCo_A);

    // Form F_Co_C by expressing spatial force F_Co_A in frame C.
    const math::RotationMatrix<T> R_CA = CalcR_AC(context).transpose();
    return R_CA * F_Co_A;
  }

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

 private:
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
    // TODO(Mitiguy) improve efficiency by implementing a frame method such as
    //  frameC().CalcRotationMatrix(context, frameA()) to mimic
    //  frameC().CalcPose(context, frameA());
    return CalcX_AC(context).rotation();
  }

  // Calculate R_AB, the rotation matrix that relates frames A and B.
  // @param[in] context The state of the multibody system.
  math::RotationMatrix<T> CalcR_AB(const systems::Context<T>& context) const {
    const math::RotationMatrix<T> R_AC = CalcR_AC(context);
    const Eigen::Quaternion<T> q_AC = R_AC.ToQuaternion();
    const T q0 = q_AC.w(), q1 = q_AC.x(), q2 = q_AC.y(), q3 = q_AC.z();
    using std::sqrt;
    const T e0 = sqrt(0.5 *(q0 + 1));
    // If q0 = -1 the e0 = 0 and the next line has a divide-by-zero error.
    // However, R_AC.ToQuaternion() guarantees q0 >= 0, so sqrt(0.5) <= e0 <= 1
    // which means the angle θₑ in e0 = cos(θₑ/2) has range  0 <= θₑ <= π/2.
    // Note: The derivation of these formulas employ double-angle formulas.
    DRAKE_ASSERT(q0 >= 0);
    const T oneOver2e0 = T(1) / (2 * e0);
    const T e1 = q1 * oneOver2e0;
    const T e2 = q2 * oneOver2e0;
    const T e3 = q3 * oneOver2e0;
    const Eigen::Quaternion<T> q_AB(e0, e1, e2, e3);
    return math::RotationMatrix<T>(q_AB);
  }

  // Calculate p_AoCo_A, the position vector from Ao to Co expressed in A.
  // @param[in] context The state of the multibody system.
  Vector3<T> Calcp_AoCo_A(const systems::Context<T>& context) const {
    return CalcX_AC(context).translation();
  }

  // Calculate p_AoCo_B, the position vector from Ao to Co expressed in B.
  // @param[in] context The state of the multibody system.
  // @see CalcBushing_xyz() returns the same result since `p_AoCo_B = [x, y, z]`
  Vector3<T> Calcp_AoCo_B(const systems::Context<T>& context) const {
    const Vector3<T> p_AoCo_A = Calcp_AoCo_A(context);
    const math::RotationMatrix<T> R_BA = CalcR_AB(context).transpose();
    return R_BA * p_AoCo_A;
  }

  // Returns X_AC, the rigid transform that relates frames A and C.
  // @param[in] context The state of the multibody system.
  math::RigidTransform<T> CalcX_AC(const systems::Context<T>& context) const {
    return frameC().CalcPose(context, frameA());
  }

  // Uses the rotation matrix R_AC that relates frames A and C to calculate the
  // RollPitchYaw angles `[roll, pitch, yaw] = [q₀, q₁, q₂]`, with
  // [`-π < q₀ <= π`, `-π/2 <= q₁ <= π/2`, `-π < q₂ <= π`].
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

  // Calculate the bushing's displacement `[x, y, z]`.
  // @param[in] context The state of the multibody system.
  // @see Calcp_AoCo_B() returns this same result since `p_AoCo_B = [x, y, z]`.
  Vector3<T> CalcBushing_xyz(const systems::Context<T>& context) const {
    return Calcp_AoCo_B(context);
  }

  // Calculate the time derivative of the bushing's displacement `[ẋ, ẏ, ż]`,
  // which is equal to  `2 * v_BCo_B`  (2 * Co's velocity in B, expressed in B)
  // which is equal to `-2 * v_BAo_B` (-2 * Ao's velocity in B, expressed in B).
  // @param[in] context The state of the multibody system.
  Vector3<T> CalcBushing_xyzDt(const systems::Context<T>& context) const {
    const SpatialVelocity<T> V_AC_A = CalcV_AC_A(context);
    const Vector3<T>& w_AC_A = V_AC_A.rotational();
    const Vector3<T>& v_ACo_A = V_AC_A.translational();
    const Vector3<T> w_AB_A = 0.5 * w_AC_A;
    const Vector3<T> p_AoCo_A = Calcp_AoCo_A(context);

    // Calculate the time-derivative in frame B of p_AoCo, derived below.
    // The results of this calculation is a vector expressed in frame A.
    // v_ACo = DtA_p_AoCo                  (definition)
    //       = DtB_p_AoCo + w_AB x p_AoCo  (Golden rule for vector derivatives)
    // DtB_p_AoCo = v_ACo - wAB x p_AoCo   (rearrange previous line).
    const Vector3<T> DtB_p_AoCo_A = v_ACo_A - w_AB_A.cross(p_AoCo_A);

    // Form the time-derivative in frame B of p_AoCo, expressed in frame B.
    const math::RotationMatrix<T> R_BA = CalcR_AB(context).transpose();
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

  // Calculate v_ACo_A, Co's translational velocity in frame A, expressed in A.
  // @param[in] context The state of the multibody system.
  // @see CalcBushing_xyzDt() for `[ẋ, ẏ, ż]` since `[ẋ, ẏ, ż] ≠ v_ACₒ_A`.
  // TODO(Mitiguy) improve efficiency by implementing a frame method such as
  //  frameC().CalcTranslationalVelocity(context, frameA(), frameA()) to mimic
  //  frameC().CalcSpatialVelocity(context, frameA(), frameA());
  Vector3<T> Calcv_ACo_A(const systems::Context<T>& context) const {
    const SpatialVelocity<T> V_ACo_A = CalcV_AC_A(context);
    return V_ACo_A.translational();
  }

  // Calculate V_AC_A, frame C's spatial velocity in frame A, expressed in A.
  // @param[in] context The state of the multibody system.
  // @see CalcBushingRollPitchYawAngleRates() for `[q̇₀, q̇₁, q̇₂]`.
  // @see CalcBushing_xyzDt() for `[ẋ, ẏ, ż]`.
  SpatialVelocity<T> CalcV_AC_A(const systems::Context<T>& context) const {
    return frameC().CalcSpatialVelocity(context, frameA(), frameA());
  }

  // Calculate the matrix N that relates q̇₀, q̇₁, q̇₂ to ωx, ωy, ωz, where frame
  // C's angular velocity in A is expressed `w_AC_A = ωx Ax + ωy Ay + ωz Az`.
  Matrix3<T> CalcQDt012ToWxyzMatrix(
      const systems::Context<T>& context) const {
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    const T q1 = rpy.pitch_angle();
    const T q2 = rpy.yaw_angle();
    const T c1 = cos(q1), s1 = sin(q1);
    const T c2 = cos(q2), s2 = sin(q2);
    const T oneOverc1 = 1.0/c1, tan1 = s1 * oneOverc1;
    Matrix3<T> mat33;
    // ⌈ q̇₀ ⌉ = ⌈ cos(q₂)/cos(q₁)  sin(q₂)/cos(q₁)   0 ⌉ ⌈ ωx ⌉
    // | q̇₁ | = |   -sin(q2)            cos(q2)      0 | | ωy |
    // ⌊ q̇₂ ⌋ = ⌊ cos(q₂)*tan(q₁)   sin(q₂)*tan(q₁)  1 ⌋ ⌊ ωz ⌋
    // clang-format off
    mat33 << c2 * oneOverc1,  s2 * oneOverc1,  0,
            -s2,              c2,              0,
             c2 * tan1,       s2 * tan1,       1;
    // clang-format on
    return mat33;
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
    // T₀ = -(k₀ q₀ + b₀ q̇₀)
    // T₁ = -(k₁ q₁ + b₁ q̇₁)
    // T₂ = -(k₂ q₂ + b₂ q̇₂)
    const Vector3<T> t012 = TorqueStiffnessConstantsTimesAngles(context) +
                            TorqueDampingConstantsTimesAngleRates(context);
    return t012;
  }

  // Calculate the bushing torque on frame C, expressed in frame A.
  // @param[in] context The state of the multibody system.
  // @related CalcBushingSpatialForceOnFrameA(),
  //          CalcBushingSpatialForceOnFrameC().
  Vector3<T> CalcBushingTorqueOnCExpressedInA(
      const systems::Context<T>& context) const {
    const Vector3<T> t012 = -CalcBushingTorque012(context);
    // The set of forces on frame C from the bushing is equivalent to a
    // torque T on frame C and a force f applied to a point Cp of C.
    // The set of forces on frame A from the bushing is equivalent to a
    // torque -T on frame A and a force -f applied to a point Ap of A.
    // Points Ap and Cp are coincident and located halfway between Aₒ and Cₒ.
    // Calculate torque `T = Tx Ax + Ty Ay + Tz Az` the bushing applies to C.
    // ⌈ Tx ⌉ = ⌈ cos(q₂)/cos(q₁)  -sin(q2)   cos(q₂)*tan(q₁) ⌉ ⌈ T₀ ⌉
    // | Ty | = | sin(q₂)/cos(q₁)   cos(q2)   sin(q₂)*tan(q₁) | | T₁ |
    // ⌊ Tz ⌋ = ⌊        0             0             1        ⌋ ⌊ T₂ ⌋
    const Matrix3<T> mat33 = CalcQDt012ToWxyzMatrix(context);
    const Vector3<T> Txyz = mat33.transpose() * t012;
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

  // Calculate the resultant bushing force on frame C expressed in frame B.
  // @param[in] context The state of the multibody system.
  Vector3<T> CalcBushingNetForceOnCExpressedInB(
      const systems::Context<T>& context) const {
    // Calculate force `f = fx Bx + fy By + fz Bz`.
    // fx = -(kx x + bx ẋ)
    // fy = -(ky y + by ẏ)
    // fz = -(kz z + bz ż)
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
