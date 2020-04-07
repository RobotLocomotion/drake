#pragma once

#include <limits>
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

/// This ForceElement models a massless flexible bushing that connects a frame A
/// of a link (body) L0 to a frame C of a link (body) L1.  The bushing can apply
/// a torque and force due to stiffness (spring) and dissipation (damper)
/// properties.
/// Frame B is the bushing frame whose origin Bo is halfway between Ao (A's
/// origin) and Co (C's origin) and whose unit vectors 𝐁𝐱, 𝐁𝐲, 𝐁𝐳 are "halfway"
/// (in an angle-axis sense) between the unit vectors of frame A and frame C.
/// Frame B is a "floating" frame in the sense that it is calculated from the
/// position and orientation of frames A and C (B is not welded to the bushing).
/// @image html multibody/tree/images/LinearBushingRollPitchYaw.png width=100%
///
/// The set of forces on frame C from the bushing is equivalent to a
/// torque 𝐭 on frame C and a force 𝐟 applied to a point Cp of C.
/// The set of forces on frame A from the bushing is equivalent to a
/// torque −𝐭 on frame A and a force −𝐟 applied to a point Ap of A.
/// Points Ap and Cp are coincident with Bo (frame B's origin).
///
/// This "quasi-symmetric" bushing force/torque model was developed at Toyota
/// Research Institute and has advantages compared to traditional bushing models
/// because it employs a bushing-centered "symmetric" frame B and it ensures the
/// moment of −𝐟 on A about Ao is equal to the moment of 𝐟 on C about Co.
/// Traditional models differ as they lack a "symmetric" frame B and apply −𝐟 at
/// Ao, which means the  moment of −𝐟 on A about Ao is always zero.  Note: This
/// bushing model is not fully symmetric since the orientation between frames A
/// and C is parameterized with roll-pitch-yaw angles [q₀ q₁ q₂].  Since these
/// angles have an inherent sequence, they are not mathematically symmetric.
///
/// The torque model depends on spring-damper "gimbal" torques `τ ≜ [τ₀ τ₁ τ₂]`
/// which themselves depend on roll-pitch-yaw angles `q ≜ [q₀ q₁ q₂]` and
/// rates `q̇ = [q̇₀ q̇₁ q̇₂]` via a diagonal torque-stiffness matrix K₀₁₂ and a
/// diagonal torque-damping matrix D₀₁₂ as <pre>
///     ⌈ τ₀ ⌉     ⌈k₀    0    0⌉ ⌈ q₀ ⌉     ⌈d₀    0    0⌉ ⌈ q̇₀ ⌉
/// τ ≜ | τ₁ | = − | 0   k₁    0| | q₁ |  −  | 0   d₁    0| | q̇₁ |
///     ⌊ τ₂ ⌋     ⌊ 0    0   k₂⌋ ⌊ q₂ ⌋     ⌊ 0    0   d₂⌋ ⌊ q̇₂ ⌋ </pre>
/// where k₀, k₁, k₂ and d₀, d₁, d₂ are torque stiffness and damping constants
/// and must have non-negative values.
/// @note τ does not represent a vector expressed in one frame.  Instead it is
/// regarded as a 3x1 array of torque scalars associated with roll-pitch yaw.
/// @note As discussed in the Advanced section below, τ is not 𝐭 `(τ ≠ 𝐭)`.
/// @note This is a "linear" bushing model as gimbal torque τ varies linearly
/// with q and q̇ as τ = τᴋ + τᴅ where τᴋ = −K₀₁₂ ⋅ q and τᴅ = −D₀₁₂ ⋅ q̇.
///
/// The bushing model for the net force 𝐟 on frame C from the bushing depends on
/// scalars x, y, z which are defined so 𝐫 (the position vector from Ao to Co)
/// can be expressed in frame B as `𝐫 ≜ p_AoCo = [x y z]ʙ = x 𝐁𝐱 + y 𝐁𝐲 + z 𝐁𝐳`.
/// The model for 𝐟 uses a diagonal force-stiffness matrix Kxyᴢ, a diagonal
/// force-damping matrix Dxyᴢ, and defines fx, fy, fz so `𝐟 = [fx fy fz]ʙ`.<pre>
/// ⌈ fx ⌉      ⌈kx    0    0⌉ ⌈ x ⌉     ⌈dx    0    0⌉ ⌈ ẋ ⌉
/// | fy | =  − | 0   ky    0| | y |  −  | 0   dy    0| | ẏ |
/// ⌊ fz ⌋      ⌊ 0    0   kz⌋ ⌊ z ⌋     ⌊ 0    0   dz⌋ ⌊ ż ⌋ </pre>
/// where kx, ky, kz and dx, dy, dz are force stiffness and damping constants
/// and must have non-negative values.
/// @note This is a "linear" bushing model as the force 𝐟 varies linearly
/// with 𝐫 and 𝐫̇̇ as 𝐟 = 𝐟ᴋ + 𝐟ᴅ where 𝐟ᴋ = −Kxyz ⋅ 𝐫 and 𝐟ᴅ = −Dxyz ⋅ 𝐫̇̇.
///
/// This bushing's constructor sets the torque stiffness/damping constants
/// `[k₀ k₁ k₂]` and `[d₀ d₁ d₂]` and the force stiffness/damping constants
/// `[kx ky kz]` and `[dx dy dz]`.  The examples below demonstrate how to model
/// various joints that have a flexible (e.g., rubber) mount.  The damping
/// values below with ? may be set to 0 or a reasonable positive number.
///
/// Bushing type                    | torque constants    | force constants
/// --------------------------------|:--------------------|:------------------
/// z-axis revolute joint           | k₀₁₂ = `[k₀ k₁ 0]`  | kxyz = `[kx ky kz]`
/// ^                               | d₀₁₂ = `[d₀ d₁ ?]`  | dxyz = `[dx dy dz]`
/// x-axis prismatic joint          | k₀₁₂ = `[k₀ k₁ k₂]` | kxyz = `[0 ky kz]`
/// ^                               | d₀₁₂ = `[d₀ d₁ d₂]` | dxyz = `[? dy dz]`
/// Ball and socket joint           | k₀₁₂ = `[0  0  0]`  | kxyz = `[kx ky kz]`
/// ^                               | d₀₁₂ = `[?  ?  ?]`  | dxyz = `[dx dy dz]`
/// Weld/fixed joint                | k₀₁₂ = `[k₀ k₁ k₂]` | kxyz = `[kx ky kz]`
/// ^                               | d₀₁₂ = `[d₀ d₁ d₂]` | dxyz = `[dx dy dz]`
///
/// Angles q₀, q₁, q₂ are calculated from frame C's orientation relative to
/// frame A, with `[−π < q₀ <= π, −π/2 <= q₁ <= π/2, −π < q₂ <= π]`,
/// hence, there is no angle wrapping and torque stiffness has a limited range.
/// Gimbal torques τ can be discontinuous if one of q₀, q₁, q₂ is discontinuous
/// and its associated torque spring constant is nonzero. For example, τ₂ is
/// discontinuous if `k₂ ≠ 0` and the bushing has a large rotation so q₂ jumps
/// from `≈ −π to π`. τ can also be discontinuous if one of q̇₀, q̇₁, q̇₂ is
/// discontinuous and its associated torque damper constant is nonzero.
/// For example, τ₀ is discontinuous if `d₀ ≠ 0` and q̇₀ is undefined (which
/// occurs when `pitch = q₁ = π/2`).  Note: Due to the relationship of 𝐭 to τ
/// shown below, 𝐭 is discontinuous if τ is discontinuous.
///
/// ### Advanced: Relationship of 𝐭 to τ.
/// To understand how "gimbal torques" τ relate to 𝐭, it helps to remember that
/// the RollPitchYaw class documentation states that a Space-fixed (extrinsic)
/// X-Y-Z rotation with roll-pitch-yaw angles [q₀ q₁ q₂] is equivalent to a
/// Body-fixed (intrinsic) Z-Y-X rotation by yaw-pitch-roll angles [q₂ q₁ q₀].
/// In the context of "gimbal torques", the Body-fixed Z-Y-X rotation sequence
/// with angles [q₂ q₁ q₀] is physical meaningful as it produces torques
/// associated with successive frames in a gimbal as τ₂ 𝐀𝐳, τ₁ 𝐏𝐲, τ₀ 𝐂𝐱,
/// where each of 𝐀𝐳, 𝐏𝐲, 𝐂𝐱 are unit vectors associated with a frame in the
/// yaw-pitch-roll rotation sequence and 𝐏𝐲 is a unit vector of the "pitch"
/// intermediate frame.  As described earlier, torque 𝐭 is the moment of the
/// bushing forces on frame C about Cp.  Scalars tx, ty, tz are defined so 𝐭 can
/// be expressed `𝐭 = [tx ty tz]ᴀ = tx 𝐀𝐱 + ty 𝐀𝐲 + tz 𝐀𝐳`.
/// As shown in code documentation, the relationship of [tx ty tz] to [τ₀ τ₁ τ₂]
/// was found by equating 𝐭's power to τ's power as 𝐭 ⋅ w_AC = τ ⋅ q̇. <pre>
/// ⌈ tx ⌉      ⌈ τ₀ ⌉            ⌈ cos(q₂)/cos(q₁)  sin(q₂)/cos(q₁)   0 ⌉
/// | ty | = Nᵀ | τ₁ |  where N = |   −sin(q2)            cos(q2)      0 |
/// ⌊ tz ⌋      ⌊ τ₂ ⌋            ⌊ cos(q₂)*tan(q₁)   sin(q₂)*tan(q₁)  1 ⌋</pre>
///
/// @note The complete theory for this bushing is documented in the source code.
/// Please look there if you want more information.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// @see math::RollPitchYaw for definitions of roll, pitch, yaw `[q₀ q₁ q₂]`.
///
/// @note Per issue #12982, do not directly or indirectly call the following
/// methods as they have not yet been implemented and throw an exception:
/// CalcPotentialEnergy(), CalcConservativePower(), CalcNonConservativePower().
template <typename T>
class LinearBushingRollPitchYaw final : public ForceElement<T> {
  // TODO(Mitiguy) Add gimbal picture at "Relationship of 𝐭 to τ".
  // TODO(Mitiguy) Per issue #12982, implement CalcPotentialEnergy(),
  //  CalcConservativePower(), CalcNonConservativePower().
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearBushingRollPitchYaw)

  /// Construct a LinearBushingRollPitchYaw B that connects frames A and C,
  /// where frame A is welded to a link L0 and frame C is welded to a link L1.
  /// @param[in] frameA frame A of link (body) L0 that connects to bushing B.
  /// @param[in] frameC frame C of link (body) L1 that connects to bushing B.
  /// @param[in] torque_stiffness_constants `[k₀ k₁ k₂]` multiply the
  /// roll-pitch-yaw angles `[q₀ q₁ q₂]` to produce the spring portion of the
  /// "gimbal" torques τ₀, τ₁, τ₂. The SI units of `k₀, k₁, k₂` are N*m/rad.
  /// @param[in] torque_damping_constants `[d₀ d₁ d₂]` multiply the
  /// roll-pitch-yaw rates `[q̇₀ q̇₁ q̇₂]` to produce the damper portion of the
  /// "gimbal" torques τ₀, τ₁, τ₂.  The SI units of `d₀, d₁, d₂` are N*m*s/rad.
  /// @param[in] force_stiffness_constants `[kx ky kz]` multiply the
  /// bushing displacements `[x y z]` to form 𝐟ᴋ, the spring portion of the
  /// force 𝐟 = [fx fy fz]ʙ.  The SI units of `kx, ky, kz` are N/m.
  /// @param[in] force_damping_constants `[dx dy dz]` multiply the
  /// bushing displacement rates `[ẋ ẏ ż]` to form 𝐟ᴅ, the damper portion of the
  /// force 𝐟 = [fx fy fz]ʙ.  The SI units of `dx, dy, dz` are N*s/m.
  /// @note The LinearBushingRollPitchYaw class documentation describes the
  /// stiffness and damping constants.
  /// @note The net moment on C about Co is affected by both the gimbal torque
  /// and the moment of 𝐟 about Co. Similarly, for the net moment on A about Ao.
  /// @note math::RollPitchYaw describes the roll pitch yaw angles q₀, q₁, q₂.
  /// The position from Ao to Co is p_AoCo_B = x 𝐁𝐱 + y 𝐁𝐲 + z 𝐁𝐳 = [x y z]ʙ.
  /// @note The ModelInstanceIndex assigned to this by the constructor is the
  /// one assigned to frame C, i.e., frameC.model_instance().
  /// @pre All the stiffness and damping constants must be non-negative.
  LinearBushingRollPitchYaw(const Frame<T>& frameA, const Frame<T>& frameC,
                            const Vector3<double>& torque_stiffness_constants,
                            const Vector3<double>& torque_damping_constants,
                            const Vector3<double>& force_stiffness_constants,
                            const Vector3<double>& force_damping_constants);

  /// Returns link (body) L0 (frame A is welded to link L0).
  const Body<T>& link0() const { return frameA().body(); }

  /// Returns link (body) L1 (frame C is welded to link L1).
  const Body<T>& link1() const { return frameC().body(); }

  /// Returns frame A, which is the frame that is welded to link (body) L0 and
  /// attached to the bushing.
  const Frame<T>& frameA() const {
    return this->get_parent_tree().get_frame(frameA_index_);
  }

  /// Returns frame C, which is the frame that is welded to link (body) L1 and
  /// attached to the bushing.
  const Frame<T>& frameC() const {
    return this->get_parent_tree().get_frame(frameC_index_);
  }

  /// Returns the torque stiffness constants `[k₀ k₁ k₂]` (units of N*m/rad).
  const Vector3<double>& torque_stiffness_constants() const {
    return torque_stiffness_constants_;
  }

  /// Returns the torque damping constants `[d₀ d₁ d₂]` (units of N*m*s/rad).
  const Vector3<double>& torque_damping_constants() const {
    return torque_damping_constants_;
  }

  /// Returns the force stiffness constants `[kx ky kz]` (units of N/m).
  const Vector3<double>& force_stiffness_constants() const {
    return force_stiffness_constants_;
  }

  /// Returns the force damping constants `[dx dy dz]` (units of N*s/m).
  const Vector3<double>& force_damping_constants() const {
    return force_damping_constants_;
  }

  /// Calculate F_A_A, the bushing's spatial force on frame A expressed in A.
  /// F_A_A contains two vectors: the moment of all bushing forces on A about Ao
  /// (−𝐭 + p_AoAp × −𝐟) and the net bushing force on A (−𝐟 expressed in A).
  /// @param[in] context The state of the multibody system.
  /// @see CalcBushingSpatialForceOnFrameC().
  SpatialForce<T> CalcBushingSpatialForceOnFrameA(
      const systems::Context<T>& context) const;

  /// Calculate F_C_C, the bushing's spatial force on frame C expressed in C.
  /// F_C_C contains two vectors: the moment of all bushing forces on C about Co
  /// (𝐭 + p_CoCp × 𝐟) and the resultant bushing force on C (𝐟 expressed in C).
  /// @param[in] context The state of the multibody system.
  /// @see CalcBushingSpatialForceOnFrameA().
  SpatialForce<T> CalcBushingSpatialForceOnFrameC(
      const systems::Context<T>& context) const;

 private:
  // Friend class for accessing protected/private internals of this class.
  friend class BushingTester;

  // The following template friend is needed to facilitate use of the private
  // constructor below for use in the method TemplatedDoCloneToScalar().
  template <typename U>
  friend class LinearBushingRollPitchYaw;

  // Many of these input parameters for this private LinearBushingRollPitchYaw
  // are described in the public constructor's documentation.
  LinearBushingRollPitchYaw(ModelInstanceIndex model_instance,
                            FrameIndex frameA_index, FrameIndex frameC_index,
                            const Vector3<double>& torque_stiffness_constants,
                            const Vector3<double>& torque_damping_constants,
                            const Vector3<double>& force_stiffness_constants,
                            const Vector3<double>& force_damping_constants);

  // TODO(Mitiguy) Per issue #12982, implement the following method.
  //  Currently it has not been implemented and throws an exception.
  T CalcPotentialEnergy(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc) const override;

  // TODO(Mitiguy) Per issue #12982, implement the following method.
  //  Currently it has not been implemented and throws an exception
  T CalcConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const override;

  // TODO(Mitiguy) Per issue #12982, implement the following method.
  //  Currently it has not been implemented and throws an exception
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
    const math::RotationMatrix<T> R_AB = CalcR_AB(R_AC);
    return R_AB;
  }

  // Calculate R_AB, the rotation matrix that relates frames A and B.
  // @param[in] R_AC The rotation matrix that relates frames A and C.
  static math::RotationMatrix<T> CalcR_AB(math::RotationMatrix<T> R_AC);

  // The efficient algorithm CalcR_AB() above is verified in debug builds by
  // calculating the `θ λ` AngleAxis from R_AC and then forming R_AB_expected
  // from the AngleAxis `θ/2 λ`.  The function below throws an exception if
  // R_AB does not match R_AB_expected to near machine precision.
  static void ThrowIfInvalidHalfAngleAxis(const math::RotationMatrix<T>& R_AC,
                                          const math::RotationMatrix<T>& R_AB);

  // Uses the rotation matrix R_AC that relates frames A and C to calculate the
  // RollPitchYaw angles `[roll pitch yaw] = [q₀ q₁ q₂]`, with the range
  // `[−π < q₀ <= π, −π/2 <= q₁ <= π/2, −π < q₂ <= π]`.
  // @param[in] context The state of the multibody system.
  math::RollPitchYaw<T> CalcBushingRollPitchYawAngles(
      const systems::Context<T>& context) const {
    return math::RollPitchYaw<T>(CalcR_AC(context));
  }

  // Calculate the time-derivative of the roll, pitch, yaw angles associated
  // with the orientation between frames A and C.
  // @param[in] context The state of the multibody system.
  // @retval `[ṙoll ṗitch ẏaw] = [q̇₀ q̇₁ q̇₂]`
  Vector3<T> CalcBushingRollPitchYawAngleRates(
      const systems::Context<T>& context) const {
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    return CalcBushingRollPitchYawAngleRates(context, rpy);
  }

  // Calculate time-derivatives of the roll, pitch, yaw angles associated with
  // the orientation of frames A and C.
  // @param[in] context The state of the multibody system.
  // @param[in] rpy RollPitchYaw angles for the orientation of frames A and C.
  // @retval `[ṙoll ṗitch ẏaw] = [q̇₀ q̇₁ q̇₂]`
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

  // Calculate X_AC, the rigid transform that relates frames A and C.
  // @param[in] context The state of the multibody system.
  math::RigidTransform<T> CalcX_AC(const systems::Context<T>& context) const {
    return frameC().CalcPose(context, frameA());
  }

  // Calculate `p_AoCo_B = [x y z]ʙ`, the position from Ao to Co expressed in B.
  // @param[in] context The state of the multibody system.
  Vector3<T> Calcp_AoCo_B(const systems::Context<T>& context) const {
    const Vector3<T> p_AoCo_A = Calcp_AoCo_A(context);
    const math::RotationMatrix<T> R_BA = CalcR_AB(context).inverse();
    return R_BA * p_AoCo_A;
  }

  // Calculate [ẋ ẏ ż] which happens to be 3x1 array associated with DtB_p_AoCo,
  // the time-derivative in B of p_AoCo (when DtB_p_AoCo is expressed in B).
  // @param[in] context The state of the multibody system.
  // @note Calcp_AoCo_B() returns `p_AoCo_B = [x y z]ʙ`.
  Vector3<T> CalcBushing_xyzDt(const systems::Context<T>& context) const;

  // Calculate w_AC_A, frame C's angular velocity in frame A, expressed in A.
  // @param[in] context The state of the multibody system.
  // @note `w_AC_A ≠ [q̇₀ q̇₁ q̇₂]`
  // @see CalcBushingRollPitchYawAngleRates() for `[q̇₀ q̇₁ q̇₂]`.
  Vector3<T> Calcw_AC_A(const systems::Context<T>& context) const {
    const SpatialVelocity<T> V_AC_A = CalcV_AC_A(context);
    return V_AC_A.rotational();
  }

  // Calculate V_AC_A, frame C's spatial velocity in frame A, expressed in A.
  // @param[in] context The state of the multibody system.
  // @see CalcBushingRollPitchYawAngleRates() for `[q̇₀ q̇₁ q̇₂]`.
  // @see CalcBushing_xyzDt() for `[ẋ ẏ ż]`.
  SpatialVelocity<T> CalcV_AC_A(const systems::Context<T>& context) const {
    return frameC().CalcSpatialVelocity(context, frameA(), frameA());
  }

  // Calculate τᴋ = [k₀q₀, k₁q₁, k₂q₂], element-wise multiplication of the
  // torque stiffness constants [k₀ k₁ k₂] and roll-pitch-yaw angles [q₀ q₁ q₂].
  // @param[in] context The state of the multibody system.
  Vector3<T> TorqueStiffnessConstantsTimesAngles(
      const systems::Context<T>& context) const {
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    return torque_stiffness_constants().cwiseProduct(rpy.vector());
  }

  // Calculate τᴅ = [d₀q̇₀, d₁q̇₁, d₂q̇₂], element-wise multiplication of the
  // torque damping constants [d₀ d₁ d₂] and roll-pitch-yaw rates [q̇₀ q̇₁ q̇₂].
  // @param[in] context The state of the multibody system.
  Vector3<T> TorqueDampingConstantsTimesAngleRates(
      const systems::Context<T>& context) const {
    const math::RollPitchYaw<T> rpy = CalcBushingRollPitchYawAngles(context);
    const Vector3<T> rpyDt = CalcBushingRollPitchYawAngleRates(context, rpy);
    return torque_damping_constants().cwiseProduct(rpyDt);
  }

  // Calculate the 3x1 array (not a vector) containing τ = τᴋ + τᴅ = [τ₀ τ₁ τ₂].
  // @param[in] context The state of the multibody system.
  Vector3<T> CalcBushingTorqueTau(const systems::Context<T>& context) const {
    // τ₀ = −(k₀ q₀ + d₀ q̇₀)
    // τ₁ = −(k₁ q₁ + d₁ q̇₁)
    // τ₂ = −(k₂ q₂ + d₂ q̇₂)
    const Vector3<T> tau_k = -TorqueStiffnessConstantsTimesAngles(context);
    const Vector3<T> tau_d = -TorqueDampingConstantsTimesAngleRates(context);
    return tau_k + tau_d;  // τ = τᴋ + τᴅ
  }

  // Calculate `𝐭 = t_Cp_A = [tx ty tz]ᴀ` the moment of all forces on frame C
  // about point Cp expressed in frame A.
  // @param[in] context The state of the multibody system.
  // @see CalcBushingSpatialForceOnFrameA(),
  //      CalcBushingSpatialForceOnFrameC().
  Vector3<T> CalcBushingTorqueOnCExpressedInA(
      const systems::Context<T>& context) const;

  // Calculate `𝐟ᴋ = [kx x, ky y, kz z]ʙ`, element-wise multiplication of the
  // force stiffness constants `[kx ky kz]` and displacements `[x y z]`.
  // @param[in] context The state of the multibody system.
  Vector3<T> ForceStiffnessConstantsTimesDisplacement(
      const systems::Context<T>& context) const {
    const Vector3<T> xyz = Calcp_AoCo_B(context);  // [x y z]ʙ
    return force_stiffness_constants().cwiseProduct(xyz);
  }

  // Calculate `𝐟ᴅ = [dx ẋ, dy ẏ, dz ż]ʙ`, element-wise multiplication of the
  // force damping constants `[dx dy dz]` and displacement rates `[ẋ ẏ ż]`.
  // @param[in] context The state of the multibody system.
  Vector3<T> ForceDampingConstantsTimesDisplacementRate(
      const systems::Context<T>& context) const {
    const Vector3<T> xyzDt = CalcBushing_xyzDt(context);
    return force_damping_constants().cwiseProduct(xyzDt);
  }

  // Calculate `𝐟 = 𝐟ᴋ + 𝐟ᴅ = f_C_B  = [fx fy fz]ʙ`, the resultant bushing
  // force on frame C expressed in frame B.
  // @param[in] context The state of the multibody system.
  Vector3<T> CalcBushingNetForceOnCExpressedInB(
      const systems::Context<T>& context) const {
    // Calculate force `𝐟 = fx 𝐁𝐱 + fy 𝐁𝐲 + fz 𝐁𝐳`.
    // fx = −(kx x + dx ẋ)
    // fy = −(ky y + dy ẏ)
    // fz = −(kz z + dz ż)
    const Vector3<T> f_k = -ForceStiffnessConstantsTimesDisplacement(context);
    const Vector3<T> f_d = -ForceDampingConstantsTimesDisplacementRate(context);
    return f_k + f_d;  // 𝐟 = 𝐟ᴋ + 𝐟ᴅ
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<ForceElement<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  const FrameIndex frameA_index_;
  const FrameIndex frameC_index_;

  const Vector3<double> torque_stiffness_constants_;
  const Vector3<double> torque_damping_constants_;
  const Vector3<double> force_stiffness_constants_;
  const Vector3<double> force_damping_constants_;
};


}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::LinearBushingRollPitchYaw)
