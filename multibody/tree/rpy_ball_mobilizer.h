#pragma once

#include <limits>
#include <memory>
#include <string>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/mobilizer_impl.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

// This mobilizer models a 3 dof gimbal joint between an inboard frame F and an
// outboard frame M that allows frame M to rotate freely with respect to F. A
// gimbal joint provides arbitrary orientation like a ball joint (and in fact we
// call it a BallRpyJoint in Drake) but with some restrictions, discussed
// below). No translational motion of M in F is allowed and the inboard frame
// origin Fo and the outboard frame origin Mo are coincident at all times.
//
// The orientation R_FM of the outboard frame M in F is parameterized with
// roll-pitch-yaw angles (also known as space-fixed x-y-z Euler angles or
// extrinsic angles). That is, the generalized coordinates q for this mobilizer
// correspond to angles q = [θ₀ θ₁ θ₂], for a sequence of rotations about the
// Fx, Fy, and Fz axes, respectively. Mathematically, rotation R_FM is given in
// terms of angles θ₀, θ₁, θ₂ by:
//
//   R_FM(q) = Rz(θ₂) * Ry(θ₁) * Rx(θ₀)
//
// where Rx(θ), Ry(θ) and Rz(θ) correspond to the elemental rotations in amount
// of θ about the Fx, Fy and Fz axes respectively. Zero θ₀, θ₁, θ₂ angles define
// the "zero configuration" which corresponds to frames F and M being
// coincident, see SetZeroState(). Angles θ₀, θ₁, θ₂ are defined to be
// positive according to the right-hand-rule with the thumb aligned in the
// direction of their respective axes.
//
// The generalized velocities v for this mobilizer correspond to the angular
// velocity w_FM of frame M in F, expressed in frame F. Note that this is _not_
// the same as the time derivative q̇ the rotation angles; you must use
// q̇ = MapVelocityToQDot(w_FM) to get the angle time derivatives.
//
// MapVelocityToQDot() maps the angular velocity w_FM to Euler angle rates
// while MapQDotToVelocity() maps Euler angles's rates to angular velocity
// w_FM. While the mapping MapVelocityToQDot() always exists, its inverse
// MapQDotToVelocity() is singular for values of θ₁ (many times referred to as
// the pitch angle) such that θ₁ = π/2 + kπ, ∀ k ∈ ℤ.
//
// @note Roll-pitch-yaw (space x-y-z) angles (extrinsic, about Fx Fy Fz) are
// equivalent to body-fixed z-y-x angles (intrinsic, about Mz M'y M''x where
// the primes indicate new axis directions after each rotation).
//
// @note The roll-pitch-yaw (space x-y-z) Euler sequence is also known as the
// Tait-Bryan angles or Cardan angles.
//
//    H_FM_F₆ₓ₃ = [ I₃ₓ₃ ]    Hdot_FM_F = 0₆ₓ₃
//                [ 0₃ₓ₃ ]
//
//    H_FM_M = R_MF ⋅ H_FM_F = [ R_MF ]
//                             [  0   ]
//
// @tparam_default_scalar
template <typename T>
class RpyBallMobilizer final : public MobilizerImpl<T, 3, 3> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RpyBallMobilizer);
  using MobilizerBase = MobilizerImpl<T, 3, 3>;
  using MobilizerBase::kNq, MobilizerBase::kNv, MobilizerBase::kNx;
  template <typename U>
  using QVector = typename MobilizerBase::template QVector<U>;
  template <typename U>
  using VVector = typename MobilizerBase::template VVector<U>;
  template <typename U>
  using HMatrix = typename MobilizerBase::template HMatrix<U>;

  // Constructor for an RpyBallMobilizer between an inboard frame F
  // inboard_frame_F and an outboard frame M outboard_frame_M granting
  // three rotational degree of freedom corresponding to angles θ₀, θ₁, θ₂ as
  // described in this class's documentation.
  RpyBallMobilizer(const SpanningForest::Mobod& mobod,
                   const Frame<T>& inboard_frame_F,
                   const Frame<T>& outboard_frame_M)
      : MobilizerBase(mobod, inboard_frame_F, outboard_frame_M) {}

  ~RpyBallMobilizer() final;

  std::unique_ptr<BodyNode<T>> CreateBodyNode(
      const BodyNode<T>* parent_node, const RigidBody<T>* body,
      const Mobilizer<T>* mobilizer) const final;

  bool has_quaternion_dofs() const final { return false; }

  // Overloads to define the suffix names for the position and velocity
  // elements.
  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  bool can_rotate() const final { return true; }
  bool can_translate() const final { return false; }

  // Retrieves from context the three roll-pitch-yaw angles θ₀, θ₁, θ₂ which
  // describe the state for this mobilizer as documented in this class's
  // documentation.
  //
  // @param[in] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @retval angles
  //   The three roll-pitch-yaw angles θ₀, θ₁, θ₂, associated with the sequence
  //   of rotations about the space fixed axes x̂, ŷ, ẑ, respectively packed and
  //   returned as a Vector3 with entries angles(0) = θ₀, angles(1) = θ₁,
  //   angles(2) = θ₂.
  Vector3<T> get_angles(const systems::Context<T>& context) const;

  // Sets in context the state for this mobilizer to have the roll-pitch-yaw
  // angles θ₀, θ₁, θ₂, provided in the input argument angles, which stores
  // them with the format angles = [θ₀, θ₁, θ₂].
  //
  // @param[out] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @param[in] angles
  //   A Vector3 which must pack values for the roll-pitch-yaw angles θ₀, θ₁,
  //   θ₂, described in this class's documentation, at entries angles(0),
  //   angles(1) and angles(2), respectively.
  // @returns a constant reference to this mobilizer.
  const RpyBallMobilizer<T>& SetAngles(systems::Context<T>* context,
                                       const Vector3<T>& angles) const;

  // Sets context so this mobilizer's generalized coordinates (roll-pitch-yaw
  // angles θ₀, θ₁, θ₂) are consistent with the given R_FM rotation matrix.
  // @param[in] context
  //   The context of the MultibodyTree that this mobilizer belongs to.
  // @param[in] R_FM
  //   The rotation matrix relating the orientation of frame F and frame M.
  // @returns a constant reference to this mobilizer.
  const RpyBallMobilizer<T>& SetFromRotationMatrix(
      systems::Context<T>* context, const math::RotationMatrix<T>& R_FM) const;

  // Retrieves from context the angular velocity w_FM of the outboard frame
  // M in the inboard frame F, expressed in F.
  //
  // @param[in] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @retval w_FM
  //   A vector in ℝ³ with the angular velocity of the outboard frame M in the
  //   inboard frame F, expressed in F.
  //
  // @note Many dynamicists follow the convention of expressing angular
  // velocity in the outboard frame M; we return it expressed in the inboard
  // frame F. That is, this method returns W_FM_F.
  Vector3<T> get_angular_velocity(const systems::Context<T>& context) const;

  // Sets in context the state for this mobilizer so that the angular
  // velocity of the outboard frame M in the inboard frame F is w_FM.
  // @param[out] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @param[in] w_FM
  //   A vector in ℝ³ with the desired angular velocity of the outboard frame M
  //   in the inboard frame F, expressed in F.
  // @returns a constant reference to this mobilizer.
  const RpyBallMobilizer<T>& SetAngularVelocity(systems::Context<T>* context,
                                                const Vector3<T>& w_FM) const;

  // Stores in state the angular velocity w_FM of the outboard frame
  // M in the inboard frame F corresponding to this mobilizer.
  //
  // @param[in] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @param[out] state
  //   On return, state will store the angular velocity w_FM of frame F in
  //   frame M.
  // @param[in] w_FM
  //   A vector in ℝ³ with the desired angular velocity of the outboard frame M
  //   in the inboard frame F, expressed in F.
  // @returns a constant reference to this mobilizer.
  const RpyBallMobilizer<T>& SetAngularVelocity(
      const systems::Context<T>& context, const Vector3<T>& w_FM,
      systems::State<T>* state) const;

  // Computes the across-mobilizer transform X_FM(q) between the inboard
  // frame F and the outboard frame M as a function of the roll-pitch-yaw angles
  // θ₀, θ₁, θ₂ stored in context.
  math::RigidTransform<T> calc_X_FM(const T* q) const {
    return math::RigidTransform<T>(math::RollPitchYaw<T>(q[0], q[1], q[2]),
                                   Vector3<T>::Zero());
  }

  /* We're not attempting to optimize the update, but could improve slightly
  since the translation never changes (always zero). */
  void update_X_FM(const T* q, math::RigidTransform<T>* X_FM) const {
    DRAKE_ASSERT(q != nullptr && X_FM != nullptr);
    *X_FM = calc_X_FM(q);
  }

  // Computes the across-mobilizer velocity V_FM(q, v) of the outboard frame
  // M measured and expressed in frame F as a function of the input generalized
  // velocity v which contains the components of the angular velocity w_FM
  // expressed in frame F. The translational velocity is always zero.
  SpatialVelocity<T> calc_V_FM(const T*, const T* v) const {
    const Eigen::Map<const Vector3<T>> w_FM(v);
    return SpatialVelocity<T>(w_FM, Vector3<T>::Zero());
  }

  // Here H_F₆ₓ₃=[I₃ₓ₃ 0₃ₓ₃]ᵀ so Hdot_F=0 and
  // A_FM_F = H_F⋅vdot + Hdot_F⋅v = [vdot, 0₃]ᵀ
  SpatialAcceleration<T> calc_A_FM(const T*, const T*, const T* vdot) const {
    const Eigen::Map<const Vector3<T>> alpha_FM(vdot);
    return SpatialAcceleration<T>(alpha_FM, Vector3<T>::Zero());
  }

  // Returns tau = H_FMᵀ⋅F. The rotational part of H is identity here and
  // the rest is zero.
  void calc_tau(const T*, const SpatialForce<T>& F_BMo_F, T* tau) const {
    DRAKE_ASSERT(tau != nullptr);
    Eigen::Map<VVector<T>> tau_as_vector(tau);
    const Vector3<T>& t_BMo_F = F_BMo_F.rotational();
    tau_as_vector = t_BMo_F;
  }

  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const final;

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const final;

  // Computes the across-mobilizer acceleration A_FM(q, v, v̇) of the
  // outboard frame M in the inboard frame F.
  // The acceleration A_FM will be a function of the generalized positions q
  // (roll-pitch-yaw angles) stored in context, of the generalized velocities
  // v (angular velocity w_FM) also stored in context and of the supplied
  // generalized accelerations vdot, which in this case correspond to angular
  // acceleration of M in F alpha_FM = Dt_F(w_FM) (see
  // @ref Dt_multibody_quantities for our notation of time derivatives in
  // different reference frames).
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const override;

  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_Mo_F,
                           Eigen::Ref<VectorX<T>> tau) const override;

  bool is_velocity_equal_to_qdot() const override { return false; }

 private:
  // Struct that consolidates sine and cosine calculations to facilitate their
  // reuse in other functions.
  // Note: This struct is exempted from the styleguide's prohibition of related
  // members in a struct by virtue of it being strictly internal (the invariants
  // are maintained internally like private members of the class).
  struct SinCosPitchYaw {
    T sin_pitch{std::numeric_limits<T>::quiet_NaN()};
    T cos_pitch{std::numeric_limits<T>::quiet_NaN()};
    T sin_yaw{std::numeric_limits<T>::quiet_NaN()};
    T cos_yaw{std::numeric_limits<T>::quiet_NaN()};
  };

  // Returns a struct with calculated sin(pitch), cos(pitch), sin(yaw),
  // cos(yaw).
  SinCosPitchYaw CalcSinPitchCosPitchSinYawCosYaw(
      const systems::Context<T>& context) const;

  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const final;

  void DoCalcNDotMatrix(const systems::Context<T>& context,
                        EigenPtr<MatrixX<T>> Ndot) const final;

  void DoCalcNplusDotMatrix(const systems::Context<T>& context,
                            EigenPtr<MatrixX<T>> NplusDot) const final;

  // Maps the generalized velocity v, which corresponds to the angular velocity
  // w_FM, to time derivatives of roll-pitch-yaw angles θ₀, θ₁, θ₂ in qdot.
  //
  // @param[in] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @param[in] v
  //   A vector of generalized velocities for this Mobilizer which should
  //   correspond to a vector in ℝ³ for an angular velocity w_FM of M in F.
  // @param[out] qdot
  //   A Vector3 packing of the time derivatives of the roll-pitch-yaw angles
  //   θ₀, θ₁, θ₂ in qdot(0), qdot(1) and qdot(2), respectively.
  //
  // @warning The mapping from Euler angle's rates to angular velocity is
  // singular for angle θ₁ such that θ₁ = π/2 + kπ, ∀ k ∈ ℤ.
  // To avoid working close to this singularity (which could potentially result
  // in large errors for qdot), this method aborts when the absolute value of
  // the cosine of θ₁ is smaller than 10⁻³, a number arbitrarily chosen to this
  // end.
  void DoMapVelocityToQDot(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& v,
                           EigenPtr<VectorX<T>> qdot) const final;

  // Implements DoMapVelocityToQDot() with pre-computed values of
  // sin(pitch), sin(yaw), cos(yaw), 1/cos(pitch).
  void DoMapVelocityToQDotImpl(const SinCosPitchYaw& sin_cos_pitch_yaw,
                               const T& cpi,
                               const Eigen::Ref<const VectorX<T>>& v,
                               EigenPtr<VectorX<T>> qdot) const;

  // Maps time derivatives of the roll-pitch-yaw angles θ₀, θ₁, θ₂ in qdot to
  // the generalized velocity v, which corresponds to the angular velocity
  // w_FM.
  //
  // @param[in] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @param[in] qdot
  //   A vector containing the time derivatives of the roll-pitch-yaw angles
  //   θ₀, θ₁, θ₂ in qdot(0), qdot(1) and qdot(2), respectively.
  // @param[out] v
  //   A vector of generalized velocities for this Mobilizer which should
  //   correspond to a vector in ℝ³ for an angular velocity w_FM of M in F.
  void DoMapQDotToVelocity(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& qdot,
                           EigenPtr<VectorX<T>> v) const final;

  // Maps vdot to qddot, which for this mobilizer is q̈ = Ṅ(q,q̇)⋅v + N(q)⋅v̇.
  // For simple mobilizers q̈ = v̇. This mobilizer's N and Ṅ are more elaborate.
  void DoMapAccelerationToQDDot(const systems::Context<T>& context,
                                const Eigen::Ref<const VectorX<T>>& vdot,
                                EigenPtr<VectorX<T>> qddot) const final;

  // Maps qddot to vdot, which for this mobilizer is v̇ = Ṅ⁺(q,q̇)⋅q̇ + N⁺(q)⋅q̈.
  // For simple mobilizers v̇ = q̈. This mobilizer's N and Ṅ⁺ are more elaborate.
  void DoMapQDDotToAcceleration(const systems::Context<T>& context,
                                const Eigen::Ref<const VectorX<T>>& qddot,
                                EigenPtr<VectorX<T>> vdot) const final;

  // Calculate the term Ṅ⁺(q,q̇)⋅q̇ which appears in v̇ = Ṅ⁺(q,q̇)⋅q̇ + N⁺(q)⋅q̈.
  Vector3<T> CalcAccelerationBiasForQDDot(const systems::Context<T>& context,
                                          const char* function_name) const;

  // Implements CalcAccelerationBiasForQDDot() with pre-computed values of
  // sin(pitch), cos(pitch), sin(yaw), cos(yaw), 1/cos(pitch).
  Vector3<T> CalcAccelerationBiasForQDDotImpl(
      const systems::Context<T>& context,
      const SinCosPitchYaw& sin_cos_pitch_yaw, const T& cpi) const;

  // Certain roll pitch yaw calculations (e.g., calculating the N(q) matrix)
  // have a singularity (divide-by-zero error) when cos(pitch) ≈ 0.
  // The tolerance 1.0e-3 is used to test whether the cosine of the pitch angle
  // is near zero, which occurs when the pitch angle ≈ π/2 ± n π (n=0, 1 2, …).
  // Throw an exception if a pitch angle is within ≈ 0.057° of a singularity.
  void ThrowIfCosPitchNearZero(const systems::Context<T>& context,
                               const T& cos_pitch,
                               const char* function_name) const {
    using std::abs;
    if (abs(cos_pitch) < 1.0e-3)
      ThrowSinceCosPitchNearZero(context, function_name);
  }

  // Ideally, ThrowIfCosPitchNearZero() is inlined by separating this function.
  [[noreturn]] void ThrowSinceCosPitchNearZero(
      const systems::Context<T>& context, const char* function_name) const;

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const override;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::RpyBallMobilizer);
