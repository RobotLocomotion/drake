#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/mobilizer_impl.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

// This mobilizer introduces six degrees of freedom to model arbitrary rigid
// body motions of an outboard frame M relative to an inboard frame F.
// Translational motions of M in F are parametrized by the position p_FM of
// frame M in F, measured in frame F. As hinted by the name of this class, the
// orientation R_FM of the outboard frame M in F is parameterized with
// roll-pitch-yaw angles (also known as space-fixed x-y-z Euler angles or
// extrinsic angles). That is, the first three generalized coordinates q for
// this mobilizer correspond to angles q=[θ₀ θ₁ θ₂], for a sequence of rotations
// about the Fx, Fy, and Fz axes, respectively. Mathematically, rotation R_FM is
// given in terms of angles θ₀, θ₁, θ₂ by:
//
//   R_FM(q) = Rz(θ₂) * Ry(θ₁) * Rx(θ₀)
//
// where Rx(θ), Ry(θ) and Rz(θ) correspond to the elemental rotations in
// amount of θ about the Fx, Fy and Fz axes respectively. Refer to
// math::RollPitchYaw for further details on this representation. Zero θ₀, θ₁,
// θ₂ angles and zero position p_FM define the "zero configuration" which
// corresponds to frames F and M being coincident, see SetZeroState(). Angles
// θ₀, θ₁, θ₂ are defined to be positive according to the right-hand-rule with
// the thumb aligned in the direction of their respective axes.
//
// The generalized velocities v for this mobilizer correspond to the angular
// velocity w_FM of frame M in F followed by the translational velocity v_FM of
// M in F, both expressed in the inboard frame F. Note that this is _not_ the
// same as the time derivative q̇ the rotation angles; you must use
// q̇ = MapVelocityToQDot(w_FM) to get the angle time derivatives.
//
// While the mapping MapQDotToVelocity() always exists, its inverse
// MapVelocityToQDot() is singular for values of θ₁ (many times referred to as
// the pitch angle) such that θ₁ = π/2 + kπ, ∀ k ∈ ℤ. In a related matter, there
// are no range limits on the allowed values of the r-p-y angles θ₀, θ₁, θ₂. All
// operations performed by this mobilizer are valid for θᵢ ∈ ℝ. The only
// operations that are not well defined at θ₁ = π/2 + kπ, ∀ k ∈ ℤ are those
// related to the kinematic mapping from angular velocity w_FM to Euler angle
// rates, namely MapVelocityToQDot() and CalcNMatrix().
//
// @note Roll-pitch-yaw (space x-y-z) angles (extrinsic, about Fx Fy Fz) are
// equivalent to body-fixed z-y-x angles (intrinsic, about Mz M'y M''x where
// the primes indicate new axis directions after each rotation).
//
// @note The roll-pitch-yaw (space x-y-z) Euler sequence is also known as the
// Tait-Bryan angles or Cardan angles.
//
//   H_FM₆ₓ₆ = I₆ₓ₆     Hdot_FM₆ₓ₆ = 0₆ₓ₆
//
//   H_FM_M = R_MF ⋅ H_FM_F = [ R_MF   0₃ₓ₃ ]
//                            [ 0₃ₓ₃   R_MF ]
//
// @tparam_default_scalar
template <typename T>
class RpyFloatingMobilizer final : public MobilizerImpl<T, 6, 6> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RpyFloatingMobilizer);
  using MobilizerBase = MobilizerImpl<T, 6, 6>;
  using MobilizerBase::kNq, MobilizerBase::kNv, MobilizerBase::kNx;
  template <typename U>
  using QVector = typename MobilizerBase::template QVector<U>;
  template <typename U>
  using VVector = typename MobilizerBase::template VVector<U>;
  template <typename U>
  using HMatrix = typename MobilizerBase::template HMatrix<U>;

  // Constructor for an RpyFloatingMobilizer between an inboard frame F
  // inboard_frame_F and an outboard frame M outboard_frame_M.
  RpyFloatingMobilizer(const SpanningForest::Mobod& mobod,
                       const Frame<T>& inboard_frame_F,
                       const Frame<T>& outboard_frame_M)
      : MobilizerBase(mobod, inboard_frame_F, outboard_frame_M) {}

  ~RpyFloatingMobilizer() final;

  std::unique_ptr<BodyNode<T>> CreateBodyNode(
      const BodyNode<T>* parent_node, const RigidBody<T>* body,
      const Mobilizer<T>* mobilizer) const final;

  bool has_quaternion_dofs() const final { return false; }

  // Overloads to define the suffix names for the position and velocity
  // elements.
  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  bool can_rotate() const final { return true; }
  bool can_translate() const final { return true; }

  // Returns the generalized positions for this mobilizer stored in context.
  // Generalized positions q for this mobilizer are packed in exactly the
  // following order: q = [θ₀, θ₁, θ₂, px_FM, py_FM, pz_FM] that is, rpy
  // angles are stored in the first three entries of the configuration vector,
  // followed by the three translational coordinates.
  Vector6<T> get_generalized_positions(
      const systems::Context<T>& context) const;

  // Returns the generalized velocities for this mobilizer stored in context.
  // Generalized velocities v for this mobilizer are packed in exactly the
  // following order: v = [wx_FM, wy_FM, wz_FM, vx_FM, vy_FM, vz_FM] that is,
  // the components of the angular velocity w_FM are stored in the first three
  // entries of the generalized velocities vector, followed by the three
  // components of the translational velocity v_FM.
  Vector6<T> get_generalized_velocities(
      const systems::Context<T>& context) const;

  // Retrieves the three roll-pitch-yaw angles θ₀, θ₁, θ₂ stored in context,
  // which describe the state for this mobilizer as documented in this class's
  // documentation.
  //
  // @param[in] context
  //   The context of the model this mobilizer belongs to.
  // @retval angles
  //   The three roll-pitch-yaw angles θ₀, θ₁, θ₂, associated with the sequence
  //   of rotations about the space fixed axes x̂, ŷ, ẑ, respectively packed and
  //   returned as a Vector3 with entries angles(0) = θ₀, angles(1) = θ₁,
  //   angles(2) = θ₂. There are no range limits for the angular values.
  Vector3<T> get_angles(const systems::Context<T>& context) const;

  // Retrieves the position p_FM stored in context.
  //
  // @param[in] context
  //   The context of the model this mobilizer belongs to.
  // @retval p_FM
  //   The position of M in F.
  Vector3<T> get_translation(const systems::Context<T>& context) const;

  // Retrieves from context the angular velocity w_FM of the outboard frame
  // M in the inboard frame F, expressed in F.
  //
  // @param[in] context
  //   The context of the model this mobilizer belongs to.
  // @retval w_FM
  //   A vector in ℝ³ with the angular velocity of the outboard frame M in the
  //   inboard frame F, expressed in F.
  Vector3<T> get_angular_velocity(const systems::Context<T>& context) const;

  // Retrieves from context the translational velocity v_FM of the outboard
  // frame M in the inboard frame F, expressed in F.
  //
  // @param[in] context
  //   The context of the model this mobilizer belongs to.
  // @retval v_FM
  //   A vector in ℝ³ with the translational velocity of the outboard frame M in
  //   the inboard frame F, expressed in F.
  Vector3<T> get_translational_velocity(
      const systems::Context<T>& context) const;

  // Stores in context the roll-pitch-yaw angles θ₀, θ₁, θ₂, provided in the
  // input argument angles, which stores the with the format angles = [θ₀,
  // θ₁, θ₂].
  //
  // @param[in,out] context
  //   The context of the model this mobilizer belongs to.
  // @param[in] angles
  //   A Vector3 which must pack values for the roll-pitch-yaw angles
  //   θ₀, θ₁, θ₂, described in this class's documentation, at entries
  //   angles(0), angles(1) and angles(2), respectively.
  // @returns a constant reference to this mobilizer.
  const RpyFloatingMobilizer<T>& SetAngles(systems::Context<T>* context,
                                           const Vector3<T>& angles) const;

  // Stores in context the position p_FM of M in F.
  //
  // @param[in,out] context
  //   The context of the model this mobilizer belongs to.
  // @param[in] p_FM
  //   Position of F in M.
  // @returns a constant reference to this mobilizer.
  const RpyFloatingMobilizer<T>& SetTranslation(systems::Context<T>* context,
                                                const Vector3<T>& p_FM) const;

  // Sets the distribution governing the random samples of the rpy angles
  // component of the mobilizer state.
  void set_random_angles_distribution(
      const Vector3<symbolic::Expression>& angles);

  // Sets the distribution governing the random samples of the translation
  // component of the mobilizer state.
  void set_random_translation_distribution(
      const Vector3<symbolic::Expression>& p_FM);

  // Sets in context the state for this mobilizer so that the angular
  // velocity of the outboard frame M in the inboard frame F is w_FM.
  // @param[in,out] context
  //   The context of the model this mobilizer belongs to.
  // @param[in] w_FM
  //   A vector in ℝ³ with the desired angular velocity of the outboard frame M
  //   in the inboard frame F, expressed in F.
  // @returns a constant reference to this mobilizer.
  const RpyFloatingMobilizer<T>& SetAngularVelocity(
      systems::Context<T>* context, const Vector3<T>& w_FM) const;

  // Stores in context the translational velocity v_FM of M in F.
  //
  // @param[in,out] context
  //   The context of the model this mobilizer belongs to.
  // @param[in] v_FM
  //   Translational velocity of F in M.
  // @returns a constant reference to this mobilizer.
  const RpyFloatingMobilizer<T>& SetTranslationalVelocity(
      systems::Context<T>* context, const Vector3<T>& v_FM) const;

  // Sets context so this mobilizer's generalized coordinates (roll-pitch-yaw
  // angles θ₀, θ₁, θ₂ and position p_FM) represent the given rigid
  // transform X_FM.
  // @param[in,out] context
  //   The context of the model that this mobilizer belongs to.
  // @param[in] X_FM
  //   Pose of M in F.
  // @returns a constant reference to this mobilizer.
  // @note Even though there is no range limit for the angles θ₀,
  // θ₁, θ₂, this specific method will generate roll-pitch-yaw angles in the
  // range -π <= θ₀ <= π, -π/2 <= θ₁ <= π/2, -π <= θ₂ <= π.
  const RpyFloatingMobilizer<T>& SetFromRigidTransform(
      systems::Context<T>* context, const math::RigidTransform<T>& X_FM) const;

  // Computes the across-mobilizer transform X_FM(q) between the inboard
  // frame F and the outboard frame M as a function of the configuration q
  // stored in context.
  math::RigidTransform<T> calc_X_FM(const T* q) const {
    return math::RigidTransform<T>(math::RollPitchYaw<T>(q[0], q[1], q[2]),
                                   Vector3<T>(q[3], q[4], q[5]));
  }

  /* There's nothing to optimize for the X_FM update. */
  void update_X_FM(const T* q, math::RigidTransform<T>* X_FM) const {
    DRAKE_ASSERT(q != nullptr && X_FM != nullptr);
    *X_FM = calc_X_FM(q);
  }

  // Computes the across-mobilizer velocity V_FM(q, v) of the outboard frame M
  // measured and expressed in frame F as a function of the input generalized
  // velocity v, packed as documented in get_generalized_velocities(). (That's
  // conveniently just V_FM already.)
  SpatialVelocity<T> calc_V_FM(const T*, const T* v) const {
    const Eigen::Map<const VVector<T>> V_FM(v);
    return SpatialVelocity<T>(V_FM);  // w_FM, v_FM
  }

  // We chose the generalized velocities for this mobilizer so that H=I, Hdot=0.
  // Therefore A_FM = H⋅vdot + Hdot⋅v = vdot.
  SpatialAcceleration<T> calc_A_FM(const T*, const T*, const T* vdot) const {
    const Eigen::Map<const VVector<T>> A_FM(vdot);
    return SpatialAcceleration<T>(A_FM);
  }

  // Returns tau = H_FMᵀ⋅F. H is identity for this mobilizer.
  void calc_tau(const T*, const SpatialForce<T>& F_BMo_F, T* tau) const {
    DRAKE_ASSERT(tau != nullptr);
    Eigen::Map<VVector<T>> tau_as_vector(tau);
    tau_as_vector = F_BMo_F.get_coeffs();
  }

  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const final;

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const final;

  // Computes the across-mobilizer acceleration A_FM(q, v, v̇) of the
  // outboard frame M in the inboard frame F.
  // The acceleration A_FM will be a function of the generalized positions q
  // and generalized velocities v stored in context and of the supplied
  // generalized accelerations vdot. vdot must contain the rates of change
  // of each of the generalized velocities components packed in the order
  // documented in get_generalized_velocities(). For this mobilizer this
  // corresponds to vdot = [alpha_FM; a_FM], with alpha_FM = Dt_F(w_FM) and
  // a_FM the angular and translational accelerations of M in F, respectively
  // (see @ref Dt_multibody_quantities for our notation of time derivatives in
  // different reference frames.)
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const final;

  // See Mobilizer::ProjectSpatialForce() for details.
  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_Mo_F,
                           Eigen::Ref<VectorX<T>> tau) const final;

  bool is_velocity_equal_to_qdot() const final { return false; }

 protected:
  std::optional<QVector<T>> DoPoseToPositions(
      const Eigen::Quaternion<T> orientation,
      const Vector3<T>& translation) const final;

  std::optional<VVector<T>> DoSpatialVelocityToVelocities(
      const SpatialVelocity<T>& velocity) const final {
    return velocity.get_coeffs();
  }

  // Implements Mobilizer's NVI, see Mobilizer::CalcNMatrix() for details.
  // @warning The mapping from angular velocity to Euler angle's rates is
  // singular for angle θ₁ such that θ₁ = π/2 + kπ, ∀ k ∈ ℤ. To avoid
  // working close to this singularity (which could potentially result in large
  // errors for qdot), this method aborts when the absolute value of the
  // cosine of θ₁ is smaller than 10⁻³, a number arbitrarily chosen to this end.
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  // Implements Mobilizer's NVI, see Mobilizer::DoCalcNplusMatrix() for details.
  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const final;

  // Maps the generalized velocity v to time derivatives of configuration
  // qdot.
  //
  // @param[in] context
  //   The context of the model this mobilizer belongs to.
  // @param[in] v
  //   A vector of generalized velocities for this mobilizer, packed as
  //   documented in get_generalized_velocities().
  // @param[out] qdot
  //   Rates of the generalized positions, packed as documented in
  //   get_generalized_positions().
  //
  // @warning The mapping from angular velocity to Euler angle's rates is
  // singular for angle θ₁ such that θ₁ = π/2 + kπ, ∀ k ∈ ℤ. To avoid
  // working close to this singularity (which could potentially result in large
  // errors for qdot), this method aborts when the absolute value of the
  // cosine of θ₁ is smaller than 10⁻³, a number arbitrarily chosen to this end.
  void DoMapVelocityToQDot(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& v,
                           EigenPtr<VectorX<T>> qdot) const final;

  // Maps time derivatives of the configuration in qdot to
  // the generalized velocities v.
  //
  // @param[in] context
  //   The context of the model this mobilizer belongs to.
  // @param[in] qdot
  //   Rates of the generalized positions, packed as documented in
  //   get_generalized_positions().
  // @param[out] v
  //   A vector of generalized velocities for this mobilizer, packed as
  //   documented in get_generalized_velocities().
  void DoMapQDotToVelocity(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& qdot,
                           EigenPtr<VectorX<T>> v) const final;

  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const final;

  std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const final;

 private:
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::RpyFloatingMobilizer);
