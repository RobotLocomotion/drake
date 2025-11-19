#pragma once

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

/* This mobilizer models a planar joint between an inboard frame F and an
 outboard frame M that enables translation along F's x and y axes and rotation
 about F's z-axis.

 The generalized coordinates for this mobilizer (x, y, θ) correspond to
 translations along the x and y axes of frame F and rotaition about the z-axis
 of frame F.
 Zero (x, y, θ) define the "zero configuration" which corresponds to frame F and
 M being coincident and aligned, see SetZeroState(). The translations (x, y)
 are defined to be positive in the direction of their respective axes and the
 rotation θ is defined to be positive according to the right-hand-rule with the
 thumb aligned in the direction of frame F's z-axis.
 The generalized velocities for this mobilizer are the rate of change of the
 coordinates, v = q̇.

 H_FM_F₆ₓ₃ = [0 0 0]    Hdot_FM_F = 0₆ₓ₃
             [0 0 0]
             [0 0 1]    R_FM(q₂) = [c -s  0]
             [1 0 0]               [s  c  0]
             [0 1 0]               [0  0  1]
             [0 0 0]

 H_FM_M = R_MF ⋅ H_FM_F       Hdot_FM_M
        = [ 0 0 0]            = [ 0  0  0]
          [ 0 0 0]              [ 0  0  0]
          [ 0 0 1]              [ 0  0  1]
          [ c s 0]              [-s  c  0]⋅v₂
          [-s c 0]              [-c -s  0]⋅v₂
          [ 0 0 0]              [ 0  0  0]

 @tparam_default_scalar */
template <typename T>
class PlanarMobilizer final : public MobilizerImpl<T, 3, 3> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PlanarMobilizer);
  using MobilizerBase = MobilizerImpl<T, 3, 3>;
  using MobilizerBase::kNq, MobilizerBase::kNv, MobilizerBase::kNx;
  template <typename U>
  using QVector = typename MobilizerBase::template QVector<U>;
  template <typename U>
  using VVector = typename MobilizerBase::template VVector<U>;
  template <typename U>
  using HMatrix = typename MobilizerBase::template HMatrix<U>;

  /* Constructor for a %PlanarMobilizer between an inboard frame F
   `inboard_frame_F` and an outboard frame M `outboard_frame_M` granting two
   translational and one rotational degrees of freedom as described in this
   class's documentation. */
  PlanarMobilizer(const SpanningForest::Mobod& mobod,
                  const Frame<T>& inboard_frame_F,
                  const Frame<T>& outboard_frame_M)
      : MobilizerBase(mobod, inboard_frame_F, outboard_frame_M) {}

  ~PlanarMobilizer() final;

  std::unique_ptr<BodyNode<T>> CreateBodyNode(
      const BodyNode<T>* parent_node, const RigidBody<T>* body,
      const Mobilizer<T>* mobilizer) const final;

  // Overloads to define the suffix names for the position and velocity
  // elements.
  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  bool can_rotate() const final { return true; }
  bool can_translate() const final { return true; }

  /* Retrieves from `context` the two translations (x, y) which describe the
   position for `this` mobilizer as documented in this class's documentation.

   @param[in] context The context of the model this mobilizer belongs to.
   @returns The two translations (x, y) packed and returned as a vector with
            entries `translations(0) = x`, `translations(1) = y`. */
  Vector2<T> get_translations(const systems::Context<T>& context) const;

  /* Sets in `context` the position for `this` mobilizer to the translations
   (x, y) provided in the input argument `translations`, which stores them with
   the format `translations = [x, y]`.

   @param[in] context The context of the model this mobilizer belongs to.
   @param[in] translations A vector which must pack values for the translations
                           (x, y) described in this class's documentation, at
                           entries `translations(0)` and `translations(1)`,
                           respectively.
   @returns A constant reference to `this` mobilizer. */
  const PlanarMobilizer<T>& set_translations(
      systems::Context<T>* context,
      const Eigen::Ref<const Vector2<T>>& translations) const;

  /* Retrieves from `context` the angle θ which describe the orientation for
   `this` mobilizer as documented in this class's documentation.

   @param[in] context The context of the model this mobilizer belongs to.
   @returns The angle θ of the mobilizer. */
  const T& get_angle(const systems::Context<T>& context) const;

  /* Sets in `context` the orientation for `this` mobilizer to the angle θ
   provided by the input argument `angle`.

   @param[in] context The context of the model this mobilizer belongs to.
   @param[in] angle The desired angle in radians.
   @returns a constant reference to `this` mobilizer. */
  const PlanarMobilizer<T>& SetAngle(systems::Context<T>* context,
                                     const T& angle) const;

  /* Retrieves from `context` the rate of change, in meters per second, of
   `this` mobilizer's translations (see get_translations()).
   @param[in] context The context of the model this mobilizer belongs to.
   @returns The rate of change of the two translations (ẋ, ẏ) returned as
            the vector [ẋ, ẏ], which is v_FM_F. */
  Vector2<T> get_translation_rates(const systems::Context<T>& context) const;

  /* Sets in `context` the rate of change, in meters per second, of `this`
   mobilizer's translations (see get_translations()) to `v_FM_F`.
   @param[in] context The context of the model this mobilizer belongs to.
   @param[in] v_FM_F The desired rate of change of `this` mobilizer's
                     translations, packed as the vector [ẋ, ẏ].
   @returns A constant reference to `this` mobilizer. */
  const PlanarMobilizer<T>& SetTranslationRates(
      systems::Context<T>* context,
      const Eigen::Ref<const Vector2<T>>& v_FM_F) const;

  /* Retrieves from `context` the rate of change, in radians per second, of
   `this` mobilizer's angle (see get_angle()).
   @param[in] context The context of the model this mobilizer belongs to.
   @returns The rate of change of `this` mobilizer's angle. */
  const T& get_angular_rate(const systems::Context<T>& context) const;

  /* Sets in `context` the rate of change, in radians per second, of `this`
   mobilizer's angle (see angle()) to `theta_dot`.
   @param[in] context The context of the model this mobilizer belongs to.
   @param[in] theta_dot The desired rate of change of `this` mobilizer's angle
                        in radians per second.
   @returns A constant reference to `this` mobilizer. */
  const PlanarMobilizer<T>& SetAngularRate(systems::Context<T>* context,
                                           const T& theta_dot) const;

  /* Computes the across-mobilizer transform `X_FM(q)` between the inboard
  frame F and the outboard frame M as a function of the configuration q stored
  in `context`. */
  math::RigidTransform<T> calc_X_FM(const T* q) const {
    return math::RigidTransform<T>(math::RotationMatrix<T>::MakeZRotation(q[2]),
                                   Vector3<T>(q[0], q[1], 0.0));
  }

  /* We're not yet attempting to optimize the X_FM update, though we could. */
  void update_X_FM(const T* q, math::RigidTransform<T>* X_FM) const {
    DRAKE_ASSERT(q != nullptr && X_FM != nullptr);
    *X_FM = calc_X_FM(q);
  }

  /* Computes the across-mobilizer velocity V_FM(q, v) of the outboard frame
  M measured and expressed in frame F as a function of the input velocity v. */
  SpatialVelocity<T> calc_V_FM(const T*, const T* v) const {
    return SpatialVelocity<T>(Vector3<T>(0.0, 0.0, v[2]),
                              Vector3<T>(v[0], v[1], 0.0));
  }

  /* Returns H_FM_F⋅vdot + Hdot_FM_F⋅v. See class description. */
  SpatialAcceleration<T> calc_A_FM(const T*, const T*, const T* vdot) const {
    return SpatialAcceleration<T>(Vector3<T>(0.0, 0.0, vdot[2]),
                                  Vector3<T>(vdot[0], vdot[1], 0.0));
  }

  /* Returns tau = H_FM_Fᵀ⋅F_F */
  void calc_tau(const T*, const SpatialForce<T>& F_BMo_F, T* tau) const {
    DRAKE_ASSERT(tau != nullptr);
    const Vector3<T>& t_B_F = F_BMo_F.rotational();       // torque
    const Vector3<T>& f_BMo_F = F_BMo_F.translational();  // force
    tau[0] = f_BMo_F[0];                                  // force along x
    tau[1] = f_BMo_F[1];                                  // force along y
    tau[2] = t_B_F[2];                                    // torque about z
  }

  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const final;

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const final;

  /* Computes the across-mobilizer acceleration `A_FM(q, v, v̇)` of the outboard
   frame M in the inboard frame F.
   By definition `A_FM = d_F(V_FM)/dt = H_FM(q) * v̇ + Ḣ_FM * v`. The
   acceleration `A_FM` will be a function of the configuration q and the
   velocity v from the `context` as well as the generalized accelerations
   `v̇ = dv/dt`, the rates of change of v.
   This method aborts in Debug builds if `vdot.size()` is not three. */
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const override;

  /* Projects the spatial force `F_Mo = [τ_Mo, f_Mo]` on `this` mobilizer's
   outboard frame M onto the rotational axis, z, and translational axes, x and
   y. Mathematically:
   <pre>
      tau = [f_Mo⋅Fx]
            [f_Mo⋅Fy]
            [τ_Mo⋅Fz]
   </pre>
   Therefore, the result of this method is the vector of torques and forces for
   each degree of freedom of `this` mobilizer.
   This method aborts in Debug builds if `tau.size()` is not three. */
  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_Mo_F,
                           Eigen::Ref<VectorX<T>> tau) const override;

  bool is_velocity_equal_to_qdot() const override { return true; }

 protected:
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const final;

  // Maps v to qdot, which for this mobilizer is q̇ = v.
  void DoMapVelocityToQDot(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& v,
                           EigenPtr<VectorX<T>> qdot) const final;

  // Maps qdot to v, which for this mobilizer is v = q̇.
  void DoMapQDotToVelocity(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& qdot,
                           EigenPtr<VectorX<T>> v) const final;

  // Generally, q̈ = Ṅ(q,q̇)⋅v + N(q)⋅v̇. For this mobilizer, Ṅ = zero matrix.
  void DoCalcNDotMatrix(const systems::Context<T>& context,
                        EigenPtr<MatrixX<T>> Ndot) const final;

  // Generally, v̇ = Ṅ⁺(q,q̇)⋅q̇ + N⁺(q)⋅q̈. For this mobilizer, Ṅ⁺ = zero matrix.
  void DoCalcNplusDotMatrix(const systems::Context<T>& context,
                            EigenPtr<MatrixX<T>> NplusDot) const final;

  // Maps vdot to qddot, which for this mobilizer is q̈ = v̇.
  void DoMapAccelerationToQDDot(const systems::Context<T>& context,
                                const Eigen::Ref<const VectorX<T>>& vdot,
                                EigenPtr<VectorX<T>> qddot) const final;

  // Maps qddot to vdot, which for this mobilizer is v̇ = q̈.
  void DoMapQDDotToAcceleration(const systems::Context<T>& context,
                                const Eigen::Ref<const VectorX<T>>& qddot,
                                EigenPtr<VectorX<T>> vdot) const final;

  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const override;

 private:
  /* Helper method to make a clone templated on ToScalar. */
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::PlanarMobilizer);
