#pragma once

#include <memory>

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

// This mobilizer fixes the relative pose of an outboard frame M to be
// coincident with an inboard frame F as if "welding" them together.
// Therefore, this mobilizer has no associated state with it.
//
// @tparam_default_scalar
template <typename T>
class WeldMobilizer final : public MobilizerImpl<T, 0, 0> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WeldMobilizer);
  using MobilizerBase = MobilizerImpl<T, 0, 0>;
  using MobilizerBase::kNq, MobilizerBase::kNv, MobilizerBase::kNx;
  template <typename U>
  using QVector = typename MobilizerBase::template QVector<U>;
  template <typename U>
  using VVector = typename MobilizerBase::template VVector<U>;
  template <typename U>
  using HMatrix = typename MobilizerBase::template HMatrix<U>;

  // Constructor for a %WeldMobilizer between the `inboard_frame_F` and
  // `outboard_frame_M`. The frames will be held coincident forever.
  WeldMobilizer(const SpanningForest::Mobod& mobod,
                const Frame<T>& inboard_frame_F,
                const Frame<T>& outboard_frame_M)
      : MobilizerBase(mobod, inboard_frame_F, outboard_frame_M) {}

  ~WeldMobilizer() final;

  std::unique_ptr<BodyNode<T>> CreateBodyNode(
      const BodyNode<T>* parent_node, const RigidBody<T>* body,
      const Mobilizer<T>* mobilizer) const final;

  // Computes the across-mobilizer transform `X_FM`, which for this mobilizer
  // is always the identity transform since F==M by construction.
  math::RigidTransform<T> calc_X_FM(const T*) const {
    return math::RigidTransform<T>();
  }

  /* We should do exactly nothing to update X_FM since it remains identity
  forever. */
  void update_X_FM(const T*, math::RigidTransform<T>* X_FM) const {
    DRAKE_ASSERT(X_FM != nullptr);
    DRAKE_ASSERT(X_FM->IsExactlyIdentity());
    // Do nothing.
  }

  /* Since X_FM is identity, X_AF * X_FM is just X_AF. */
  math::RigidTransform<T> post_multiply_by_X_FM(
      const math::RigidTransform<T>& X_AF,
      const math::RigidTransform<T>&) const {
    return X_AF;
  }

  /* Since X_FM is identity, X_FM * X_MB is just X_MB. */
  math::RigidTransform<T> pre_multiply_by_X_FM(
      const math::RigidTransform<T>&,
      const math::RigidTransform<T>& X_MB) const {
    return X_MB;
  }

  /* Since R_FM is identity, applying it to a vector does nothing. */
  Vector3<T> apply_R_FM(const math::RotationMatrix<T>&,
                        const Vector3<T>& v_M) const {
    return v_M;
  }

  // Computes the across-mobilizer velocity V_FM which for this mobilizer is
  // always zero since the outboard frame M is fixed to the inboard frame F.
  SpatialVelocity<T> calc_V_FM(const T*, const T*) const {
    return SpatialVelocity<T>::Zero();
  }

  /* Computes the across-mobilizer acceleration A_FM which for this mobilizer is
  always zero. */
  SpatialAcceleration<T> calc_A_FM(const T*, const T*, const T*) const {
    return SpatialAcceleration<T>::Zero();
  }

  // Does nothing since there are no taus.
  void calc_tau(const T*, const SpatialForce<T>&, T*) const {}

  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>&) const final;

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>&,
      const Eigen::Ref<const VectorX<T>>&) const final;

  // Computes the across-mobilizer acceleration `A_FM` which for this mobilizer
  // is always zero since the outboard frame M is fixed to the inboard frame F.
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const final;

  // Since this mobilizer has no generalized velocities associated with it,
  // this override is a no-op.
  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_Mo_F,
                           Eigen::Ref<VectorX<T>> tau) const final;

  bool is_velocity_equal_to_qdot() const override { return true; }

  bool can_rotate() const final { return false; }
  bool can_translate() const final { return false; }

 protected:
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const final;

  // Generally, q̈ = Ṅ(q,q̇)⋅v + N(q)⋅v̇. For this mobilizer, Ṅ = 0x0 matrix.
  void DoCalcNDotMatrix(const systems::Context<T>& context,
                        EigenPtr<MatrixX<T>> Ndot) const final;

  // Generally, v̇ = Ṅ⁺(q,q̇)⋅q̇ + N⁺(q)⋅q̈. For this mobilizer, Ṅ⁺ = 0x0 matrix.
  void DoCalcNplusDotMatrix(const systems::Context<T>& context,
                            EigenPtr<MatrixX<T>> NplusDot) const final;

  // This override is a no-op since this mobilizer has no generalized
  // velocities associated with it.
  void DoMapVelocityToQDot(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& v,
                           EigenPtr<VectorX<T>> qdot) const final;

  // This override is a no-op since this mobilizer has no generalized
  // velocities associated with it.
  void DoMapQDotToVelocity(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& qdot,
                           EigenPtr<VectorX<T>> v) const final;

  // This override is a no-op since this mobilizer has no generalized
  // velocities associated with it.
  void DoMapAccelerationToQDDot(const systems::Context<T>& context,
                                const Eigen::Ref<const VectorX<T>>& vdot,
                                EigenPtr<VectorX<T>> qddot) const final;

  // This override is a no-op since this mobilizer has no generalized
  // velocities associated with it.
  void DoMapQDDotToAcceleration(const systems::Context<T>& context,
                                const Eigen::Ref<const VectorX<T>>& qddot,
                                EigenPtr<VectorX<T>> vdot) const final;

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
    class ::drake::multibody::internal::WeldMobilizer);
