#pragma once

#include <limits>
#include <memory>
#include <string>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_constant_curvature_trajectory.h"
#include "drake/math/wrap_to.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/mobilizer_impl.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

/* A Mobilizer that describes the motion of the mobilized frame M along a
 piecewise constant curvature path contained in a plane.

 The path is specified as a PiecewiseConstantCurvatureTrajectory, refer to that
 class documentation for further details on parameterization, conventions and
 notation used. (The "A" frame there corresponds to the "F" frame here; "M" is
 the same in both places.)

 This mobilizer grants a single degree of freedom q that corresponds to the
 length s (in meters) along the path. The generalized velocity v = q̇ (= ṡ)
 corresponds to the signed magnitude of the tangential velocity. The mobilized
 frame M is defined according to the convention documented in
 PiecewiseConstantCurvatureTrajectory. That is, axis Mx is the tangent to the
 trajectory, Mz equals the (constant) normal p̂ to the plane, and My = Mz ⨯ Mx.
 It is not required that M coincides with F at distance q = 0.

 At any given point along the path, the turning rate ρ(q) (units of 1/m) is
 defined such the angular velocity is given by ω_FM = ρ⋅v⋅Mz_F, i.e. |ρ(q)|
 corresponds to the path's curvature and its sign is defined via the right-hand
 rule about the plane's normal Mz.

 If the specified trajectory is periodic, the mobilizer describes a trajectory
 of length s_f that satisfies X_FM(q) = X_FM(q + s_f) and ρ(q) = ρ(q + s_f).

 Therefore the hinge matrix H_FM ∈ ℝ⁶ˣ¹ and its time derivative are

    H_FM_F(q) = [ρ(q)⋅Mz_F(q)]        Hdot_FM_F(q,v) = [       0      ]
                [     Mx_F(q)],                        [ρ(q)⋅v⋅My_F(q)]

    (Recall that the time derivative taken in F of a vector r⃗ fixed in a frame
     M is given by ω_FM ⨯ r⃗, so DtF(Mx) = (ρ⋅v⋅Mz) ⨯ Mx = ρ⋅v⋅My.)

 This is of course much nicer in M:
    H_FM_M(q) = [0 0 ρ(q) 1 0 0]ᵀ     Hdot_FM_M = [ 0₆ ]ᵀ

 The angular component of Hdot_FM is zero since ρ(q) is constant within each
 piecewise segment in PiecewiseConstantCurvatureTrajectory.

 @tparam_default_scalar */
template <typename T>
class CurvilinearMobilizer final : public MobilizerImpl<T, 1, 1> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CurvilinearMobilizer);
  using MobilizerBase = MobilizerImpl<T, 1, 1>;
  using MobilizerBase::kNq, MobilizerBase::kNv, MobilizerBase::kNx;
  template <typename U>
  using QVector = typename MobilizerBase::template QVector<U>;
  template <typename U>
  using VVector = typename MobilizerBase::template VVector<U>;
  template <typename U>
  using HMatrix = typename MobilizerBase::template HMatrix<U>;

  /* Constructor for a CurvilinearMobilizer describing the motion of outboard
   frame M along `curvilinear_path` with pose X_FM in the inboard frame F. Refer
   to the class documentation for further details on notation and conventions.

   `curvilinear_path` provides the pose X_FM(s) along a piecewise constant
   curvature trajectory. In particular, X_FM(s=0) does not need to be identity
   matrix.

   @param mobod Topological information for this mobilizer.
   @param inboard_frame_F the inboard frame F.
   @param outboard_frame_M the outboard frame M.
   @param curvilinear_path the curvilinear path defining X_FM(q). */
  CurvilinearMobilizer(
      const SpanningForest::Mobod& mobod, const Frame<T>& inboard_frame_F,
      const Frame<T>& outboard_frame_M,
      const trajectories::PiecewiseConstantCurvatureTrajectory<double>&
          curvilinear_path);

  ~CurvilinearMobilizer() final;

  std::unique_ptr<internal::BodyNode<T>> CreateBodyNode(
      const internal::BodyNode<T>* parent_node, const RigidBody<T>* body,
      const Mobilizer<T>* mobilizer) const final;

  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  bool can_rotate() const final { return true; }
  bool can_translate() const final { return true; }

  /* Gets the distance of travel along the path of the mobilizer from
   the provided context in meters.
   @param context The context of the model this mobilizer belongs to.
   @returns The distance coordinate of the mobilizer in the context. */
  const T& get_distance(const systems::Context<T>& context) const;

  /* Sets the distance of travel along the path of the mobilizer from
   the provided context in meters.
   @param context The context of the model this mobilizer belongs to.
   @param distance The desired distance coordinate of the mobilizer.
   @returns a const reference to this mobilizer */
  const CurvilinearMobilizer<T>& SetDistance(systems::Context<T>* context,
                                             const T& distance) const;

  /* Gets the tangential velocity of the mobilizer from the provided context in
   meters per second.
   @param context The context of the model this mobilizer belongs to.
   @returns The velocity coordinate of the mobilizer in the context. */
  const T& get_tangential_velocity(const systems::Context<T>& context) const;

  /* Sets the tangential velocity of the mobilizer from the provided context in
   meters per second.
   @param context The context of the model this mobilizer belongs to.
   @param tangential_velocity The desired velocity coordinate of the mobilizer.
   @returns a const reference to this mobilizer */
  const CurvilinearMobilizer<T>& SetTangentialVelocity(
      systems::Context<T>* context, const T& tangential_velocity) const;

  /* Computes the across-mobilizer transform X_FM(q) as a function of the
   distance traveled along the mobilizer's path.
   @param q The distance traveled along the mobilizer's path in meters.
   @returns The across-mobilizer transform X_FM(q). */
  math::RigidTransform<T> calc_X_FM(const T* q) const;

  /* We're not attempting to optimize the X_FM update. */
  void update_X_FM(const T* q, math::RigidTransform<T>* X_FM) const {
    DRAKE_ASSERT(q != nullptr && X_FM != nullptr);
    *X_FM = calc_X_FM(q);
  }

  /* Computes the across-mobilizer spatial velocity V_FM(q, v) as a function of
   the distance traveled and tangential velocity along the mobilizer's path.
   @param q The distance traveled along the mobilizer's path in meters.
   @param v The tangential velocity along the mobilizer's path in meters per
   second.
   @returns The across-mobilizer spatial velocity V_FM(q, v). */
  SpatialVelocity<T> calc_V_FM(const T* q, const T* v) const;

  /* Alternate function that returns the spatial velocity expressed in M. */
  SpatialVelocity<T> calc_V_FM_M(const math::RigidTransform<T>&, const T* q,
                                 const T* v) const;

  /* Computes the across-mobilizer spatial acceleration A_FM(q, v, v̇) as a
   function of the distance traveled, tangential velocity, and tangential
   acceleration along the mobilizer's path.
   @param q The distance traveled along the mobilizer's path in meters.
   @param v The tangential velocity along the mobilizer's path in meters per
   second.
   @param vdot The tangential acceleration along the mobilizer's path in meters
   per second squared.
   @returns The across-mobilizer spatial acceleration A_FM(q, v, v̇). */
  SpatialAcceleration<T> calc_A_FM(const T* q, const T* v, const T* vdot) const;

  /* Alternate function that returns the spatial acceleration expressed in M. */
  SpatialAcceleration<T> calc_A_FM_M(const math::RigidTransform<T>&, const T* q,
                                     const T* v, const T* vdot) const;

  /* Projects the spatial force `F_BMo_F` on `this` mobilizer's outboard
   frame M onto the path tangent. Mathematically, tau = H_FM_Fᵀ⋅F_BMo_F.

   The result of this projection is the magnitude of a force (in N) along
   the path that would cause the same acceleration as F_BMo_F.

   @param q The distance traveled along the mobilizer's path in meters.
   @param F_BMo_F The spatial force applied to the mobilized body B at Mo,
   expressed in F.
   @param[out] tau A pointer to store the resulting generalized force in
   Newtons.
   @pre tau is not the nullptr.
   @pre tau->size() equals one. */
  void calc_tau(const T* q, const SpatialForce<T>& F_BMo_F, T* tau) const;

  /* Alternate projection function that expects the force to be expressed in the
   M frame instead of F (producing the same resulting tau). Returns
   tau = H_FM_Mᵀ⋅F_BMo_M (see class comments). */
  void calc_tau_from_M(const math::RigidTransform<T>&, const T* q,
                       const SpatialForce<T>& F_BMo_M, T* tau) const;

  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const final;

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const final;

  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const override;

  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_BMo_F,
                           Eigen::Ref<VectorX<T>> tau) const override;

  bool is_velocity_equal_to_qdot() const override { return true; }

 protected:
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const final;

  // Generally, q̈ = Ṅ(q,q̇)⋅v + N(q)⋅v̇. For this mobilizer, Ṅ = zero matrix.
  void DoCalcNDotMatrix(const systems::Context<T>& context,
                        EigenPtr<MatrixX<T>> Ndot) const final;

  // Generally, v̇ = Ṅ⁺(q,q̇)⋅q̇ + N⁺(q)⋅q̈. For this mobilizer, Ṅ⁺ = zero matrix.
  void DoCalcNplusDotMatrix(const systems::Context<T>& context,
                            EigenPtr<MatrixX<T>> NplusDot) const final;

  // Maps v to qdot, which for this mobilizer is q̇ = v.
  void DoMapVelocityToQDot(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& v,
                           EigenPtr<VectorX<T>> qdot) const final;

  // Maps qdot to v, which for this mobilizer is v = q̇.
  void DoMapQDotToVelocity(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& qdot,
                           EigenPtr<VectorX<T>> v) const final;

  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const override;

 private:
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  trajectories::PiecewiseConstantCurvatureTrajectory<T> curvilinear_path_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::CurvilinearMobilizer);
