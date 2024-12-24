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
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

using drake::trajectories::PiecewiseConstantCurvatureTrajectory;

namespace drake {
namespace multibody {
namespace internal {

// Tolerance used to check joint periodicity.
static constexpr double kCurvilinearJointPeriodicityTolerance =
    1e3 * std::numeric_limits<double>::epsilon();

/** A Mobilizer which allows two frames to translate and rotate relatively to
 one another along a curvilinear path composed of line segments and circular
 arcs within a plane.

 The path is specified as a PiecewiseConstantCurvatureTrajectory.

 The mobilizer may be specified to be periodic, representing a path shaped as a
 closed loop. In this case, the path must return to the starting pose at its end
 distance s_f [m]. The transfrom X_FM(q) from the CurvilinearMobilizer's inboard
 frame F to the outboard frame M in this case is periodic with period s_f
 (X_FM(q) = X_FM(q + s_f)).

 The single generalized coordinate q [m] introduced by this mobilizer
 corresponds to the distance of travel along the path. The transfrom X_FM(q)
 from the CurvilinearMobilizer's inboard frame F to the outboard frame M is set
 to be the path-aligned pose at distance s(q) [m], available through
 PiecewiseConstantCurvatureTrajectory::CalcPose. For aperiodic trajectories,
 s(q) = q. For periodic trajectories, s(q) is wrapped using modular arithmetic,
 i.e. s(q) = q - s_f⋅floor(q/s_f) [m].  The generalized velocity v = q̇ [m/s] is
 the tangential velocity along the path.

 The path lies within a plane with normal axis p̂, equal to the z axis of the
 mobilized frame Mz. p̂ is not necessarily equal to Fz, but p̂_F is constant.

 At any given point along the path, a scalar turning rate ρ(q) [1/m] is defined
 via right-hand rule about the planar axis p̂, such that the angular velocity
 across the mobilizer is w_FM = ρ⋅p̂⋅v. The hinge matrix and time derivative
 are

    H_FM₆ₓ₁(q) = [       Mx_F(q),     Hdot_FM₆ₓ₁(q) = [ρ(q)⋅My_F(q)⋅v,
                    ρ(q)⋅Mz_F(q)],                                  0].

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

  /** Constructor for a CurvilinearMobilizer between the inboard frame F
   and the outboard frame M granting a single degree of freedom expressed by the
   pose X_FM(q) of the path `curvilinear_path` at distance q [m] along the path.
   The path is required to be a closed loop if the parameter `is_periodic` is
   set to `true`.

   @param mobod information for the mobilized body attached to frame M
   @param inboard_frame_F the inboard frame F
   @param outboard_frame_M the outboard frame M
   @param curvilinear_path the curvilinear path defining X_FM
   @param is_periodic if true, the mobilizer is periodic, and the path must be a
   closed loop.

   @throws std::exception if `is_periodic` is true, but `curvilinear_path` is
   not (nearly) periodic. */
  CurvilinearMobilizer(
      const SpanningForest::Mobod& mobod, const Frame<T>& inboard_frame_F,
      const Frame<T>& outboard_frame_M,
      const PiecewiseConstantCurvatureTrajectory<double>& curvilinear_path,
      bool is_periodic)
      : MobilizerBase(mobod, inboard_frame_F, outboard_frame_M),
        curvilinear_path_(curvilinear_path),
        is_periodic_(is_periodic) {
    bool path_is_periodic = curvilinear_path.IsNearlyPeriodic(
        kCurvilinearJointPeriodicityTolerance);
    if (is_periodic_ && !path_is_periodic) {
      // The path not periodic, but the mobilizer is.
      throw std::runtime_error(
          "CurvilinearMobilizer: Periodic mobilizer must be constructed with "
          "periodic path.");
    }
  }

  ~CurvilinearMobilizer() final;

  std::unique_ptr<internal::BodyNode<T>> CreateBodyNode(
      const internal::BodyNode<T>* parent_node, const RigidBody<T>* body,
      const Mobilizer<T>* mobilizer) const final;

  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  bool can_rotate() const final { return true; }
  bool can_translate() const final { return true; }

  /** Gets the distance of travel along the path of the mobilizer from
   the provided context in meters.
   @param context The context of the model this mobilizer belongs to.
   @returns The distance coordinate of the mobilizer in the context. */
  const T& get_distance(const systems::Context<T>& context) const;

  /** Sets the distance of travel along the path of the mobilizer from
   the provided context in meters.
   @param context The context of the model this mobilizer belongs to.
   @param distance The desired distance coordinate of the mobilizer.
   @returns a const reference to this mobilizer */
  const CurvilinearMobilizer<T>& SetDistance(systems::Context<T>* context,
                                             const T& distance) const;

  /** Gets the tangential velocity of the mobilizer from the provided context in
   meters per second.
   @param context The context of the model this mobilizer belongs to.
   @returns The velocity coordinate of the mobilizer in the context. */
  const T& get_tangential_velocity(const systems::Context<T>& context) const;

  /** Sets the tangential velocity of the mobilizer from the provided context in
   meters per second.
   @param context The context of the model this mobilizer belongs to.
   @param tangential_velocity The desired velocity coordinate of the mobilizer.
   @returns a const reference to this mobilizer */
  const CurvilinearMobilizer<T>& SetTangentialVelocity(
      systems::Context<T>* context, const T& tangential_velocity) const;

  /** Calculates the distance along the path s(q) [m] associated with a
   generalized coordinate q [m].

   For aperiodic paths, this function is the identity (s(q) = q).

   For periodic paths of length s_f [m], the mobilizer returns to the start of
   the path after each full length of travel, meaning that s must be "wrapped"
   to the path domain [0, s_f):

   <pre>
      s(q) = q mod s_f.
   </pre>

   @param q The generalized coordinate of the mobilizer.
   @returns The position s along the path associated with q. */
  T calc_s(const T& q) const {
    return is_periodic_ ? math::wrap_to(q, T(0.), curvilinear_path_.length())
                        : q;
  }

  /** Computes the across-mobilizer transform X_FM(q) as a function of the
   distance traveled along the mobilizer's path.
   @param q The distance traveled along the mobilizer's path in meters.
   @returns The across-mobilizer transform X_FM(q). */
  math::RigidTransform<T> calc_X_FM(const T* q) const {
    const T s = calc_s(q[0]);
    return curvilinear_path_.CalcPose(s);
  }

  /** Computes the across-mobilizer spatial velocity V_FM(q, v) as a function of
   the distance traveled and tangential velocity along the mobilizer's path.
   @param q The distance traveled along the mobilizer's path in meters.
   @param v The tangential velocity along the mobilizer's path in meters per
   second.
   @returns The across-mobilizer spatial velocity V_FM(q, v). */
  SpatialVelocity<T> calc_V_FM(const T* q, const T* v) const {
    const T s = calc_s(q[0]);
    return curvilinear_path_.CalcSpatialVelocity(s, v[0]);
  }

  /** Computes the across-mobilizer spatial acceleration A_FM(q, v, v̇) as a
   function of the distance traveled, tangential velocity, and tangential
   acceleration along the mobilizer's path.
   @param q The distance traveled along the mobilizer's path in meters.
   @param v The tangential velocity along the mobilizer's path in meters per
   second.
   @param vdot The tangential acceleration along the mobilizer's path in meters
   per second squared.
   @returns The across-mobilizer spatial acceleration A_FM(q, v, v̇). */
  SpatialAcceleration<T> calc_A_FM(const T* q, const T* v,
                                   const T* vdot) const {
    const T s = calc_s(q[0]);
    return curvilinear_path_.CalcSpatialAcceleration(s, v[0], vdot[0]);
  }

  /** Projects the spatial force `F_Mo_F` on `this` mobilizer's outboard
   frame M onto the path tangent. Mathematically,
   this is the dot product of `F_Mo_F` with the spatial derivative of
   the path w.r.t. distance traveled q: F_Mo_F⋅H_FM.

   For this mobilizer, H_FM(q) = d/dv V_FM_F(q, v) is numerically equal to the
   spatial velocity V_FM_F(q, 1) evaluated at the unit velocity v = 1 [m/s]:

   <pre>
      tau = F_BMo_F.dot(V_FM_F(q, 1))
   </pre>

   The result of this method is the scalar equivalent to
   a force applied on the path, pointed along the path's tangential axis and
   measured in Newtons.


   @param q The distance traveled along the mobilizer's path in meters.
   @param F_BMo_F The spatial force applied to the child body at Mo, expressed
   in F.
   @param[out] tau A pointer to store the resulting generalized force in
   Newtons.
  */
  void calc_tau(const T* q, const SpatialForce<T>& F_BMo_F, T* tau) const {
    DRAKE_ASSERT(tau != nullptr);
    const T v(1.);
    // Computes tau = H_FM(q)⋅F_Mo_F, equivalent to V_FM(q, 1)⋅F_Mo_F.
    tau[0] = calc_V_FM(q, &v).dot(F_BMo_F);
  }

  /** Calculates and stores the across-mobilizer transform X_FM given
   the generalized coordinate stored in a given context.
   @param context The context of the model this mobilizer belongs to.
   @returns The across-mobilizer transform X_FM(q). */
  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const final;

  /** Calculates and stores the across-mobilizer spatial velocity V_FM given
   the generalized coordinate stored in a given context and a provided
   tangential velocity.
   @param context The context of the model this mobilizer belongs to.
   @param v The tangential velocity along the mobilizer's path in meters per
   second.
   @returns The across-mobilizer spatial velocity V_FM(q, v).
   @note This method aborts in Debug builds if v.size() is not one. */
  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const final;

  /** Calculates and stores the across-mobilizer spatial acceleration A_FM given
   the generalized coordinate and velocity stored in a given context and a
   provided tangential acceleration.
   @param context The context of the model this mobilizer belongs to.
   @param vdot The tangential acceleration along the mobilizer's path in meters
   per second squared.
   @returns The across-mobilizer spatial acceleration A_FM(q, v, v̇).
   @note This method aborts in Debug builds if vdot.size() is not one. */
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const override;

  /** Projects the spatial force `F_Mo_F` on `this` mobilizer's outboard
   frame M onto the path tangent. Mathematically,
   this is the dot product of `F_Mo_F` with the spatial derivative of
   the path w.r.t. distance traveled q: F_Mo_F⋅H_FM.

   For this mobilizer, H_FM(q) = d/dv V_FM(q, v) is numerically equal to the
   spatial velocity V_FM(q, 1) evaluated at the unit velocity v = 1 [m/s]:

   <pre>
      tau = F_Mo_F.dot(V_FM(q, 1)).
   </pre>

   The result of this method is the scalar equivalent to
   a force applied on the path, pointed along the path's tangential axis and
   measured in Newtons.
   @param context The context of the model this mobilizer belongs to.
   @param F_BMo_F The spatial force applied to the child body at Mo, expressed
   in F.
   @param[out] tau A reference to store the resulting generalized force in
   Newtons.
   @note This method aborts in Debug builds if `tau.size()` is not one.*/
  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_BMo_F,
                           Eigen::Ref<VectorX<T>> tau) const override;

  bool is_velocity_equal_to_qdot() const override { return true; }

  void MapVelocityToQDot(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& v,
                         EigenPtr<VectorX<T>> qdot) const override;

  void MapQDotToVelocity(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         EigenPtr<VectorX<T>> v) const override;

 protected:
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const final;

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

  PiecewiseConstantCurvatureTrajectory<T> curvilinear_path_;
  bool is_periodic_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::CurvilinearMobilizer);
