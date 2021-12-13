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
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

// This Mobilizer allows two frames to rotate relatively to one another around
// an axis that is constant when measured in either this mobilizer's inboard or
// outboard frames, while the distance between the two frames does not vary.
// To fully specify this mobilizer a user must provide the inboard frame F,
// the outboard (or "mobilized") frame M and the axis `axis_F` (expressed in
// frame F) about which frame M rotates with respect to F.
// The single generalized coordinate q introduced by this mobilizer
// corresponds to the rotation angle in radians of frame M with respect to
// frame F about the rotation axis `axis_F`. When `q = 0`, frames F and M are
// coincident. The rotation angle is defined to be positive according to the
// right-hand-rule with the thumb aligned in the direction of the `axis_F`.
// Notice that the components of the rotation axis as expressed in
// either frame F or M are constant. That is, `axis_F` and `axis_M` remain
// unchanged w.r.t. both frames by this mobilizer's motion.
//
// @tparam_default_scalar
template <typename T>
class RevoluteMobilizer final : public MobilizerImpl<T, 1, 1> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RevoluteMobilizer)

  // Constructor for a %RevoluteMobilizer between the inboard frame F
  // `inboard_frame_F` and the outboard frame M `outboard_frame_F` granting a
  // single rotational degree of freedom about axis `axis_F` expressed in the
  // inboard frame F.
  // @pre `axis_F` must be a non-zero vector with norm at least root square of
  // machine epsilon. This vector can have any length (subject to the norm
  // restriction above), only the direction is used.
  // @throws std::exception if the L2 norm of `axis_F` is less than the square
  // root of machine epsilon.
  RevoluteMobilizer(const Frame<T>& inboard_frame_F,
                    const Frame<T>& outboard_frame_M,
                    const Vector3<double>& axis_F) :
      MobilizerBase(inboard_frame_F, outboard_frame_M), axis_F_(axis_F) {
    double kEpsilon = std::sqrt(std::numeric_limits<double>::epsilon());
    DRAKE_THROW_UNLESS(!axis_F.isZero(kEpsilon));
    axis_F_.normalize();
  }

  // Overloads to define the suffix names for the position and velocity
  // elements.
  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  // @retval axis_F The rotation axis as a unit vector expressed in the inboard
  //                frame F.
  const Vector3<double>& revolute_axis() const { return axis_F_; }

  // Gets the rotation angle of `this` mobilizer from `context`. See class
  // documentation for sign convention.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @returns The angle coordinate of `this` mobilizer in the `context`.
  const T& get_angle(const systems::Context<T>& context) const;

  // Sets the `context` so that the generalized coordinate corresponding to the
  // rotation angle of `this` mobilizer equals `angle`.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @param[in] angle The desired angle in radians.
  // @returns a constant reference to `this` mobilizer.
  const RevoluteMobilizer<T>& set_angle(
      systems::Context<T>* context, const T& angle) const;

  // Gets the rate of change, in radians per second, of `this` mobilizer's
  // angle (see get_angle()) from `context`. See class documentation for the
  // angle sign convention.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @returns The rate of change of `this` mobilizer's angle in the `context`.
  const T& get_angular_rate(const systems::Context<T> &context) const;

  // Sets the rate of change, in radians per second, of this `this` mobilizer's
  // angle to `theta_dot`. The new rate of change `theta_dot` gets stored in
  // `context`.
  // See class documentation for the angle sign convention.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @param[in] theta_dot The desired rate of change of `this` mobilizer's
  // angle in radians per second.
  // @returns a constant reference to `this` mobilizer.
  const RevoluteMobilizer<T>& set_angular_rate(
      systems::Context<T> *context, const T& theta_dot) const;

  // Computes the across-mobilizer transform `X_FM(q)` between the inboard
  // frame F and the outboard frame M as a function of the rotation angle
  // about this mobilizer's axis (@see revolute_axis().)
  // The generalized coordinate q for `this` mobilizer (the rotation angle) is
  // stored in `context`.
  // This method aborts in Debug builds if `v.size()` is not one.
  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const override;

  // Computes the across-mobilizer velocity `V_FM(q, v)` of the outboard frame
  // M measured and expressed in frame F as a function of the rotation angle
  // and input angular velocity `v` about this mobilizer's axis
  // (@see revolute_axis()).
  // The generalized coordinate q for `this` mobilizer (the rotation angle) is
  // stored in `context`.
  // This method aborts in Debug builds if `v.size()` is not one.
  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const override;

  // Computes the across-mobilizer acceleration `A_FM(q, v, v̇)` of the
  // outboard frame M in the inboard frame F.
  // By definition `A_FM = d_F(V_FM)/dt = H_FM(q) * v̇ + Ḣ_FM * v`.
  // The acceleration `A_FM` will be a function of the rotation angle q, its
  // rate of change v for the current state in `context` and of the input
  // generalized acceleration `v̇ = dv/dt`, the rate of change of v.
  // See class documentation for the angle sign convention.
  // This method aborts in Debug builds if `vdot.size()` is not one.
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const override;

  // Projects the spatial force `F_Mo_F` on `this` mobilizer's outboard
  // frame M onto its rotation axis (@see revolute_axis().) Mathematically:
  // <pre>
  //    tau = F_Mo_F.rotational().dot(axis_F)
  // </pre>
  // Therefore, the result of this method is the scalar value of the torque at
  // the axis of `this` mobilizer.
  // This method aborts in Debug builds if `tau.size()` is not one.
  void ProjectSpatialForce(
      const systems::Context<T>& context,
      const SpatialForce<T>& F_Mo_F,
      Eigen::Ref<VectorX<T>> tau) const override;

  void MapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const override;

  void MapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const override;

 protected:
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  void DoCalcNplusMatrix(
      const systems::Context<T>& context,
      EigenPtr<MatrixX<T>> Nplus) const final;

  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const override;

 private:
  typedef MobilizerImpl<T, 1, 1> MobilizerBase;
  // Bring the handy number of position and velocities MobilizerImpl enums into
  // this class' scope. This is useful when writing mathematical expressions
  // with fixed-sized vectors since we can do things like Vector<T, nq>.
  // Operations with fixed-sized quantities can be optimized at compile time
  // and therefore they are highly preferred compared to the very slow dynamic
  // sized quantities.
  using MobilizerBase::kNq;
  using MobilizerBase::kNv;

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // Default joint axis expressed in the inboard frame F.
  Vector3<double> axis_F_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::RevoluteMobilizer)
