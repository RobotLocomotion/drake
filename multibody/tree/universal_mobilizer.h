#pragma once

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

// This mobilizer models a universal joint between an inboard frame F and an
// outboard frame M that enables rotation about F's x-axis followed by rotation
// about M's y-axis. No translational motion of M in F is allowed and the
// inboard frame origin `Fo` and the outboard frame origin `Mo` are coincident
// at all times.
//
// The generalized coordinates for this mobilizer correspond to angles (θ₁, θ₂)
// for a sequence of body-fixed rotations about the x-axis of frame F, and the
// y-axis of frame M  respectively. Defining an intermediate frame I, the first
// rotation defines I with respect to F.  The x-axis of F and I are aligned and
// the axes are offset by the rotation, θ₁, about their shared x-axis.  Frame M
// is then defined to share the same y-axis as I and is offset by the rotation,
// θ₂, about their shared y-axis.
// Mathematically, rotation `R_FM` is given in terms of angles (θ₁, θ₂) by:
// <pre>
//   R_FM(q) = R_FI(θ₁) * R_IM(θ₂)
// </pre>
// where `R_FI(θ₁)` defines the orientation of I in F as an elemental rotation
// of amount θ₁ about the x-axis of frame F and `R_IM(θ₂)` defines the
// orientation of M in I as an elemental rotation of amount θ₂ about the y-axis
// of frame I (also the y-axis of frame M).
// Zero θ₁, θ₂ angles define the "zero configuration" which corresponds to
// frames F, I, and M being coincident, see set_zero_state(). Angles (θ₁, θ₂)
// are defined to be positive according to the right-hand-rule with the thumb
// aligned in the direction of their respective axes. The generalized
// velocities for this mobilizer are the rate of change of the angles, v = q̇.
//
// @tparam_default_scalar
template <typename T>
class UniversalMobilizer final : public MobilizerImpl<T, 2, 2> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UniversalMobilizer)

  // Constructor for a %UniversalMobilizer between an inboard frame F
  // `inboard_frame_F` and an outboard frame M `outboard_frame_M` granting
  // two rotational degrees of freedom corresponding to angles θ₁, θ₂ as
  // described in this class's documentation.
  UniversalMobilizer(const Frame<T>& inboard_frame_F,
                     const Frame<T>& outboard_frame_M)
      : MobilizerBase(inboard_frame_F, outboard_frame_M) {}

  // Overloads to define the suffix names for the position and velocity
  // elements.
  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  // Retrieves from `context` the two angles, (θ₁, θ₂) which describe the state
  // for `this` mobilizer as documented in this class's documentation.
  //
  // @param[in] context The context of the model this mobilizer belongs to.
  // @returns angles The two angles (θ₁, θ₂) packed and returned as a vector
  //                 with entries `angles(0) = θ₁`, `angles(1) = θ₂`.
  Vector2<T> get_angles(const systems::Context<T>& context) const;

  // Sets in `context` the state for `this` mobilizer to the angles (θ₁, θ₂)
  // provided in the input argument `angles`, which stores them with the format
  // `angles = [θ₁, θ₂]`.
  //
  // @param[in] context The context of the model this mobilizer belongs to.
  // @param[in] angles A vector which must pack values for the angles (θ₁, θ₂)
  //                   described in this class's documentation, at entries
  //                   `angles(0)` and `angles(1)`, respectively.
  // @returns a constant reference to `this` mobilizer.
  const UniversalMobilizer<T>& set_angles(systems::Context<T>* context,
                                          const Vector2<T>& angles) const;

  // Retrieves from `context` the rate of change, in radians per second, of
  // `this` mobilizer's angles (see get_angles()).
  // @param[in] context The context of the model this mobilizer belongs to.
  // @returns angles_dot The rate of change of the two angles (ω₁, ω₂) returned
  //                     as the vector [ω₁, ω₂].
  Vector2<T> get_angular_rates(const systems::Context<T>& context) const;

  // Sets in `context` the rate of change, in radians per second, of this
  // `this` mobilizer's angles (see get_angles()) to `angles_dot`.
  // @param[in] context The context of the model this mobilizer belongs to.
  // @param[in] angles_dot The desired rate of change, ω₁, ω₂, packed as the
  //                       vector [ω₁, ω₂].
  // @returns a constant reference to `this` mobilizer.
  const UniversalMobilizer<T>& set_angular_rates(
      systems::Context<T>* context, const Vector2<T>& angles_dot) const;

  // Computes the across-mobilizer transform `X_FM(q)` between the inboard
  // frame F and the outboard frame M as a function of the angles (θ₁, θ₂)
  // stored in `context`.
  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const override;

  // Computes the across-mobilizer velocity `V_FM(q, v)` of the outboard frame
  // M measured and expressed in frame F as a function of the angles (θ₁, θ₂)
  // stored in `context` and of the input angular rates v, formatted as
  // in get_angular_rates().
  // This method aborts in Debug builds if `v.size()` is not two.
  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const override;

  // Computes the across-mobilizer acceleration `A_FM(q, v, v̇)` of the
  // outboard frame M in the inboard frame F.
  // By definition `A_FM = d_F(V_FM)/dt = H_FM(q) * v̇ + Ḣ_FM * v`.
  // The acceleration `A_FM` will be a function of the rotation angles q (θ₁,
  // θ₂) and their rates of change v (ω₁, ω₂) from the `context` as well as the
  // generalized accelerations `v̇ = dv/dt`, the rates of change of v.
  // This method aborts in Debug builds if `vdot.size()` is not two.
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const override;

  // Projects the spatial force `F_Mo = [τ_Mo, f_Mo]` on `this` mobilizer's
  // outboard frame M onto the axes of rotation, x and y. Mathematically:
  // <pre>
  //    tau = [τ_Mo⋅Fx]
  //          [τ_Mo⋅My]
  // </pre>
  // Therefore, the result of this method is the vector of torques about each
  // rotation axis of `this` mobilizer.
  // This method aborts in Debug builds if `tau.size()` is not two.
  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_Mo_F,
                           Eigen::Ref<VectorX<T>> tau) const override;

  // Performs the identity mapping from v to qdot since, for this mobilizer,
  // v = q̇.
  void MapVelocityToQDot(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& v,
                         EigenPtr<VectorX<T>> qdot) const override;

  // Performs the identity mapping from qdot to v since, for this mobilizer,
  // v = q̇.
  void MapQDotToVelocity(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         EigenPtr<VectorX<T>> v) const override;

 protected:
  // Calculates the rotational part of matrix H and optionally its derivative.
  // See Mobilizer documentation for notation.
  Eigen::Matrix<T, 3, 2> CalcHwMatrix(const systems::Context<T>& context,
                                      Vector3<T>* Hw_dot = nullptr) const;

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
  typedef MobilizerImpl<T, 2, 2> MobilizerBase;
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
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::UniversalMobilizer)
