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

/* A Revolute Mobilizer allows two frames to rotate relative to one another
around an axis that is constant when measured in either of the frames, while the
frame origins remain coincident. To fully specify this mobilizer a user must
provide the inboard ("fixed") frame F, the outboard ("mobilized") frame M and
the axis about which frame M rotates with respect to F. The axis vector can be
considered axis_F (expressed in frame F) or axis_M (expressed in frame M) since
the measure numbers are identical in either frame. We provide mobilizer
instances for only cases where the axis is one of the coordinate axis 100, 010,
or 001. It is up to the Revolute Joint to choose F and M frames properly to
implement the user's specification of parent frame Jp, child frame Jc, and a
general rotation axis.

The single generalized coordinate q introduced by this mobilizer corresponds to
the rotation angle in radians of frame M with respect to frame F about the
rotation axis. When q = 0, frames F and M are coincident. The rotation angle is
defined to be positive according to the right-hand-rule with the thumb aligned
in the direction of the axis. When the mobilizer is reversed from the joint, the
rotation axis must be chosen opposite the user's specified joint axis so that q
and v have the expected meanings. ("Reversed" means that the mobilizer's inboard
body is the joint's child body and the mobilizer's outboard body is the joint's
parent body.)

Notice that the components of the rotation axis as expressed in either frame F
or M are constant. That is, axis_F and axis_M remain identical and unchanged
w.r.t. both frames by this mobilizer's motion.

H_FM_F₆ₓ₁=[axis_F 0₃]ᵀ     Hdot_FM_F₆ₓ₁ = 0₆
H_FM_M₆ₓ₁=[axis_M 0₃]ᵀ     Hdot_FM_M₆ₓ₁ = 0₆
   where axis_M == axis_F

@tparam_default_scalar */
template <typename T>
class RevoluteMobilizer : public MobilizerImpl<T, 1, 1> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RevoluteMobilizer);

  using MobilizerBase = MobilizerImpl<T, 1, 1>;
  using MobilizerBase::kNq, MobilizerBase::kNv, MobilizerBase::kNx;
  template <typename U>
  using QVector = typename MobilizerBase::template QVector<U>;
  template <typename U>
  using VVector = typename MobilizerBase::template VVector<U>;
  template <typename U>
  using HMatrix = typename MobilizerBase::template HMatrix<U>;

  // Overloads to define the suffix names for the position and velocity
  // elements.
  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  bool can_rotate() const final { return true; }
  bool can_translate() const final { return false; }

  // @retval axis_F The rotation axis as a unit vector expressed in the inboard
  //                frame F.
  const Vector3<double>& revolute_axis() const { return axis_FM(); }

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
  const RevoluteMobilizer<T>& SetAngle(systems::Context<T>* context,
                                       const T& angle) const;

  // Gets the rate of change, in radians per second, of `this` mobilizer's
  // angle (see get_angle()) from `context`. See class documentation for the
  // angle sign convention.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @returns The rate of change of `this` mobilizer's angle in the `context`.
  const T& get_angular_rate(const systems::Context<T>& context) const;

  // Sets the rate of change, in radians per second, of this `this` mobilizer's
  // angle to `theta_dot`. The new rate of change `theta_dot` gets stored in
  // `context`.
  // See class documentation for the angle sign convention.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @param[in] theta_dot The desired rate of change of `this` mobilizer's
  // angle in radians per second.
  // @returns a constant reference to `this` mobilizer.
  const RevoluteMobilizer<T>& SetAngularRate(systems::Context<T>* context,
                                             const T& theta_dot) const;
  bool is_velocity_equal_to_qdot() const override { return true; }

  // Maps v to qdot, which for this mobilizer is q̇ = v.
  void MapVelocityToQDot(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& v,
                         EigenPtr<VectorX<T>> qdot) const final;

  // Maps qdot to v, which for this mobilizer is v = q̇.
  void MapQDotToVelocity(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         EigenPtr<VectorX<T>> v) const final;

  // Maps vdot to qddot, which for this mobilizer is q̈ = v̇.
  void MapAccelerationToQDDot(const systems::Context<T>& context,
                              const Eigen::Ref<const VectorX<T>>& vdot,
                              EigenPtr<VectorX<T>> qddot) const final;

  // Maps qddot to vdot, which for this mobilizer is v̇ = q̈.
  void MapQDDotToAcceleration(const systems::Context<T>& context,
                              const Eigen::Ref<const VectorX<T>>& qddot,
                              EigenPtr<VectorX<T>> vdot) const final;

 protected:
  // Constructor for a %RevoluteMobilizer between the inboard frame F
  // `inboard_frame_F` and the outboard frame M `outboard_frame_F` granting a
  // single rotational degree of freedom about some axis common to both frames.
  RevoluteMobilizer(const SpanningForest::Mobod& mobod,
                    const Frame<T>& inboard_frame_F,
                    const Frame<T>& outboard_frame_M)
      : MobilizerBase(mobod, inboard_frame_F, outboard_frame_M) {}

  // @pre axis_FM is a unit vector.
  void set_axis_FM(const Vector3<double>& axis_FM) { axis_FM_ = axis_FM; }

  const Vector3<double>& axis_FM() const { return axis_FM_; }

  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const final;

 private:
  // Default joint axis expressed identically in frames F and M.
  Vector3<double> axis_FM_;
};

/* Revolute mobilizer with rotation axis aligned with one of the F and M frame
axes. */
template <typename T, int axis>
class RevoluteMobilizerAxial final : public RevoluteMobilizer<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RevoluteMobilizerAxial);

  using RevoluteMobilizer<T>::axis_FM;

  RevoluteMobilizerAxial(const SpanningForest::Mobod& mobod,
                         const Frame<T>& inboard_frame_F,
                         const Frame<T>& outboard_frame_M)
      : RevoluteMobilizer<T>(mobod, inboard_frame_F, outboard_frame_M) {
    Vector3<double> rotation_axis = Vector3<double>::Zero();
    rotation_axis[axis] = 1;
    this->set_axis_FM(rotation_axis);
  }

  std::unique_ptr<BodyNode<T>> CreateBodyNode(
      const BodyNode<T>* parent_node, const RigidBody<T>* body,
      const Mobilizer<T>* mobilizer) const final;
  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const final;

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const final;

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
      const Eigen::Ref<const VectorX<T>>& vdot) const final;

  // Projects the spatial force `F_Mo_F` on `this` mobilizer's outboard
  // frame M onto its rotation axis (@see revolute_axis().) Mathematically:
  // <pre>
  //    tau = F_Mo_F.rotational().dot(axis_F)
  // </pre>
  // Therefore, the result of this method is the scalar value of the torque at
  // the axis of `this` mobilizer.
  // This method aborts in Debug builds if `tau.size()` is not one.
  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_Mo_F,
                           Eigen::Ref<VectorX<T>> tau) const final;

  // Computes the across-mobilizer transform `X_FM(q)` between the inboard
  // frame F and the outboard frame M as a function of the rotation angle
  // about this mobilizer's axis (@see revolute_axis().)
  // The generalized coordinate q for `this` mobilizer (the rotation angle) is
  // stored in `context`.
  // This method aborts in Debug builds if `v.size()` is not one.
  math::RigidTransform<T> calc_X_FM(const T* q) const {
    // TODO(sherm1) Make a generic RotationMatrix function for this.
    auto make_R_FM = [](const T& angle) {
      if constexpr (axis == 0)
        return math::RotationMatrix<T>::MakeXRotation(angle);
      if constexpr (axis == 1)
        return math::RotationMatrix<T>::MakeYRotation(angle);
      if constexpr (axis == 2)
        return math::RotationMatrix<T>::MakeZRotation(angle);
    };
    return math::RigidTransform<T>(make_R_FM(q[0]), Vector3<T>::Zero());
  }

  void update_X_FM(const T* q, math::RigidTransform<T>* X_FM) const {
    DRAKE_ASSERT(q != nullptr && X_FM != nullptr);
    math::RigidTransform<T>::template UpdateAxialRotation<axis>(q[0], &*X_FM);
  }

  Vector3<T> apply_X_FM(const math::RigidTransform<T>& X_FM,
                        const Vector3<T>& v_M) const {
    return math::RigidTransform<T>::template ApplyAxialRotation<axis>(X_FM,
                                                                      v_M);
  }

  Vector3<T> apply_R_FM(const math::RotationMatrix<T>& R_FM,
                        const Vector3<T>& v_M) const {
    return math::RotationMatrix<T>::template ApplyAxialRotation<axis>(R_FM,
                                                                      v_M);
  }

  math::RigidTransform<T> compose_with_X_FM(
      const math::RigidTransform<T>& X_AF,
      const math::RigidTransform<T>& X_FM) const {
    math::RigidTransform<T> X_AM;
    X_AF.template ComposeWithAxialRotation<axis>(X_FM, &X_AM);
    return X_AM;
  }

  // TODO(sherm1) Velocity & acceleration APIs should be reworked to perform
  //  only simplified updates.

  // Computes the across-mobilizer spatial velocity V_FM(q, v) of the outboard
  // frame M measured and expressed in frame F as a function of the input
  // angular velocity `v` about this mobilizer's axis (@see revolute_axis()).
  SpatialVelocity<T> calc_V_FM(const T*, const T* v) const {
    return SpatialVelocity<T>(v[0] * axis_FM(),  // axis_F, 3 flops
                              Vector3<T>::Zero());
  }

  SpatialVelocity<T> calc_V_FM_M(const math::RigidTransform<T>&, const T*,
                                 const T* v) const {
    return SpatialVelocity<T>(v[0] * axis_FM(),  // axis_M, 3 flops
                              Vector3<T>::Zero());
  }

  // Here H_F₆ₓ₁=[axis_F, 0₃]ᵀ so Hdot_F = 0 and
  // A_FM_F = H_F⋅vdot + Hdot_F⋅v = [axis_F⋅vdot, 0₃]ᵀ
  SpatialAcceleration<T> calc_A_FM(const T*, const T*, const T* vdot) const {
    return SpatialAcceleration<T>(vdot[0] * axis_FM(),  // axis_F, 3 flops
                                  Vector3<T>::Zero());
  }

  // Here H_M₆ₓ₁=[axis_M, 0₃]ᵀ so Hdot_M = 0 and
  // A_FM_M = H_M⋅vdot + Hdot_M⋅v = [axis_M⋅vdot, 0₃]ᵀ
  // But axis_M == axis_F.
  SpatialAcceleration<T> calc_A_FM_M(const math::RigidTransform<T>&, const T*,
                                     const T*, const T* vdot) const {
    return SpatialAcceleration<T>(vdot[0] * axis_FM(),  // axis_M, 3 flops
                                  Vector3<T>::Zero());
  }

  // Returns tau = H_FM_Fᵀ⋅F_F, where H_FM_Fᵀ = [axis_Fᵀ 0₃ᵀ].
  void calc_tau(const T*, const SpatialForce<T>& F_BMo_F, T* tau) const {
    DRAKE_ASSERT(tau != nullptr);
    const Vector3<T>& t_BMo_F = F_BMo_F.rotational();
    tau[0] = t_BMo_F[axis];
  }

  // Returns tau = H_FM_Mᵀ⋅F_M, where H_FM_Mᵀ = [axis_Mᵀ 0₃ᵀ] and
  // axis_M == 100, 010, or 001.
  void calc_tau_from_M(const math::RigidTransform<T>&, const T*,
                       const SpatialForce<T>& F_BMo_M, T* tau) const {
    DRAKE_ASSERT(tau != nullptr);
    const Vector3<T>& t_BMo_M = F_BMo_M.rotational();
    tau[0] = t_BMo_M[axis];
  }

 private:
  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const final;

  std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const final;

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::RevoluteMobilizer);

#define DRAKE_DECLARE_REVOLUTE_MOBILIZER(axis)                                \
  extern template class ::drake::multibody::internal::RevoluteMobilizerAxial< \
      double, axis>;                                                          \
  extern template class ::drake::multibody::internal::RevoluteMobilizerAxial< \
      ::drake::AutoDiffXd, axis>;                                             \
  extern template class ::drake::multibody::internal::RevoluteMobilizerAxial< \
      ::drake::symbolic::Expression, axis>

DRAKE_DECLARE_REVOLUTE_MOBILIZER(0);
DRAKE_DECLARE_REVOLUTE_MOBILIZER(1);
DRAKE_DECLARE_REVOLUTE_MOBILIZER(2);

#undef DRAKE_DECLARE_REVOLUTE_MOBILIZER
