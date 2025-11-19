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

/* A Prismatic Mobilizer allows two frames to translate relative to one another
along an axis that is constant when measured in either of the frames, while the
frame coordinate axes remain aligned (that is, there is no rotation). The
translation axis must be one of the coordinate axes x, y, or z. To fully specify
this mobilizer we need an inboard ("fixed") frame F, an outboard ("mobilized")
frame M and the coordinate axis along which frame M translates with respect to F
(see PrismaticMobilizerAxial below). The axis vector can be considered axis_F
(expressed in frame F) or axis_M (expressed in frame M) since the components are
identical in either frame.

The restriction to translating along a coordinate axis means that the transform
X_FM has special structure that can be exploited for speed (it is an "axial
translation transform" atX_FM; search for the Doxygen tag "special_xform_def" in
drake/math/rigid_transform.h for a definition). Velocity and acceleration
quantities are simplified also. In robotics, the prismatic joint is common
so it is important that we handle its implementation efficiently.

The single generalized coordinate q introduced by this mobilizer corresponds to
the translation distance in meters of frame M with respect to frame F along the
translation axis. When q = 0, frames F and M are coincident. The translation is
defined to be positive in the direction of the translation axis.

Notice that the components of the rotation axis as expressed in either frame F
or M are constant. That is, axis_F and axis_M remain identical and unchanged
w.r.t. both frames by this mobilizer's motion.

H_FM_F₆ₓ₁=[0₃ axis_F]ᵀ     Hdot_FM_F₆ₓ₁ = 0₆
H_FM_M₆ₓ₁=[0₃ axis_M]ᵀ     Hdot_FM_M₆ₓ₁ = 0₆
   where axis_M == axis_F

@tparam_default_scalar */

// PrismaticMobilizer base class.
// This is an abstract base class to provide high-level access to the common
// features of the axial prismatic mobilizer concrete classes (defined below),
// intended for use by the Joint and PrismaticJoint APIs. Internal algorithms,
// however, should be templatized on the specific prismatic mobilizer instance
// to permit inline access to performance-critical functions.
template <typename T>
class PrismaticMobilizer : public MobilizerImpl<T, 1, 1> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrismaticMobilizer);

  using MobilizerBase = MobilizerImpl<T, 1, 1>;
  using MobilizerBase::kNq, MobilizerBase::kNv, MobilizerBase::kNx;
  template <typename U>
  using QVector = typename MobilizerBase::template QVector<U>;
  template <typename U>
  using VVector = typename MobilizerBase::template VVector<U>;
  template <typename U>
  using HMatrix = typename MobilizerBase::template HMatrix<U>;

  ~PrismaticMobilizer() override;

  // Overloads to define the suffix names for the position and velocity
  // elements.
  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  bool can_rotate() const final { return false; }
  bool can_translate() const final { return true; }

  // @retval axis The translation axis as a unit vector expressed identically
  // in both the F and M frames. This will be one of the coordinate axes,
  // which will have been constructed to be aligned with the user's specified
  // translation axis for the PrismaticJoint this is implementing.
  const Vector3<double>& translation_axis() const { return axis_; }

  // Gets the translational distance for `this` mobilizer from `context`. See
  // class documentation for sign convention details.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @returns The translation coordinate of `this` mobilizer in the `context`.
  const T& get_translation(const systems::Context<T>& context) const;

  // Sets `context` so that the generalized coordinate corresponding to the
  // translation for `this` mobilizer equals `translation`.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @param[in] translation The desired translation in meters.
  // @returns a constant reference to `this` mobilizer.
  const PrismaticMobilizer<T>& SetTranslation(systems::Context<T>* context,
                                              const T& translation) const;

  // Gets the rate of change, in meters per second, of `this` mobilizer's
  // translation (see get_translation()) from `context`. See class
  // documentation for the translation sign convention.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @returns The rate of change of `this` mobilizer's translation in the
  // `context`.
  const T& get_translation_rate(const systems::Context<T>& context) const;

  // Sets the rate of change, in meters per second, of `this` mobilizer's
  // translation to `translation_dot`. The new rate of change `translation_dot`
  // gets stored in `context`.
  // See class documentation for the translation sign convention.
  // @param[in] context The context of the MultibodyTree this mobilizer
  //                    belongs to.
  // @param[in] translation_dot The desired rate of change of `this`
  // mobilizer's translation in meters per second.
  // @returns a constant reference to `this` mobilizer.
  const PrismaticMobilizer<T>& SetTranslationRate(
      systems::Context<T>* context, const T& translation_dot) const;

  bool is_velocity_equal_to_qdot() const final { return true; }

 protected:
  // Constructor for a PrismaticMobilizer between the inboard frame F and the
  // outboard frame M, granting a single translational degree of freedom along
  // a coordinate axis (axis=0, 1, or 2) common to both frames.
  PrismaticMobilizer(const SpanningForest::Mobod& mobod,
                     const Frame<T>& inboard_frame_F,
                     const Frame<T>& outboard_frame_M, int axis);

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

  // Maps vdot to qddot, which for this mobilizer is q̈ = v̇.
  void DoMapAccelerationToQDDot(const systems::Context<T>& context,
                                const Eigen::Ref<const VectorX<T>>& vdot,
                                EigenPtr<VectorX<T>> qddot) const final;

  // Maps qddot to vdot, which for this mobilizer is v̇ = q̈.
  void DoMapQDDotToAcceleration(const systems::Context<T>& context,
                                const Eigen::Ref<const VectorX<T>>& qddot,
                                EigenPtr<VectorX<T>> vdot) const final;

 private:
  // Joint axis expressed identically in frames F and M. This must be one of
  // the coordinate axes 100, 010, or 001.
  Vector3<double> axis_;
};

// Prismatic mobilizer with translation axis aligned with one of the F and M
// frame axes.
template <typename T, int axis>
  requires(0 <= axis && axis <= 2)
class PrismaticMobilizerAxial final : public PrismaticMobilizer<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrismaticMobilizerAxial);

  PrismaticMobilizerAxial(const SpanningForest::Mobod& mobod,
                          const Frame<T>& inboard_frame_F,
                          const Frame<T>& outboard_frame_M)
      : PrismaticMobilizer<T>(mobod, inboard_frame_F, outboard_frame_M, axis) {}

  ~PrismaticMobilizerAxial() final;

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
  // By definition `A_FM = d_F(V_FM)/dt`. The acceleration `A_FM` will be a
  // function of the translation distance q, its rate of change v for the
  // current state in `context` and of the input generalized acceleration
  // `v̇ = dv/dt`, the rate of change of v.
  // See class documentation for the translation sign convention.
  // This method aborts in Debug builds if `vdot.size()` is not one.
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const final;

  // Projects the spatial force `F_Mo_F` on `this` mobilizer's outboard
  // frame M onto its translation axis (see translation_axis().)
  // Mathematically: <pre>
  //    tau = F_Mo_F.translational().dot(axis)
  // </pre>
  // Therefore, the result of this method is the scalar value of the linear
  // force along the axis of `this` mobilizer.
  // This method aborts in Debug builds if `tau.size()` is not one.
  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_Mo_F,
                           Eigen::Ref<VectorX<T>> tau) const final;

  // Computes the across-mobilizer transform `X_FM(q)` between the inboard
  // frame F and the outboard frame M as a function of the translation distance
  // along this mobilizer's axis (see translation_axis().)
  math::RigidTransform<T> calc_X_FM(const T* q) const {
    DRAKE_ASSERT(q != nullptr);
    Eigen::Vector3<T> p_FM;
    p_FM[kX] = q[0];
    p_FM[kY] = 0;
    p_FM[kZ] = 0;
    return math::RigidTransform<T>(p_FM);
  }

  // We only need to write q into the appropriate slot of X_FM to update. */
  void update_X_FM(const T* q, math::RigidTransform<T>* X_FM) const {
    DRAKE_ASSERT(q != nullptr && X_FM != nullptr);
    math::RigidTransform<T>::template UpdateAxialTranslation<axis>(*q, X_FM);
  }

  // Returns X_AM = X_AF * atX_FM, exploiting known structure of atX_FM.
  math::RigidTransform<T> post_multiply_by_X_FM(
      const math::RigidTransform<T>& X_AF,
      const math::RigidTransform<T>& atX_FM) const {
    math::RigidTransform<T> X_AM;
    X_AF.template PostMultiplyByAxialTranslation<axis>(atX_FM, &X_AM);
    return X_AM;
  }

  // Returns X_FB = atX_FM * X_MB, exploiting known structure of atX_FM.
  math::RigidTransform<T> pre_multiply_by_X_FM(
      const math::RigidTransform<T>& atX_FM,
      const math::RigidTransform<T>& X_MB) const {
    math::RigidTransform<T> X_FB;
    X_MB.template PreMultiplyByAxialTranslation<axis>(atX_FM, &X_FB);
    return X_FB;
  }

  // Since R_FM is identity, applying it to a vector does nothing.
  Vector3<T> apply_R_FM(const math::RotationMatrix<T>&,
                        const Vector3<T>& v_M) const {
    return v_M;
  }

  // Computes the across-mobilizer velocity `V_FM(q, v)` of the outboard frame M
  // measured and expressed in frame F as a function of the input translational
  // velocity v along this mobilizer's axis (see translation_axis()).
  SpatialVelocity<T> calc_V_FM(const T*, const T* v) const {
    DRAKE_ASSERT(v != nullptr);
    Eigen::Vector3<T> v_FM;
    v_FM[kX] = v[0];
    v_FM[kY] = 0;
    v_FM[kZ] = 0;
    return SpatialVelocity<T>(Vector3<T>::Zero(), v_FM);
  }

  SpatialAcceleration<T> calc_A_FM(const T*, const T*, const T* vdot) const {
    DRAKE_ASSERT(vdot != nullptr);
    Eigen::Vector3<T> a_FM;
    a_FM[kX] = vdot[0];
    a_FM[kY] = 0;
    a_FM[kZ] = 0;
    return SpatialAcceleration<T>(Vector3<T>::Zero(), a_FM);
  }

  // Returns tau = H_FM_Fᵀ⋅F, where H_FM_Fᵀ = [0₃ᵀ axis_Fᵀ].
  void calc_tau(const T*, const SpatialForce<T>& F_BMo_F, T* tau) const {
    DRAKE_ASSERT(tau != nullptr);
    const Vector3<T>& f_BMo_F = F_BMo_F.translational();
    tau[0] = f_BMo_F[axis];
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

  // Write algorithms as though the axes were x, y, and z; modular arithmetic
  // here ensures they will work correctly for axis=0, 1, or 2.
  static constexpr int kX = axis, kY = (axis + 1) % 3, kZ = (axis + 2) % 3;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::PrismaticMobilizer);

#define DRAKE_DECLARE_PRISMATIC_MOBILIZER(axis)                                \
  extern template class ::drake::multibody::internal::PrismaticMobilizerAxial< \
      double, axis>;                                                           \
  extern template class ::drake::multibody::internal::PrismaticMobilizerAxial< \
      ::drake::AutoDiffXd, axis>;                                              \
  extern template class ::drake::multibody::internal::PrismaticMobilizerAxial< \
      ::drake::symbolic::Expression, axis>

DRAKE_DECLARE_PRISMATIC_MOBILIZER(0);
DRAKE_DECLARE_PRISMATIC_MOBILIZER(1);
DRAKE_DECLARE_PRISMATIC_MOBILIZER(2);

#undef DRAKE_DECLARE_PRISMATIC_MOBILIZER
