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

// This mobilizer models a universal joint between an inboard frame F and an
// outboard frame M that enables rotation about F's x-axis followed by rotation
// about M's y-axis. No translational motion of M in F is allowed and the
// inboard frame origin `Fo` and the outboard frame origin `Mo` are coincident
// at all times.
//
// The generalized coordinates for this mobilizer correspond to angles (θ₀, θ₁)
// for a sequence of body-fixed rotations about the x-axis of frame F, and the
// y-axis of frame M  respectively. Defining an intermediate frame I, the first
// rotation defines I with respect to F.  The x-axis of F and I are aligned and
// the axes are offset by the rotation, θ₀, about their shared x-axis.  Frame M
// is then defined to share the same y-axis as I and is offset by the rotation,
// θ₁, about their shared y-axis.
// Mathematically, rotation `R_FM` is given in terms of angles (θ₀, θ₁) by:
// <pre>
//   R_FM(q) = R_FI(θ₀) * R_IM(θ₁)
// </pre>
// where `R_FI(θ₀)` defines the orientation of I in F as an elemental rotation
// of amount θ₀ about the x-axis of frame F and `R_IM(θ₁)` defines the
// orientation of M in I as an elemental rotation of amount θ₁ about the y-axis
// of frame I (also the y-axis of frame M).
// Zero θ₀, θ₁ angles define the "zero configuration" which corresponds to
// frames F, I, and M being coincident, see SetZeroState(). Angles (θ₀, θ₁)
// are defined to be positive according to the right-hand-rule with the thumb
// aligned in the direction of their respective axes. The generalized
// velocities for this mobilizer are the rate of change of the angles, v = q̇.
//
//    H_FM_F₆ₓ₂ = [Hw_FM₃ₓ₂]        Hw_FM_F = [ 1   0   ]       Hv_FM_F = 0₃ₓ₂
//                [Hv_FM₃ₓ₂]                  [ 0 c(q₀) ]
//                                            [ 0 s(q₀) ]
//
// Hdot_FM_F₆ₓ₂ = [Hwdot_FM₃ₓ₂]  Hwdot_FM_F = [ 0    0     ]  Hvdot_FM_F = 0₃ₓ₂
//                [Hvdot_FM₃ₓ₂]               [ 0 -v₀s(q₀) ]
//                                            [ 0  v₀c(q₀) ]
//
// @tparam_default_scalar
template <typename T>
class UniversalMobilizer final : public MobilizerImpl<T, 2, 2> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UniversalMobilizer);
  using MobilizerBase = MobilizerImpl<T, 2, 2>;
  using MobilizerBase::kNq, MobilizerBase::kNv, MobilizerBase::kNx;
  template <typename U>
  using QVector = typename MobilizerBase::template QVector<U>;
  template <typename U>
  using VVector = typename MobilizerBase::template VVector<U>;
  template <typename U>
  using HMatrix = typename MobilizerBase::template HMatrix<U>;

  // Constructor for a %UniversalMobilizer between an inboard frame F
  // `inboard_frame_F` and an outboard frame M `outboard_frame_M` granting
  // two rotational degrees of freedom corresponding to angles θ₀, θ₁ as
  // described in this class's documentation.
  UniversalMobilizer(const SpanningForest::Mobod& mobod,
                     const Frame<T>& inboard_frame_F,
                     const Frame<T>& outboard_frame_M)
      : MobilizerBase(mobod, inboard_frame_F, outboard_frame_M) {}

  ~UniversalMobilizer() final;

  std::unique_ptr<BodyNode<T>> CreateBodyNode(
      const BodyNode<T>* parent_node, const RigidBody<T>* body,
      const Mobilizer<T>* mobilizer) const final;

  // Overloads to define the suffix names for the position and velocity
  // elements.
  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  bool can_rotate() const final { return true; }
  bool can_translate() const final { return false; }

  // Retrieves from `context` the two angles, (θ₀, θ₁) which describe the state
  // for `this` mobilizer as documented in this class's documentation.
  //
  // @param[in] context The context of the model this mobilizer belongs to.
  // @returns angles The two angles (θ₀, θ₁) packed and returned as a vector
  //                 with entries `angles(0) = θ₀`, `angles(1) = θ₁`.
  Vector2<T> get_angles(const systems::Context<T>& context) const;

  // Sets in `context` the state for `this` mobilizer to the angles (θ₀, θ₁)
  // provided in the input argument `angles`, which stores them with the format
  // `angles = [θ₀, θ₁]`.
  //
  // @param[in] context The context of the model this mobilizer belongs to.
  // @param[in] angles A vector which must pack values for the angles (θ₀, θ₁)
  //                   described in this class's documentation, at entries
  //                   `angles(0)` and `angles(1)`, respectively.
  // @returns a constant reference to `this` mobilizer.
  const UniversalMobilizer<T>& SetAngles(systems::Context<T>* context,
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
  const UniversalMobilizer<T>& SetAngularRates(
      systems::Context<T>* context, const Vector2<T>& angles_dot) const;

  // Computes the across-mobilizer transform `X_FM(q)` between the inboard
  // frame F and the outboard frame M as a function of the angles (θ₀, θ₁)
  // stored in `context`.
  math::RigidTransform<T> calc_X_FM(const T* q) const {
    const T s0 = sin(q[0]), c0 = cos(q[0]);
    const T s1 = sin(q[1]), c1 = cos(q[1]);
    Matrix3<T> R_FM_matrix;
    // clang-format off
    R_FM_matrix <<   c1,    0.0,  s1,
                   s0 * s1, c0,  -s0 * c1,
                  -c0 * s1, s0,   c0 * c1;
    // clang-format on
    return math::RigidTransform<T>(
        math::RotationMatrix<T>::MakeUnchecked(R_FM_matrix),
        Vector3<T>::Zero());
  }

  /* We're not attempting to optimize the update, but could improve slightly
  since the translation never changes (always zero). */
  void update_X_FM(const T* q, math::RigidTransform<T>* X_FM) const {
    DRAKE_ASSERT(q != nullptr && X_FM != nullptr);
    *X_FM = calc_X_FM(q);
  }

  // Computes the across-mobilizer velocity V_FM(q, v) of the outboard frame
  // M measured and expressed in frame F as a function of the angles (θ₀, θ₁)
  // stored in context and of the input angular rates v, formatted as
  // in get_angular_rates().
  // TODO(sherm1) Should not have to recalculate H_FM(q) here.
  SpatialVelocity<T> calc_V_FM(const T* q, const T* v) const {
    const Eigen::Map<const VVector<T>> w(v);
    const Eigen::Matrix<T, 3, 2> Hw = this->CalcHwMatrix(q);
    return SpatialVelocity<T>(Hw * w, Vector3<T>::Zero());
  }

  //                 Hwᵀ        Hvᵀ                 Hw_dotᵀ        Hv_dotᵀ
  // Here H₆ₓ₂=[1   0     0   | 0₃]ᵀ  Hdot = [         0₃         |  0₃  ]
  //           [0 c(q₀) s(q₀) | 0₃]          [ 0 -v₀s(q₀) v₀c(q₀) |  0₃  ]
  //
  // So A_FM = H⋅vdot + Hdot⋅v = [Hw⋅vdot + Hw_dot⋅v, 0₃]ᵀ
  SpatialAcceleration<T> calc_A_FM(const T* q, const T* v,
                                   const T* vdot) const {
    Vector3<T> Hw_dot_col1;
    const Eigen::Matrix<T, 3, 2> Hw = this->CalcHwMatrix(q, v, &Hw_dot_col1);
    const Eigen::Map<const VVector<T>> wdot(vdot);
    return SpatialAcceleration<T>(Hw * wdot + Hw_dot_col1 * v[1],
                                  Vector3<T>::Zero());
  }

  // Returns tau = H_FM_Fᵀ ⋅ F_F. See above for the structure of H.
  void calc_tau(const T* q, const SpatialForce<T>& F_BMo_F, T* tau) const {
    DRAKE_ASSERT(q != nullptr && tau != nullptr);
    Eigen::Map<VVector<T>> tau_as_vector(tau);
    const Vector3<T>& t_B_F = F_BMo_F.rotational();  // torque
    const Eigen::Matrix<T, 3, 2> Hw_FM = this->CalcHwMatrix(q);
    tau_as_vector = Hw_FM.transpose() * t_B_F;
  }

  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const final;

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const final;

  // Computes the across-mobilizer acceleration `A_FM(q, v, v̇)` of the
  // outboard frame M in the inboard frame F.
  // By definition `A_FM = d_F(V_FM)/dt = H_FM(q) * v̇ + Ḣ_FM * v`.
  // The acceleration `A_FM` will be a function of the rotation angles q (θ₀,
  // θ₁) and their rates of change v (ω₁, ω₂) from the `context` as well as the
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
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // Calculates the rotational part of matrix H and optionally its derivative.
  // See Mobilizer documentation for notation. If you want the derivative, pass
  // both v and Hw_dot.
  Eigen::Matrix<T, 3, 2> CalcHwMatrix(const T* q, const T* v = nullptr,
                                      Vector3<T>* Hw_dot = nullptr) const;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::UniversalMobilizer);
