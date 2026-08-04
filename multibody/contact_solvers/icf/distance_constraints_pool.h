#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/icf/abstract_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"
#include "drake/multibody/contact_solvers/icf/holonomic_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/icf/holonomic_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

// Forward declaration to break circular dependencies.
template <typename T>
class IcfModel;

/* A pool of distance constraints between pairs of bodies.

Each distance constraint connects distinct bodies A and B, constraining the
Euclidean distance d between a point P on A and a point Q on B to a free length
ℓ. This is a single (scalar) holonomic constraint equation:
  g = d − ℓ = 0 ∈ ℝ.
Adapted from the SAP distance constraint (see sap_distance_constraint.h),
this is a compliant (spring-damper) constraint: with p̂ the unit vector from P
to Q, stiffness k and damping c, the scalar impulse is
     γ = δt⋅(−k⋅(d−ℓ) − c⋅ḋ) ∈ ℝ
applied as γ⋅p̂ on B at Q and −γ⋅p̂ on A at P.

Unlike other holonomic constraints (e.g. weld, ball), the distance constraint
exposes stiffness and damping parameters to the user, allowing the modeling of
a linear spring-damper between two points. If the stiffness is set to +∞, the
constraint uses the near-rigid approximation to approximate a rigid distance
constraint.

@tparam_nonsymbolic_scalar */
template <typename T>
class DistanceConstraintsPool
    : public HolonomicConstraintsPool<T, 1, DistanceConstraintsPool<T>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DistanceConstraintsPool);

  explicit DistanceConstraintsPool(const IcfModel<T>* parent_model)
      : HolonomicConstraintsPool<T, 1, DistanceConstraintsPool<T>>(
            parent_model) {}

  /* The scalar constraint function g₀ = d − ℓ is symmetric under swapping the
  points P and Q, so (unlike the weld/ball) it does not negate on an A/B flip
  during model reduction. See HolonomicConstraintsPool::kFlipNegatesG0(). */
  static constexpr bool FlipNegatesG0() { return false; }

  /* Sets the k-th distance constraint.
  @param index The index of the constraint within the pool.
  @param bodyA The index of body A. May be anchored.
  @param bodyB The index of body B. Must not be anchored.
  @param p_AP_W Position of constraint point P in body A, expressed in world.
  @param p_BQ_W Position of constraint point Q in body B, expressed in world.
  @param p_hat_W Unit vector from P to Q, expressed in world.
  @param g0 The constraint function at q₀, g₀ = d₀ − ℓ.
  @param stiffness The constraint stiffness k in N/m (may be +∞ for near-rigid).
  @param damping The constraint damping c in N⋅s/m.
  @pre indices in range, bodyA ≠ bodyB, bodyB not anchored, stiffness > 0,
       damping ≥ 0. */
  void Set(int index, int bodyA, int bodyB, const Vector3<T>& p_AP_W,
           const Vector3<T>& p_BQ_W, const Vector3<T>& p_hat_W, const T& g0,
           const T& stiffness, const T& damping);

  /* Hooks required by HolonomicConstraintsPool (CRTP). */
  Vector1<T> CalcConstraintVelocity(int k, const Vector6<T>& V_WB,
                                    const Vector6<T>* V_WA) const;
  void CalcSpatialImpulses(int k, const Vector1<T>& gamma, Vector6<T>* Gamma_Bo,
                           Vector6<T>* Gamma_Ao) const;
  void CalcHessianBlocks(int k, const T& R_inv, Matrix6<T>* G_Bp,
                         Matrix6<T>* G_Ap, Matrix6<T>* G_cross) const;
  const DistanceConstraintsDataPool<T>& GetDataPool(
      const IcfData<T>& data) const {
    return data.distance_constraints_data();
  }
  void ResizeGeometry(int num_constraints);
  void ReduceGeometryInto(DistanceConstraintsPool<T>* reduced, int k,
                          bool flip) const;

  /* Testing-only access. */
  const EigenPool<Vector3<T>>& p_AP_W() const { return p_AP_W_; }
  const EigenPool<Vector3<T>>& p_BQ_W() const { return p_BQ_W_; }
  const EigenPool<Vector3<T>>& p_hat_W() const { return p_hat_W_; }

 private:
  EigenPool<Vector3<T>> p_AP_W_;
  EigenPool<Vector3<T>> p_BQ_W_;
  EigenPool<Vector3<T>> p_hat_W_;
};
static_assert(IsAbstractConstraintsPool<DistanceConstraintsPool>);
static_assert(IsHolonomicConstraintsDerived<DistanceConstraintsPool>);

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        DistanceConstraintsPool);
