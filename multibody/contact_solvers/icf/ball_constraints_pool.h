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

/* A pool of ball constraints between pairs of bodies.

Each ball constraint connects distinct bodies A and B, enforcing coincidence of
a point P on A and a point Q on B. The constraint function is defined as
  g = p_PQ = 0 ∈ ℝ³
where p_PQ is the relative translation between the constraint points P and Q.
Unlike the weld constraint, the ball constraint leaves the relative orientation
of the two bodies free (constraining 3 translational DOFs, no rotational DOFs).

Following the SAP weld constraint formulation (see sap_weld_constraint.h), we
define the constraint velocity as the relative translational velocity of points
Am (fixed to A) and Bm (fixed to B) coincident at the midpoint M between P and
Q:
  vc = v_W_AmBm ∈ ℝ³
and the convex cost as:
  ℓ(vc) = ½(v̂ − vc)ᵀR⁻¹(v̂ − vc)
where R is a 3×3 diagonal "near-rigid" regularization matrix and v̂ is a bias
velocity computed from g₀, the constraint function evaluated at q₀.

This produces impulse γ ≜ −dℓ(vc)/dvc ∈ ℝ³ which we use to apply equal and
opposite impulses at Am and Bm (co-located at the midpoint M), ensuring
conservation of angular momentum and satisfaction of Newton's third law.
Our sign convention is such that γ is the impulse on B, so the impulse on A
is −γ.

N.B. Applying the impulse at the midpoint M is a deliberate deviation from the
SAP ball constraint (see sap_ball_constraint.h), which applies the impulse at
the points P and Q directly and therefore does NOT conserve angular momentum
when P and Q are not coincident (it introduces a small moment of order
O(‖γ‖⋅‖p_PQ‖)). The midpoint formulation used here matches the ICF weld
constraint and conserves angular momentum. Concretely, the ball constraint is
the translational (lower) half of the weld constraint.

Like patch and weld constraints, ball constraints involve two bodies and can
introduce cross-clique coupling (off-diagonal blocks in the Hessian) when the
two bodies belong to different cliques.

@tparam_nonsymbolic_scalar */
template <typename T>
class BallConstraintsPool
    : public HolonomicConstraintsPool<T, 3, BallConstraintsPool<T>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BallConstraintsPool);

  /* Constructs an empty pool. */
  explicit BallConstraintsPool(const IcfModel<T>* parent_model)
      : HolonomicConstraintsPool<T, 3, BallConstraintsPool<T>>(parent_model) {}

  /* Sets the k-th ball constraint.

  @param index The index of the constraint within the pool.
  @param bodyA The index of body A in the IcfModel. May be anchored.
  @param bodyB The index of body B in the IcfModel. Must not be anchored.
  @param p_AP_W Position of constraint point P in body A, expressed in world.
  @param p_BQ_W Position of constraint point Q in body B, expressed in world.
  @param p_PQ_W Position of Q relative to P, expressed in world.

  Calling this function several times with the same `index` overwrites the
  previous constraint for that index.

  @pre all indices are in range, bodyA ≠ bodyB, bodyB not anchored. */
  void Set(int index, int bodyA, int bodyB, const Vector3<T>& p_AP_W,
           const Vector3<T>& p_BQ_W, const Vector3<T>& p_PQ_W);

  /* Hooks required by HolonomicConstraintsPool. */
  Vector3<T> CalcConstraintVelocity(int k, const Vector6<T>& V_WB,
                                    const Vector6<T>* V_WA) const;
  void CalcSpatialImpulses(int k, const Vector3<T>& gamma, Vector6<T>* Gamma_Bo,
                           Vector6<T>* Gamma_Ao) const;
  void CalcHessianBlocks(int k, const T& R_inv, Matrix6<T>* G_Bp,
                         Matrix6<T>* G_Ap, Matrix6<T>* G_cross) const;
  const BallConstraintsDataPool<T>& GetDataPool(const IcfData<T>& data) const {
    return data.ball_constraints_data();
  }
  void ResizeGeometry(int num_constraints);
  void ReduceGeometryInto(BallConstraintsPool<T>* reduced, int k,
                          bool flip) const;

  /* Testing only access. */
  const EigenPool<Vector3<T>>& p_AP_W() const { return p_AP_W_; }
  const EigenPool<Vector3<T>>& p_BQ_W() const { return p_BQ_W_; }
  // Relative translation, which for a ball is also the constraint function
  // g₀ = p_PQ_W (stored by the base).
  const EigenPool<Vector3<T>>& p_PQ_W() const { return this->g0(); }

 private:
  // Per-constraint data, all indexed by constraint index k.
  EigenPool<Vector3<T>> p_AP_W_;  // Position of P in A, expressed in W.
  EigenPool<Vector3<T>> p_BQ_W_;  // Position of Q in B, expressed in W.
};
static_assert(IsAbstractConstraintsPool<BallConstraintsPool>);

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        BallConstraintsPool);
