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

/* A pool of weld constraints between pairs of bodies.

Each weld constraint connects distinct bodies A and B, enforcing coincidence
of a frame P on A and a frame Q on B. The constraint function is defined as
  g = (a_PQ, p_PoQo) = 0 ∈ ℝ⁶
where a_PQ = θ⋅k is the Euler vector for the relative rotation R_PQ, and
p_PoQo is the relative translation between the constraint frames P and Q.

Following the SAP weld constraint formulation (see sap_weld_constraint.h), we
define the constraint velocity as the relative spatial velocity of points Am
(fixed to A) and Bm (fixed to B) coincident at the midpoint M between P and Q:
  vc = V_W_AmBm ∈ ℝ⁶
and the convex cost as:
  ℓ(vc) = ½(v̂ − vc)ᵀR⁻¹(v̂ − vc)
where R is a 6×6 diagonal "near-rigid" regularization matrix and v̂ is a bias
velocity computed from g₀, the constraint function evaluated at q₀.

This produces spatial impulse γ ≜ −dℓ(vc)/dvc = (γᵣ, γₜ) ∈ ℝ⁶ which we use to
apply equal and opposite impulses at Am and Bm (co-located at the midpoint M),
ensuring conservation of angular momentum and satisfaction of Newton's third
law. Our sign convention is such that γ is the impulse on B, so the impulse on A
is −γ.

Like patch constraints, weld constraints involve two bodies and can introduce
cross-clique coupling (off-diagonal blocks in the Hessian) when the two bodies
belong to different cliques.

@tparam_nonsymbolic_scalar */
template <typename T>
class WeldConstraintsPool
    : public HolonomicConstraintsPool<T, 6, WeldConstraintsPool<T>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WeldConstraintsPool);

  /* Constructs an empty pool. */
  explicit WeldConstraintsPool(const IcfModel<T>* parent_model)
      : HolonomicConstraintsPool<T, 6, WeldConstraintsPool<T>>(parent_model) {}

  /* Sets the k-th weld constraint.

  @param index The index of the constraint within the pool.
  @param bodyA The index of body A in the IcfModel. May be anchored.
  @param bodyB The index of body B in the IcfModel. Must not be anchored.
  @param p_AP_W Position of constraint point P in body A, expressed in world.
  @param p_BQ_W Position of constraint point Q in body B, expressed in world.
  @param p_PoQo_W Position of Qo relative to Po, expressed in world.
  @param a_PQ_W Euler vector a_PQ = θ⋅k for relative rotation, in world frame.

  Calling this function several times with the same `index` overwrites the
  previous constraint for that index.

  @pre all indices are in range, bodyA ≠ bodyB, bodyB not anchored. */
  void Set(int index, int bodyA, int bodyB, const Vector3<T>& p_AP_W,
           const Vector3<T>& p_BQ_W, const Vector3<T>& p_PoQo_W,
           const Vector3<T>& a_PQ_W);

  /* Hooks required by HolonomicConstraintsPool (CRTP). */
  Vector6<T> CalcConstraintVelocity(int k, const Vector6<T>& V_WB,
                                    const Vector6<T>* V_WA) const;
  void CalcSpatialImpulses(int k, const Vector6<T>& gamma, Vector6<T>* Gamma_Bo,
                           Vector6<T>* Gamma_Ao) const;
  void CalcHessianBlocks(int k, const T& R_inv, Matrix6<T>* G_Bp,
                         Matrix6<T>* G_Ap, Matrix6<T>* G_cross) const;
  const WeldConstraintsDataPool<T>& GetDataPool(const IcfData<T>& data) const {
    return data.weld_constraints_data();
  }
  void ResizeGeometry(int num_constraints);
  void ReduceGeometryInto(WeldConstraintsPool<T>* reduced, int k,
                          bool flip) const;

  /* Testing only access. */
  const EigenPool<Vector3<T>>& p_AP_W() const { return p_AP_W_; }
  const EigenPool<Vector3<T>>& p_BQ_W() const { return p_BQ_W_; }
  const EigenPool<Vector3<T>>& p_PoQo_W() const { return p_PoQo_W_; }

 private:
  // Per-constraint data, all indexed by constraint index k.
  EigenPool<Vector3<T>> p_AP_W_;    // Position of P in A, expressed in W.
  EigenPool<Vector3<T>> p_BQ_W_;    // Position of Q in B, expressed in W.
  EigenPool<Vector3<T>> p_PoQo_W_;  // Relative translation, expressed in W.
};
static_assert(IsAbstractConstraintsPool<WeldConstraintsPool>);

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        WeldConstraintsPool);
