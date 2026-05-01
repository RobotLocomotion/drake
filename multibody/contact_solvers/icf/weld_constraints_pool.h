#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/reduced_mapping.h"
#include "drake/multibody/contact_solvers/icf/weld_constraints_data_pool.h"

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
class WeldConstraintsPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WeldConstraintsPool);

  /* Constructs an empty pool. */
  explicit WeldConstraintsPool(const IcfModel<T>* parent_model);

  ~WeldConstraintsPool();

  /* Returns a reference to the parent model. */
  const IcfModel<T>& model() const { return *model_; }

  /* Returns the total number of weld constraints stored in this pool. */
  int num_constraints() const { return ssize(body_pairs_); }

  /* Resizes the constraints pool to store the given number of weld constraints.

  @warning After resizing, constraints may hold invalid data until Set() is
  called for each constraint index in [0, num_constraints()). */
  void Resize(int num_constraints);

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

  /* Computes the sparsity pattern for the pool. Clique i is connected to
  clique j > i iff sparsity[i] contains j. */
  void CalcSparsityPattern(std::vector<std::vector<int>>* sparsity) const;

  /* Precomputes the iteration-invariant Hessian blocks for every weld
  constraint. Because the weld cost ℓ(vc) = ½(v̂ − vc)ᵀR⁻¹(v̂ − vc) is
  purely quadratic in the constraint velocity, its Hessian ∂²ℓ/∂v² depends
  only on the regularization R and the (constant) constraint Jacobians. This
  method must be called after all Set() calls and before AccumulateHessian()
  is used. */
  void PrecomputeHessianBlocks();

  /* Computes problem data as a function of the body spatial velocities V_WB for
  the full IcfModel. */
  void CalcData(const EigenPool<Vector6<T>>& V_WB,
                WeldConstraintsDataPool<T>* weld_data) const;

  /* Adds the gradient contribution of this constraint, ∇ℓ(v) = −Jᵀγ, to the
  model-wide gradient. */
  void AccumulateGradient(const IcfData<T>& data, VectorX<T>* gradient) const;

  /* Adds the contribution of this constraint to the model-wide Hessian. */
  void AccumulateHessian(
      const IcfData<T>& data,
      contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>*
          hessian) const;

  /* Computes the first and second derivatives of the constraint cost
  ℓ̃(α) = ℓ(v + α⋅w).

  @param weld_data Constraint data computed at v + α⋅w.
  @param U_WB Body spatial velocities when generalized velocities equal w.
         I.e., U_WB = J_WB⋅w.
  @param[out] dcost the first derivative dℓ̃/dα on output.
  @param[out] d2cost the second derivative d²ℓ̃/dα² on output. */
  void CalcCostAlongLine(const WeldConstraintsDataPool<T>& weld_data,
                         const EigenPool<Vector6<T>>& U_WB, T* dcost,
                         T* d2cost) const;

  void Reduce(const ReducedMapping& mapping,
              WeldConstraintsPool<T>* reduced) const;

  /* Testing only access. */
  const std::vector<std::pair<int, int>>& body_pairs() const {
    return body_pairs_;
  }

 private:
  const IcfModel<T>* const model_;  // The parent model.

  // Body pairs involved in each weld constraint, (bodyA, bodyB).
  // bodyB is always dynamic (not anchored).
  std::vector<std::pair<int, int>> body_pairs_;

  // Per-constraint data, all indexed by constraint index k.
  EigenPool<Vector3<T>> p_AP_W_;    // Position of P in A, expressed in W.
  EigenPool<Vector3<T>> p_BQ_W_;    // Position of Q in B, expressed in W.
  EigenPool<Vector3<T>> p_PoQo_W_;  // Relative translation, expressed in W.

  std::vector<Vector6<T>> g0_;  // Constraint function at start of step.

  // Near-rigid regularization per constraint.
  // R depends on the current time step δt and is computed in
  // PrecomputeHessianBlocks(), which must be called whenever δt changes.
  // R is a diagonal 6×6 regularization matrix: R = diag(Rᵣ, Rₜ).
  std::vector<Vector6<T>> R_;  // The diagonal regularization matrix.

  // TODO(sherm1) Consider whether this should be using EigenPools to
  //  reduce memory use.
  // Precomputed Hessian blocks for each weld constraint, populated by
  // PrecomputeHessianBlocks(). These are iteration-invariant because the weld
  // cost Hessian depends only on R and the constant constraint Jacobians.
  struct HessianBlock {
    int c_b{-1};         // Clique index for body B (always valid).
    int c_a{-1};         // Clique index for body A (-1 if anchored).
    MatrixX<T> H_BB;     // Diagonal block for body B's clique.
    MatrixX<T> H_AA;     // Diagonal block for body A's clique (if dynamic).
    MatrixX<T> H_cross;  // Off-diagonal (or same-clique cross) block.
    int cross_row{-1};   // Block row index for the cross term.
    int cross_col{-1};   // Block column index for the cross term.
    bool a_is_dynamic{false};  // True when body A is not anchored.
  };
  std::vector<HessianBlock> hessian_blocks_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        WeldConstraintsPool);
