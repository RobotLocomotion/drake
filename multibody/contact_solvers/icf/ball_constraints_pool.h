#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/abstract_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/ball_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/reduced_mapping.h"

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
class BallConstraintsPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BallConstraintsPool);

  /* Constructs an empty pool. */
  explicit BallConstraintsPool(const IcfModel<T>* parent_model);

  ~BallConstraintsPool();

  /* @see IsAbstractConstraintsPool. */
  const IcfModel<T>& model() const { return *model_; }
  int num_constraints() const { return ssize(body_pairs_); }
  void AccumulateGradient(const IcfData<T>& data, VectorX<T>* gradient) const;
  void AccumulateHessian(
      const IcfData<T>& data,
      contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>*
          hessian) const;
  void ReduceInto(const ReducedMapping& mapping,
                  BallConstraintsPool<T>* reduced_pool) const;

  /* Resizes the constraints pool to store the given number of ball constraints.

  @warning After resizing, constraints may hold invalid data until Set() is
  called for each constraint index in [0, num_constraints()). */
  void Resize(int num_constraints);

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

  /* Computes the sparsity pattern for the pool. Clique i is connected to
  clique j > i iff sparsity[i] contains j. */
  void CalcSparsityPattern(std::vector<std::vector<int>>* sparsity) const;

  /* Precomputes the iteration-invariant Hessian blocks for every ball
  constraint. Because the ball cost ℓ(vc) = ½(v̂ − vc)ᵀR⁻¹(v̂ − vc) is
  purely quadratic in the constraint velocity, its Hessian ∂²ℓ/∂v² depends
  only on the regularization R and the (constant) constraint Jacobians. This
  method must be called after all Set() calls and before AccumulateHessian()
  is used. */
  void PrecomputeHessianBlocks();

  /* Computes problem data as a function of the body spatial velocities V_WB for
  the full IcfModel. */
  void CalcData(const EigenPool<Vector6<T>>& V_WB,
                BallConstraintsDataPool<T>* ball_data) const;

  /* Computes the first and second derivatives of the constraint cost
  ℓ̃(α) = ℓ(v + α⋅w).

  @param ball_data Constraint data computed at v + α⋅w.
  @param U_WB Body spatial velocities when generalized velocities equal w.
         I.e., U_WB = J_WB⋅w.
  @param[out] dcost the first derivative dℓ̃/dα on output.
  @param[out] d2cost the second derivative d²ℓ̃/dα² on output. */
  void CalcCostAlongLine(const BallConstraintsDataPool<T>& ball_data,
                         const EigenPool<Vector6<T>>& U_WB, T* dcost,
                         T* d2cost) const;

  /* Testing only access. */
  const std::vector<std::pair<int, int>>& body_pairs() const {
    return body_pairs_;
  }
  const EigenPool<Vector3<T>>& p_AP_W() const { return p_AP_W_; }
  const EigenPool<Vector3<T>>& p_BQ_W() const { return p_BQ_W_; }
  const EigenPool<Vector3<T>>& p_PQ_W() const { return p_PQ_W_; }
  const EigenPool<Vector3<T>>& g0() const { return p_PQ_W_; }
  const EigenPool<Vector3<T>>& R() const { return R_; }
  int hessian_blocks_size() const { return ssize(hessian_blocks_); }

 private:
  const IcfModel<T>* const model_;  // The parent model.

  // Body pairs involved in each ball constraint, (bodyA, bodyB).
  // bodyB is always dynamic (not anchored).
  std::vector<std::pair<int, int>> body_pairs_;

  // Per-constraint data, all indexed by constraint index k.
  EigenPool<Vector3<T>> p_AP_W_;  // Position of P in A, expressed in W.
  EigenPool<Vector3<T>> p_BQ_W_;  // Position of Q in B, expressed in W.
  // Relative translation, expressed in W. For a ball constraint this is also
  // the constraint function at the start of the step, g₀ = p_PQ_W.
  EigenPool<Vector3<T>> p_PQ_W_;

  // Near-rigid regularization per constraint.
  // R depends on the current time step δt and is computed in
  // PrecomputeHessianBlocks(), which must be called whenever δt changes.
  // R is a diagonal 3×3 regularization matrix.
  EigenPool<Vector3<T>> R_;  // The diagonal regularization matrix.

  // Precomputed Hessian blocks for each ball constraint, populated by
  // PrecomputeHessianBlocks(). These are iteration-invariant because the ball
  // cost Hessian depends only on R and the constant constraint Jacobians.
  struct HessianBlock {
    int c_B{-1};         // Clique index for body B (always valid).
    int c_A{-1};         // Clique index for body A (-1 if anchored).
    MatrixX<T> H_BB;     // Diagonal block for body B's clique.
    MatrixX<T> H_AA;     // Diagonal block for body A's clique (if dynamic).
    MatrixX<T> H_cross;  // Off-diagonal (or same-clique cross) block.
    int cross_row{-1};   // Block row index for the cross term.
    int cross_col{-1};   // Block column index for the cross term.
    bool A_is_dynamic{false};  // True when body A is not anchored.
  };
  std::vector<HessianBlock> hessian_blocks_;
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
