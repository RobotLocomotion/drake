#pragma once

#include <concepts>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"
#include "drake/multibody/contact_solvers/icf/holonomic_constraints_data_pool.h"
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

/* Shifts a spatial impulse applied to a rigid body from a point B to a point A,
both fixed to the same body. Given the spatial impulse Γ_B applied at B and the
position p_AB of B relative to A, returns the equivalent spatial impulse Γ_A at
A:
  Γ_A = Φ(p_AB)⋅Γ_B,
where Φ(p) is the rigid shift operator:
   Φ(p) =
   | I₃  pₓ |
   | 0   I₃ |

All quantities must be expressed in the same frame.

Γ is represented in code as "Gamma". */
template <typename T>
Vector6<T> ShiftSpatialImpulse(const Vector6<T>& Gamma_B,
                               const Vector3<T>& p_AB) {
  Vector6<T> Gamma_A;
  Gamma_A.template head<3>() =
      Gamma_B.template head<3>() + p_AB.cross(Gamma_B.template tail<3>());
  Gamma_A.template tail<3>() = Gamma_B.template tail<3>();
  return Gamma_A;
}

/* CRTP base class implementing the shared machinery for a pool of holonomic
constraints between pairs of bodies. Weld, ball, and distance constraints are
all holonomic: each has a convex cost
  ℓ(vc) = ½(v̂ − vc)ᵀ⋅R⁻¹⋅(v̂ − vc),
where vc ∈ ℝᴺ is the constraint velocity, v̂ a bias velocity from the constraint
function g₀, and R a (near-rigid or compliant) regularization. The impulse is
γ = R⁻¹⋅(v̂ − vc), applied to body B and (negated) to body A.

This base owns all the constraint-agnostic machinery: the regularization R, the
bias v̂, the cost, gradient, block Hessian, the sparsity pattern, and model
reduction.

Each concrete pool supplies the following through the CRTP. The signatures are
checked by IsHolonomicConstraintsDerived (declared below).

Numeric hooks:
  - CalcConstraintVelocity() → vc, the constraint velocity (ℝᴺ).
  - CalcSpatialImpulses()    → (Γ_Bo, Γ_Ao), the constraint impulse γ shifted to
                               the body origins of B and A.
  - CalcHessianBlocks()      → the body-space Hessian blocks
                               G_Xp = Φ(p_X)ᵀ⋅G⋅Φ(p_X) with G = R⁻¹.

Bookkeeping hooks:
  - GetDataPool()       → the derived's typed data pool within an IcfData.
  - ResizeGeometry()    → (re)sizes the derived's own geometry storage.
  - ReduceGeometryInto()→ copies one constraint's geometry into a reduced pool.
  - kFlipNegatesG0      → a static constexpr trait (see below).

@tparam_nonsymbolic_scalar
@tparam N The number of constraint equations.
@tparam Derived The concrete constraint pool. */
template <typename T, int N, typename Derived>
class HolonomicConstraintsPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HolonomicConstraintsPool);

  ~HolonomicConstraintsPool();

  using ConstraintVector = Eigen::Matrix<T, N, 1>;
  using DataPool = HolonomicConstraintsDataPool<T, N>;

  /* Whether the constraint function g₀ negates when a reduced-model constraint
  flips A/B (see ReduceInto). This is true when g₀ is a "relative" quantity that
  reverses under swapping the two constraint points P and Q — e.g. the weld's
  (a_PQ, p_PoQo) or the ball's p_PQ. A concrete pool whose g₀ is invariant under
  the swap (e.g. the distance's scalar d − ℓ) hides this with a `false` value.
  The constraint velocity vc is invariant under the flip either way (the derived
  flips its geometry to keep it so), so g₀ and vc must transform consistently
  for the bias v̂ = −g₀/(δt + τ) to stay correct.

  WARNING: because this trait has a default, a concrete pool whose g₀ does not
  follow the default gets NO compile error if it forgets to shadow it — the
  reduced-model bias v̂ is then silently wrong. A new pool must confirm the
  correct value (there is Reduce test coverage that exercises the flip). */
  static constexpr bool kFlipNegatesG0 = true;

  /* @see IsAbstractConstraintsPool. */
  const IcfModel<T>& model() const { return *model_; }
  int num_constraints() const { return ssize(body_pairs_); }
  void AccumulateGradient(const IcfData<T>& data, VectorX<T>* gradient) const;
  void AccumulateHessian(
      const IcfData<T>& data,
      contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>*
          hessian) const;
  void ReduceInto(const ReducedMapping& mapping, Derived* reduced_pool) const;

  /* Resizes the pool (generic data + the derived's geometry) to store the given
  number of constraints. Constraints hold invalid data until Set(). */
  void Resize(int num_constraints);

  /* Computes the sparsity pattern for the pool. Clique i is connected to
  clique j > i iff sparsity[i] contains j. */
  void CalcSparsityPattern(std::vector<std::vector<int>>* sparsity) const;

  /* Precomputes the iteration-invariant regularization R, bias v̂, and Hessian
  blocks. Must be called after all Set() calls and whenever the time step
  changes (R and v̂ depend on δt). */
  void PrecomputeHessianBlocks();

  /* Computes problem data (impulses γ and cost) from the body spatial
  velocities V_WB. */
  void CalcData(const EigenPool<Vector6<T>>& V_WB, DataPool* data) const;

  /* Computes the first and second derivatives of the cost ℓ̃(α) = ℓ(v + α⋅w).
  @param data Constraint data computed at v + α⋅w.
  @param U_WB Body spatial velocities evaluated at v + α⋅w. */
  void CalcCostAlongLine(const DataPool& data,
                         const EigenPool<Vector6<T>>& U_WB, T* dcost,
                         T* d2cost) const;

  /* Testing-only access to the generic per-constraint data. */
  const std::vector<std::pair<int, int>>& body_pairs() const {
    return body_pairs_;
  }
  const EigenPool<ConstraintVector>& g0() const { return g0_; }
  const std::vector<T>& stiffness() const { return stiffness_; }
  const std::vector<T>& damping() const { return damping_; }
  const std::vector<T>& R() const { return R_; }
  const EigenPool<ConstraintVector>& v_hat() const { return v_hat_; }
  int hessian_blocks_size() const { return ssize(hessian_blocks_); }

 protected:
  explicit HolonomicConstraintsPool(const IcfModel<T>* parent_model);

  /* Resizes only the generic base-owned arrays (not the derived's geometry). */
  void ResizeCommon(int num_constraints);

  /* Stores the generic per-constraint data. Called by the derived's Set() after
  it stores its geometry.
  @pre index in range, bodyB not anchored.
  @pre stiffness > 0
  @pre damping >= 0 */
  void SetCommon(int index, int bodyA, int bodyB, const ConstraintVector& g0,
                 const T& stiffness, const T& damping);

  // Precomputed, iteration-invariant Hessian blocks. Rank-N per constraint.
  struct HessianBlock {
    int c_B{-1};         // Clique index for body B (always valid).
    int c_A{-1};         // Clique index for body A (-1 if anchored).
    MatrixX<T> H_BB;     // Diagonal block for body B's clique.
    MatrixX<T> H_AA;     // Diagonal block for body A's clique (if dynamic).
    MatrixX<T> H_cross;  // Off-diagonal (or same-clique cross) block.
    int cross_row{-1};
    int cross_col{-1};
    bool A_is_dynamic{false};
  };

  const IcfModel<T>* const model_;  // The parent model.

  // Body pairs (bodyA, bodyB); bodyB is always dynamic (not anchored).
  std::vector<std::pair<int, int>> body_pairs_;

  // Constraint function at the start of the step, g₀ ∈ ℝᴺ, per constraint.
  EigenPool<ConstraintVector> g0_;

  // User compliance per constraint (stiffness +∞ / damping 0 ⇒ near-rigid).
  std::vector<T> stiffness_;
  std::vector<T> damping_;

  // Scalar regularization R and bias v̂ per constraint, computed in
  // PrecomputeHessianBlocks() (depend on the time step, so recomputed on
  // UpdateTimeStep).
  std::vector<T> R_;
  EigenPool<ConstraintVector> v_hat_;

  std::vector<HessianBlock> hessian_blocks_;

 private:
  Derived& mutable_derived() { return *static_cast<Derived*>(this); }
  const Derived& derived() const { return *static_cast<const Derived*>(this); }
};

/* Provides documentation and type-signature checking for the per-constraint
"hooks" a concrete pool must supply to HolonomicConstraintsPool through the
CRTP.

Clients invoke the concept on the conforming pool after its definition is
complete:

@code
static_assert(IsHolonomicConstraintsDerived<WeldConstraintsPool>);
@endcode
*/
template <template <typename> typename Pool>
concept IsHolonomicConstraintsDerived = requires(
    Pool<double> pool, Pool<double>* reduced, const IcfData<double>& data,
    const Vector6<double>& V_WB, const Vector6<double>* V_WA,
    typename Pool<double>::ConstraintVector gamma, double R_inv,
    Vector6<double>* spatial_impulse, Matrix6<double>* hessian_block, int k,
    bool flip) {
  /* Computes vc ∈ ℝᴺ, the constraint velocity of constraint k. V_WA is null
  when body A is anchored. */
  {
    pool.CalcConstraintVelocity(k, V_WB, V_WA)
  } -> std::same_as<typename Pool<double>::ConstraintVector>;

  /* Shifts the constraint impulse γ of constraint k to the body origins,
  writing Γ_Bo on B and (when non-null) Γ_Ao on A. */
  pool.CalcSpatialImpulses(k, gamma, spatial_impulse, spatial_impulse);

  /* Builds the body-space Hessian blocks G_Xp = Φ(p_X)ᵀ⋅G⋅Φ(p_X) of constraint
  k. The A and cross blocks are written only when non-null (A dynamic). */
  pool.CalcHessianBlocks(k, R_inv, hessian_block, hessian_block, hessian_block);

  /* Returns this pool's typed data pool within `data`. */
  {
    pool.GetDataPool(data)
  } -> std::same_as<const typename Pool<double>::DataPool&>;

  /* (Re)sizes the derived's own geometry storage. */
  pool.ResizeGeometry(k);

  /* Copies constraint k's geometry into `reduced`, flipping A/B if `flip`. */
  pool.ReduceGeometryInto(reduced, k, flip);

  /* Whether g₀ negates when a reduced constraint flips A/B. */
  { Pool<double>::kFlipNegatesG0 } -> std::convertible_to<bool>;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
