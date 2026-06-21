#pragma once

#include <span>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/abstract_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/coupler_constraints_data_pool.h"
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

/* A pool of coupler constraints (qᵢ - ρqⱼ = Δq) linking generalized positions
(qᵢ, qⱼ) with gear ratio ρ and offset Δq.

Each coupler constraint is associated with a convex cost ℓ(v) that is added to
the overall ICF cost. The gradient of this cost produces an impulse γ = -∇ℓ(v)
that enforces the constraint when the convex ICF problem is solved.

Coupler constraints only apply to joints within the same clique, so they do not
change the sparsity structure of the problem.
TODO(vincekurtz): consider relaxing this requirement.

@tparam_nonsymbolic_scalar */
template <typename T>
class CouplerConstraintsPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CouplerConstraintsPool);

  /* Constructs an empty pool. */
  explicit CouplerConstraintsPool(const IcfModel<T>* parent_model);

  ~CouplerConstraintsPool();

  /* @see IsAbstractConstraintsPool. */
  const IcfModel<T>& model() const { return *model_; }
  int num_constraints() const { return constraint_to_clique_.size(); }
  void AccumulateGradient(const IcfData<T>& data, VectorX<T>* gradient) const;
  void AccumulateGradient(const IcfData<T>& data,
                          std::span<const int> constraints,
                          VectorX<T>* gradient) const;
  void AccumulateHessian(
      const IcfData<T>& data,
      contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>*
          hessian) const;
  void AccumulateHessian(
      const IcfData<T>& data, std::span<const int> constraints,
      std::span<const int> clique_to_block, int island,
      contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>*
          hessian) const;
  void ReduceInto(const ReducedMapping& mapping,
                  CouplerConstraintsPool<T>* reduced_pool) const;

  /* Resizes the constraints pool to store the given number of constraints.

  @warning After resizing, constraints may hold invalid data until Set() is
  called for each constraint index in [0, num_constraints()). */
  void Resize(const int num_constraints);

  /* Sets the given coupler constraint, qᵢ − ρqⱼ = Δq between two DoFs (i, j) in
  the given clique.

  @param index The index of the constraint within the pool,
               must be in [0, num_constraints()).
  @param clique The clique index where the constraint is applied. Both DoFs
                must belong to the same clique.
  @param i The clique-local index of the first DoF.
  @param j The clique-local index of the second DoF.
  @param qi The current position of the first DoF.
  @param qj The current position of the second DoF.
  @param gear_ratio The gear ratio ρ.
  @param offset The offset Δq.

  Calling this function several times with the same `index` overwrites the
  previous constraint for that index. */
  void Set(int index, int clique, int i, int j, const T& qi, const T& qj,
           T gear_ratio, T offset);

  /* Computes problem data as a function of the generalized velocities v for the
  full IcfModel. */
  void CalcData(const VectorX<T>& v,
                CouplerConstraintsDataPool<T>* coupler_data) const;

  /* Island-filtered overload: computes data only for the constraints whose
  indices are listed in `constraints` and returns the sum of their costs (it
  does not write the pool-wide cost scalar, which is shared across islands). */
  T CalcData(const VectorX<T>& v, std::span<const int> constraints,
             CouplerConstraintsDataPool<T>* coupler_data) const;

  /* Computes the first and second derivatives of the constraint cost
  ℓ̃ (α) = ℓ(v + α⋅w).

  @param coupler_data Constraint data computed at v + α⋅w.
  @param w The search direction.
  @param[out] dcost the first derivative dℓ̃ /dα on output.
  @param[out] d2cost the second derivative d²ℓ̃ /dα² on output.

  TODO(vincekurtz): factor out common documentation among constraints. */
  void CalcCostAlongLine(const CouplerConstraintsDataPool<T>& coupler_data,
                         const VectorX<T>& w, T* dcost, T* d2cost) const;

  /* Island-filtered overload: derivatives for only the listed constraints. */
  void CalcCostAlongLine(const CouplerConstraintsDataPool<T>& coupler_data,
                         const VectorX<T>& w, std::span<const int> constraints,
                         T* dcost, T* d2cost) const;

  /* Testing only access. */
  const std::vector<int>& constraint_to_clique() const {
    return constraint_to_clique_;
  }
  const std::vector<std::pair<int, int>>& dofs() const { return dofs_; }
  const std::vector<T>& gear_ratio() const { return gear_ratio_; }
  const std::vector<T>& g_hat() const { return g_hat_; }
  const std::vector<T>& R() const { return R_; }

 private:
  const IcfModel<T>* const model_;  // The parent model.

  // Clique for the k-th constraint, of size num_constraints().
  std::vector<int> constraint_to_clique_;

  // DOFs (i, j) for the k-th constraint, of size num_constraints(). In a
  // reduced pool (made by ReduceInto()), either DOF can be absent, represented
  // by a -1 value. Both DOFs cannot be absent.
  std::vector<std::pair<int, int>> dofs_;

  // Gear ratio ρ per constraint, of size num_constraints().
  std::vector<T> gear_ratio_;

  // Regularization and bias per constraint, of size num_constraints().
  std::vector<T> g_hat_;  // The true bias is v̂ = ĝ / δt.
  std::vector<T> R_;
};
static_assert(IsAbstractConstraintsPool<CouplerConstraintsPool>);

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        CouplerConstraintsPool);
