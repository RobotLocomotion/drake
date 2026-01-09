#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/coupler_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"

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

  /* Returns a reference to the parent model. */
  const IcfModel<T>& model() const { return *model_; }

  /* Returns the total number of constraints stored in this pool. */
  int num_constraints() const { return constraint_to_clique_.size(); }

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

  /* Adds the gradient contribution of this constraint, ∇ℓ(v) = −γ, to the
  model-wide gradient. */
  void AccumulateGradient(const IcfData<T>& data, VectorX<T>* gradient) const;

  /* Adds the contribution of this constraint to the model-wide Hessian. */
  void AccumulateHessian(
      const IcfData<T>& data,
      contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>*
          hessian) const;

  /* Computes the first and second derivatives of the constraint cost
  ℓ̃ (α) = ℓ(v + α⋅w).

  @param coupler_data Constraint data computed at v + α⋅w.
  @param w The search direction.
  @param[out] dcost the first derivative dℓ̃ /dα on output.
  @param[out] d2cost the second derivative d²ℓ̃ /dα² on output.

  TODO(vincekurtz): factor out common documentation among constraints. */
  void CalcCostAlongLine(const CouplerConstraintsDataPool<T>& coupler_data,
                         const VectorX<T>& w, T* dcost, T* d2cost) const;

  /* Testing only access. */
  const std::vector<int>& constraint_to_clique() const {
    return constraint_to_clique_;
  }
  const std::vector<std::pair<int, int>>& dofs() const { return dofs_; }
  const std::vector<T>& gear_ratio() const { return gear_ratio_; }

 private:
  const IcfModel<T>* const model_;  // The parent model.

  // Clique for the k-th constraint, of size num_constraints().
  std::vector<int> constraint_to_clique_;

  // DOFs (i, j) for the k-th constraint, of size num_constraints().
  std::vector<std::pair<int, int>> dofs_;

  // Gear ratio ρ per constraint, of size num_constraints().
  std::vector<T> gear_ratio_;

  // Regularization and bias per constraint, of size num_constraints().
  std::vector<T> g_hat_;  // The true bias is v̂ = ĝ / δt.
  std::vector<T> R_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        CouplerConstraintsPool);
