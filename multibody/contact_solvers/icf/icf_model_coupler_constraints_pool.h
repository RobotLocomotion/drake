#pragma once

#ifndef DRAKE_ICF_MODEL_NESTED_CLASS_INCLUDES
#error Do not directly include this file; instead, use icf_model.h.
#endif

#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/icf/coupler_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* A pool of coupler constraints (qᵢ - ρqⱼ = Δq) linking generalized positions
(qᵢ, qⱼ) with gear ratio ρ and offset Δq.

Coupler constraints only apply to joints within the same clique, so they do not
change the sparsity structure of the problem.
TODO(vincekurtz): consider relaxing this requirement. */
template <typename T>
class IcfModel<T>::CouplerConstraintsPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CouplerConstraintsPool);

  /* Constructor for an empty pool. */
  CouplerConstraintsPool(const IcfModel<T>* parent_model)
      : model_(parent_model) {
    DRAKE_ASSERT(parent_model != nullptr);
  }

  /* Resets, zeroing out the constraints while keeping memory allocated. */
  void Clear();

  /* Resize the constraints pool to store the given number of constraints. */
  void Resize(const int num_constraints);

  /* Set the given coupler constraint, qᵢ − ρqⱼ−Δq = 0, between the i-th and
  j-th DoFs of the given clique.

  @param index The index of the constraint within the pool,
               must be in [0, num_constraints()).
  @param clique The clique index where the constraint is applied. Both DoFs
                must belong to the same clique.
  @param i The index of the first DoF within `clique`.
  @param j The index of the second DoF within `clique`.
  @param qi The current position of the i-th DoF.
  @param qj The current position of the j-th DoF.
  @param gear_ratio The gear ratio ρ.
  @param offset The offset Δq. */
  void Set(int index, int clique, int i, int j, const T& qi, const T& qj,
           T gear_ratio, T offset);

  /* Update only the time step for this constraint pool, leaving the constraints
  otherwise unchanged. */
  void UpdateTimeStep(const T& old_time_step, const T& time_step);

  /* Compute problem data from the given generalized velocities v, and store in
  the given data struct. */
  void CalcData(const VectorX<T>& v,
                CouplerConstraintsDataPool<T>* coupler_data) const;

  /* Add the gradient contribution of this constraint, ∇ℓ = −Jᵀ⋅γ, to the
  overall gradient.

  TODO(amcastro-tri): factor out this method into a
  GeneralizedVelocitiesConstraintsPool parent class, along with other common
  functionality to all constraint pools on generalized velocities. */
  void AccumulateGradient(const IcfData<T>& data, VectorX<T>* gradient) const;

  /* Add the Hessian contribution of this constraint to the overall Hessian. */
  void AccumulateHessian(
      const IcfData<T>& data,
      contact_solvers::internal::BlockSparseSymmetricMatrixT<T>* hessian) const;

  /* Compute the first and second derivatives of ℓ(α) = ℓ(v + αw) at α = 0. Used
  for exact line search. */
  void ProjectAlongLine(const CouplerConstraintsDataPool<T>& coupler_data,
                        const VectorX<T>& w, T* dcost, T* d2cost) const;

  /* Total number of constraints. */
  int num_constraints() const { return constraint_to_clique_.size(); }

  /* Return a reference to the parent model. */
  const IcfModel<T>& model() const { return *model_; }

 private:
  const IcfModel<T>* model_{nullptr};  // The parent model.

  // Clique for the k-th constraint. Of size num_constraints().
  std::vector<int> constraint_to_clique_;
  std::vector<std::pair<int, int>> dofs_;  // (i, j) pair of coupled dofs.
  std::vector<T> gear_ratio_;

  // Regularization and bias per constraint.
  std::vector<T> v_hat_;
  std::vector<T> R_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
