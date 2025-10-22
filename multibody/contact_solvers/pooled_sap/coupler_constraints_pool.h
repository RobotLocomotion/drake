#pragma once

#ifndef DRAKE_POOLED_SAP_INCLUDED
#error Do not include this file. Use "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h" // NOLINT
#endif

#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/pooled_sap/coupler_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/eigen_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

/**
 * A pool of coupler constraints (qᵢ - ρqⱼ = Δq) linking generalized positions
 * (qᵢ, qⱼ) with gear ratio ρ and offset Δq.
 *
 * Coupler constraints only apply to joints within the same clique, so they do
 * not change the sparsity structure of the problem.
 * TODO(vincekurtz): consider relaxing this requirement.
 */
template <typename T>
class PooledSapModel<T>::CouplerConstraintsPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CouplerConstraintsPool);

  // Constructor for an empty pool.
  CouplerConstraintsPool(const PooledSapModel<T>* parent_model)
      : model_(parent_model) {
    DRAKE_ASSERT(parent_model != nullptr);
  }

  // Resets, zeroing out the constraints while keeping memory allocated.
  void Clear();

  // Resize the constraints pool to store the given number of constraints.
  void Resize(const int num_constraints);

  /**
   * Add a coupler constraint qᵢ − ρqⱼ−Δq = 0 between the i-th and j-th DoFs of
   * the given clique.
   *
   * @param index The index of the constraint within the pool,
   *               must be in [0, num_constraints()).
   * @param clique The clique index where the constraint is applied. Both DoFs
   *               must belong to the same clique.
   * @param i The index of the first DoF within `clique`.
   * @param j The index of the second DoF within `clique`.
   * @param qi The current position of the i-th DoF.
   * @param qj The current position of the j-th DoF.
   * @param gear_ratio The gear ratio ρ.
   * @param offset The offset Δq.
   *
   */
  void Add(int index, int clique, int i, int j, const T& qi, const T& qj,
           T gear_ratio, T offset);

  // Resize the data object (stores information that changes between iterations)
  // to match this constraints pool.
  void ResizeData(CouplerConstraintsDataPool<T>* coupler_data) const;

  // Compute problem data from the given generalized velocities v, and store in
  // the given data struct.
  void CalcData(const VectorX<T>& v,
                CouplerConstraintsDataPool<T>* coupler_data) const;

  // Add the gradient constribution of this constraint, ∇ℓ = −Jᵀ⋅γ, to the
  // overall gradient.
  // TODO(amcastro-tri): factor out this method into a
  // GeneralizedVelocitiesConstraintsPool parent class, along with other common
  // functionality to all constraint pools on generalized velocities.
  void AccumulateGradient(const PooledSapData<T>& data,
                          VectorX<T>* gradient) const;

  // Add the Hessian contribution of this constraint to the overall Hessian.
  void AccumulateHessian(
      const PooledSapData<T>& data,
      internal::BlockSparseSymmetricMatrixT<T>* hessian) const;

  // Compute the first and second derivatives of ℓ(α) = ℓ(v + αw) at α = 0. Used
  // for exact line search.
  void ProjectAlongLine(const CouplerConstraintsDataPool<T>& coupler_data,
                        const VectorX<T>& w, T* dcost, T* d2cost) const;

  // Total number of constraints.
  int num_constraints() const { return constraint_to_clique_.size(); }

  // Each constraint has exactly one equation associated with it.
  int num_constraint_equations() const { return constraint_to_clique_.size(); }

  // Return a reference to the parent model.
  const PooledSapModel<T>& model() const { return *model_; }

 private:
  const PooledSapModel<T>* model_{nullptr};  // The parent model.

  // Clique for the k-th constraint. Of size num_constraints().
  std::vector<int> constraint_to_clique_;
  std::vector<std::pair<int, int>> dofs_;  // (i, j) pair of coupled dofs.
  std::vector<T> gear_ratio_;

  // Regularization and bias per constraint.
  std::vector<T> v_hat_;
  std::vector<T> R_;
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
