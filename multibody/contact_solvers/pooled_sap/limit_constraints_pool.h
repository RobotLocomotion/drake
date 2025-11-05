#pragma once

#ifndef DRAKE_POOLED_SAP_INCLUDED
#error Do not include this file. Use "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h" // NOLINT
#endif

#include <algorithm>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/pooled_sap/eigen_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/limit_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

/**
 * A pool of limit constraints, qu ≥ q ≥ ql.
 */
template <typename T>
class PooledSapModel<T>::LimitConstraintsPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LimitConstraintsPool);

  // Constructor for an empty pool.
  LimitConstraintsPool(const PooledSapModel<T>* parent_model)
      : model_(parent_model) {
    DRAKE_ASSERT(parent_model != nullptr);
  }

  // Reset, zeroing out the constraints while keeping memory allocated.
  void Clear();

  // Re-allocate memory as needed, setting all constraints as infinite by
  // default so they are disabled to start with.
  void Resize(const std::vector<int>& constrained_clique_sizes,
              const std::vector<int>& constraint_to_clique);

  /**
   * Set the limit constraint parameters for the given clique and DoF.
   *
   * @param index The index of this limit constraint in the pool.
   * @param clique The clique to which this limit constraint applies.
   * @param dof The degree of freedom within the clique to which this limit
   * constraint applies.
   * @param q0 The current configuration value for this DoF.
   * @param ql The lower limit for this DoF.
   * @param qu The upper limit for this DoF.
   */
  void Set(int index, int clique, int dof, const T& q0, const T& ql,
           const T& qu);

  // Update the time step, leaving all other parameters unchanged.
  void UpdateTimeStep(const T& old_dt, const T& new_dt);

  // Upper and lower limits
  T& lower_limit(int k, int dof) { return ql_[k](dof); }
  T& upper_limit(int k, int dof) { return qu_[k](dof); }

  // Initial configuration q0
  T& configuration(int k, int dof) { return q0_[k](dof); }

  // Near-rigid regularization
  T& regularization(int k, int dof) { return R_[k](dof); }
  const T regularization(int k, int dof) const { return R_[k](dof); }

  // Upper and lower bound velocity targets
  T& vl_hat(int k, int dof) { return vl_hat_[k](dof); }
  T& vu_hat(int k, int dof) { return vu_hat_[k](dof); }
  const T vl_hat(int k, int dof) const { return vl_hat_[k](dof); }
  const T vu_hat(int k, int dof) const { return vu_hat_[k](dof); }

  // Compute problem data for the given generalized velocities `v`.
  void CalcData(const VectorX<T>& v,
                LimitConstraintsDataPool<T>* limit_data) const;

  // Add the gradient contribution of this constraint, ∇ℓ = −γ, to the overall
  // gradient.
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
  void ProjectAlongLine(const LimitConstraintsDataPool<T>& limit_data,
                        const VectorX<T>& w, VectorX<T>* v_sized_scratch,
                        T* dcost, T* d2cost) const;

  // Total number of limit constraints.
  int num_constraints() const { return constraint_to_clique_.size(); }

  // Number of velocities for each limit constraint.
  const std::vector<int>& constraint_sizes() const { return constraint_sizes_; }

  // Return a reference to the parent model.
  const PooledSapModel<T>& model() const { return *model_; }

 private:
  // Compute cost, gradient, and Hessian contribution for a single limit
  // constraint.
  T CalcLimitData(const T& v_hat, const T& R, const T& v, T* gamma, T* G) const;

  const PooledSapModel<T>* model_{nullptr};  // The parent model.

  // Clique for the k-th constraint. Of size num_constraints().
  std::vector<int> constraint_to_clique_;

  // Clique size for the k-th constraint. Of size num_constraints().
  std::vector<int> constraint_sizes_;

  EigenPool<VectorX<T>> ql_;
  EigenPool<VectorX<T>> qu_;
  EigenPool<VectorX<T>> q0_;
  EigenPool<VectorX<T>> vl_hat_;
  EigenPool<VectorX<T>> vu_hat_;
  EigenPool<VectorX<T>> R_;
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
