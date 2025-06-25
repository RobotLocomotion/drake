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
#include "drake/multibody/contact_solvers/pooled_sap/coupler_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/eigen_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"
#include "drake/multibody/contact_solvers/pooled_sap/sap_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

/* A pool of limit constraints organized by cliques. */
template <typename T>
class PooledSapModel<T>::CouplerConstraintsPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CouplerConstraintsPool);

  int num_constraints() const { return constraint_to_clique_.size(); }

  // Total number of constraint equations. Each constraint has a clique c and
  // size model().clique_size(c).
  int num_constraint_equations() const { return constraint_to_clique_.size(); }

  /* Constructor for an empty pool. */
  CouplerConstraintsPool(const PooledSapModel<T>* parent_model)
      : model_(parent_model) {
    DRAKE_ASSERT(parent_model != nullptr);
  }

  /* Returns reference to the parent model. */
  const PooledSapModel<T>& model() const { return *model_; }

  void Clear() {
    constraint_to_clique_.clear();
    dofs_.clear();
    gear_ratio_.clear();
    v_hat_.clear();
    R_.clear();
  }

  void Reset() {
    Clear();
    // TODO(amcastro-tri): Reserve capacity for all couplers in the model.
  }

  /* Enforces the constraint g = qᵢ − ρqⱼ−Δq = 0, between the i-th and j-th DoFs
   of `clique`. */
  int Add(int clique, int i, int j, const T& qi, const T& qj, T gear_ratio,
          T offset) {
    const int k = constraint_to_clique_.size();
    constraint_to_clique_.push_back(clique);
    dofs_.emplace_back(i, j);
    gear_ratio_.push_back(gear_ratio);

    const T dt = model().time_step();
    const double beta = 0.1;
    const double eps = beta * beta / (4 * M_PI * M_PI) / (1 + beta / M_PI);

    const T g0 = qi - gear_ratio * qj - offset;
    v_hat_.push_back(-g0 / (dt * (1.0 + beta / M_PI)));

    const auto w_clique = model().get_clique_delassus(clique);
    // Approximation of W = Jᵀ⋅M⁻¹⋅J, with
    //  J = [0 ... 1 ... -ρ ... 0]
    //             ↑      ↑
    //             i      j
    const T w = w_clique(i) + gear_ratio * gear_ratio * w_clique(j);

    R_.push_back(eps * w);

    return k;
  }

  T& regularization(int k) { return R_[k]; }
  T& v_hat(int k) { return v_hat_[k]; }

  void ResizeData(CouplerConstraintsDataPool<T>* coupler_data) const;

  void CalcData(const VectorX<T>& v,
                CouplerConstraintsDataPool<T>* coupler_data) const;

  // TODO(amcastro-tri): factor out this method into a
  // GeneralizedVelocitiesConstraintsPool parent class, along with other common
  // functionality to all constraint pools on generalized velocities.
  void AccumulateGradient(const SapData<T>& data, VectorX<T>* gradient) const;

  void AccumulateHessian(
      const SapData<T>& data,
      internal::BlockSparseSymmetricMatrixT<T>* hessian) const;

  void ProjectAlongLine(const CouplerConstraintsDataPool<T>& coupler_data,
                        const VectorX<T>& w, T* dcost, T* d2cost) const;

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
