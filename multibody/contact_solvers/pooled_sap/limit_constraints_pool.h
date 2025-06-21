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
#include "drake/multibody/contact_solvers/pooled_sap/sap_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

/* A pool of gain constraints organized by cliques.

 A clique can have a gain constraint that models generalized forces on the
 clique according to:

  τ = clamp(−K⋅v + b, e)

 where K is a positive semi-definite diagonal gain matrix, b is a bias term and
 e is an effort limit. Generalized impulses for that clique are thus γ = δt⋅τ.
 */
template <typename T>
class PooledSapModel<T>::LimitConstraintsPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LimitConstraintsPool);

  int num_constraints() const { return constraint_to_clique_.size(); }

  // Total number of constriant equations. Each constraint has a clique c and
  // size model().clique_size(c).
  int num_constraint_equations() const { return q0_.size(); }

  /* Constructor for an empty pool. */
  LimitConstraintsPool(const PooledSapModel<T>* parent_model)
      : model_(parent_model) {
    DRAKE_ASSERT(parent_model != nullptr);
  }

  /* Returns reference to the parent model. */
  const PooledSapModel<T>& model() const { return *model_; }

  void Clear() {
    clique_to_constraint_.clear();
    constraint_to_clique_.clear();
    constraint_sizes_.clear();
    ql_.Clear();
    qu_.Clear();
    q0_.Clear();
    vl_hat_.Clear();
    vu_hat_.Clear();
    R_.Clear();
  }

  void Reset() {
    Clear();

    const int nc = model().num_cliques();
    DRAKE_DEMAND(nc > 0);
    const std::vector<int>& clique_sizes = model().clique_sizes();

    clique_to_constraint_.resize(nc);
    std::fill(clique_to_constraint_.begin(), clique_to_constraint_.end(), -1);

    // At most all cliques involved.
    constraint_to_clique_.reserve(nc);
    constraint_sizes_.reserve(nc);

    ql_.Reserve(clique_sizes);
    qu_.Reserve(clique_sizes);
    q0_.Reserve(clique_sizes);
    vl_hat_.Reserve(clique_sizes);
    vu_hat_.Reserve(clique_sizes);
    R_.Reserve(clique_sizes);
  }

  /* If not yet added, adds a limit constraint for the given `clique`. */
  int MaybeAdd(int clique) {
    const int index = num_constraints();
    if (clique_to_constraint_[clique] < 0) {
      const int nv = model().clique_size(clique);
      clique_to_constraint_[clique] = index;
      constraint_to_clique_.push_back(clique);
      constraint_sizes_.push_back(nv);  // Upper + lower per clique.
      ql_.Add(nv, 1).setConstant(-std::numeric_limits<double>::infinity());
      qu_.Add(nv, 1).setConstant(std::numeric_limits<double>::infinity());
      q0_.Add(nv, 1).setConstant(0.0);
      R_.Add(nv, 1).setConstant(std::numeric_limits<double>::infinity());

      // For v_hat =  infinity, the constraint can never be active, and thus
      // gamma = 0, unless someone sets another v_hat.
      vl_hat_.Add(nv, 1).setConstant(-std::numeric_limits<double>::infinity());
      vu_hat_.Add(nv, 1).setConstant(-std::numeric_limits<double>::infinity());
      return index;
    } else {
      return index - 1;
    }
  }

  int Add(int clique, int dof, const T& q0, const T& ql, const T& qu) {
    const int k = MaybeAdd(clique);
    lower_limit(k, dof) = ql;
    upper_limit(k, dof) = qu;
    configuration(k, dof) = q0;

    const T dt = model().time_step();
    const double beta = 0.1;
    const double eps = beta * beta / (4 * M_PI * M_PI) * (1 + beta / M_PI);

    const auto w_clique = model().get_clique_delassus(clique);
    regularization(k, dof) = eps * w_clique(dof);
    vl_hat(k, dof) = (ql - q0) / (dt * (1.0 + beta));
    vu_hat(k, dof) = (q0 - qu) / (dt * (1.0 + beta));

    return k;
  }

  T& lower_limit(int k, int dof) { return ql_[k](dof); }
  T& upper_limit(int k, int dof) { return qu_[k](dof); }
  T& configuration(int k, int dof) { return q0_[k](dof); }
  T& regularization(int k, int dof) { return R_[k](dof); }
  T& vl_hat(int k, int dof) { return vl_hat_[k](dof); }
  T& vu_hat(int k, int dof) { return vu_hat_[k](dof); }
  const T regularization(int k, int dof) const { return R_[k](dof); }
  const T vl_hat(int k, int dof) const { return vl_hat_[k](dof); }
  const T vu_hat(int k, int dof) const { return vu_hat_[k](dof); }

  void ResizeData(LimitConstraintsDataPool<T>* limit_data) const;

  void CalcData(const VectorX<T>& v,
                LimitConstraintsDataPool<T>* limit_data) const;

  // TODO(amcastro-tri): factor out this method into a
  // GeneralizedVelocitiesConstraintsPool parent class, along with other common
  // functionality to all constraint pools on generalized velocities.
  void AccumulateGradient(const SapData<T>& data, VectorX<T>* gradient) const;

  void AccumulateHessian(
      const SapData<T>& data,
      internal::BlockSparseSymmetricMatrixT<T>* hessian) const;

  void ProjectAlongLine(const LimitConstraintsDataPool<T>& limit_data,
                        const VectorX<T>& w, VectorX<T>* v_sized_scratch,
                        T* dcost, T* d2cost) const;

 private:
  T CalcLimitData(const T& v_hat, const T& R, const T& v, T* gamma, T* G) const;

  const PooledSapModel<T>* model_{nullptr};  // The parent model.

  // Constraint index or -1. Of size num_cliques.
  std::vector<int> clique_to_constraint_;

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
