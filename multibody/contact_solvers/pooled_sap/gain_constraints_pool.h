#pragma once

#ifndef DRAKE_POOLED_SAP_INCLUDED
#error Do not include this file. Use "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h" // NOLINT
#endif

#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/pooled_sap/eigen_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/gain_constraints_data_pool.h"
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
class PooledSapModel<T>::GainConstraintsPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GainConstraintsPool);

  int num_constraints() const { return clique_.size(); }

  // Total number of constriant equations. Each constraint has a clique c and
  // size model().clique_size(c).
  int num_constraint_equations() const { return b_.size(); }

  /* Constructor for an empty pool. */
  GainConstraintsPool(const PooledSapModel<T>* parent_model)
      : model_(parent_model) {
    DRAKE_ASSERT(parent_model != nullptr);
  }

  /* Returns reference to the parent model. */
  const PooledSapModel<T>& model() const { return *model_; }

  void Clear() {
    clique_.clear();
    constraint_sizes_.clear();
    K_.Clear();
    b_.Clear();
    le_.Clear();
    ue_.Clear();
  }

  void Reset() { Clear(); }

  void Resize(const std::vector<int>& sizes) {
    clique_.resize(sizes.size());
    constraint_sizes_.resize(sizes.size());
    K_.Resize(sizes);
    b_.Resize(sizes);
    le_.Resize(sizes);
    ue_.Resize(sizes);
  }

  /* Adds gain constraint for `clique`.
   @param i The index of this gain constraint in the pool.
   @param clique The clique to which this gain constraint applies.
   @param K The diagonal entries of gain matrix K. They must be positive or
   zero.
   @param b The bias term.
   @param e The vector of effort limits for each DoF of the clique.
   @pre K, b, e are of size model().clique_size(clique). */
  void Add(const int i, int clique, const VectorX<T>& K, const VectorX<T>& b,
           const VectorX<T>& e) {
    const int nv = model().clique_size(clique);
    DRAKE_DEMAND(K.size() == nv);
    DRAKE_DEMAND(b.size() == nv);
    DRAKE_DEMAND(e.size() == nv);
    clique_[i] = clique;
    constraint_sizes_[i] = nv;
    K_[i] = K;
    b_[i] = b;
    le_[i] = -e;
    ue_[i] = e;
  }

  void ResizeData(GainConstraintsDataPool<T>* gain_data) const;

  void CalcData(const VectorX<T>& v,
                GainConstraintsDataPool<T>* gain_data) const;

  // TODO(amcastro-tri): factor out this method into a
  // GeneralizedVelocitiesConstraintsPool parent class, along with other common
  // functionality to all constraint pools on generalized velocities.
  void AccumulateGradient(const SapData<T>& data, VectorX<T>* gradient) const;

  void AccumulateHessian(
      const SapData<T>& data,
      internal::BlockSparseSymmetricMatrixT<T>* hessian) const;

  void ProjectAlongLine(const GainConstraintsDataPool<T>& gain_data,
                        const VectorX<T>& w, VectorX<T>* v_sized_scratch,
                        T* dcost, T* d2cost) const;

 private:
  /* Computes yᵢ = clamp(xᵢ). */
  T Clamp(int k, const Eigen::Ref<const VectorX<T>>& v,
          EigenPtr<VectorX<T>> gamma, EigenPtr<MatrixX<T>> G) const {
    const int n = v.size();
    DRAKE_ASSERT(gamma->size() == n);
    DRAKE_ASSERT(G->rows() == n);
    DRAKE_ASSERT(G->cols() == n);
    using std::max;
    using std::min;

    const T& dt = model().time_step();

    T cost = 0;
    for (int i = 0; i < n; ++i) {
      const T& ki = K_[k][i];
      const T& bi = b_[k][i];
      const T& lei = le_[k][i];
      const T& uei = ue_[k][i];
      const T& vi = v[i];
      T& gi = (*gamma)[i];
      T& Gi = (*G)(i, i);

      const T yi = -ki * vi + bi;

      if (yi < lei) {
        // Below lower limit.
        gi = dt * lei;
        Gi = 0.0;
        if (ki > 0) {
          cost += gi * (yi - 0.5 * lei) / ki;
        } else {
          cost -= gi * vi;  // Zero gain case.
        }
      } else if (yi > uei) {
        // Above upper limit.
        gi = dt * uei;
        Gi = 0.0;
        if (ki > 0) {
          cost += gi * (yi - 0.5 * uei) / ki;
        } else {
          cost -= gi * vi;  // Zero gain case.
        }
      } else {
        // Within limit.
        gi = dt * yi;
        Gi = dt * ki;
        if (ki > 0) {
          cost += 0.5 * yi * yi * dt / ki;
        } else {
          cost -= gi * vi;  // Zero gain case.
        }
      }
    }

    return cost;
  }

  const PooledSapModel<T>* model_{nullptr};  // The parent model.

  // We always add gain constraints per-clique.
  std::vector<int> clique_;            // Clique the k-th gain belongs to.
  std::vector<int> constraint_sizes_;  // Clique size for the k-th constraint.
  EigenPool<VectorX<T>> K_;
  EigenPool<VectorX<T>> b_;
  EigenPool<VectorX<T>> le_;  // Lower effort limit.
  EigenPool<VectorX<T>> ue_;  // Upper effort limit.
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
