// NOLINTNEXTLINE(build/include): prevent complaint re patch_constraints_pool.h

#include <numeric>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/multibody/contact_solvers/pooled_sap/eigen_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

template <typename T>
void PooledSapModel<T>::LimitConstraintsPool::ResizeData(
    LimitConstraintsDataPool<T>* limit_data) const {
  limit_data->Resize(constraint_sizes_);
}

template <typename T>
T PooledSapModel<T>::LimitConstraintsPool::CalcLimitData(const T& v_hat,
                                                         const T& R, const T& v,
                                                         T* gamma, T* G) const {
  T cost = 0;
  *(gamma) = 0;
  *(G) = 0;

  if (v < v_hat) {
    const T dv = (v - v_hat);
    cost += 0.5 * dv * dv / R;
    (*gamma) = -dv / R;
    (*G) = 1.0 / R;
  }

  return cost;
}

template <typename T>
void PooledSapModel<T>::LimitConstraintsPool::CalcData(
    const VectorX<T>& v, LimitConstraintsDataPool<T>* limit_data) const {
  DRAKE_ASSERT(limit_data != nullptr);
  using VectorXView = typename EigenPool<VectorX<T>>::ElementView;
  using MatrixXView = typename EigenPool<MatrixX<T>>::ElementView;

  T& cost = limit_data->cost();
  cost = 0;
  for (int k = 0; k < num_constraints(); ++k) {
    const int c = constraint_to_clique_[k];
    const int nv = model().clique_size(c);
    auto vk = model().clique_segment(c, v);
    VectorXView gamma_lower = limit_data->gamma_lower(k);
    VectorXView gamma_upper = limit_data->gamma_upper(k);
    MatrixXView G_lower = limit_data->G_lower(k);
    MatrixXView G_upper = limit_data->G_upper(k);
    for (int i = 0; i < nv; ++i) {
      // i-th lower limit for constraint k (clique c).
      const T vl = vk(i);
      cost += CalcLimitData(vl_hat(k, i), regularization(k, i), vl,
                            &gamma_lower(i), &G_lower(i, i));

      // i-th upper limit for constraint k (clique c).
      const T vu = -vk(i);
      cost += CalcLimitData(vu_hat(k, i), regularization(k, i), vu,
                            &gamma_upper(i), &G_upper(i, i));
    }
  }
}

template <typename T>
void PooledSapModel<T>::LimitConstraintsPool::AccumulateGradient(
    const SapData<T>& data, VectorX<T>* gradient) const {
  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstElementView;

  const LimitConstraintsDataPool<T>& limit_data =
      data.cache().limit_constraints_data;

  for (int k = 0; k < num_constraints(); ++k) {
    const int c = constraint_to_clique_[k];
    auto gradient_c = model().clique_segment(c, gradient);
    ConstVectorXView gamma_lower = limit_data.gamma_lower(k);
    ConstVectorXView gamma_upper = limit_data.gamma_upper(k);

    // For this constraint vc = [v; -v], i.e. J = [1; -1]^ᵀ.
    // Therefore ∇ℓ =γᵤ − γₗ:
    gradient_c += gamma_upper;
    gradient_c -= gamma_lower;
  }
}

template <typename T>
void PooledSapModel<T>::LimitConstraintsPool::AccumulateHessian(
    const SapData<T>& data,
    internal::BlockSparseSymmetricMatrixT<T>* hessian) const {
  const LimitConstraintsDataPool<T>& limit_data =
      data.cache().limit_constraints_data;

  for (int k = 0; k < num_constraints(); ++k) {
    const int c = constraint_to_clique_[k];
    hessian->AddToBlock(c, c, limit_data.G_lower(k));
    hessian->AddToBlock(c, c, limit_data.G_upper(k));
  }
}

template <typename T>
void PooledSapModel<T>::LimitConstraintsPool::ProjectAlongLine(
    const LimitConstraintsDataPool<T>& limit_data, const VectorX<T>& w,
    VectorX<T>* v_sized_scratch, T* dcost, T* d2cost) const {
  const int nv = model().num_velocities();
  DRAKE_ASSERT(v_sized_scratch != nullptr);
  DRAKE_ASSERT(v_sized_scratch->size() == nv);

  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstElementView;
  using ConstMatrixXView = typename EigenPool<MatrixX<T>>::ConstElementView;

  *dcost = 0.0;
  *d2cost = 0.0;
  for (int k = 0; k < num_constraints(); ++k) {
    const int c = constraint_to_clique_[k];
    auto w_c = model().clique_segment(c, w);
    auto G_times_w = model().clique_segment(c, v_sized_scratch);

    // Lower limit contribution.
    ConstVectorXView gamma_lower = limit_data.gamma_lower(k);
    ConstMatrixXView G_lower = limit_data.G_lower(k);
    G_times_w.noalias() = G_lower.diagonal().asDiagonal() * w_c;
    (*dcost) -= w_c.dot(gamma_lower);
    (*d2cost) += w_c.dot(G_times_w);

    // Upper limit contribution.
    ConstVectorXView gamma_upper = limit_data.gamma_upper(k);
    ConstMatrixXView G_upper = limit_data.G_upper(k);
    G_times_w.noalias() = G_upper.diagonal().asDiagonal() * w_c;
    (*dcost) += w_c.dot(gamma_upper);
    (*d2cost) += w_c.dot(G_times_w);
  }
}

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::pooled_sap::PooledSapModel<
    double>::LimitConstraintsPool;
template class ::drake::multibody::contact_solvers::pooled_sap::PooledSapModel<
    drake::AutoDiffXd>::LimitConstraintsPool;
