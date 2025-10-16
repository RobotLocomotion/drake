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
void PooledSapModel<T>::GainConstraintsPool::ResizeData(
    GainConstraintsDataPool<T>* gain_data) const {
  gain_data->Resize(constraint_sizes_);
}

template <typename T>
void PooledSapModel<T>::GainConstraintsPool::CalcData(
    const VectorX<T>& v, GainConstraintsDataPool<T>* gain_data) const {
  DRAKE_ASSERT(gain_data != nullptr);

  using VectorXView = typename EigenPool<VectorX<T>>::ElementView;
  using MatrixXView = typename EigenPool<MatrixX<T>>::ElementView;

  T& cost = gain_data->cost();
  cost = 0;
  for (int k = 0; k < num_constraints(); ++k) {
    const int c = clique_[k];
    auto vk = model().clique_segment(c, v);
    VectorXView gk = gain_data->gamma(k);
    MatrixXView Gk = gain_data->G(k);
    cost += Clamp(k, vk, &gk, &Gk);
  }
}

template <typename T>
void PooledSapModel<T>::GainConstraintsPool::AccumulateGradient(
    const PooledSapData<T>& data, VectorX<T>* gradient) const {
  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstElementView;

  const GainConstraintsDataPool<T>& gain_data =
      data.cache().gain_constraints_data;

  for (int k = 0; k < num_constraints(); ++k) {
    const int c = clique_[k];
    auto gradient_c = model().clique_segment(c, gradient);
    ConstVectorXView gk = gain_data.gamma(k);
    gradient_c -= gk;
  }
}

template <typename T>
void PooledSapModel<T>::GainConstraintsPool::AccumulateHessian(
    const PooledSapData<T>& data,
    internal::BlockSparseSymmetricMatrixT<T>* hessian) const {
  using ConstMatrixXView = typename EigenPool<MatrixX<T>>::ConstElementView;

  const GainConstraintsDataPool<T>& gain_data =
      data.cache().gain_constraints_data;

  for (int k = 0; k < num_constraints(); ++k) {
    const int c = clique_[k];
    ConstMatrixXView Gk = gain_data.G(k);
    hessian->AddToBlock(c, c, Gk);
  }
}

template <typename T>
void PooledSapModel<T>::GainConstraintsPool::ProjectAlongLine(
    const GainConstraintsDataPool<T>& gain_data, const VectorX<T>& w,
    VectorX<T>* v_sized_scratch, T* dcost, T* d2cost) const {
  const int nv = model().num_velocities();
  DRAKE_ASSERT(v_sized_scratch != nullptr);
  DRAKE_ASSERT(v_sized_scratch->size() == nv);

  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstElementView;
  using ConstMatrixXView = typename EigenPool<MatrixX<T>>::ConstElementView;

  *dcost = 0.0;
  *d2cost = 0.0;
  for (int k = 0; k < num_constraints(); ++k) {
    const int c = clique_[k];
    auto w_c = model().clique_segment(c, w);
    ConstVectorXView gk = gain_data.gamma(k);
    ConstMatrixXView Gk = gain_data.G(k);

    auto G_times_w = model().clique_segment(c, v_sized_scratch);
    G_times_w.noalias() = Gk.diagonal().asDiagonal() * w_c;

    (*dcost) -= w_c.dot(gk);
    (*d2cost) += w_c.dot(G_times_w);
  }
}

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::pooled_sap::PooledSapModel<
    double>::GainConstraintsPool;
template class ::drake::multibody::contact_solvers::pooled_sap::PooledSapModel<
    drake::AutoDiffXd>::GainConstraintsPool;
