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
void PooledSapModel<T>::CouplerConstraintsPool::ResizeData(
    CouplerConstraintsDataPool<T>* coupler_data) const {
  coupler_data->Resize(num_constraints());
}

template <typename T>
void PooledSapModel<T>::CouplerConstraintsPool::CalcData(
    const VectorX<T>& v, CouplerConstraintsDataPool<T>* coupler_data) const {
  DRAKE_ASSERT(coupler_data != nullptr);

  T& cost = coupler_data->cost();
  cost = 0;
  for (int k = 0; k < num_constraints(); ++k) {
    const int c = constraint_to_clique_[k];
    auto vk = model().clique_segment(c, v);
    const int i = dofs_[k].first;
    const int j = dofs_[k].second;
    const T& rho = gear_ratio_[k];
    const T& v_hat = v_hat_[k];
    const T& R = R_[k];

    const T vi = vk(i);
    const T vj = vk(j);
    const T vc = vi - rho * vj;  // Constraint velocity.

    const T gamma = -(vc - v_hat) / R;
    coupler_data->gamma(k) = gamma;
    cost += 0.5 * (v_hat - vc) * gamma;
  }
}

template <typename T>
void PooledSapModel<T>::CouplerConstraintsPool::AccumulateGradient(
    const PooledSapData<T>& data, VectorX<T>* gradient) const {
  const CouplerConstraintsDataPool<T>& coupler_data =
      data.cache().coupler_constraints_data;

  for (int k = 0; k < num_constraints(); ++k) {
    const int c = constraint_to_clique_[k];
    auto gradient_c = model().clique_segment(c, gradient);
    const int i = dofs_[k].first;
    const int j = dofs_[k].second;
    const T& rho = gear_ratio_[k];

    //  J = [0 ... 1 ... -ρ ... 0]
    //             ↑      ↑
    //             i      j
    // Thus: ∇ℓ = −Jᵀ⋅γ = [0 ... -γₖ ... ρ⋅γₖ ... 0]ᵀ.
    const T& gamma = coupler_data.gamma(k);
    gradient_c(i) -= gamma;
    gradient_c(j) += rho * gamma;
  }
}

template <typename T>
void PooledSapModel<T>::CouplerConstraintsPool::AccumulateHessian(
    const PooledSapData<T>& data,
    internal::BlockSparseSymmetricMatrixT<T>* hessian) const {
  unused(data);

  for (int k = 0; k < num_constraints(); ++k) {
    const int c = constraint_to_clique_[k];
    const int i = dofs_[k].first;
    const int j = dofs_[k].second;
    const T& rho = gear_ratio_[k];
    const T& R = R_[k];

    auto& Hc = hessian->diagonal_block(c);
    // clang-format off
    Hc(i, i) += 1.0 / R;  Hc(i, j) -= rho / R;
    Hc(j, i) -= rho / R;  Hc(j, j) += rho * rho / R;
    // clang-format on
  }
}

template <typename T>
void PooledSapModel<T>::CouplerConstraintsPool::ProjectAlongLine(
    const CouplerConstraintsDataPool<T>& coupler_data, const VectorX<T>& w,
    T* dcost, T* d2cost) const {
  *dcost = 0.0;
  *d2cost = 0.0;
  for (int k = 0; k < num_constraints(); ++k) {
    const int c = constraint_to_clique_[k];
    const int i = dofs_[k].first;
    const int j = dofs_[k].second;
    const T& rho = gear_ratio_[k];
    const T& R = R_[k];
    const T& gamma = coupler_data.gamma(k);
    auto w_c = model().clique_segment(c, w);

    const T wi = w_c(i);
    const T wj = w_c(j);
    const T vw = wi - rho * wj;  // "constraint velocity" evaluated on w.

    (*dcost) -= gamma * vw;
    (*d2cost) += vw * vw / R;
  }
}

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::pooled_sap::PooledSapModel<
    double>::CouplerConstraintsPool;
template class ::drake::multibody::contact_solvers::pooled_sap::PooledSapModel<
    drake::AutoDiffXd>::CouplerConstraintsPool;
