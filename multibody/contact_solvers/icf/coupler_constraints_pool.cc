// NOLINTNEXTLINE(build/include): prevent complaint re patch_constraints_pool.h

#include <numeric>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"
#include "drake/multibody/contact_solvers/icf/icf.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {

template <typename T>
void IcfModel<T>::CouplerConstraintsPool::Clear() {
  constraint_to_clique_.clear();
  dofs_.clear();
  gear_ratio_.clear();
  v_hat_.clear();
  R_.clear();
}

template <typename T>
void IcfModel<T>::CouplerConstraintsPool::Resize(const int num_constraints) {
  constraint_to_clique_.resize(num_constraints);
  dofs_.resize(num_constraints);
  gear_ratio_.resize(num_constraints);
  v_hat_.resize(num_constraints);
  R_.resize(num_constraints);
}

template <typename T>
void IcfModel<T>::CouplerConstraintsPool::Set(int index, int clique, int i,
                                              int j, const T& qi, const T& qj,
                                              T gear_ratio, T offset) {
  DRAKE_ASSERT(index >= 0 && index < num_constraints());
  DRAKE_ASSERT(i >= 0 && i < model().clique_size(clique));
  DRAKE_ASSERT(j >= 0 && j < model().clique_size(clique));

  constraint_to_clique_[index] = clique;
  dofs_[index] = std::make_pair(i, j);
  gear_ratio_[index] = gear_ratio;

  const double beta = 0.1;
  const double eps = beta * beta / (4 * M_PI * M_PI) / (1 + beta / M_PI);

  const T g0 = qi - gear_ratio * qj - offset;

  // N.B. We have v̂ = −g₀ / (δt (1 + β/π)), but we only want to use the time
  // step δt = model().time_step() at the time of solving, in case the δt
  // changes between when the constraint is set and when the problem is solved.
  // Thus we store v̂ = −g₀ / (1 + β/π) and scale by 1/δt in CalcData().
  v_hat_[index] = -g0 / (1.0 + beta / M_PI);

  const auto w_clique = model().clique_delassus(clique);
  // Approximation of W = Jᵀ⋅M⁻¹⋅J, with
  //  J = [0 ... 1 ... -ρ ... 0]
  //             ↑      ↑
  //             i      j
  const T w = w_clique(i) + gear_ratio * gear_ratio * w_clique(j);

  R_[index] = eps * w;
}

template <typename T>
void IcfModel<T>::CouplerConstraintsPool::CalcData(
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
    const T& v_hat = v_hat_[k] / model().time_step();
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
void IcfModel<T>::CouplerConstraintsPool::AccumulateGradient(
    const IcfData<T>& data, VectorX<T>* gradient) const {
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
void IcfModel<T>::CouplerConstraintsPool::AccumulateHessian(
    const IcfData<T>& data,
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
void IcfModel<T>::CouplerConstraintsPool::ProjectAlongLine(
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

}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::icf::IcfModel<
    double>::CouplerConstraintsPool;
template class ::drake::multibody::contact_solvers::icf::IcfModel<
    drake::AutoDiffXd>::CouplerConstraintsPool;
