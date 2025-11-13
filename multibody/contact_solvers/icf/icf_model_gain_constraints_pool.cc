#include <algorithm>

#include "drake/multibody/contact_solvers/icf/icf_model.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T>
using BlockSparseSymmetricMatrixT =
    contact_solvers::internal::BlockSparseSymmetricMatrixT<T>;

template <typename T>
void IcfModel<T>::GainConstraintsPool::Clear() {
  clique_.clear();
  constraint_sizes_.clear();
  K_.Clear();
  b_.Clear();
  le_.Clear();
  ue_.Clear();
}

template <typename T>
void IcfModel<T>::GainConstraintsPool::Resize(std::span<const int> sizes) {
  clique_.resize(sizes.size());
  constraint_sizes_.resize(sizes.size());
  const int num_elements = ssize(sizes);
  K_.Resize(num_elements, sizes, {});
  b_.Resize(num_elements, sizes, {});
  le_.Resize(num_elements, sizes, {});
  ue_.Resize(num_elements, sizes, {});
}

template <typename T>
void IcfModel<T>::GainConstraintsPool::Set(const int index, int clique,
                                           const VectorX<T>& K,
                                           const VectorX<T>& b,
                                           const VectorX<T>& e) {
  DRAKE_ASSERT(index >= 0 && index < num_constraints());
  const int nv = model().clique_size(clique);
  DRAKE_DEMAND(K.size() == nv);
  DRAKE_DEMAND(b.size() == nv);
  DRAKE_DEMAND(e.size() == nv);
  clique_[index] = clique;
  constraint_sizes_[index] = nv;
  K_[index] = K;
  b_[index] = b;
  le_[index] = -e;
  ue_[index] = e;
}

template <typename T>
void IcfModel<T>::GainConstraintsPool::CalcData(
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
void IcfModel<T>::GainConstraintsPool::AccumulateGradient(
    const IcfData<T>& data, VectorX<T>* gradient) const {
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
void IcfModel<T>::GainConstraintsPool::AccumulateHessian(
    const IcfData<T>& data, BlockSparseSymmetricMatrixT<T>* hessian) const {
  const GainConstraintsDataPool<T>& gain_data =
      data.cache().gain_constraints_data;

  for (int k = 0; k < num_constraints(); ++k) {
    const int c = clique_[k];
    ConstMatrixXView Gk = gain_data.G(k);
    hessian->AddToBlock(c, c, Gk);
  }
}

template <typename T>
void IcfModel<T>::GainConstraintsPool::ProjectAlongLine(
    const GainConstraintsDataPool<T>& gain_data, const VectorX<T>& w,
    VectorX<T>* v_sized_scratch, T* dcost, T* d2cost) const {
  const int nv = model().num_velocities();
  DRAKE_ASSERT(v_sized_scratch != nullptr);
  DRAKE_ASSERT(v_sized_scratch->size() == nv);

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

template <typename T>
T IcfModel<T>::GainConstraintsPool::Clamp(int k,
                                          const Eigen::Ref<const VectorX<T>>& v,
                                          EigenPtr<VectorX<T>> gamma,
                                          EigenPtr<MatrixX<T>> G) const {
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

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::icf::internal::IcfModel<
    double>::GainConstraintsPool;
template class ::drake::multibody::contact_solvers::icf::internal::IcfModel<
    drake::AutoDiffXd>::GainConstraintsPool;
