#include "drake/multibody/contact_solvers/icf/gain_constraints_pool.h"

#include <algorithm>

#include "drake/multibody/contact_solvers/icf/icf_model.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using contact_solvers::internal::BlockSparseSymmetricMatrix;
using Eigen::VectorBlock;

template <typename T>
GainConstraintsPool<T>::GainConstraintsPool(const IcfModel<T>* parent_model)
    : model_(parent_model) {
  DRAKE_DEMAND(parent_model != nullptr);
}

template <typename T>
GainConstraintsPool<T>::~GainConstraintsPool() = default;

template <typename T>
void GainConstraintsPool<T>::Resize(std::span<const int> sizes) {
  clique_.resize(sizes.size());
  constraint_size_.resize(sizes.size());
  const int num_elements = ssize(sizes);
  K_.Resize(num_elements, sizes);
  b_.Resize(num_elements, sizes);
  le_.Resize(num_elements, sizes);
  ue_.Resize(num_elements, sizes);
}

// TODO(rpoyner-tri): here is a memory allocation to eventually remove. The
// call to Set is copying (allocation) for K and b and e when the caller's
// provided type is an Eigen block, but the Set function requires a VectorX
// explicitly, not a Ref.
template <typename T>
void GainConstraintsPool<T>::Set(int index, int clique, const VectorX<T>& K,
                                 const VectorX<T>& b, const VectorX<T>& e) {
  DRAKE_ASSERT(0 <= index && index < num_constraints());
  const int nv = model().clique_size(clique);
  DRAKE_ASSERT(K.size() == nv);
  DRAKE_ASSERT(b.size() == nv);
  DRAKE_ASSERT(e.size() == nv);
  clique_[index] = clique;
  constraint_size_[index] = nv;
  K_[index] = K;
  b_[index] = b;
  le_[index] = -e;
  ue_[index] = e;
}

template <typename T>
void GainConstraintsPool<T>::CalcData(
    const VectorX<T>& v, GainConstraintsDataPool<T>* gain_data) const {
  DRAKE_ASSERT(gain_data != nullptr);

  T& cost = gain_data->mutable_cost();
  cost = 0;
  for (int k = 0; k < num_constraints(); ++k) {
    const int c = clique_[k];
    VectorBlock<const VectorX<T>> vk = model().clique_segment(c, v);
    VectorXView gamma_k = gain_data->mutable_gamma(k);
    VectorXView Gk = gain_data->mutable_G(k);
    cost += Clamp(k, vk, &gamma_k, &Gk);
  }
}

template <typename T>
void GainConstraintsPool<T>::AccumulateGradient(const IcfData<T>& data,
                                                VectorX<T>* gradient) const {
  DRAKE_ASSERT(gradient != nullptr);

  const GainConstraintsDataPool<T>& gain_data = data.gain_constraints_data();
  for (int k = 0; k < num_constraints(); ++k) {
    const int c = clique_[k];
    VectorBlock<VectorX<T>> gradient_c =
        model().mutable_clique_segment(c, gradient);
    ConstVectorXView gamma_k = gain_data.gamma(k);
    gradient_c -= gamma_k;
  }
}

template <typename T>
void GainConstraintsPool<T>::AccumulateHessian(
    const IcfData<T>& data,
    BlockSparseSymmetricMatrix<MatrixX<T>>* hessian) const {
  DRAKE_ASSERT(hessian != nullptr);

  const GainConstraintsDataPool<T>& gain_data = data.gain_constraints_data();
  for (int k = 0; k < num_constraints(); ++k) {
    const int c = clique_[k];

    // TODO(vincekurtz): use data.scratch() to avoid allocating Gk here.
    const MatrixX<T> Gk = gain_data.G(k).asDiagonal();
    hessian->AddToBlock(c, c, Gk);
  }
}

template <typename T>
void GainConstraintsPool<T>::CalcCostAlongLine(
    const GainConstraintsDataPool<T>& gain_data, const VectorX<T>& w,
    EigenPool<VectorX<T>>* Gw_scratch, T* dcost, T* d2cost) const {
  DRAKE_ASSERT(Gw_scratch != nullptr);
  DRAKE_ASSERT(dcost != nullptr);
  DRAKE_ASSERT(d2cost != nullptr);
  EigenPool<VectorX<T>>& Gw_pool = *Gw_scratch;

  *dcost = 0.0;
  *d2cost = 0.0;
  for (int k = 0; k < num_constraints(); ++k) {
    const int c = clique_[k];
    VectorBlock<const VectorX<T>> w_c = model().clique_segment(c, w);
    Gw_pool.Resize(1, model().clique_size(c), 1);
    VectorXView G_times_w = Gw_pool[0];

    ConstVectorXView gamma_k = gain_data.gamma(k);
    ConstVectorXView Gk = gain_data.G(k);
    G_times_w = Gk.asDiagonal() * w_c;
    (*dcost) -= w_c.dot(gamma_k);
    (*d2cost) += w_c.dot(G_times_w);
  }
}

template <typename T>
T GainConstraintsPool<T>::Clamp(int k, const Eigen::Ref<const VectorX<T>>& v,
                                EigenPtr<VectorX<T>> gamma,
                                EigenPtr<VectorX<T>> G) const {
  const int nv = v.size();
  DRAKE_DEMAND(gamma->size() == nv);
  DRAKE_DEMAND(G->size() == nv);
  using std::max;
  using std::min;

  const T& dt = model().time_step();

  T cost = 0;
  for (int i = 0; i < nv; ++i) {
    const T& ki = K_[k][i];
    const T& bi = b_[k][i];
    const T& lei = le_[k][i];
    const T& uei = ue_[k][i];
    const T& vi = v[i];
    T& gamma_i = (*gamma)[i];
    T& Gi = (*G)[i];

    const T yi = -ki * vi + bi;

    if (yi < lei) {
      // Below lower limit.
      gamma_i = dt * lei;
      Gi = 0.0;
      if (ki > 0) {
        cost += gamma_i * (yi - 0.5 * lei) / ki;
      } else {
        cost -= gamma_i * vi;  // Zero gain case.
      }
    } else if (yi > uei) {
      // Above upper limit.
      gamma_i = dt * uei;
      Gi = 0.0;
      if (ki > 0) {
        cost += gamma_i * (yi - 0.5 * uei) / ki;
      } else {
        cost -= gamma_i * vi;  // Zero gain case.
      }
    } else {
      // Within limit.
      gamma_i = dt * yi;
      Gi = dt * ki;
      if (ki > 0) {
        cost += 0.5 * yi * yi * dt / ki;
      } else {
        cost -= gamma_i * vi;  // Zero gain case.
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

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        GainConstraintsPool);
