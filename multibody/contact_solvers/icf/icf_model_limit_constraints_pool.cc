#include <limits>
#include <vector>

#include "drake/common/unused.h"
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
void IcfModel<T>::LimitConstraintsPool::Clear() {
  constraint_to_clique_.clear();
  constraint_sizes_.clear();
  ql_.Clear();
  qu_.Clear();
  q0_.Clear();
  vl_hat_.Clear();
  vu_hat_.Clear();
  R_.Clear();
}

template <typename T>
void IcfModel<T>::LimitConstraintsPool::Resize(
    std::span<const int> constrained_clique_sizes,
    std::span<const int> constraint_to_clique) {
  DRAKE_DEMAND(constrained_clique_sizes.size() == constraint_to_clique.size());
  const int num_elements = ssize(constrained_clique_sizes);
  ql_.Resize(num_elements, constrained_clique_sizes, {});
  qu_.Resize(num_elements, constrained_clique_sizes, {});
  q0_.Resize(num_elements, constrained_clique_sizes, {});
  vl_hat_.Resize(num_elements, constrained_clique_sizes, {});
  vu_hat_.Resize(num_elements, constrained_clique_sizes, {});
  R_.Resize(num_elements, constrained_clique_sizes, {});
  constraint_sizes_.assign(constrained_clique_sizes.begin(),
                           constrained_clique_sizes.end());
  constraint_to_clique_.assign(constraint_to_clique.begin(),
                               constraint_to_clique.end());

  // All constraints are disabled (e.g., infinite bounds) by default. This
  // allows us to add a limit constraint on only one DoF in a multi-DoF
  // clique, for example.
  // TODO(vincekurtz): consider a setConstant method in EigenPool.
  for (int k = 0; k < num_constraints(); ++k) {
    ql_[k].setConstant(-std::numeric_limits<double>::infinity());
    qu_[k].setConstant(std::numeric_limits<double>::infinity());
    q0_[k].setConstant(0.0);
    R_[k].setConstant(std::numeric_limits<double>::infinity());
    vl_hat_[k].setConstant(-std::numeric_limits<double>::infinity());
    vu_hat_[k].setConstant(-std::numeric_limits<double>::infinity());
  }
}

template <typename T>
void IcfModel<T>::LimitConstraintsPool::Set(int index, int clique, int dof,
                                            const T& q0, const T& ql,
                                            const T& qu) {
  lower_limit(index, dof) = ql;
  upper_limit(index, dof) = qu;
  configuration(index, dof) = q0;

  const double beta = 0.1;
  const double eps = beta * beta / (4 * M_PI * M_PI) * (1 + beta / M_PI);

  const auto w_clique = model().clique_delassus(clique);
  regularization(index, dof) = eps * w_clique(dof);

  // Eventually we will use
  //  v̂ₗ = (qₗ − q₀) / (δt (1 + β))
  //  v̂ᵤ = (q₀ − qᵤ) / (δt (1 + β))
  // However, since model.time_step() may change between now and when we
  // actually solve the problem, we neglect the 1/δt factor for now, and
  // will scale v̂ by 1/δt in CalcData().
  vl_hat(index, dof) = (ql - q0) / (1.0 + beta);
  vu_hat(index, dof) = (q0 - qu) / (1.0 + beta);
}

template <typename T>
T IcfModel<T>::LimitConstraintsPool::CalcLimitData(const T& v_hat, const T& R,
                                                   const T& v, T* gamma,
                                                   T* G) const {
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
void IcfModel<T>::LimitConstraintsPool::CalcData(
    const VectorX<T>& v, LimitConstraintsDataPool<T>* limit_data) const {
  DRAKE_ASSERT(limit_data != nullptr);
  using VectorXView = typename EigenPool<VectorX<T>>::ElementView;
  using MatrixXView = typename EigenPool<MatrixX<T>>::ElementView;

  const T& dt = model().time_step();
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
      cost += CalcLimitData(vl_hat(k, i) / dt, regularization(k, i), vl,
                            &gamma_lower(i), &G_lower(i, i));

      // i-th upper limit for constraint k (clique c).
      const T vu = -vk(i);
      cost += CalcLimitData(vu_hat(k, i) / dt, regularization(k, i), vu,
                            &gamma_upper(i), &G_upper(i, i));
    }
  }
}

template <typename T>
void IcfModel<T>::LimitConstraintsPool::AccumulateGradient(
    const IcfData<T>& data, VectorX<T>* gradient) const {
  const LimitConstraintsDataPool<T>& limit_data =
      data.cache().limit_constraints_data;

  for (int k = 0; k < num_constraints(); ++k) {
    const int c = constraint_to_clique_[k];
    auto gradient_c = model().clique_segment(c, gradient);
    ConstVectorXView gamma_lower = limit_data.gamma_lower(k);
    ConstVectorXView gamma_upper = limit_data.gamma_upper(k);

    // For this constraint vc = [v; -v], i.e. J = [1; -1]^ᵀ.
    // Therefore ∇ℓ = γᵤ − γₗ:
    gradient_c += gamma_upper;
    gradient_c -= gamma_lower;
  }
}

template <typename T>
void IcfModel<T>::LimitConstraintsPool::AccumulateHessian(
    const IcfData<T>& data, BlockSparseSymmetricMatrixT<T>* hessian) const {
  const LimitConstraintsDataPool<T>& limit_data =
      data.cache().limit_constraints_data;

  for (int k = 0; k < num_constraints(); ++k) {
    const int c = constraint_to_clique_[k];
    hessian->AddToBlock(c, c, limit_data.G_lower(k));
    hessian->AddToBlock(c, c, limit_data.G_upper(k));
  }
}

template <typename T>
void IcfModel<T>::LimitConstraintsPool::ProjectAlongLine(
    const LimitConstraintsDataPool<T>& limit_data, const VectorX<T>& w,
    VectorX<T>* v_sized_scratch, T* dcost, T* d2cost) const {
  const int nv = model().num_velocities();
  DRAKE_ASSERT(v_sized_scratch != nullptr);
  DRAKE_ASSERT(v_sized_scratch->size() == nv);

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

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::icf::internal::IcfModel<
    double>::LimitConstraintsPool;
template class ::drake::multibody::contact_solvers::icf::internal::IcfModel<
    drake::AutoDiffXd>::LimitConstraintsPool;
