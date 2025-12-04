#include "drake/multibody/contact_solvers/icf/limit_constraints_pool.h"

#include <limits>
#include <vector>

#include "drake/multibody/contact_solvers/icf/icf_model.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using contact_solvers::internal::BlockSparseSymmetricMatrix;
using Eigen::VectorBlock;

template <typename T>
LimitConstraintsPool<T>::LimitConstraintsPool(const IcfModel<T>* parent_model)
    : model_(parent_model) {
  DRAKE_ASSERT(parent_model != nullptr);
}

template <typename T>
LimitConstraintsPool<T>::~LimitConstraintsPool() = default;

template <typename T>
void LimitConstraintsPool<T>::Resize(
    std::span<const int> sizes, std::span<const int> constraint_to_clique) {
  DRAKE_DEMAND(sizes.size() == constraint_to_clique.size());
  const int num_elements = ssize(sizes);
  ql_.Resize(num_elements, sizes);
  qu_.Resize(num_elements, sizes);
  q0_.Resize(num_elements, sizes);
  gl_hat_.Resize(num_elements, sizes);
  gu_hat_.Resize(num_elements, sizes);
  R_.Resize(num_elements, sizes);
  constraint_size_.assign(sizes.begin(), sizes.end());
  clique_.assign(constraint_to_clique.begin(), constraint_to_clique.end());

  // All constraints are disabled (e.g., infinite bounds) by default. This
  // allows us to add a limit constraint on only one DoF in a multi-DoF
  // clique, for example.
  // TODO(vincekurtz): consider a setConstant method in EigenPool.
  for (int k = 0; k < num_constraints(); ++k) {
    ql_[k].setConstant(-std::numeric_limits<double>::infinity());
    qu_[k].setConstant(std::numeric_limits<double>::infinity());
    q0_[k].setConstant(0.0);
    R_[k].setConstant(std::numeric_limits<double>::infinity());
    gl_hat_[k].setConstant(-std::numeric_limits<double>::infinity());
    gu_hat_[k].setConstant(-std::numeric_limits<double>::infinity());
  }
}

template <typename T>
void LimitConstraintsPool<T>::Set(int index, int clique, int dof, const T& q0,
                                  const T& ql, const T& qu) {
  ql_[index](dof) = ql;
  qu_[index](dof) = qu;
  q0_[index](dof) = q0;

  // Near-rigid regularization [Castro et al., 2022].
  const double beta = 0.1;
  const double eps = beta * beta / (4 * M_PI * M_PI) * (1 + beta / M_PI);

  // Approximation of W = Jᵀ⋅M⁻¹⋅J = M⁻¹ ≈ diag(M)⁻¹.
  ConstVectorXView w_clique = model().clique_diagonal_mass_inverse(clique);
  R_[index](dof) = eps * w_clique(dof);

  // Eventually we will use
  //  v̂ₗ = (qₗ − q₀) / (δt (1 + β))
  //  v̂ᵤ = (q₀ − qᵤ) / (δt (1 + β))
  // However, since model.time_step() may change between now and when we
  // actually solve the problem, we precompute
  //  ĝₗ = v̂ₗ⋅δt = (qₗ − q₀) / (1 + β)
  //  ĝᵤ = v̂ᵤ⋅δt = (q₀ − qᵤ) / (1 + β)
  // so that we can compute v̂ = ĝ/δt from the current time step in calls to
  // CalcData().
  gl_hat_[index](dof) = (ql - q0) / (1.0 + beta);
  gu_hat_[index](dof) = (q0 - qu) / (1.0 + beta);
}

template <typename T>
void LimitConstraintsPool<T>::CalcData(
    const VectorX<T>& v, LimitConstraintsDataPool<T>* limit_data) const {
  DRAKE_ASSERT(limit_data != nullptr);

  const T& dt = model().time_step();
  T& cost = limit_data->mutable_cost();
  cost = 0;
  for (int k = 0; k < num_constraints(); ++k) {
    const int c = clique_[k];
    const int nv = model().clique_size(c);
    VectorBlock<const VectorX<T>> vk = model().clique_segment(c, v);
    VectorXView gamma_lower = limit_data->mutable_gamma_lower(k);
    VectorXView gamma_upper = limit_data->mutable_gamma_upper(k);
    VectorXView G_lower = limit_data->mutable_G_lower(k);
    VectorXView G_upper = limit_data->mutable_G_upper(k);
    for (int i = 0; i < nv; ++i) {
      // i-th lower limit for constraint k (clique c).
      const T vl = vk(i);
      cost += CalcLimitData(gl_hat_[k](i) / dt, R_[k](i), vl, &gamma_lower(i),
                            &G_lower(i));

      // i-th upper limit for constraint k (clique c).
      const T vu = -vk(i);
      cost += CalcLimitData(gu_hat_[k](i) / dt, R_[k](i), vu, &gamma_upper(i),
                            &G_upper(i));
    }
  }
}

template <typename T>
void LimitConstraintsPool<T>::AccumulateGradient(const IcfData<T>& data,
                                                 VectorX<T>* gradient) const {
  const LimitConstraintsDataPool<T>& limit_data = data.limit_constraints_data();

  for (int k = 0; k < num_constraints(); ++k) {
    const int c = clique_[k];
    VectorBlock<VectorX<T>> gradient_c =
        model().mutable_clique_segment(c, gradient);
    ConstVectorXView gamma_lower = limit_data.gamma_lower(k);
    ConstVectorXView gamma_upper = limit_data.gamma_upper(k);

    // For this constraint vc = [v; -v], i.e. J = [1; -1]^ᵀ.
    // Therefore ∇ℓ = γᵤ − γₗ:
    gradient_c += gamma_upper;
    gradient_c -= gamma_lower;
  }
}

template <typename T>
void LimitConstraintsPool<T>::AccumulateHessian(
    const IcfData<T>& data,
    BlockSparseSymmetricMatrix<MatrixX<T>>* hessian) const {
  const LimitConstraintsDataPool<T>& limit_data = data.limit_constraints_data();

  for (int k = 0; k < num_constraints(); ++k) {
    const int c = clique_[k];

    const MatrixX<T> G_lower = limit_data.G_lower(k).asDiagonal();
    const MatrixX<T> G_upper = limit_data.G_upper(k).asDiagonal();
    hessian->AddToBlock(c, c, G_lower);
    hessian->AddToBlock(c, c, G_upper);
  }
}

template <typename T>
void LimitConstraintsPool<T>::CalcCostAlongLine(
    const LimitConstraintsDataPool<T>& limit_data, const VectorX<T>& w,
    EigenPool<VectorX<T>>* Gw_scratch, T* dcost, T* d2cost) const {
  DRAKE_ASSERT(Gw_scratch != nullptr);
  EigenPool<VectorX<T>>& Gw_pool = *Gw_scratch;

  (*dcost) = 0.0;
  (*d2cost) = 0.0;
  for (int k = 0; k < num_constraints(); ++k) {
    const int c = clique_[k];
    VectorBlock<const VectorX<T>> w_c = model().clique_segment(c, w);
    Gw_pool.Resize(1, model().clique_size(c), 1);
    VectorXView G_times_w = Gw_pool[0];

    // Lower limit contribution.
    ConstVectorXView gamma_lower = limit_data.gamma_lower(k);
    ConstVectorXView G_lower = limit_data.G_lower(k);
    G_times_w = G_lower.asDiagonal() * w_c;
    (*dcost) -= w_c.dot(gamma_lower);
    (*d2cost) += w_c.dot(G_times_w);

    // Upper limit contribution.
    ConstVectorXView gamma_upper = limit_data.gamma_upper(k);
    ConstVectorXView G_upper = limit_data.G_upper(k);
    G_times_w = G_upper.asDiagonal() * w_c;
    (*dcost) += w_c.dot(gamma_upper);
    (*d2cost) += w_c.dot(G_times_w);
  }
}

template <typename T>
T LimitConstraintsPool<T>::CalcLimitData(const T& v_hat, const T& R, const T& v,
                                         T* gamma, T* G) const {
  T cost = 0;
  (*gamma) = 0;
  (*G) = 0;

  if (v < v_hat) {
    const T dv = (v - v_hat);
    cost += 0.5 * dv * dv / R;
    (*gamma) = -dv / R;
    (*G) = 1.0 / R;
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
        LimitConstraintsPool);
