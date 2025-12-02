#include "drake/multibody/contact_solvers/icf/icf_model.h"

#include <utility>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using contact_solvers::internal::BlockSparseSymmetricMatrix;
using contact_solvers::internal::BlockSparsityPattern;
using Eigen::VectorBlock;

template <typename T>
IcfModel<T>::IcfModel() : params_{std::make_unique<IcfParameters<T>>()} {}

template <typename T>
void IcfModel<T>::ResetParameters(std::unique_ptr<IcfParameters<T>> params) {
  DRAKE_DEMAND(params != nullptr);
  params_ = std::move(params);

  // Set some sizes.
  num_velocities_ = v0().size();
  num_bodies_ = ssize(this->params().body_to_clique);
  num_cliques_ = ssize(this->params().clique_sizes);
  DRAKE_DEMAND(num_cliques_ > 0);
  max_clique_size_ = *std::max_element(this->params().clique_sizes.begin(),
                                       this->params().clique_sizes.end());

  // Compute the initial spatial velocity V_WB0 = J_WB⋅v0 for each body.
  V_WB0_.Resize(num_bodies_, 6, 1);
  CalcBodySpatialVelocities(v0(), &V_WB0_);

  // Set the scaling factor diag(M)^{-1/2} for convergence checks.
  scale_factor_ = M0().diagonal().cwiseInverse().cwiseSqrt();

  // Define the sparse dynamics matrix A = M + δt D and the diagonal Delassus
  // operator estimate W = diag(M)⁻¹.
  const std::vector<int>& clique_sizes = this->params().clique_sizes;
  const int num_cliques = ssize(clique_sizes);
  A_.Resize(num_cliques, clique_sizes, clique_sizes);
  clique_delassus_.Resize(num_cliques, clique_sizes);
  for (int c = 0; c < num_cliques_; ++c) {
    const int v_start = this->params().clique_start[c];
    const int nv = clique_sizes[c];
    A_[c] = M0().block(v_start, v_start, nv, nv);
    A_[c].diagonal() += time_step() * D0().segment(v_start, nv);
    clique_delassus_[c] =
        M0().block(v_start, v_start, nv, nv).diagonal().cwiseInverse();
  }

  // Set the linear cost term r = A v₀ - δt k₀.
  Av0_.resize(num_velocities_);
  MultiplyByDynamicsMatrix(v0(), &Av0_);
  r_ = Av0_ - time_step() * k0();

  VerifyInvariants();

  // The old sparsity pattern is now invalid. It must be recomputed with
  // SetSparsityPattern().
  sparsity_pattern_ = nullptr;
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>> IcfModel<T>::clique_segment(
    int clique, const VectorX<T>& full_vector) const {
  DRAKE_ASSERT(full_vector.size() == num_velocities_);
  return full_vector.segment(params().clique_start[clique],
                             params().clique_sizes[clique]);
}

template <typename T>
Eigen::VectorBlock<VectorX<T>> IcfModel<T>::mutable_clique_segment(
    int clique, VectorX<T>* full_vector) const {
  DRAKE_ASSERT(full_vector != nullptr);
  DRAKE_ASSERT(full_vector->size() == num_velocities_);
  return full_vector->segment(params().clique_start[clique],
                              params().clique_sizes[clique]);
}

template <typename T>
void IcfModel<T>::ResizeData(IcfData<T>* data) const {
  data->Resize(num_bodies_, num_velocities_, max_clique_size_);
}

template <typename T>
void IcfModel<T>::CalcData(const VectorX<T>& v, IcfData<T>* data) const {
  // Set generalized velocities v.
  data->set_v(v);

  // Set momentum cost (1/2 v'Av - r'v) and gradient (Av - r).
  CalcMomentumTerms(v, data);

  // Compute spatial velocities for all bodies.
  EigenPool<Vector6<T>>& V_WB = data->mutable_V_WB();
  CalcBodySpatialVelocities(v, &V_WB);

  // TODO(#23769): Add constraint contributions as well. For now all we have is
  // the unconstrained momentum contribution.
  data->set_cost(data->momentum_cost());
}

template <typename T>
std::unique_ptr<BlockSparseSymmetricMatrix<MatrixX<T>>>
IcfModel<T>::MakeHessian(const IcfData<T>& data) const {
  auto hessian = std::make_unique<BlockSparseSymmetricMatrix<MatrixX<T>>>(
      sparsity_pattern());
  UpdateHessian(data, hessian.get());
  return hessian;
}

template <typename T>
void IcfModel<T>::UpdateHessian(
    const IcfData<T>&, BlockSparseSymmetricMatrix<MatrixX<T>>* hessian) const {
  hessian->SetZero();

  // Initialize hessian = A (block diagonal).
  for (int c = 0; c < num_cliques_; ++c) {
    ConstMatrixXView A_clique = A(c);
    hessian->AddToBlock(c, c, A_clique);
  }

  // TODO(#23769): Add constraints' contributions. IcfData will be used in this
  // case.
}

template <typename T>
void IcfModel<T>::CalcSearchDirectionData(
    const IcfData<T>& data, const VectorX<T>& w,
    IcfSearchDirectionData<T>* search_direction_data) const {
  search_direction_data->w.resize(num_velocities_);
  search_direction_data->U.Resize(num_bodies_, 6, 1);

  // We'll use search_direction_data->w as scratch, to avoid memory allocation.
  VectorX<T>& temp = search_direction_data->w;
  MultiplyByDynamicsMatrix(w, &temp);  // temp = A⋅w

  const VectorX<T>& v = data.v();
  search_direction_data->a = temp.dot(w);       // a = ‖w‖²
  const T vAw = v.dot(temp);                    // vAw = vᵀ⋅A⋅w
  search_direction_data->b = vAw - w.dot(r());  // b = vᵀ⋅A⋅w - w⋅r
  search_direction_data->c = data.momentum_cost();

  // It is now safe to overwrite with the desired value.
  search_direction_data->w = w;

  // U = J⋅w.
  CalcBodySpatialVelocities(w, &search_direction_data->U);
}

template <typename T>
T IcfModel<T>::CalcCostAlongLine(
    const T& alpha, const IcfData<T>& data,
    const IcfSearchDirectionData<T>& search_direction, T* dcost_dalpha,
    T* d2cost_dalpha2) const {
  const T& a = search_direction.a;
  const T& b = search_direction.b;
  const T& c = search_direction.c;

  EigenPool<Vector6<T>>& V_WB_alpha = data.scratch().V_WB_alpha;
  DRAKE_ASSERT(V_WB_alpha.size() == num_bodies_);

  VectorXView v_alpha = data.scratch().v_alpha[0];
  DRAKE_ASSERT(v_alpha.size() == num_velocities_);
  v_alpha.noalias() = data.v() + alpha * search_direction.w;

  // Compute momentum contributions:
  T cost = (0.5 * a * alpha + b) * alpha + c;  // = aα²/2 + bα + c
  *dcost_dalpha = a * alpha + b;
  *d2cost_dalpha2 = a;

  // TODO(#23769): Add constraint contributions.

  return cost;
}

template <typename T>
void IcfModel<T>::SetSparsityPattern() {
  DRAKE_DEMAND(params_ != nullptr);

  // N.B. we make a copy here because block_sizes will be moved into the
  // BlockSparsityPattern at the end of this function.
  std::vector<int> block_sizes = params().clique_sizes;

  // Build diagonal entries in the sparsity pattern.
  std::vector<std::vector<int>> sparsity(num_cliques_);
  for (int i = 0; i < num_cliques_; ++i) {
    sparsity[i].emplace_back(i);
  }

  // TODO(#23769): Build off-diagonal entries in the sparsity pattern from
  // constraints.

  sparsity_pattern_ = std::make_unique<BlockSparsityPattern>(
      std::move(block_sizes), std::move(sparsity));
}

template <typename T>
void IcfModel<T>::UpdateTimeStep(const T& time_step) {
  DRAKE_DEMAND(time_step > 0);
  DRAKE_DEMAND(params_ != nullptr);

  // Linearized dynamics matrix A = M + δt⋅D
  for (int c = 0; c < num_cliques_; ++c) {
    const int v_start = params().clique_start[c];
    const int nv = params().clique_sizes[c];
    A_[c] = M0().block(v_start, v_start, nv, nv);
    A_[c].diagonal() += time_step * D0().segment(v_start, nv);
    clique_delassus_[c] =
        M0().block(v_start, v_start, nv, nv).diagonal().cwiseInverse();
  }

  // Linear cost term r = A v₀ - δt k₀
  MultiplyByDynamicsMatrix(v0(), &Av0_);
  r_ = Av0_ - time_step * k0();

  params_->time_step = time_step;
}

template <typename T>
void IcfModel<T>::VerifyInvariants() const {
  DRAKE_DEMAND(params_ != nullptr);
  DRAKE_DEMAND(time_step() > 0);

  DRAKE_DEMAND(num_bodies_ > 0);
  DRAKE_DEMAND(num_velocities_ > 0);
  DRAKE_DEMAND(num_cliques_ > 0);
  DRAKE_DEMAND(max_clique_size_ > 0);

  DRAKE_DEMAND(v0().size() == num_velocities_);
  DRAKE_DEMAND(M0().rows() == num_velocities_);
  DRAKE_DEMAND(M0().cols() == num_velocities_);
  DRAKE_DEMAND(D0().size() == num_velocities_);
  DRAKE_DEMAND(k0().size() == num_velocities_);
  DRAKE_DEMAND(scale_factor().size() == num_velocities_);

  DRAKE_DEMAND(params().J_WB.size() == num_bodies_);
  DRAKE_DEMAND(V_WB0_.size() == num_bodies_);
  DRAKE_DEMAND(ssize(params().body_mass) == num_bodies_);

  DRAKE_DEMAND(A_.size() == num_cliques_);
  DRAKE_DEMAND(r_.size() == num_velocities_);
  DRAKE_DEMAND(Av0_.size() == num_velocities_);
  DRAKE_DEMAND(clique_delassus_.size() == num_cliques_);

  DRAKE_DEMAND(ssize(params().clique_start) == num_cliques_ + 1);
  DRAKE_DEMAND(ssize(params().clique_sizes) == num_cliques_);
  DRAKE_DEMAND(ssize(params().body_to_clique) == num_bodies_);
  DRAKE_DEMAND(ssize(params().body_is_floating) == num_bodies_);

  for (int b = 0; b < num_bodies_; ++b) {
    const int c = body_to_clique(b);
    DRAKE_DEMAND(c < num_cliques_);
    if (c >= 0) {
      DRAKE_DEMAND(J_WB(b).cols() == A(c).rows());
    }
  }

  int nv = 0;
  for (int c = 0; c < num_cliques_; ++c) {
    const int clique_nv = clique_size(c);
    DRAKE_DEMAND(A(c).rows() == clique_nv);
    DRAKE_DEMAND(A(c).cols() == clique_nv);
    nv += clique_nv;
  }
  DRAKE_DEMAND(nv == num_velocities_);
}

template <typename T>
void IcfModel<T>::MultiplyByDynamicsMatrix(const VectorX<T>& v,
                                           VectorX<T>* result) const {
  DRAKE_ASSERT(v.size() == num_velocities_);
  DRAKE_ASSERT(result != nullptr);
  DRAKE_ASSERT(result->size() == num_velocities_);

  for (int c = 0; c < num_cliques_; ++c) {
    ConstMatrixXView A_clique = A(c);
    VectorBlock<const VectorX<T>> v_clique = clique_segment(c, v);
    VectorBlock<VectorX<T>> Av_clique = mutable_clique_segment(c, result);
    Av_clique.noalias() = A_clique * v_clique;  // Required to avoid allocation!
  }
}

template <typename T>
void IcfModel<T>::CalcMomentumTerms(const VectorX<T>& v,
                                    IcfData<T>* data) const {
  DRAKE_ASSERT(v.size() == num_velocities_);
  VectorX<T>& Av = data->mutable_Av();
  VectorXView Av_minus_r = data->scratch().Av_minus_r[0];
  DRAKE_ASSERT(Av_minus_r.size() == num_velocities_);

  // Cost.
  MultiplyByDynamicsMatrix(v, &Av);
  Av_minus_r = 0.5 * Av - r_;
  data->set_momentum_cost(v.dot(Av_minus_r));

  // Gradient.
  data->mutable_gradient() = Av - r_;
}

template <typename T>
void IcfModel<T>::CalcBodySpatialVelocities(
    const VectorX<T>& v, EigenPool<Vector6<T>>* V_pool) const {
  EigenPool<Vector6<T>>& spatial_velocities = *V_pool;
  DRAKE_ASSERT(v.size() == num_velocities_);
  DRAKE_ASSERT(spatial_velocities.size() == num_bodies_);
  for (int b = 0; b < num_bodies_; ++b) {
    const int c = body_to_clique(b);
    Vector6<T>& V_WB = spatial_velocities[b];
    if (c >= 0) {
      VectorBlock<const VectorX<T>> v_clique = clique_segment(c, v);
      if (is_floating(b)) {
        V_WB = v_clique;
      } else {
        V_WB = J_WB(b) * v_clique;
      }
    } else {
      V_WB.setZero();  // Anchored body.
    }
  }
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfModel);
