#include "drake/multibody/contact_solvers/icf/icf_model.h"

#include <utility>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T>
void IcfModel<T>::ResetParameters(std::unique_ptr<IcfParameters<T>> params) {
  DRAKE_ASSERT(params != nullptr);
  params_ = std::move(params);

  // Set some sizes
  num_velocities_ = v0().size();
  num_bodies_ = ssize(this->params().body_to_clique);
  num_cliques_ = ssize(this->params().clique_sizes);

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

  // Set the linear cost term r = A v₀ - δt k₀
  Av0_.resize(num_velocities_);
  MultiplyByDynamicsMatrix(v0(), &Av0_);
  r_ = Av0_ - time_step() * k0();

  // Compute the initial spatial velocity V_WB0 = J_WB⋅v0 for each body
  V_WB0_.Resize(num_bodies_, 6, 1);
  CalcBodySpatialVelocities(v0(), &V_WB0_);

  // Set the scaling factor diag(M)^{-1/2} for convergence checks
  scale_factor_ = M0().diagonal().cwiseInverse().cwiseSqrt();

  VerifyInvariants();
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>> IcfModel<T>::clique_segment(
    int clique, const VectorX<T>& x) const {
  DRAKE_ASSERT(x.size() == num_velocities());
  return x.segment(params().clique_start[clique],
                   params().clique_sizes[clique]);
}

template <typename T>
Eigen::VectorBlock<VectorX<T>> IcfModel<T>::clique_segment(
    int clique, VectorX<T>* x) const {
  DRAKE_ASSERT(x != nullptr);
  DRAKE_ASSERT(x->size() == num_velocities());
  return x->segment(params().clique_start[clique],
                    params().clique_sizes[clique]);
}

template <typename T>
void IcfModel<T>::VerifyInvariants() const {
  DRAKE_DEMAND(time_step() > 0);
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
  DRAKE_ASSERT(v.size() == num_velocities());
  DRAKE_ASSERT(result != nullptr);
  DRAKE_ASSERT(result->size() == num_velocities());

  for (int c = 0; c < num_cliques(); ++c) {
    ConstMatrixXView A_clique = A(c);
    const auto v_clique = clique_segment(c, v);
    auto Av_clique = clique_segment(c, result);
    Av_clique.noalias() = A_clique * v_clique;  // Required to avoid allocation!
  }
}

template <typename T>
void IcfModel<T>::CalcMomentumTerms(const IcfData<T>& data,
                                    typename IcfData<T>::Cache* cache) const {
  // Data.
  const VectorX<T>& v = data.v();
  VectorX<T>& Av = cache->Av;

  // Scratch data.
  VectorX<T>& Av_minus_r = data.scratch().Av_minus_r;
  DRAKE_ASSERT(Av_minus_r.size() == num_velocities());

  // Cost.
  MultiplyByDynamicsMatrix(v, &Av);
  Av_minus_r = 0.5 * Av - r_;
  cache->momentum_cost = v.dot(Av_minus_r);

  // Gradient.
  cache->gradient = Av - r_;
}

template <typename T>
void IcfModel<T>::CalcBodySpatialVelocities(
    const VectorX<T>& v, EigenPool<Vector6<T>>* V_pool) const {
  EigenPool<Vector6<T>>& spatial_velocities = *V_pool;
  DRAKE_ASSERT(v.size() == num_velocities());
  DRAKE_ASSERT(spatial_velocities.size() == num_bodies());
  spatial_velocities[0].setZero();  // World's spatial velocity.
  for (int b = 1; b < num_bodies(); ++b) {
    const int c = body_to_clique(b);
    Vector6<T>& V_WB = spatial_velocities[b];
    if (c >= 0) {
      auto v_clique = clique_segment(c, v);
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

template <typename T>
void IcfModel<T>::ResizeData(IcfData<T>* data) const {
  const int max_clique_size = *std::max_element(params().clique_sizes.begin(),
                                                params().clique_sizes.end());
  data->Resize(num_bodies_, num_velocities_, max_clique_size);
}

template <typename T>
void IcfModel<T>::CalcData(const VectorX<T>& v, IcfData<T>* data) const {
  // Set generalized velocities v
  data->v() = v;

  // Set momentum cost (1/2 v'Av - r'v) and gradient (Av - r).
  typename IcfData<T>::Cache& cache = data->cache();
  CalcMomentumTerms(*data, &cache);

  // Compute spatial velocities for all bodies.
  CalcBodySpatialVelocities(v, &cache.spatial_velocities);

  // TODO(#23769): Add constraint contributions as well. For now all we have is
  // the unconstrained momentum contribution.
  cache.cost = cache.momentum_cost;
}

template <typename T>
std::unique_ptr<BlockSparseSymmetricMatrixT<T>> IcfModel<T>::MakeHessian(
    const IcfData<T>& data) const {
  auto hessian =
      std::make_unique<BlockSparseSymmetricMatrixT<T>>(sparsity_pattern());
  UpdateHessian(data, hessian.get());
  return hessian;
}

template <typename T>
void IcfModel<T>::UpdateHessian(const IcfData<T>&,
                                BlockSparseSymmetricMatrixT<T>* hessian) const {
  hessian->SetZero();

  // Initialize hessian = A (block diagonal).
  for (int c = 0; c < num_cliques(); ++c) {
    ConstMatrixXView A_clique = A(c);
    hessian->AddToBlock(c, c, A_clique);
  }

  // TODO(#23769): Add constraints' contributions. IcfData will be used in this
  // case.
}

template <typename T>
void IcfModel<T>::SetSparsityPattern() {
  // N.B. we make a copy here because block_sizes will be moved into the
  // BlockSparsityPattern at the end of this function.
  std::vector<int> block_sizes = params().clique_sizes;

  // Build diagonal entries in the sparsity pattern.
  std::vector<std::vector<int>> sparsity(num_cliques());
  for (int i = 0; i < num_cliques(); ++i) {
    sparsity[i].emplace_back(i);
  }

  // TODO(#23769): Build off-diagonal entries in the sparsity pattern from
  // constraints.

  sparsity_pattern_ = std::make_unique<BlockSparsityPattern>(
      std::move(block_sizes), std::move(sparsity));
}

template <typename T>
void IcfModel<T>::UpdateSearchDirection(
    const IcfData<T>& data, const VectorX<T>& w,
    SearchDirectionData<T>* search_data) const {
  search_data->w.resize(num_velocities());
  search_data->U.Resize(num_bodies(), 6, 1);

  // We'll use search_data->w as scratch, to avoid memory allocation.
  auto& tmp = search_data->w;
  MultiplyByDynamicsMatrix(w, &tmp);  // tmp = A⋅w

  const VectorX<T>& v = data.v();
  search_data->a = tmp.dot(w);        // a = ‖w‖²
  const T vAw = v.dot(tmp);           // vAw = vᵀ⋅A⋅w
  search_data->b = vAw - w.dot(r());  // b = vᵀ⋅A⋅w - w⋅r
  search_data->c = data.cache().momentum_cost;

  search_data->w = w;  // it is now safe to overwrite with the desired value.

  // U = J⋅w.
  CalcBodySpatialVelocities(w, &search_data->U);
}

template <typename T>
T IcfModel<T>::CalcCostAlongLine(const T& alpha, const IcfData<T>& data,
                                 const SearchDirectionData<T>& search_direction,
                                 T* dcost_dalpha, T* d2cost_dalpha2) const {
  const T& a = search_direction.a;
  const T& b = search_direction.b;
  const T& c = search_direction.c;

  auto& V_WB_alpha = data.scratch().V_WB_alpha;
  DRAKE_ASSERT(V_WB_alpha.size() == num_bodies());

  auto& v_alpha = data.scratch().v_alpha;
  DRAKE_ASSERT(v_alpha.size() == num_velocities());
  v_alpha.noalias() = data.v() + alpha * search_direction.w;

  // Compute momentum contributions:
  T cost = (0.5 * a * alpha + b) * alpha + c;  // = aα²/2 + bα + c
  *dcost_dalpha = a * alpha + b;
  *d2cost_dalpha2 = a;

  // TODO(#23769): Add constraint contributions.

  return cost;
}

template <typename T>
void IcfModel<T>::UpdateTimeStep(const T& time_step) {
  DRAKE_DEMAND(time_step > 0);

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

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfModel);
