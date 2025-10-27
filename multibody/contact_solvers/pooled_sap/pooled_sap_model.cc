// NOLINTNEXTLINE(build/include): prevent complaint re pooled_sap_model.h

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

template <typename T>
using MatrixXView = typename EigenPool<MatrixX<T>>::ElementView;
template <typename T>
using ConstMatrixXView = typename EigenPool<MatrixX<T>>::ConstElementView;

template <typename T>
Eigen::VectorBlock<const VectorX<T>> PooledSapModel<T>::clique_segment(
    int clique, const VectorX<T>& x) const {
  DRAKE_ASSERT(x.size() == num_velocities());
  return x.segment(clique_start_[clique], clique_sizes_[clique]);
}

template <typename T>
Eigen::VectorBlock<VectorX<T>> PooledSapModel<T>::clique_segment(
    int clique, VectorX<T>* x) const {
  DRAKE_ASSERT(x != nullptr);
  DRAKE_ASSERT(x->size() == num_velocities());
  return x->segment(clique_start_[clique], clique_sizes_[clique]);
}

template <typename T>
void PooledSapModel<T>::ResetParameters(
    std::unique_ptr<PooledSapParameters<T>> params) {
  // Check validity of the new parameters
  DRAKE_DEMAND(params != nullptr);
  DRAKE_DEMAND(params->time_step > 0);
  const int num_bodies = ssize(params->body_cliques);
  DRAKE_DEMAND(num_bodies > 0);
  DRAKE_DEMAND(ssize(params->body_is_floating) == num_bodies);
  DRAKE_DEMAND(ssize(params->body_mass) == num_bodies);
  DRAKE_DEMAND(params->body_cliques[0] < 0);  // Always for the world.
  const int num_cliques = params->A.size();
  DRAKE_DEMAND(ssize(params->clique_nu) == num_cliques);

  DRAKE_DEMAND(params->J_WB.size() == num_bodies);
  for (int b = 0; b < num_bodies; ++b) {
    const int c = params->body_cliques[b];
    DRAKE_DEMAND(c < num_cliques);
    if (c >= 0) {
      DRAKE_DEMAND(params->J_WB[b].cols() == params->A[c].rows());
    }
  }

  // Pre-compute the size of each clique and its starting index in the
  // full velocity vector.
  clique_sizes_.clear();
  clique_start_.clear();
  clique_sizes_.reserve(num_cliques);
  clique_start_.reserve(num_cliques + 1);
  clique_start_.push_back(0);
  int num_velocities = 0;

  for (int c = 0; c < num_cliques; ++c) {
    const int clique_nv = params->A[c].rows();
    DRAKE_DEMAND(params->A[c].cols() == clique_nv);
    DRAKE_DEMAND(params->clique_nu[c] <= clique_nv);

    // Here we are pushing the start for the next clique, c+1.
    clique_start_.push_back(clique_start_.back() + clique_nv);
    clique_sizes_.push_back(clique_nv);

    num_velocities += clique_nv;
  }

  DRAKE_DEMAND(params->r.size() == num_velocities);
  DRAKE_DEMAND(params->v0.size() == num_velocities);
  DRAKE_DEMAND(params->effort_limits.size() == num_velocities);

  // All checks passed; set the new parameters.
  params_ = std::move(params);
  num_velocities_ = num_velocities;
  num_bodies_ = num_bodies;

  // Reset all constraint pools.
  coupler_constraints_pool_.Clear();
  gain_constraints_pool_.Clear();
  limit_constraints_pool_.Clear();
  patch_constraints_pool_.Clear();

  // Pre-compute spatial velocity for each body at the current state.
  V_WB0_.Resize(num_bodies_);
  CalcBodySpatialVelocities(params_->v0, &V_WB0_);

  // Set the diagonal approximation of the Delassus operator for constraints
  // for which the Jacobian J is the identity matrix.
  clique_delassus_.Resize(clique_sizes_, clique_sizes_);
  for (int c = 0; c < params_->A.size(); ++c) {
    clique_delassus_[c] = params_->A[c].diagonal().cwiseInverse();
  }
}

template <typename T>
void PooledSapModel<T>::MultiplyByDynamicsMatrix(const VectorX<T>& v,
                                                 VectorX<T>* result) const {
  DRAKE_ASSERT(v.size() == num_velocities());
  DRAKE_ASSERT(result != nullptr);
  DRAKE_ASSERT(result->size() == num_velocities());

  const auto& A = params().A;
  for (int c = 0; c < A.size(); ++c) {
    ConstMatrixXView<T> A_clique = A[c];
    const int start = clique_start_[c];
    const int nv = clique_sizes_[c];
    const auto v_clique = v.segment(start, nv);
    auto Av_clique = result->segment(start, nv);
    Av_clique.noalias() = A_clique * v_clique;  // Required to avoid allocation!
  }
}

template <typename T>
void PooledSapModel<T>::CalcMomentumTerms(
    const PooledSapData<T>& data,
    typename PooledSapData<T>::Cache* cache) const {
  // Parameters.
  const auto& r = params().r;

  // Data.
  const VectorX<T>& v = data.v();
  VectorX<T>& Av = cache->Av;

  // Scratch data.
  data.scratch().Clear();
  VectorX<T>& Av_minus_r = data.scratch().Av_minus_r;
  Av_minus_r.resize(num_velocities());

  MultiplyByDynamicsMatrix(v, &Av);

  // Cost.
  Av_minus_r = 0.5 * Av - r;
  cache->momentum_cost = v.dot(Av_minus_r);

  // Gradient.
  cache->gradient = Av - r;
}

template <typename T>
void PooledSapModel<T>::CalcBodySpatialVelocities(
    const VectorX<T>& v, EigenPool<Vector6<T>>* V_pool) const {
  EigenPool<Vector6<T>>& spatial_velocities = *V_pool;
  spatial_velocities[0].setZero();  // World's spatial velocity.
  for (int b = 1; b < num_bodies_; ++b) {
    const int c = params().body_cliques[b];
    Vector6<T>& V_WB = spatial_velocities[b];
    if (c >= 0) {
      auto v_clique = clique_segment(c, v);
      if (is_floating(b)) {
        V_WB = v_clique;
      } else {
        auto J_WB = params().J_WB[b];
        V_WB = J_WB * v_clique;
      }
    } else {
      V_WB.setZero();  // Anchored body.
    }
  }
}

template <typename T>
void PooledSapModel<T>::CalcData(const VectorX<T>& v,
                                 PooledSapData<T>* data) const {
  // Set generalized velocities v
  data->v() = v;

  // Set momentum cost (1/2 v'Av - r'v) and gradient (Av - r).
  typename PooledSapData<T>::Cache& cache = data->cache();
  CalcMomentumTerms(*data, &cache);

  // Compute spatial velocities for all bodies.
  CalcBodySpatialVelocities(v, &cache.spatial_velocities);

  // Compute constraint data.
  // TODO(CENIC): factor out common functionality into a ConstraintsPool class.
  coupler_constraints_pool_.CalcData(v, &cache.coupler_constraints_data);
  gain_constraints_pool_.CalcData(v, &cache.gain_constraints_data);
  limit_constraints_pool_.CalcData(v, &cache.limit_constraints_data);
  patch_constraints_pool_.CalcData(cache.spatial_velocities,
                                   &cache.patch_constraints_data);

  // Accumulate gradient contributions from constraints.
  coupler_constraints_pool_.AccumulateGradient(*data, &cache.gradient);
  gain_constraints_pool_.AccumulateGradient(*data, &cache.gradient);
  limit_constraints_pool_.AccumulateGradient(*data, &cache.gradient);
  patch_constraints_pool_.AccumulateGradient(*data, &cache.gradient);

  // Accumulate cost contributions from constraints.
  cache.cost = cache.momentum_cost;
  cache.cost += cache.coupler_constraints_data.cost();
  cache.cost += cache.gain_constraints_data.cost();
  cache.cost += cache.limit_constraints_data.cost();
  cache.cost += cache.patch_constraints_data.cost();
}

template <typename T>
std::unique_ptr<internal::BlockSparseSymmetricMatrixT<T>>
PooledSapModel<T>::MakeHessian(const PooledSapData<T>& data) const {
  auto hessian = std::make_unique<internal::BlockSparseSymmetricMatrixT<T>>(
      sparsity_pattern());
  UpdateHessian(data, hessian.get());
  return hessian;
}

template <typename T>
void PooledSapModel<T>::UpdateHessian(
    const PooledSapData<T>& data,
    internal::BlockSparseSymmetricMatrixT<T>* hessian) const {
  hessian->SetZero();

  // Initialize hessian = A (block diagonal).
  const auto& A = params().A;
  for (int c = 0; c < A.size(); ++c) {
    ConstMatrixXView<T> A_clique = A[c];
    hessian->AddToBlock(c, c, A_clique);
  }

  // Add constraints' contributions.
  coupler_constraints_pool_.AccumulateHessian(data, hessian);
  gain_constraints_pool_.AccumulateHessian(data, hessian);
  limit_constraints_pool_.AccumulateHessian(data, hessian);
  patch_constraints_pool_.AccumulateHessian(data, hessian);
}

template <typename T>
void PooledSapModel<T>::SetSparsityPattern() {
  std::vector<int> block_sizes = clique_sizes_;
  const int num_nodes = block_sizes.size();

  // Build diagonal entries in the sparsity pattern.
  std::vector<std::vector<int>> sparsity(num_nodes);
  for (int i = 0; i < num_nodes; ++i) {
    sparsity[i].emplace_back(i);
  }

  // Build off-diagonal entries in the sparsity pattern.
  // TODO(CENIC): account for other constraints that may change the sparsity
  // pattern.
  patch_constraints_pool_.CalcSparsityPattern(&sparsity);

  sparsity_pattern_ = std::make_unique<internal::BlockSparsityPattern>(
      std::move(block_sizes), std::move(sparsity));
}

template <typename T>
void PooledSapModel<T>::ResizeData(PooledSapData<T>* data) const {
  // TODO(vincekurtz): treat patch constraints more like the other constraints.
  data->Resize(num_bodies_, num_velocities_,
               patch_constraints_pool_.patch_sizes());
  coupler_constraints_pool_.ResizeData(&data->cache().coupler_constraints_data);
  gain_constraints_pool_.ResizeData(&data->cache().gain_constraints_data);
  limit_constraints_pool_.ResizeData(&data->cache().limit_constraints_data);
}

template <typename T>
void PooledSapModel<T>::UpdateSearchDirection(
    const PooledSapData<T>& data, const VectorX<T>& w,
    SearchDirectionData<T>* search_data) const {
  search_data->w.resize(num_velocities());
  search_data->U.Resize(num_bodies());

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
T PooledSapModel<T>::CalcCostAlongLine(
    const T& alpha, const PooledSapData<T>& data,
    const SearchDirectionData<T>& search_direction, PooledSapData<T>* scratch,
    T* dcost_dalpha, T* d2cost_dalpha2) const {
  (void)scratch;
  const T& a = search_direction.a;
  const T& b = search_direction.b;
  const T& c = search_direction.c;

  // N.B. We'll use data.scratch() for these, while we'll use
  // scratch->scratch() below for constraints to avoid overwriting these locals
  // by mistake.
  auto& V_WB_alpha = data.scratch().V_WB_alpha;
  V_WB_alpha.Clear();
  V_WB_alpha.Resize(num_bodies());
  auto& v_alpha = data.scratch().v_alpha;
  v_alpha.resize(num_velocities());

  v_alpha.noalias() = data.v() + alpha * search_direction.w;

  // Compute momentum contributions:
  T cost = (0.5 * a * alpha + b) * alpha + c;  // = aα²/2 + bα + c
  *dcost_dalpha = a * alpha + b;
  *d2cost_dalpha2 = a;

  T constraint_dcost, constraint_d2cost;

  // Add coupler constraints contributions:
  {
    // TODO(vincekurtz): resize scratch space earlier, and only once.
    coupler_constraints_pool_.ResizeData(
        &data.scratch().coupler_constraints_data);
    coupler_constraints_pool_.CalcData(
        v_alpha, &data.scratch().coupler_constraints_data);
    coupler_constraints_pool_.ProjectAlongLine(
        data.scratch().coupler_constraints_data, search_direction.w,
        &constraint_dcost, &constraint_d2cost);
    cost += data.scratch().coupler_constraints_data.cost();
    *dcost_dalpha += constraint_dcost;
    *d2cost_dalpha2 += constraint_d2cost;
  }

  // Add gain constraints contributions:
  {
    data.scratch().Gw_gain.resize(num_velocities());
    gain_constraints_pool_.ResizeData(&data.scratch().gain_constraints_data);
    gain_constraints_pool_.CalcData(v_alpha,
                                    &data.scratch().gain_constraints_data);
    gain_constraints_pool_.ProjectAlongLine(
        data.scratch().gain_constraints_data, search_direction.w,
        &data.scratch().Gw_gain, &constraint_dcost, &constraint_d2cost);
    cost += data.scratch().gain_constraints_data.cost();
    *dcost_dalpha += constraint_dcost;
    *d2cost_dalpha2 += constraint_d2cost;
  }

  // Add limit constraints contributions:
  {
    data.scratch().Gw_limit.resize(num_velocities());
    limit_constraints_pool_.ResizeData(&data.scratch().limit_constraints_data);
    limit_constraints_pool_.CalcData(v_alpha,
                                     &data.scratch().limit_constraints_data);
    limit_constraints_pool_.ProjectAlongLine(
        data.scratch().limit_constraints_data, search_direction.w,
        &data.scratch().Gw_limit, &constraint_dcost, &constraint_d2cost);
    cost += data.scratch().limit_constraints_data.cost();
    *dcost_dalpha += constraint_dcost;
    *d2cost_dalpha2 += constraint_d2cost;
  }

  // Add patch constraints contributions:
  {
    data.scratch().patch_constraints_data.Resize(
        patch_constraints_pool_.patch_sizes());
    CalcBodySpatialVelocities(v_alpha, &V_WB_alpha);
    patch_constraints_pool_.CalcData(V_WB_alpha,
                                     &data.scratch().patch_constraints_data);
    patch_constraints_pool_.ProjectAlongLine(
        data.scratch().patch_constraints_data, search_direction.U,
        &data.scratch().U_AbB_W_pool, &constraint_dcost, &constraint_d2cost);
    cost += data.scratch().patch_constraints_data.cost();
    *dcost_dalpha += constraint_dcost;
    *d2cost_dalpha2 += constraint_d2cost;
  }

  return cost;
}

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::pooled_sap::PooledSapModel);
