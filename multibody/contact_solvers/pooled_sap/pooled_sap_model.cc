// NOLINTNEXTLINE(build/include): prevent complaint re pooled_sap_model.h

#include <memory>
#include <set>
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

/* A⋅v = r + Jᵀ⋅γ,
 which corresponds to the convex cost:
    ℓ(v) = 1/2‖v‖²−r⋅v + ℓ(vc).
*/

template <typename T>
void PooledSapModel<T>::MultiplyByDynamicsMatrix(const VectorX<T>& v,
                                                 VectorX<T>* result) const {
  DRAKE_ASSERT(v.size() == num_velocities());
  DRAKE_ASSERT(result != nullptr);
  DRAKE_ASSERT(result->size() == num_velocities());

  const auto& A = params().A;
  result->setZero();
  for (int c = 0; c < A.size(); ++c) {
    // Compute A*v
    ConstMatrixXView<T> A_clique = A[c];
    const int start = clique_start_[c];
    const int nv = clique_sizes_[c];
    const auto v_clique = v.segment(start, nv);
    auto Av_clique = result->segment(start, nv);
    Av_clique.noalias() = A_clique * v_clique;  // Required to avoid allocation!
  }
}

/* Computes:
  - Av
  - momentum_cost */
template <typename T>
void PooledSapModel<T>::CalcMomentumTerms(
    const SapData<T>& data, typename SapData<T>::Cache* cache) const {
  // Parameters.
  const auto& r = params().r;

  // Data.
  const VectorX<T>& v = data.v();
  VectorX<T>& Av = cache->Av;

  // Scratch data.
  data.scratch().Clear();
  auto tmp = data.scratch().VectorX_pool.Add(num_velocities(), 1);

  MultiplyByDynamicsMatrix(v, &Av);

  // Cost.
  tmp = 0.5 * Av - r;
  cache->momentum_cost = v.dot(tmp);

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
      auto J_WB = params().J_WB[b];
      V_WB = J_WB * v_clique;
    } else {
      V_WB.setZero();  // Anchored body.
    }
  }
}

template <typename T>
void PooledSapModel<T>::CalcData(const VectorX<T>& v, SapData<T>* data) const {
  data->v() = v;
  typename SapData<T>::Cache& cache = data->cache();
  CalcMomentumTerms(*data, &cache);
  CalcBodySpatialVelocities(v, &cache.spatial_velocities);
  patch_constraints_pool_.CalcData(cache.spatial_velocities,
                                   &cache.patch_constraints_data);

  // Include patch constraints contributions.
  // TODO(amcastro-tri): factor out this function into a ConstraintPool class,
  // along with clique data size and other common per-pool functionality.
  patch_constraints_pool_.AccumulateGradient(*data, &cache.gradient);

  cache.cost = cache.momentum_cost + cache.patch_constraints_data.cost();
}

template <typename T>
std::unique_ptr<internal::BlockSparseSymmetricMatrixT<T>>
PooledSapModel<T>::MakeHessian(const SapData<T>& data) const {
  auto hessian = std::make_unique<internal::BlockSparseSymmetricMatrixT<T>>(
      CalcSparsityPattern());
  UpdateHessian(data, hessian.get());
  return hessian;
}

template <typename T>
void PooledSapModel<T>::UpdateHessian(
    const SapData<T>& data,
    internal::BlockSparseSymmetricMatrixT<T>* hessian) const {
  hessian->SetZero();

  // Initialize hessian = diag(A).
  const auto& A = params().A;
  for (int c = 0; c < A.size(); ++c) {
    ConstMatrixXView<T> A_clique = A[c];
    hessian->AddToBlock(c, c, A_clique);
  }

  // Add constraints' contributions.
  patch_constraints_pool_.AccumulateHessian(data, hessian);
}

template <typename T>
internal::BlockSparsityPattern PooledSapModel<T>::CalcSparsityPattern() const {
  std::vector<int> block_sizes = clique_sizes_;
  const int num_nodes = block_sizes.size();

  /* Build diagonal entry in sparsity pattern. */
  std::vector<std::vector<int>> sparsity(num_nodes);
  for (int i = 0; i < num_nodes; ++i) {
    sparsity[i].emplace_back(i);
  }

  /* Build off-diagonal entry in sparsity pattern. */
  // TODO(amcastro-tri): Make parent ConstraintsPool for all constraints pool
  // types.
  patch_constraints_pool_.CalcSparsityPattern(&sparsity);

  return internal::BlockSparsityPattern(std::move(block_sizes),
                                        std::move(sparsity));
}

template <typename T>
void PooledSapModel<T>::ResizeData(SapData<T>* data) const {
  data->Resize(num_bodies_, num_velocities_, clique_sizes_,
               patch_constraints_pool_.patch_sizes());
}

template <typename T>
void PooledSapModel<T>::UpdateSearchDirection(
    const SapData<T>& data, const VectorX<T>& w,
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
    const T& alpha, const SapData<T>& data,
    const SearchDirectionData<T>& search_direction, SapData<T>* scratch,
    T* dcost_dalpha, T* d2cost_dalpha2) const {
  typename SapData<T>::Cache& cache_alpha = scratch->cache();

  const T& a = search_direction.a;
  const T& b = search_direction.b;
  const T& c = search_direction.c;

  // N.B. We'll use data.scratch() for these, while we'll use
  // scratch->scratch() below for constraints to avoid overwriting these locals
  // by mistake.
  auto& V_WB_alpha = data.scratch().Vector6_pool;
  V_WB_alpha.Clear();
  V_WB_alpha.Resize(num_bodies());
  auto& v_alpha = data.scratch().v_pool;
  v_alpha.resize(num_velocities());

  v_alpha.noalias() = data.v() + alpha * search_direction.w;

  // Compute momentum contributions:
  T cost = (0.5 * a * alpha + b) * alpha + c;  // = aα²/2 + bα + c
  *dcost_dalpha = a * alpha + b;
  *d2cost_dalpha2 = a;

  // Add constraints contributions:
  T constraint_dcost, constraint_d2cost;
  CalcBodySpatialVelocities(v_alpha, &V_WB_alpha);
  patch_constraints_pool_.CalcData(V_WB_alpha,
                                   &cache_alpha.patch_constraints_data);
  patch_constraints_pool_.ProjectAlongLine(
      cache_alpha.patch_constraints_data, search_direction.U,
      &scratch->scratch(), &constraint_dcost, &constraint_d2cost);
  cost += cache_alpha.patch_constraints_data.cost();
  *dcost_dalpha += constraint_dcost;
  *d2cost_dalpha2 += constraint_d2cost;

  return cost;
}

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::pooled_sap::PooledSapModel);
