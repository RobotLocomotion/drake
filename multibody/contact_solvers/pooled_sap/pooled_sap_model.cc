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

/* Computes:
  - Av
  - momentum_cost */
template <typename T>
void PooledSapModel<T>::CalcMomentumTerms(
    const SapData<T>& data, typename SapData<T>::Cache* cache) const {
  // Parameters.
  const auto& A = params().A;
  const auto& r = params().r;

  // Data.
  const VectorX<T>& v = data.v();
  VectorX<T>& Av = cache->Av;

  // Scratch data.
  data.scratch().Clear();
  auto tmp = data.scratch().VectorX_pool.Add(num_velocities(), 1);

  MatrixX<T>& H = cache->hessian;
  H.setZero();

  Av.setZero();
  for (int c = 0; c < A.size(); ++c) {
    // Compute A*v
    ConstMatrixXView<T> A_clique = A[c];
    const int start = clique_start_[c];
    const int nv = clique_sizes_[c];
    const auto v_clique = v.segment(start, nv);
    auto Av_clique = Av.segment(start, nv);
    Av_clique.noalias() = A_clique * v_clique;  // Required to avoid allocation!
    // Initialize H = diag(A).
    H.block(start, start, nv, nv) = A_clique;
  }

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
    if (c >= 0) {
      auto v_clique = clique_segment(c, v);
      auto J_WB = params().J_WB[b];
      Vector6<T>& V_WB = spatial_velocities[b];
      V_WB = J_WB * v_clique;
    }
  }
}

template <typename T>
void PooledSapModel<T>::CalcData(const VectorX<T>& v, SapData<T>* data) const {
  data->v() = v;
  typename SapData<T>::Cache& cache = data->cache();
  CalcMomentumTerms(*data, &cache);
  CalcBodySpatialVelocities(v, &cache.spatial_velocities);
  patch_constraints_pool_.CalcData(*data, &cache.patch_constraints_data);

  // Include patch constraints contributions.
  // TODO(amcastro-tri): factor out this function into a ConstraintPool class,
  // along with clique data size and other common per-pool functionality.
  patch_constraints_pool_.AccumulateGradientAndHessian(*data, &cache.gradient,
                                                       &cache.hessian);

  // Complete lower triangle.
  cache.hessian.template triangularView<Eigen::StrictlyLower>() =
      cache.hessian.template triangularView<Eigen::StrictlyUpper>().transpose();

  cache.cost = cache.momentum_cost + cache.patch_constraints_data.cost();
}

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::pooled_sap::PooledSapModel);
