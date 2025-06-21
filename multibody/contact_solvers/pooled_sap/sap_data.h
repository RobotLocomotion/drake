#pragma once

#include <memory>
#include <numeric>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/linear_solve.h"
#include "drake/multibody/contact_solvers/block_sparse_cholesky_solver.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/pooled_sap/gain_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/limit_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/patch_constraints_data_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

/* SAP generalized velocities v and SAP quantities function of v.

[Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An Unconstrained
Convex Formulation of Compliant Contact. Available at
https://arxiv.org/abs/2110.10107 */
template <typename T>
class SapData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapData);

  struct Cache {
    void Resize(int num_bodies, int num_velocities,
                const std::vector<int>& clique_sizes,
                const std::vector<int>& patch_sizes) {
      (void)clique_sizes;
      const int nv = num_velocities;
      Av.resize(nv);
      gradient.resize(nv);
      spatial_velocities.Resize(num_bodies);
      patch_constraints_data.Resize(num_velocities, patch_sizes);

      // These are members in BlockSparseSuperNodalSolver, along sugar.
      // std::unique_ptr<BlockSparseSymmetricMatrix> H_;
      // BlockSparseCholeskySolver<Eigen::MatrixXd> solver_;
    }

    T momentum_cost{0};
    T constraints_cost{0};
    T cost;
    VectorX<T> Av;        // = A * v.
    VectorX<T> gradient;  // Of size num_velocities().

    // Rigid body spatial velocities, V_WB.
    EigenPool<Vector6<T>> spatial_velocities;

    // Type-specific constraint pools.
    GainConstraintsDataPool<T> gain_constraints_data;
    LimitConstraintsDataPool<T> limit_constraints_data;
    PatchConstraintsDataPool<T> patch_constraints_data;
  };

  /* Struct to store pre-allocated scratch space.
   This space is not intended for long-term storage and is often cleared or
   overwritten as needed. */
  struct Scratch {
    /* Clears all data without changing capacity. */
    void Clear() {
      Vector6_pool.Clear();
      VectorX_pool.Clear();
      MatrixX_pool.Clear();
    }
    // Meant for velocity sized vectors that do not change size. Update to
    // EigenPool when AutoDiffXd is better supported.
    VectorX<T> v_pool;
    EigenPool<Vector6<T>> Vector6_pool;
    EigenPool<VectorX<T>> VectorX_pool;
    EigenPool<MatrixX<T>> MatrixX_pool;
  };

  /* Default constructor for empty data. */
  SapData() = default;

  /* @param num_velocities Total number of generalized velocities.
     @param patch_sizes Number of contact pairs for each patch.
     @param patch_num_velocities Number of participating velocities per patch.
     */
  void Resize(int num_bodies, int num_velocities,
              const std::vector<int>& clique_sizes,
              const std::vector<int>& patch_sizes) {
    v_.resize(num_velocities);
    cache_.Resize(num_bodies, num_velocities, clique_sizes, patch_sizes);
  }

  int num_velocities() const { return v_.size(); }

  int num_patches() const {
    return cache_.patch_constraints_data.num_patches();
  }

  int num_gains() const {
    return cache_.gain_constraints_data.num_constraints();
  }

  int num_limits() const {
    return cache_.limit_constraints_data.num_constraints();
  }

  const VectorX<T>& v() const { return v_; }
  VectorX<T>& v() { return v_; }

  const Cache& cache() const { return cache_; }
  Cache& cache() { return cache_; }

  Scratch& scratch() const { return scratch_; }

 private:
  VectorX<T> v_;  // Generalized velocities.
  Cache cache_;   // All other quantities function of v.
  // We allow PooledSapModel methods to write on the scratch as needed.
  mutable Scratch scratch_;
};

template <typename T>
struct SearchDirectionData {
  // DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SearchDirectionData);

  VectorX<T> w;  // Search direction.

  // Precomputed terms:
  //   ℓ(α) = ℓ(v+α⋅w) = aα²/2 + bα + c + ℓᶜ(v+α⋅w),
  // where ℓᶜ(v+α⋅w) is the constraints cost.
  T a;                      // = ‖w‖²/2 (A norm)
  T b;                      // = w⋅(v+r)
  T c;                      // = ‖v‖²/2 + r⋅v  (momentum cost at v)
  EigenPool<Vector6<T>> U;  // U = J⋅w.
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
