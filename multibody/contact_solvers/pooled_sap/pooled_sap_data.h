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
#include "drake/multibody/contact_solvers/pooled_sap/coupler_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/gain_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/limit_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/patch_constraints_data_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

/**
 * Data for the SAP problem minᵥ ℓ(v; q₀, v₀, δt).
 *
 * This class stores all data that depends on the current generalized velocity
 * v, and therefore changes at each solver iteration. That is in contrast with
 * PooledSapModel, which changes with (q₀, v₀, δt) but remains constant for
 * different values of v during the optimization process.
 */
template <typename T>
class PooledSapData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PooledSapData);

  // The cache holds quantities that are computed from v, so they can be reused.
  struct Cache {
    void Resize(int num_bodies, int num_velocities,
                const std::vector<int>& patch_sizes) {
      const int nv = num_velocities;
      Av.resize(nv);
      gradient.resize(nv);
      spatial_velocities.Resize(num_bodies);
      patch_constraints_data.Resize(patch_sizes);
    }

    T momentum_cost{0};
    T constraints_cost{0};
    T cost;
    VectorX<T> Av;        // = A * v.
    VectorX<T> gradient;  // Of size num_velocities().

    // Rigid body spatial velocities, V_WB.
    EigenPool<Vector6<T>> spatial_velocities;

    // Type-specific constraint pools.
    CouplerConstraintsDataPool<T> coupler_constraints_data;
    GainConstraintsDataPool<T> gain_constraints_data;
    LimitConstraintsDataPool<T> limit_constraints_data;
    PatchConstraintsDataPool<T> patch_constraints_data;
  };

  // Struct to store pre-allocated scratch space. This space is not intended for
  // long-term storage and is often cleared or overwritten as needed.
  struct Scratch {
    // Clear all data without changing capacity.
    void Clear() {
      Vector6_pool.Clear();
      H_BB_pool.Clear();
      H_AA_pool.Clear();
      H_AB_pool.Clear();
      H_BA_pool.Clear();
      GJa_pool.Clear();
      GJb_pool.Clear();
    }
    // TODO(CENIC): this is meant for velocity sized vectors that do not change
    // size. Update to EigenPool when AutoDiffXd is better supported.
    VectorX<T> v_pool;
    EigenPool<Vector6<T>> Vector6_pool;
    EigenPool<MatrixX<T>> MatrixX_pool;

    // Data for Hessian accumulation. These pools will only ever hold one
    // element, but using pools instead of a single MatrixX<T> allows us to
    // avoid extra heap allocations, as their sizes change frequently.
    EigenPool<MatrixX<T>> H_BB_pool;
    EigenPool<MatrixX<T>> H_AA_pool;
    EigenPool<MatrixX<T>> H_AB_pool;
    EigenPool<MatrixX<T>> H_BA_pool;
    EigenPool<Matrix6X<T>> GJa_pool;
    EigenPool<Matrix6X<T>> GJb_pool;
  };

  /* Default constructor for empty data. */
  PooledSapData() = default;

  /**
   * Resizes the data to accommodate the given problem, typically called at the
   * beginning of each solve/time step.
   *
   * @param num_velocities Total number of generalized velocities.
   * @param patch_sizes Number of contact pairs for each patch.
   * @param patch_num_velocities Number of participating velocities per patch.
   *
   * TODO(vincekurtz): consider fixing num_bodies and num_velocities at
   * construction, and only resizing based on patch_sizes here.
   */
  void Resize(int num_bodies, int num_velocities,
              const std::vector<int>& patch_sizes) {
    v_.resize(num_velocities);
    cache_.Resize(num_bodies, num_velocities, patch_sizes);
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

  int num_couplers() const {
    return cache_.coupler_constraints_data.num_constraints();
  }

  const VectorX<T>& v() const { return v_; }
  VectorX<T>& v() { return v_; }

  const Cache& cache() const { return cache_; }
  Cache& cache() { return cache_; }

  Scratch& scratch() const { return scratch_; }

 private:
  VectorX<T> v_;  // Generalized velocities.
  Cache cache_;   // All other quantities that are computed from v.

  // We allow PooledSapModel methods to write on the scratch as needed.
  // TODO(CENIC): figure out a better/cleaner way to handle scratch space.
  mutable Scratch scratch_;
};

template <typename T>
struct SearchDirectionData {
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
