#pragma once

#include <span>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/icf/coupler_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/icf/gain_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/icf/limit_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/icf/patch_constraints_data_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Data for the ICF problem minᵥ ℓ(v; q₀, v₀, δt).

This class stores all data that depends on the current generalized velocity
v, and therefore changes at each solver iteration. That is in contrast with
IcfModel, which changes with (q₀, v₀, δt) but remains constant for different
values of v during the optimization process. */
template <typename T>
class IcfData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfData);

  /* The cache holds quantities that are computed from v, so they can be reused.
   */
  struct Cache {
    void Resize(int num_bodies, int num_velocities, int num_couplers,
                std::span<const int> gain_sizes,
                std::span<const int> limit_sizes,
                std::span<const int> patch_sizes);

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

  /* Struct to store pre-allocated scratch space. Unlike the cache, this scratch
  space is for intermediate computations, and is often cleared or overwritten as
  needed. */
  struct Scratch {
    /* Clear all data without changing capacity. */
    void Clear();

    /* Resize the scratch space, allocating memory as needed. */
    void Resize(int num_bodies, int num_velocities, int max_clique_size,
                int num_couplers, std::span<const int> gain_sizes,
                std::span<const int> limit_sizes,
                std::span<const int> patch_sizes);

    // Scratch space for CalcMomentumTerms
    VectorX<T> Av_minus_r;

    // Scratch space for CalcCostAlongLine
    EigenPool<Vector6<T>> V_WB_alpha;
    VectorX<T> v_alpha;

    // Scratch space for constraint projection in CalcCostAlongLine
    VectorX<T> Gw_gain;
    VectorX<T> Gw_limit;
    EigenPool<Vector6<T>> U_AbB_W_pool;

    // Scratch data pools for CalcCostAlongLine
    CouplerConstraintsDataPool<T> coupler_constraints_data;
    GainConstraintsDataPool<T> gain_constraints_data;
    LimitConstraintsDataPool<T> limit_constraints_data;
    PatchConstraintsDataPool<T> patch_constraints_data;

    // Scratch space for Hessian accumulation. These pools will only ever hold
    // one element, but using pools instead of a single MatrixX<T> allows us to
    // avoid extra heap allocations, as their sizes change frequently.
    EigenPool<MatrixX<T>> H_BB_pool;
    EigenPool<MatrixX<T>> H_AA_pool;
    EigenPool<MatrixX<T>> H_AB_pool;
    EigenPool<MatrixX<T>> H_BA_pool;
    EigenPool<Matrix6X<T>> GJa_pool;
    EigenPool<Matrix6X<T>> GJb_pool;
  };

  /* Default constructor for empty data. */
  IcfData() = default;

  /* Resizes the data to accommodate the given problem, typically called at the
  beginning of each solve/time step.

  @param num_bodies Total number of bodies in the model.
  @param num_velocities Total number of generalized velocities.
  @param max_clique_size Maximum number of velocities in any clique.
  @param num_couplers Number of coupler constraints.
  @param gain_sizes Number of velocities for each gain constraint.
  @param limit_sizes Number of velocities for each limit constraint.
  @param patch_sizes Number of contact pairs for each patch constraint.

  TODO(vincekurtz): consider fixing num_bodies and num_velocities at
  construction, and only resizing based on patch_sizes here. */
  void Resize(int num_bodies, int num_velocities, int max_clique_size,
              int num_couplers, std::span<const int> gain_sizes,
              std::span<const int> limit_sizes,
              std::span<const int> patch_sizes);

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

  // We allow IcfModel methods to write on the scratch as needed.
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

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfData);
