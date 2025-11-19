#pragma once

#include <span>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Data for performing efficient exact line search. */
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

/* Data for the ICF problem minᵥ ℓ(v; q₀, v₀, δt).

This class stores all data that depends on the current generalized velocity
v, and therefore changes at each solver iteration. That is in contrast with
IcfModel, which changes with (q₀, v₀, δt) but remains constant for different
values of v during the optimization process. */
template <typename T>
class IcfData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfData);

  /* Cache holds quantities that are computed from v, so they can be reused. */
  struct Cache {
    void Resize(int num_bodies, int num_velocities);

    T momentum_cost{0};
    T constraints_cost{0};
    T cost;
    VectorX<T> Av;        // = A * v.
    VectorX<T> gradient;  // Of size num_velocities().

    // Rigid body spatial velocities, V_WB.
    EigenPool<Vector6<T>> spatial_velocities;
  };

  /* Struct to store pre-allocated scratch space. Unlike the cache, this scratch
  space is for intermediate computations, and is often cleared or overwritten as
  needed. */
  struct Scratch {
    /* Clears all data without changing capacity. */
    void Clear();

    /* Resizes the scratch space, allocating memory as needed. */
    void Resize(int num_bodies, int num_velocities, int max_clique_size);

    // Scratch space for CalcMomentumTerms
    VectorX<T> Av_minus_r;

    // Scratch space for CalcCostAlongLine
    EigenPool<Vector6<T>> V_WB_alpha;
    VectorX<T> v_alpha;

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

  /* Constructs empty data. */
  IcfData() = default;

  /* Resizes the data to accommodate the given problem, typically called at the
  beginning of each solve/time step.

  @param num_bodies Total number of bodies in the model.
  @param num_velocities Total number of generalized velocities.
  @param max_clique_size Maximum number of velocities in any clique. */
  void Resize(int num_bodies, int num_velocities, int max_clique_size);

  /* Returns the number of generalized velocities in the system. */
  int num_velocities() const { return v_.size(); }

  /* Returns the generalized velocity vector v. */
  const VectorX<T>& v() const { return v_; }
  VectorX<T>& v() { return v_; }

  /* Returns a cache of intermediate quanities computed from v. */
  const Cache& cache() const { return cache_; }
  Cache& cache() { return cache_; }

  /* Returns a mutable scratch space for intermediate computations. We allow
  IcfModel to write on the scratch as needed. */
  Scratch& scratch() const { return scratch_; }

 private:
  VectorX<T> v_;
  Cache cache_;
  mutable Scratch scratch_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfData);
