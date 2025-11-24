#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Data for performing efficient exact line search.

For line search, we consider the cost
  ℓ(v) = 0.5 vᵀAv - rᵀv + ℓᶜ(v)
along a search direction w as a function of the step size α:
  ℓ̃ (α) = ℓ(v + α⋅w).
Here A and r define the unconstrained momentum cost, and ℓᶜ(v) is the
constraints cost.

This struct holds precomputed terms (e.g., a, b, c) that allow us to efficiently
evaluate
  ℓ̃ (α) = aα²/2 + bα + c + ℓᶜ(v+α⋅w),
and its derivatives for different values of α. */
template <typename T>
struct SearchDirectionData {
  VectorX<T> w;  // Search direction.

  // Precomputed quadratic coefficients
  T a;  // = ‖w‖²/2 (A norm)
  T b;  // = w⋅(v+r)
  T c;  // = ‖v‖²/2 + r⋅v  (momentum cost at v)

  // Spatial velocities U = J⋅w for each body.
  EigenPool<Vector6<T>> U;
};

/* Data for the ICF problem minᵥ ℓ(v; q₀, v₀, δt).

This class stores all data that depends on the current generalized velocity
v, and therefore changes at each solver iteration. That is in contrast with
IcfModel, which changes with (q₀, v₀, δt) but remains constant for different
values of v during the optimization process.

For most data elements, we provide a getter and a setter, e.g.,
  const T& cost() const;
  void set_cost(const T& cost);
For elements where setting would require require allocation, we instead offer
a mutable getter, e.g.,
  EigenPool<Vector6<T>>& mutable_V_WB(); */
template <typename T>
class IcfData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfData);

  /* Struct to store pre-allocated scratch space. Unlike the cache, this scratch
  space is for intermediate computations, and is often cleared or overwritten as
  needed. */
  struct Scratch {
    /* Clears all data without changing capacity. */
    void ClearPools();

    /* Resizes the scratch space, allocating memory as needed. */
    void Resize(int num_bodies, int num_velocities, int max_clique_size);

    // Scratch space for CalcMomentumTerms
    VectorX<T> Av_minus_r;  // A⋅v - r, size num_velocities()

    // Scratch space for CalcCostAlongLine
    EigenPool<Vector6<T>> V_WB_alpha;  // body spatial velocities at v + α⋅w.
    VectorX<T> v_alpha;                // v + α⋅w, size num_velocities()

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

  ~IcfData();

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

  /* Sets the generalized velocity vector v. In debug builds, sets member data
  (other than v) to NaN to enforce that any downstream quantities are recomputed
  after v changes. */
  void set_v(const VectorX<T>& v);

  /* Returns the pool of rigid body spatial velocities, V_WB. Size is
  num_bodies(). */
  const EigenPool<Vector6<T>>& V_WB() const { return V_WB_; }
  EigenPool<Vector6<T>>& mutable_V_WB() { return V_WB_; }

  /* Returns the linearized dynamics matrix times velocities, A⋅v. Size is
   * num_velocities().*/
  const VectorX<T>& Av() const { return Av_; }
  VectorX<T>& mutable_Av() { return Av_; }

  /* Returns the momentum cost 0.5 vᵀA v - rᵀv. */
  const T& momentum_cost() const { return momentum_cost_; }
  void set_momentum_cost(const T& cost) { momentum_cost_ = cost; }

  /* Returns the constraints cost ℓᶜ(v). */
  const T& constraints_cost() const { return constraints_cost_; }
  void set_constraints_cost(const T& cost) { constraints_cost_ = cost; }

  /* Returns the total cost ℓ(v) = 0.5 vᵀA v - rᵀv + ℓᶜ(v). */
  const T& cost() const { return cost_; }
  void set_cost(const T& cost) { cost_ = cost; }

  /* Returns the total gradient ∇ℓ(v). Size is num_velocities(). */
  const VectorX<T>& gradient() const { return gradient_; }
  VectorX<T>& mutable_gradient() { return gradient_; }

  /* Returns a mutable scratch space for intermediate computations. We allow
  IcfModel to write on the scratch as needed. */
  Scratch& scratch() const { return scratch_; }

 private:
  VectorX<T> v_;                // Generalized velocities v
  EigenPool<Vector6<T>> V_WB_;  // Rigid body spatial velocities V_WB
  VectorX<T> Av_;               // A⋅v
  T momentum_cost_{0};          // 0.5 vᵀAv - rᵀv
  T constraints_cost_{0};       // ℓᶜ(v)
  T cost_{0};                   // Total cost ℓ(v) = 0.5 vᵀA v - rᵀv + ℓᶜ(v)
  VectorX<T> gradient_;         // Total cost gradient ∇ℓ(v)

  mutable Scratch scratch_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfData);
