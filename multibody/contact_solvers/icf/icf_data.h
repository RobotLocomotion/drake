#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/icf/coupler_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"
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
values of v during the optimization process.

For most data elements, we provide a getter and a setter, e.g.,
  const T& cost() const;
  void set_cost(const T& cost);
For elements where setting would require require allocation, we instead offer
a mutable getter, e.g.,
  EigenPool<Vector6<T>>& mutable_V_WB();
Note that mutable getters should only be used for setting values, not for
resizing.

@tparam_nonsymbolic_scalar */
template <typename T>
class IcfData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfData);

  /* Struct to store pre-allocated scratch space. This scratch space is for
  intermediate computations, and is often cleared or overwritten as needed. To
  avoid nested function calls that accidentally overwrite the scratch space of
  previous calls in the stack, we name these variables for their specific uses.

  @warning Accidentally overwriting values in the scratch is not prevented by
  this class. Calling code must ensure that scratch space is used safely. */
  struct Scratch {
    /* Resizes the scratch space, allocating memory as needed. */
    void Resize(int num_bodies, int num_velocities, int max_clique_size,
                int num_couplers, std::span<const int> gain_sizes,
                std::span<const int> limit_sizes,
                std::span<const int> patch_sizes);

    // Scratch space for CalcMomentumTerms. Holds at most one vector of size
    // IcfModel::num_velocities().
    EigenPool<VectorX<T>> Av_minus_r;

    // Scratch space for CalcCostAlongLine.
    // Body spatial velocities at v + α⋅w. Holds at most IcfModel::num_bodies()
    // vectors.
    EigenPool<Vector6<T>> V_WB_alpha;

    // Patch spatial velocities, holds at most
    // PatchConstraintsPool::num_patches() vectors.
    EigenPool<Vector6<T>> U_AbB_W;

    // Generalized velocities at v + α⋅w. Holds at most one vector of size
    // IcfModel::num_velocities().
    EigenPool<VectorX<T>> v_alpha;

    // Scratch space for constraint projection in CalcCostAlongLine. Each of
    // these pools holds at most one vector of size max_clique_size().
    EigenPool<VectorX<T>> Gw_gain;
    EigenPool<VectorX<T>> Gw_limit;

    // Scratch data pools for CalcCostAlongLine.
    CouplerConstraintsDataPool<T> coupler_constraints_data;
    GainConstraintsDataPool<T> gain_constraints_data;
    LimitConstraintsDataPool<T> limit_constraints_data;
    PatchConstraintsDataPool<T> patch_constraints_data;

    // Scratch space for coupler constraints Hessian accumulation. Holds at most
    // one matrix of size max_clique_size() x max_clique_size().
    EigenPool<MatrixX<T>> H_cc_pool;

    // Scratch space for Hessian accumulation. These pools will only hold at
    // most one element, but using pools instead of a single MatrixX<T> allows
    // us to avoid extra heap allocations, as their sizes change frequently.
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
  @param max_clique_size Maximum number of velocities in any clique.
  @param num_couplers Number of coupler constraints.
  @param gain_sizes Number of velocities for each gain constraint.
  @param limit_sizes Number of velocities for each limit constraint.
  @param patch_sizes Number of contact pairs for each patch constraint, of size
                     equal to the number of patches. */
  void Resize(int num_bodies, int num_velocities, int max_clique_size,
              int num_couplers, std::span<const int> gain_sizes,
              std::span<const int> limit_sizes,
              std::span<const int> patch_sizes);

  /* Returns the number of generalized velocities in the system. */
  int num_velocities() const { return v_.size(); }

  /* Returns the generalized velocity vector v. */
  const VectorX<T>& v() const { return v_; }

  /* Sets the generalized velocity vector v. In debug builds, sets member data
  (other than v) to NaN to enforce that any downstream quantities are recomputed
  after v changes. */
  void set_v(const VectorX<T>& v);

  /* Returns the pool of rigid body spatial velocities, V_WB. Size is
  num_bodies().

  N.B. We use Vector6<T> rather than SpatialVelocity<T> for compatibility with
  EigenPool, though these represent the same quantities. The first three entries
  are angular velocity, and the last three entries are linear velocity. */
  const EigenPool<Vector6<T>>& V_WB() const { return V_WB_; }
  EigenPool<Vector6<T>>& mutable_V_WB() { return V_WB_; }

  /* Returns the linearized dynamics matrix times velocities, A⋅v. Size is
  num_velocities().*/
  const VectorX<T>& Av() const { return Av_; }
  VectorX<T>& mutable_Av() { return Av_; }

  /* Returns the momentum cost 0.5 vᵀA v - rᵀv. */
  const T& momentum_cost() const { return momentum_cost_; }
  void set_momentum_cost(const T& cost) { momentum_cost_ = cost; }

  /* Returns the total cost ℓ(v) = 0.5 vᵀA v - rᵀv + ℓᶜ(v). */
  const T& cost() const { return cost_; }
  void set_cost(const T& cost) { cost_ = cost; }

  /* Returns the total gradient ∇ℓ(v). Size is num_velocities(). */
  const VectorX<T>& gradient() const { return gradient_; }
  VectorX<T>& mutable_gradient() { return gradient_; }

  /* Returns the data pool for coupler constraints. */
  const CouplerConstraintsDataPool<T>& coupler_constraints_data() const {
    return coupler_constraints_data_;
  }
  CouplerConstraintsDataPool<T>& mutable_coupler_constraints_data() {
    return coupler_constraints_data_;
  }

  /* Returns the data pool for external gain (e.g., actuation) constraints. */
  const GainConstraintsDataPool<T>& gain_constraints_data() const {
    return gain_constraints_data_;
  }
  GainConstraintsDataPool<T>& mutable_gain_constraints_data() {
    return gain_constraints_data_;
  }

  /* Returns the data pool for joint limit constraints. */
  const LimitConstraintsDataPool<T>& limit_constraints_data() const {
    return limit_constraints_data_;
  }
  LimitConstraintsDataPool<T>& mutable_limit_constraints_data() {
    return limit_constraints_data_;
  }

  /* Returns the data pool for contact constraints. */
  const PatchConstraintsDataPool<T>& patch_constraints_data() const {
    return patch_constraints_data_;
  }
  PatchConstraintsDataPool<T>& mutable_patch_constraints_data() {
    return patch_constraints_data_;
  }

  /* Returns a mutable scratch space for intermediate computations. We allow
  IcfModel to write on the scratch as needed. */
  Scratch& scratch() const { return scratch_; }

 private:
  VectorX<T> v_;                // Generalized velocities v
  EigenPool<Vector6<T>> V_WB_;  // Rigid body spatial velocities V_WB
  VectorX<T> Av_;               // A⋅v
  T momentum_cost_{0};          // 0.5 vᵀAv - rᵀv
  T cost_{0};                   // Total cost ℓ(v) = 0.5 vᵀA v - rᵀv + ℓᶜ(v)
  VectorX<T> gradient_;         // Total cost gradient ∇ℓ(v)

  // Type-specific constraint pools.
  CouplerConstraintsDataPool<T> coupler_constraints_data_;
  GainConstraintsDataPool<T> gain_constraints_data_;
  LimitConstraintsDataPool<T> limit_constraints_data_;
  PatchConstraintsDataPool<T> patch_constraints_data_;

  mutable Scratch scratch_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfData);
