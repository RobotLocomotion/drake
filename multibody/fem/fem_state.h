#pragma once

#include <memory>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/multibody/fem/fem_indexes.h"
#include "drake/multibody/fem/fem_state_system.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace fem {

// TODO(xuchenhan-tri): Move this class into internal namespace.
/** %FemState provides access to private workspace FEM state and per-element
 state-dependent data. %FemState comes in two flavors, an "owned" version that
 owns the state and data stored in the private workspace, and a "shared" version
 that doesn't own them. Because the "owned" version owns the state, one can
 modify it (e.g. through SetPositions). On the other hand, the "shared" version
 only provides a window into the const state and data, so one cannot modify it.
 For example, calling SetPositions on a "shared" FemState throws an exception. A
 "shared" FemState is cheap to construct and should never persist. It is
 advisable to acquire it for evaluation in a limited scope (e.g., in a calc
 method of a cache entry) and then discard it.
 @tparam_default_scalar */
template <typename T>
class FemState {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemState);

  /** Creates an "owned" version of %FemState that allocates and accesses states
   and cached data using the provided `system`. The %FemState created with this
   constructor owns the states and data.
   @pre system != nullptr. */
  explicit FemState(const internal::FemStateSystem<T>* system);

  /** Creates a "shared" version of %FemState that accesses states and cached
   data in the given `context`. The %FemState created with this constructor
   doesn't own the states and data.
   @pre system != nullptr.
   @pre context != nullptr.
   @pre system and context are compatible. */
  FemState(const internal::FemStateSystem<T>* system,
           const systems::Context<T>* context);

  /** Returns an std::vector of per-element data in `this` %FemState.
   @param[in] cache_index    The cache index of the per-element data.
   @tparam Data the per-element data type.
   @throws std::exception if the per-element data value doesn't actually have
   type `Data`. */
  template <typename Data>
  const std::vector<Data>& EvalElementData(
      const systems::CacheIndex cache_index) const {
    const systems::Context<T>& context = get_context();
    return system_->get_cache_entry(cache_index)
        .template Eval<std::vector<Data>>(context);
  }

  /** @name    Getters and setters for the FEM states
   @anchor fem_state_setters_and_getters
   The FEM states include positions, time step positions (the positions at the
   previous time step t₀), velocities, and accelerations. Positions and
   velocities are actual state variables, while the accelerations are the saved
   result of previous calculations required by certain integrators such as
   NewmarkScheme.
   @{ */
  const VectorX<T>& GetPositions() const;
  const VectorX<T>& GetPreviousStepPositions() const;
  const VectorX<T>& GetVelocities() const;
  const VectorX<T>& GetAccelerations() const;
  void SetPositions(const Eigen::Ref<const VectorX<T>>& q);
  void SetTimeStepPositions(const Eigen::Ref<const VectorX<T>>& q0);
  void SetVelocities(const Eigen::Ref<const VectorX<T>>& v);
  void SetAccelerations(const Eigen::Ref<const VectorX<T>>& a);
  /* @} */

  /** Makes `this` %FemState an exact copy of the given `other` %FemState.
   @throws std::exception if num_dofs() of `this` %FemState and `other`
   %FemState are not the same.
   @throws std::exception if `this` %FemState is not owned (see class
   documentation). */
  void CopyFrom(const FemState<T>& other);

  /** Returns the number of degrees of freedom in the FEM model and state. */
  int num_dofs() const {
    return get_context()
        .get_discrete_state(system_->fem_position_index())
        .size();
  }

  /** Returns the number of nodes in the FEM model. */
  int num_nodes() const { return num_dofs() / 3; }

  /** Returns true if this FemState is constructed from the given system. */
  bool is_created_from_system(const internal::FemStateSystem<T>& system) const {
    return &system == system_;
  }

  /** Returns an identical copy of `this` FemState. */
  std::unique_ptr<FemState<T>> Clone() const;

 private:
  const systems::Context<T>& get_context() const {
    DRAKE_DEMAND((owned_context_ == nullptr) ^ (context_ == nullptr));
    return context_ ? *context_ : *owned_context_;
  }

  systems::Context<T>& get_mutable_context() {
    if (owned_context_ == nullptr)
      throw std::runtime_error("Trying to mutate a shared FemState.");
    return *owned_context_;
  }

  const internal::FemStateSystem<T>* system_{nullptr};
  /* One and only one of `owned_context_` and `context_` should be non-null for
   a given FemState. */
  /* Owned context that contains the FEM states and data. */
  copyable_unique_ptr<systems::Context<T>> owned_context_{nullptr};
  /* Referenced context that contains the FEM states and data. */
  const systems::Context<T>* context_{nullptr};
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake
