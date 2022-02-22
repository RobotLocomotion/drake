#pragma once

#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/multibody/fem/fem_indexes.h"
#include "drake/multibody/fem/fem_state_manager.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace fem {

/** %FemState provides access to private workspace FEM state and per-element
 state-dependent data. %FemState comes in two flavors, a "baked" version that
 owns the state and data stored in the private workspace, and a "live" version
 that doesn't own them. Becuase the "baked" version owns the state, one can
 modify it (e.g. through SetPositions). On the other hand, the "live" version
 only provides a window into the const state and data, so one cannot modify it.
 For example, calling SetPositions on a "live" FemState throws an exception. A
 "live" FemState is cheap to construct and should never persist. It is advisable
 to acquire it for evaluation in a limited scope (e.g., in a calc method of a
 cache entry) and then discard it.
 @tparam_nonsymbolic_scalar */
template <typename T>
class FemState {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemState);

  /** Creates a "baked" version of %FemState that allocates and accesses states
   and cached data using the provided `manager`. The %FemState created with this
   constructor owns the states and data.
   @pre manager != nullptr. */
  explicit FemState(const internal::FemStateManager<T>* manager);

  /** Creates a  "live" version of %FemState that accesses states and cached
   data in the given `context`. The %FemState created with this constructor
   doesn't own the states and data.
   @pre manager != nullptr.
   @pre context != nullptr.
   @pre manager and context are compatible. */
  FemState(const internal::FemStateManager<T>* manager,
           const systems::Context<T>* context);

  /** Returns per-element data in `this` %FemState.
   @param[in] cache_index    The cache index of the per-element data.
   @param[in] element_index  The element for which the data is evaluated.
   @tparam Data the per-element data type.
   @throws std::exception if the per-element data value doesn't actually have
   type `Data`.
   @throws std::exception if `element_index` is larger than the number of
   elements in the FEM model. */
  template <typename Data>
  const Data& EvalElementData(const systems::CacheIndex cache_index,
                              FemElementIndex element_index) const {
    const systems::Context<T>& context = get_context();
    const auto& element_data = manager_->get_cache_entry(cache_index)
                                   .template Eval<std::vector<Data>>(context);
    DRAKE_THROW_UNLESS(element_index < element_data.size());
    return element_data[element_index];
  }

  /** @name    Getters and setters for the FEM states
   @anchor fem_state_setters_and_getters
   The FEM states include positions, velocities and accelerations.
   @{ */
  const VectorX<T>& GetPositions() const;
  const VectorX<T>& GetVelocities() const;
  const VectorX<T>& GetAccelerations() const;
  void SetPositions(const Eigen::Ref<const VectorX<T>>& q);
  void SetVelocities(const Eigen::Ref<const VectorX<T>>& v);
  void SetAccelerations(const Eigen::Ref<const VectorX<T>>& a);
  /* @} */

  /** Returns the number of degrees of freedom in the FEM model and state. */
  int num_dofs() const {
    return get_context()
        .get_discrete_state(manager_->fem_position_index())
        .size();
  }

 private:
  const systems::Context<T>& get_context() const {
    DRAKE_DEMAND((owned_context_ == nullptr) ^ (context_ == nullptr));
    return context_ ? *context_ : *owned_context_;
  }

  systems::Context<T>& get_mutable_context() {
    if (owned_context_ == nullptr)
      throw std::runtime_error("Trying to mutate a live FemState.");
    return *owned_context_;
  }

  const internal::FemStateManager<T>* manager_{nullptr};
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
