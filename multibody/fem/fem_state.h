#pragma once

#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/multibody/fem/fem_indexes.h"
#include "drake/multibody/fem/fem_state_manager.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace fem {

/** %FemState provides access to private workspace FEM state and per-element
 state-dependent data.
 @tparam_nonsymbolic_scalar */
template <typename T>
class FemState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemState);

  /** Creates an %FemState that allocates and accesses states and cached data
   using the provided `state_info`. */
  explicit FemState(const internal::FemStateManager<T>* fem_state_manager);

  /** Returns per-element data in `this` %FemState.
   @tparam Data the per-element data type.
   @throws std::exception if the per-element data value doesn't actually have
   type V.
   @throws std::exception if `element_index` is larger than the number of
   elements in the FEM model. */
  template <typename Data>
  const Data& EvalElementData(FemElementIndex element_index) const {
    const auto& element_data =
        manager_->get_cache_entry(manager_->element_data_index())
            .template Eval<std::vector<Data>>(*context_);
    DRAKE_THROW_UNLESS(element_index < element_data.size());
    return element_data[element_index];
  }

  /** @name    Getters and setters for the FEM states
   @anchor setters_and_getters
   The FEM states include positions, velocities and accelerations.
   @{ */
  const VectorX<T>& GetPositions() const;
  const VectorX<T>& GetVelocities() const;
  const VectorX<T>& GetAccelerations() const;
  void SetPositions(const Eigen::Ref<const VectorX<T>>& q);
  void SetVelocities(const Eigen::Ref<const VectorX<T>>& v);
  void SetAccelerations(const Eigen::Ref<const VectorX<T>>& a);
  /* @} */

  /** Returns the number of degrees of freedoms in the FEM model and state. */
  int num_dofs() const {
    return context_->get_discrete_state(manager_->fem_position_index()).size();
  }

 private:
  const internal::FemStateManager<T>* manager_;
  /* Owned contexts that contains the FEM states and data. */
  copyable_unique_ptr<systems::Context<T>> context_{nullptr};
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake
