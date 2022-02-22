#pragma once

#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/multibody/fem/fem_indexes.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace fem {

namespace internal {

// TODO(xuchenhan-tri): Consider using a factory method to ensure FemStateInfo
//  is valid.
/* FemStateInfo contains information required to allocate and access FEM
 states and cache entries. Usually created with FemModel::AllocateFemState.
 @tparam_nonsymbolic_scalar */
template <typename T>
struct FemStateInfo {
  /* The system that manages the discrete states and cache entries in the FEM
   state. It must have the following discrete states and cache entry allocated.
  */
  const systems::LeafSystem<T>* system{nullptr};
  /* State and cache indexes. */
  systems::DiscreteStateIndex fem_position_index;
  systems::DiscreteStateIndex fem_velocity_index;
  systems::DiscreteStateIndex fem_acceleration_index;
  systems::CacheIndex element_data_index;
  /* The model ID of the model that consumes the state and the cached data. */
  FemModelId model_id;
};

}  // namespace internal

/** %FemState provides access to private workspace FEM state and per-element
 state-dependent data.
 @tparam_nonsymbolic_scalar */
template <typename T>
class FemState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemState);

  /** Creates an %FemState that allocates and accesses states and cached data
   using the provided `state_info`. */
  explicit FemState(const internal::FemStateInfo<T>& state_info);

  /** Returns per-element data in `this` %FemState.
   @tparam Data the per-element data type.
   @throws std::exception if the per-element data value doesn't actually have
   type V.
   @throws std::exception if `element_index` is larger than the number of
   elements in the FEM model. */
  template <typename Data>
  const Data& EvalElementData(FemElementIndex element_index) const {
    const auto& element_data =
        info_.system->get_cache_entry(info_.element_data_index)
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
    return context_->get_discrete_state(info_.fem_position_index).size();
  }

  /** Returns the identifier of the FEM model that creates and consumes the
   states and data in `this` %FemState. */
  FemModelId model_id() const { return info_.model_id; }

 private:
  internal::FemStateInfo<T> info_;
  /* Owned contexts that contains the FEM states and data. */
  copyable_unique_ptr<systems::Context<T>> context_{nullptr};
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake
