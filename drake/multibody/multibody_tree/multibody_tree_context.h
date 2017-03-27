#pragma once

#include <memory>
#include <vector>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/position_kinematics_cache.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace multibody {

/// Right now a dummy class to avoid dependency on systems::Context<T>.
///
template <typename T>
class MultibodyTreeContext: public systems::LeafContext<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyTreeContext)

  MultibodyTreeContext(int num_bodies) : systems::LeafContext<T>() {
    using systems::AbstractValue;
    using systems::BasicVector;
    using systems::CacheTicket;
    using systems::Context;
    using systems::ContinuousState;
    using systems::LeafContext;
    using systems::Value;

    // Allocate continuous state.
    // TODO(amcastro-tri): Allocate this state as needed. Right now this is just
    // a proof of concept assuming all bodies are free. Number of positions
    // and velocities will come from the MultibodyTreeTopology once mobilizers
    // are introduced and the topology is compiled.
    const int num_positions = 6 * num_bodies;
    const int num_velocities = 6 * num_bodies;
    const int num_states = num_positions + num_velocities;

    // TODO(amcastro-tri): Consider to inherit a more specific BasicVector.
    // See EndlessRoadCar<T>::AllocateContinuousState().
    auto xc = std::make_unique<ContinuousState<T>>(
        std::make_unique<BasicVector<T>>(num_states),
        num_positions /* num_q */, num_velocities /* num_v */, 0 /* num_z */);
    this->set_continuous_state(std::move(xc));

    // Creates cache entries in the context.
    // TODO(amcastro-tri): provide dependency on the generalized positions
    // vector.
    position_kinematics_ticket_ = this->CreateCacheEntry({});
    this->InitCachedValue(
        position_kinematics_ticket_,
        std::make_unique<Value<PositionKinematicsCache<T>>>(num_bodies));
  }

  Eigen::VectorBlock<const VectorX<T>> get_positions() const {
    return dynamic_cast<systems::BasicVector<T>&>(
        this->get_continuous_state()->get_generalized_position()).get_value();
  }

  Eigen::VectorBlock<const VectorX<T>> get_velocities() const {
    return dynamic_cast<systems::BasicVector<T>&>(
        this->get_continuous_state()->get_generalized_velocity()).get_value();
  }

  Eigen::VectorBlock<VectorX<T>> get_mutable_positions() {
    return dynamic_cast<systems::BasicVector<T>*>(
        this->get_mutable_continuous_state()->
            get_mutable_generalized_position())->get_mutable_value();
  }

  Eigen::VectorBlock<VectorX<T>> get_mutable_velocities() {
    return dynamic_cast<systems::BasicVector<T>*>(
        this->get_mutable_continuous_state()->
            get_mutable_generalized_velocity())->get_mutable_value();
  }

  void validate_position_kinematics_cache() {
    this->validate_cache_entry(position_kinematics_ticket_);
  }

  const PositionKinematicsCache<T>& get_position_kinematics() const {
    using systems::AbstractValue;
    using systems::Value;
    if (!this->is_cache_entry_valid(position_kinematics_ticket_)) {
      throw std::runtime_error(
          "Attempting to retrieve an invalidated cache entry.");
    }
    const AbstractValue* value =
        this->GetCachedValue(position_kinematics_ticket_);
    const Value<PositionKinematicsCache<T>>* unpacked =
        dynamic_cast<const Value<PositionKinematicsCache<T>>*>(value);
    DRAKE_DEMAND(unpacked != nullptr);
    return unpacked->get_value();
  }

  PositionKinematicsCache<T>* get_mutable_position_kinematics() const {
    using systems::AbstractValue;
    using systems::Value;
    AbstractValue* value =
        this->GetMutableCachedValue(position_kinematics_ticket_);
    Value<PositionKinematicsCache<T>>* unpacked =
        dynamic_cast<Value<PositionKinematicsCache<T>>*>(value);
    DRAKE_DEMAND(unpacked != nullptr);
    return &unpacked->get_mutable_value();
  }

 private:
  systems::CacheTicket position_kinematics_ticket_;
};

}  // namespace multibody
}  // namespace drake
