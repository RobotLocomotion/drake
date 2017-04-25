#pragma once

#include <memory>
#include <vector>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/position_kinematics_cache.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace multibody {

/// %MultibodyTreeContext is an object that contains all the information
/// needed to uniquely determine the state of a MultibodyTree.
/// %MultibodyTreeContext provides a collection of convenient access methods to
/// retrieve generalized positions, velocities, cache entries, etc. Users should
/// not need to make calls to these methods directly but rather access
/// portions of a MultibodyTree state through the API provided by
/// the different MultibodyTree elements such as Body, Frame, etc.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
template <typename T>
class MultibodyTreeContext: public systems::LeafContext<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyTreeContext)

  MultibodyTreeContext(const MultibodyTreeTopology& topology) :
      systems::LeafContext<T>(), topology_(topology) {
    using systems::AbstractValue;
    using systems::BasicVector;
    using systems::CacheTicket;
    using systems::Context;
    using systems::ContinuousState;
    using systems::LeafContext;
    using systems::Value;

    // Allocate continuous state.
    const int num_bodies = topology_.get_num_bodies();
    const int num_positions = topology_.num_positions;
    const int num_velocities = topology_.num_velocities;
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

  /// Returns an Eigen expression of the vector of generalized positions.
  Eigen::VectorBlock<const VectorX<T>> get_positions() const {
    return safe_cast<systems::BasicVector<T>>(
        this->get_continuous_state()->get_generalized_position()).get_value();
  }

  /// Returns an Eigen expression of the vector of generalized velocities.
  Eigen::VectorBlock<const VectorX<T>> get_velocities() const {
    return safe_cast<systems::BasicVector<T>>(
        this->get_continuous_state()->get_generalized_velocity()).get_value();
  }

  /// Returns a mutable Eigen expression of the vector of generalized positions.
  Eigen::VectorBlock<VectorX<T>> get_mutable_positions() {
    return safe_cast<systems::BasicVector<T>>(
        this->get_mutable_continuous_state()->
            get_mutable_generalized_position())->get_mutable_value();
  }

  /// Returns a mutable Eigen expression of the vector of generalized
  /// velocities.
  Eigen::VectorBlock<VectorX<T>> get_mutable_velocities() {
    return safe_cast<systems::BasicVector<T>>(
        this->get_mutable_continuous_state()->
            get_mutable_generalized_velocity())->get_mutable_value();
  }

  /// Validates cache entry corresponding to the PositionKinematicsCache.
  void validate_position_kinematics_cache() {
    this->validate_cache_entry(position_kinematics_ticket_);
  }

  /// Returns a constant reference to the position kinematics cache entry.
  /// @throws std::runtime_error if the position kinematics cache entry was not
  /// validated.
  const PositionKinematicsCache<T>& get_position_kinematics() const {
    using systems::AbstractValue;
    using systems::Value;
    DRAKE_ASSERT_VOID(VerifyPositionKinematicsCacheIsValid());
    const AbstractValue* value =
        this->GetCachedValue(position_kinematics_ticket_);
    auto unpacked = safe_cast<const Value<PositionKinematicsCache<T>>>(value);
    return unpacked->get_value();
  }

  /// Returns a mutable reference to the position kinematics cache entry.
  PositionKinematicsCache<T>* get_mutable_position_kinematics() const {
    using systems::AbstractValue;
    using systems::Value;
    AbstractValue* value =
        this->GetMutableCachedValue(position_kinematics_ticket_);
    auto unpacked = safe_cast<Value<PositionKinematicsCache<T>>>(value);
    return &unpacked->get_mutable_value();
  }

 private:
  const MultibodyTreeTopology topology_;
  systems::CacheTicket position_kinematics_ticket_;

  // Helper method to verify if the position kinematics cache is valid and
  // throw an exception if not.
  void VerifyPositionKinematicsCacheIsValid() const {
    if (!this->is_cache_entry_valid(position_kinematics_ticket_)) {
      throw std::runtime_error(
          "Attempting to retrieve an invalidated position kinematics cache"
          " entry.");
    }
  }

  // Helper methods to safely switch between static_cast in Release builds to
  // dynamic_cast in Debug builds.

  // Const type version.
  template<class ToType, class FromType>
  static const ToType& safe_cast(const FromType& from) {
    static_assert(std::is_convertible<ToType*, FromType*>::value,
                  "FromType is not convertible to ToType.");
#ifndef NDEBUG
    return dynamic_cast<const ToType&>(from);
#else
    return static_cast<const ToType&>(from);
#endif
  }

  // Mutable type version.
  template<class ToType, class FromType>
  static ToType* safe_cast(FromType* from) {
    static_assert(std::is_convertible<ToType*, FromType*>::value,
                  "FromType is not convertible to ToType.");
#ifndef NDEBUG
    return dynamic_cast<ToType*>(from);
#else
    return static_cast<ToType*>(from);
#endif
  }
};

}  // namespace multibody
}  // namespace drake
