#pragma once

#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/position_kinematics_cache.h"
#include "drake/multibody/multibody_tree/velocity_kinematics_cache.h"
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

  explicit MultibodyTreeContext(const MultibodyTreeTopology& topology) :
      systems::LeafContext<T>(), topology_(topology) {
    using systems::AbstractValue;
    using systems::BasicVector;
    using systems::CacheTicket;
    using systems::Context;
    using systems::ContinuousState;
    using systems::LeafContext;
    using systems::Value;

    // Allocate continuous state.
    const int num_positions = topology_.get_num_positions();
    const int num_velocities = topology_.get_num_velocities();
    const int num_states = topology_.get_num_states();

    // TODO(amcastro-tri): Consider inheriting a more specific BasicVector.
    // See EndlessRoadCar<T>::AllocateContinuousState().
    auto xc = std::make_unique<ContinuousState<T>>(
        std::make_unique<BasicVector<T>>(num_states),
        num_positions, num_velocities, 0);
    this->set_continuous_state(std::move(xc));

    // TODO(amcastro-tri): Create cache entries.
    // For instance, for PositionKinematicsCache so that it doesn't get
    // re-allocated and re-computed every time is needed.
  }

  /// Returns the size of the generalized positions vector.
  int get_num_positions() const {
    return this->get_continuous_state().get_generalized_position().size();
  }

  /// Returns the size of the generalized velocities vector.
  int get_num_velocities() const {
    return this->get_continuous_state().get_generalized_velocity().size();
  }

  /// Returns an Eigen expression of the vector of generalized positions.
  Eigen::VectorBlock<const VectorX<T>> get_positions() const {
    return get_state_segment(0, get_num_positions());
  }

  /// Returns an Eigen expression of the vector of generalized velocities.
  Eigen::VectorBlock<const VectorX<T>> get_velocities() const {
    return get_state_segment(get_num_positions(), get_num_velocities());
  }

  /// Returns a mutable Eigen expression of the vector of generalized positions.
  Eigen::VectorBlock<VectorX<T>> get_mutable_positions() {
    return get_mutable_state_segment(0, get_num_positions());
  }

  /// Returns a mutable Eigen expression of the vector of generalized
  /// velocities.
  Eigen::VectorBlock<VectorX<T>> get_mutable_velocities() {
    return get_mutable_state_segment(get_num_positions(), get_num_velocities());
  }

  /// Returns a const fixed-size Eigen::VectorBlock of `count` elements
  /// referencing a segment in the state vector with its first element
  /// at `start`.
  template <int count>
  Eigen::VectorBlock<const VectorX<T>, count> get_state_segment(
      int start) const {
    // We know that MultibodyTreeContext is a LeafContext and therefore the
    // continuous state vector must be a BasicVector.
    // TODO(amcastro-tri): make use of VectorBase::get_contiguous_vector() once
    // PR #6049 gets merged.
    Eigen::VectorBlock<const VectorX<T>> xc =
        dynamic_cast<const systems::BasicVector<T>&>(
            this->get_continuous_state().get_vector()).get_value();
    // xc.nestedExpression() resolves to "VectorX<T>&" since the continuous
    // state is a BasicVector.
    // If we do return xc.segment() directly, we would instead get a
    // Block<Block<VectorX>>, which is very different from Block<VectorX>.
    return xc.nestedExpression().template segment<count>(start);
  }

  /// Returns a mutable fixed-size Eigen::VectorBlock of `count` elements
  /// referencing a segment in the state vector with its first element
  /// at `start`.
  template <int count>
  Eigen::VectorBlock<VectorX<T>, count> get_mutable_state_segment(int start) {
    // We know that MultibodyTreeContext is a LeafContext and therefore the
    // continuous state vector must be a BasicVector.
    // TODO(amcastro-tri): make use of VectorBase::get_contiguous_vector() once
    // PR #6049 gets merged.
    Eigen::VectorBlock<VectorX<T>> xc =
        dynamic_cast<systems::BasicVector<T>&>(
            this->get_mutable_continuous_state().get_mutable_vector()).
            get_mutable_value();
    // xc.nestedExpression() resolves to "VectorX<T>&" since the continuous
    // state is a BasicVector.
    // If we do return xc.segment() directly, we would instead get a
    // Block<Block<VectorX>>, which is very different from Block<VectorX>.
    return xc.nestedExpression().template segment<count>(start);
  }

  /// Returns a const fixed-size Eigen::VectorBlock of `count` elements
  /// referencing a segment in the state vector with its first element
  /// at `start`.
  Eigen::VectorBlock<const VectorX<T>> get_state_segment(
      int start, int count) const {
    // We know that MultibodyTreeContext is a LeafContext and therefore the
    // continuous state vector must be a BasicVector.
    // TODO(amcastro-tri): make use of VectorBase::get_contiguous_vector() once
    // PR #6049 gets merged.
    Eigen::VectorBlock<const VectorX<T>> xc =
        dynamic_cast<const systems::BasicVector<T>&>(
            this->get_continuous_state().get_vector()).get_value();
    // xc.nestedExpression() resolves to "const VectorX<T>&" since the
    // continuous state is a BasicVector.
    // If we do return xc.segment() directly, we would instead get a
    // Block<Block<VectorX>>, which is very different from Block<VectorX>.
    return xc.nestedExpression().segment(start, count);
  }

  /// Returns a mutable fixed-size Eigen::VectorBlock of `count` elements
  /// referencing a segment in the state vector with its first element
  /// at `start`.
  Eigen::VectorBlock<VectorX<T>> get_mutable_state_segment(
      int start, int count) {
    // We know that MultibodyTreeContext is a LeafContext and therefore the
    // continuous state vector must be a BasicVector.
    // TODO(amcastro-tri): make use of VectorBase::get_contiguous_vector() once
    // PR #6049 gets merged.
    Eigen::VectorBlock<VectorX<T>> xc =
        dynamic_cast<systems::BasicVector<T>&>(
            this->get_mutable_continuous_state().get_mutable_vector()).
            get_mutable_value();
    // xc.nestedExpression() resolves to "VectorX<T>&" since the continuous
    // state is a BasicVector.
    // If we do return xc.segment() directly, we would instead get a
    // Block<Block<VectorX>>, which is very different from Block<VectorX>.
    return xc.nestedExpression().segment(start, count);
  }

 private:
  const MultibodyTreeTopology topology_;
};

}  // namespace multibody
}  // namespace drake
