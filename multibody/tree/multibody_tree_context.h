#pragma once

#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace multibody {
namespace internal {

// TODO(sherm1) Remove this class and just use a LeafContext directly.
/// %MultibodyTreeContext is an object that contains all the information
/// needed to uniquely determine the state of a MultibodyTree.
/// %MultibodyTreeContext provides a collection of convenient access methods to
/// retrieve generalized positions and velocities. Users should
/// not need to make calls to these methods directly but rather access
/// portions of a MultibodyTree state through the API provided by
/// the different MultibodyTree elements such as Body, Frame, etc.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
template <typename T>
class MultibodyTreeContext final : public systems::LeafContext<T> {
 public:
  /// @name Does not allow copy, move, or assignment.
  //@{
  // Copy constructor is private for use in implementing Clone().
  MultibodyTreeContext(MultibodyTreeContext&&) = delete;
  MultibodyTreeContext& operator=(const MultibodyTreeContext&) = delete;
  MultibodyTreeContext& operator=(MultibodyTreeContext&&) = delete;
  //@}

  /// Instantiates a %MultibodyTreeContext for a MultibodyTree with a given
  /// `topology`. The stored state is continuous.
  explicit MultibodyTreeContext(const MultibodyTreeTopology& topology) :
      MultibodyTreeContext(topology, false) {}

  /// Instantiates a %MultibodyTreeContext for a MultibodyTree with a given
  /// `topology`.
  /// @param[in] discrete_state
  ///   `true` if the state is to be stored as discrete state. Otherwise the
  ///   state is stored as continuous state. `false` if the state is to be
  ///   stored as continuous state.
  MultibodyTreeContext(
      const MultibodyTreeTopology& topology, bool discrete_state) :
      systems::LeafContext<T>(),
      topology_(topology),
      is_state_discrete_(discrete_state) {
  }

  std::unique_ptr<systems::ContextBase> DoCloneWithoutPointers() const final {
    return std::unique_ptr<systems::ContextBase>(
        new MultibodyTreeContext<T>(*this));
  }

  /// Returns the size of the generalized positions vector.
  int num_positions() const {
    return topology_.num_positions();
  }

  /// Returns the size of the generalized velocities vector.
  int num_velocities() const {
    return topology_.num_velocities();
  }

  /// Returns `true` if the state is discrete and `false` if the state is
  /// continuous.
  bool is_state_discrete() const {
    return is_state_discrete_;
  }

  /// Returns a const reference to the state vector stored in `this` context as
  /// an `Eigen::VectorBlock<const VectorX<T>>`.
  Eigen::VectorBlock<const VectorX<T>> get_state_vector() const {
    DRAKE_ASSERT(this->get_num_discrete_state_groups() <= 1);
    const systems::BasicVector<T>& state_vector =
        is_state_discrete() ? this->get_discrete_state(0) :
        dynamic_cast<const systems::BasicVector<T>&>(
            this->get_continuous_state().get_vector());
    return state_vector.get_value();
  }

  /// Returns a mutable reference to the state vector stored in `this` context
  /// as an `Eigen::VectorBlock<VectorX<T>>`.
  Eigen::VectorBlock<VectorX<T>> get_mutable_state_vector() {
    return get_mutable_state_vector(&this->get_mutable_state());
  }

  /// Returns a mutable reference to the state vector stored in `state` which
  /// must be the state associated with `this` context, as an
  /// `Eigen::VectorBlock<VectorX<T>>`.
  /// @pre `state` must be a systems::State<T> created by the same System
  /// that created this Context.
  Eigen::VectorBlock<VectorX<T>> get_mutable_state_vector(
      systems::State<T>* state) const {
    DRAKE_ASSERT(this->get_num_discrete_state_groups() <= 1);
    DRAKE_ASSERT(is_state_discrete() ==
                 (this->get_num_discrete_state_groups() == 1));
    systems::BasicVector<T>& state_vector =
        is_state_discrete() ? state->get_mutable_discrete_state(0) :
        dynamic_cast<systems::BasicVector<T>&>(
            state->get_mutable_continuous_state().get_mutable_vector());
    return state_vector.get_mutable_value();
  }

  /// Returns an Eigen expression of the vector of generalized positions.
  Eigen::VectorBlock<const VectorX<T>> get_positions() const {
    return get_state_segment(0, num_positions());
  }

  /// Returns an Eigen expression of the vector of generalized velocities.
  Eigen::VectorBlock<const VectorX<T>> get_velocities() const {
    return get_state_segment(num_positions(), num_velocities());
  }

  /// Returns a mutable Eigen expression of the vector of generalized positions.
  Eigen::VectorBlock<VectorX<T>> get_mutable_positions() {
    return get_mutable_state_segment(0, num_positions());
  }

  /// Returns a mutable Eigen expression of the vector of generalized
  /// velocities.
  Eigen::VectorBlock<VectorX<T>> get_mutable_velocities() {
    return get_mutable_state_segment(num_positions(), num_velocities());
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
    Eigen::VectorBlock<const VectorX<T>> x = get_state_vector();
    // xc.nestedExpression() resolves to "VectorX<T>&" since the continuous
    // state is a BasicVector.
    // If we do return xc.segment() directly, we would instead get a
    // Block<Block<VectorX>>, which is very different from Block<VectorX>.
    return x.nestedExpression().template segment<count>(start);
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
    Eigen::VectorBlock<VectorX<T>> x = get_mutable_state_vector();
    // xc.nestedExpression() resolves to "VectorX<T>&" since the continuous
    // state is a BasicVector.
    // If we do return xc.segment() directly, we would instead get a
    // Block<Block<VectorX>>, which is very different from Block<VectorX>.
    return x.nestedExpression().template segment<count>(start);
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
    Eigen::VectorBlock<const VectorX<T>> x = get_state_vector();
    // xc.nestedExpression() resolves to "const VectorX<T>&" since the
    // continuous state is a BasicVector.
    // If we do return xc.segment() directly, we would instead get a
    // Block<Block<VectorX>>, which is very different from Block<VectorX>.
    return x.nestedExpression().segment(start, count);
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
    Eigen::VectorBlock<VectorX<T>> x = get_mutable_state_vector();
    // xc.nestedExpression() resolves to "VectorX<T>&" since the continuous
    // state is a BasicVector.
    // If we do return xc.segment() directly, we would instead get a
    // Block<Block<VectorX>>, which is very different from Block<VectorX>.
    return x.nestedExpression().segment(start, count);
  }

 private:
  MultibodyTreeContext(const MultibodyTreeContext& source) = default;

  const MultibodyTreeTopology topology_;

  // If `true`, this context stores a discrete state. If `false` the state is
  // stored as continuous state.
  bool is_state_discrete_{false};
};

}  // namespace internal

/// WARNING: This will be removed on or around 2019/03/01.
template <typename T>
using MultibodyTreeContext
DRAKE_DEPRECATED(
    "This public alias is deprecated, and will be removed around 2019/03/01.")
    = internal::MultibodyTreeContext<T>;

}  // namespace multibody
}  // namespace drake
