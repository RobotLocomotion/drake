#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/pointer_cast.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/supervector.h"

namespace drake {
namespace systems {

/// %DiagramContinuousState is a ContinuousState consisting of Supervectors
/// xc, q, v, z over the corresponding entries in a set of referenced
/// ContinuousState objects, which may or may not be owned by this
/// %DiagramContinuousState. This is done recursively since any of the
/// referenced ContinuousState objects could themselves be
/// %DiagramContinuousState objects. The actual numerical data is always
/// contained in the leaf ContinuousState objects at the bottom of the tree.
///
/// This object is used both for a Diagram's actual continuous state variables
/// xc (with partitions q, v, z) and for the time derivatives xdot (qdot, vdot,
/// zdot). Cloning a %DiagramContinuousState results in an object with identical
/// structure, but which owns the referenced ContinuousState objects, regardless
/// of whether the original had ownership.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
class DiagramContinuousState final: public ContinuousState<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramContinuousState)

  /// Constructs a ContinuousState that is composed of other ContinuousStates,
  /// which are not owned by this object and must outlive it.
  ///
  /// The DiagramContinuousState vector xc = [q v z] will have the same
  /// ordering as the `substates` parameter, which should be the order of
  /// the Diagram itself. That is, the substates should be indexed by
  /// SubsystemIndex in the same order as the subsystems are. */
  explicit DiagramContinuousState(std::vector<ContinuousState<T>*> substates)
      : ContinuousState<T>(
            Span(substates, x_selector), Span(substates, q_selector),
            Span(substates, v_selector), Span(substates, z_selector)),
        substates_(std::move(substates)) {
    DRAKE_ASSERT(internal::IsNonNull(substates_));
  }

  /// Constructs a ContinuousState that is composed (recursively) of other
  /// ContinuousState objects, ownership of which is transferred here.
  explicit DiagramContinuousState(
      std::vector<std::unique_ptr<ContinuousState<T>>> substates)
      : DiagramContinuousState<T>(internal::Unpack(substates)) {
      owned_substates_ = std::move(substates);
    DRAKE_ASSERT(internal::IsNonNull(owned_substates_));
  }

  ~DiagramContinuousState() override {}

  /// Creates a deep copy of this %DiagramContinuousState, with the same
  /// substructure but with new, owned data. Intentionally shadows the
  /// ContinuousState::Clone() method but with a more-specific return type so
  /// you don't have to downcast.
  std::unique_ptr<DiagramContinuousState> Clone() const {
    // Note that DoClone() below cannot be overridden so we can count on the
    // concrete type being DiagramContinuousState.
    return static_pointer_cast<DiagramContinuousState>(DoClone());
  }

  int num_substates() const { return static_cast<int>(substates_.size()); }

  /// Returns the continuous state at the given `index`. Aborts if `index` is
  /// out-of-bounds.
  const ContinuousState<T>& get_substate(int index) const {
    DRAKE_DEMAND(0 <= index && index < num_substates());
    DRAKE_DEMAND(substates_[index] != nullptr);
    return *substates_[index];
  }

  /// Returns the continuous state at the given `index`. Aborts if `index` is
  /// out-of-bounds.
  ContinuousState<T>& get_mutable_substate(int index) {
    return const_cast<ContinuousState<T>&>(get_substate(index));
  }

 private:
  // This completely replaces the base class default DoClone() so must
  // take care of the base class members as well as the local ones.
  // The returned concrete object is a DiagramContinuousState<T>.
  std::unique_ptr<ContinuousState<T>> DoClone() const final {
    std::vector<std::unique_ptr<ContinuousState<T>>> owned_states;
    // Make deep copies regardless of whether they were owned.
    for (auto state : substates_) owned_states.push_back(state->Clone());
    return std::make_unique<DiagramContinuousState>(std::move(owned_states));
  }

  // Returns a Supervector over the x, q, v, or z components of each
  // substate in `substates`, as indicated by `selector`.
  static std::unique_ptr<VectorBase<T>> Span(
      const std::vector<ContinuousState<T>*>& substates,
      std::function<VectorBase<T>&(ContinuousState<T>*)> selector) {
    std::vector<VectorBase<T>*> sub_xs;
    for (const auto& substate : substates) {
      DRAKE_DEMAND(substate != nullptr);
      sub_xs.push_back(&selector(substate));
    }
    return std::make_unique<Supervector<T>>(sub_xs);
  }

  // Returns the entire state vector in `xc`.
  static VectorBase<T>& x_selector(ContinuousState<T>* xc) {
    return xc->get_mutable_vector();
  }
  // Returns the generalized position vector in `xc`.
  static VectorBase<T>& q_selector(ContinuousState<T>* xc) {
    return xc->get_mutable_generalized_position();
  }
  // Returns the generalized velocity vector in `xc`.
  static VectorBase<T>& v_selector(ContinuousState<T>* xc) {
    return xc->get_mutable_generalized_velocity();
  }
  // Returns the misc continuous state vector in `xc`.
  static VectorBase<T>& z_selector(ContinuousState<T>* xc) {
    return xc->get_mutable_misc_continuous_state();
  }

  // Pointers to the underlying ContinuousStates that provide the actual
  // values. If these are owned, the pointers are equal to the pointers in
  // owned_substates_.
  std::vector<ContinuousState<T>*> substates_;
  // Owned pointers to ContinuousState objects that hold the actual values.
  // The only purpose of these pointers is to maintain ownership. They may be
  // populated at construction time, and are never accessed thereafter.
  std::vector<std::unique_ptr<ContinuousState<T>>> owned_substates_;
};

}  // namespace systems
}  // namespace drake
