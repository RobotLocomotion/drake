#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/state.h"

namespace drake {
namespace systems {

/// DiagramState is a State, annotated with pointers to all the mutable
/// substates that it spans.
template <typename T>
class DiagramState : public State<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramState)

  /// Constructs a DiagramState consisting of @p size substates.
  explicit DiagramState<T>(int size);

  /// Sets the substate at @p index to @p substate, or aborts if @p index is
  /// out of bounds. Does not take ownership of @p substate, which must live
  /// as long as this object.
  void set_substate(int index, State<T>* substate) {
    DRAKE_DEMAND(index >= 0 && index < num_substates());
    substates_[index] = substate;
  }

  /// Sets the substate at @p index to @p substate, or aborts if @p index is
  /// out of bounds.
  void set_and_own_substate(int index, std::unique_ptr<State<T>> substate) {
    set_substate(index, substate.get());
    owned_substates_[index] = std::move(substate);
  }

  /// Returns the substate at @p index.
  const State<T>& get_substate(int index) const {
    DRAKE_DEMAND(index >= 0 && index < num_substates());
    return *substates_[index];
  }

  /// Returns the substate at @p index.
  State<T>& get_mutable_substate(int index) {
    DRAKE_DEMAND(index >= 0 && index < num_substates());
    return *substates_[index];
  }

  /// Finalizes this state as a span of all the constituent substates.
  void Finalize();

 private:
  int num_substates() const {
    return static_cast<int>(substates_.size());
  }

  bool finalized_{false};
  std::vector<State<T>*> substates_;
  std::vector<std::unique_ptr<State<T>>> owned_substates_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramState)
