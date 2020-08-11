#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram_continuous_state.h"
#include "drake/systems/framework/diagram_discrete_values.h"
#include "drake/systems/framework/framework_common.h"
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
  explicit DiagramState<T>(int size) :
      State<T>(),
      substates_(size),
      owned_substates_(size) {}

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
  void Finalize() {
    DRAKE_DEMAND(!finalized_);
    finalized_ = true;
    std::vector<ContinuousState<T>*> sub_xcs;
    sub_xcs.reserve(num_substates());
    std::vector<DiscreteValues<T>*> sub_xds;
    std::vector<AbstractValue*> sub_xas;
    for (State<T>* substate : substates_) {
      // Continuous
      sub_xcs.push_back(&substate->get_mutable_continuous_state());
      // Discrete
      sub_xds.push_back(&substate->get_mutable_discrete_state());
      // Abstract (no substructure)
      AbstractValues& xa = substate->get_mutable_abstract_state();
      for (int i_xa = 0; i_xa < xa.size(); ++i_xa) {
        sub_xas.push_back(&xa.get_mutable_value(i_xa));
      }
    }

    // This State consists of a continuous, discrete, and abstract state, each
    // of which is a spanning vector over the continuous, discrete, and abstract
    // parts of the constituent states.  The spanning vectors do not own any
    // of the actual memory that contains state variables. They just hold
    // pointers to that memory.
    this->set_continuous_state(
        std::make_unique<DiagramContinuousState<T>>(sub_xcs));
    this->set_discrete_state(
        std::make_unique<DiagramDiscreteValues<T>>(sub_xds));
    this->set_abstract_state(std::make_unique<AbstractValues>(sub_xas));
  }

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
