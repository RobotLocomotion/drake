#include "drake/systems/framework/diagram_continuous_state.h"

#include <utility>

#include "drake/common/pointer_cast.h"
#include "drake/systems/framework/supervector.h"

namespace drake {
namespace systems {

template <typename T>
DiagramContinuousState<T>::DiagramContinuousState(
    std::vector<ContinuousState<T>*> substates)
    : ContinuousState<T>(
          Span(substates, x_selector), Span(substates, q_selector),
          Span(substates, v_selector), Span(substates, z_selector)),
      substates_(std::move(substates)) {
  DRAKE_ASSERT(internal::IsNonNull(substates_));
}

template <typename T>
DiagramContinuousState<T>::DiagramContinuousState(
    std::vector<std::unique_ptr<ContinuousState<T>>> substates)
    : DiagramContinuousState<T>(internal::Unpack(substates)) {
  owned_substates_ = std::move(substates);
  DRAKE_ASSERT(internal::IsNonNull(owned_substates_));
}

template <typename T>
DiagramContinuousState<T>::~DiagramContinuousState() {}

template <typename T>
std::unique_ptr<DiagramContinuousState<T>>
DiagramContinuousState<T>::Clone() const {
  // We are sure of the type here because DoClone() is final.  However,
  // we'll still use the `..._or_throw` spelling as a sanity check.
  return dynamic_pointer_cast_or_throw<DiagramContinuousState>(
      ContinuousState<T>::Clone());
}

template <typename T>
std::unique_ptr<ContinuousState<T>> DiagramContinuousState<T>::DoClone() const {
  std::vector<std::unique_ptr<ContinuousState<T>>> owned_states;
  // Make deep copies regardless of whether they were owned.
  for (auto state : substates_) owned_states.push_back(state->Clone());
  return std::make_unique<DiagramContinuousState>(std::move(owned_states));
}

template <typename T>
std::unique_ptr<VectorBase<T>> DiagramContinuousState<T>::Span(
    const std::vector<ContinuousState<T>*>& substates,
    std::function<VectorBase<T>&(ContinuousState<T>*)> selector) {
  std::vector<VectorBase<T>*> sub_xs;
  for (const auto& substate : substates) {
    DRAKE_DEMAND(substate != nullptr);
    sub_xs.push_back(&selector(substate));
  }
  return std::make_unique<Supervector<T>>(sub_xs);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramContinuousState)
