#include "drake/systems/framework/diagram_state.h"

#include "drake/systems/framework/diagram_continuous_state.h"
#include "drake/systems/framework/diagram_discrete_values.h"

namespace drake {
namespace systems {

template <typename T>
DiagramState<T>::DiagramState(int size)
    : State<T>(),
      substates_(size),
      owned_substates_(size) {}

template <typename T>
void DiagramState<T>::Finalize() {
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

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramState)
