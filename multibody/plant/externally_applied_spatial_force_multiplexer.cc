#include "drake/multibody/plant/externally_applied_spatial_force_multiplexer.h"

#include <algorithm>

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {

template <typename T>
ExternallyAppliedSpatialForceMultiplexer<
    T>::ExternallyAppliedSpatialForceMultiplexer(int num_inputs)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<ExternallyAppliedSpatialForceMultiplexer>{}) {
  DRAKE_DEMAND(num_inputs >= 0);
  for (int i = 0; i < num_inputs; ++i) {
    this->DeclareAbstractInputPort(systems::kUseDefaultName, Value<ListType>());
  }
  this->DeclareAbstractOutputPort(
      systems::kUseDefaultName,
      &ExternallyAppliedSpatialForceMultiplexer<T>::CombineInputsToOutput);
}

template <typename T>
template <typename U>
ExternallyAppliedSpatialForceMultiplexer<T>::
    ExternallyAppliedSpatialForceMultiplexer(
        const ExternallyAppliedSpatialForceMultiplexer<U>& other)
    : ExternallyAppliedSpatialForceMultiplexer(other.num_input_ports()) {}

template <typename T>
ExternallyAppliedSpatialForceMultiplexer<
    T>::~ExternallyAppliedSpatialForceMultiplexer() = default;

template <typename T>
void ExternallyAppliedSpatialForceMultiplexer<T>::CombineInputsToOutput(
    const systems::Context<T>& context, ListType* output) const {
  output->clear();
  for (int i = 0; i < this->num_input_ports(); ++i) {
    const ListType& values_i =
        this->get_input_port(i).template Eval<ListType>(context);
    std::copy(values_i.begin(), values_i.end(), std::back_inserter(*output));
  }
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ExternallyAppliedSpatialForceMultiplexer);
