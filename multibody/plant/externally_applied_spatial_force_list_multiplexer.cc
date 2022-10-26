#include "drake/multibody/plant/externally_applied_spatial_force_list_multiplexer.h"

#include <algorithm>

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {

template <typename T>
ExternallyAppliedSpatialForceListMultiplexer<T>::
ExternallyAppliedSpatialForceListMultiplexer(int num_inputs)
      : systems::LeafSystem<T>(
          systems::SystemTypeTag<
              ExternallyAppliedSpatialForceListMultiplexer>{}) {
    DRAKE_DEMAND(num_inputs >= 0);
    for (int i = 0; i < num_inputs; ++i) {
      this->DeclareAbstractInputPort(
          systems::kUseDefaultName,
          Value<ListType>());
    }
    this->DeclareAbstractOutputPort(
        "combined",
        &ExternallyAppliedSpatialForceListMultiplexer<T>
            ::CombineInputsToOutput);
  }

template <typename T>
template <typename U>
ExternallyAppliedSpatialForceListMultiplexer<T>::
ExternallyAppliedSpatialForceListMultiplexer(
    const ExternallyAppliedSpatialForceListMultiplexer<U>& other)
    : ExternallyAppliedSpatialForceListMultiplexer(other.num_input_ports()) { }

template <typename T>
void ExternallyAppliedSpatialForceListMultiplexer<T>::CombineInputsToOutput(
    const systems::Context<T>& context,
    ListType* output) const {
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
    class ::drake::multibody::ExternallyAppliedSpatialForceListMultiplexer)
