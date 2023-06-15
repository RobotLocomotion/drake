#include "drake/systems/framework/leaf_output_port.h"

namespace drake::systems {

template <typename T>
void LeafOutputPort<T>::ThrowIfInvalidPortValueType(
    const Context<T>& context, const AbstractValue& proposed) const {
  const CacheEntryValue& cache_value =
      cache_entry().get_cache_entry_value(context);
  const AbstractValue& value = cache_value.PeekAbstractValueOrThrow();

  if (proposed.type_info() != value.type_info()) {
    throw std::logic_error(
        fmt::format("OutputPort::Calc(): expected output type {} "
                    "but got {} for {}.",
                    value.GetNiceTypeName(), proposed.GetNiceTypeName(),
                    PortBase::GetFullDescription()));
  }
}

template <typename T>
const AbstractValue& LeafOutputPort<T>::DoEval(
    const Context<T>& context) const {
  if (!context.get_cache().has_cache_entry_value(cache_entry().cache_index())) {
    throw std::runtime_error(fmt::format(
        "This Context does not have a cache entry for {}. Make sure that the "
        "context is created *after* declaring any output ports.",
        cache_entry().description()));
  }
  return cache_entry().EvalAbstract(context);
}

}  // namespace drake::systems

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafOutputPort)
