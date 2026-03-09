#include "drake/systems/framework/input_port.h"

namespace drake {
namespace systems {
template <typename T>
InputPort<T>::InputPort(const System<T>* system,
                        internal::SystemMessageInterface* system_interface,
                        internal::SystemId system_id, std::string name,
                        InputPortIndex index, DependencyTicket ticket,
                        PortDataType data_type, int size,
                        const std::optional<RandomDistribution>& random_type,
                        EvalAbstractCallback eval,
                        ValueProducer::AllocateCallback alloc)
    : InputPortBase(system_interface, system_id, std::move(name), index, ticket,
                    data_type, size, random_type, std::move(eval),
                    std::move(alloc)),
      system_(DRAKE_DEREF(system)) {
  // Check the precondition on identical parameters; note that comparing as
  // void* is only valid because we have single inheritance.
  DRAKE_DEMAND(static_cast<const void*>(system) == system_interface);
}
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::InputPort);
