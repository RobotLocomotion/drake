#include "drake/systems/framework/diagram_output_port.h"

namespace drake {
namespace systems {

template <typename T>
DiagramOutputPort<T>::~DiagramOutputPort() = default;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramOutputPort);
