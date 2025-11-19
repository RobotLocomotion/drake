#include "drake/systems/framework/system_visitor.h"

namespace drake {
namespace systems {

template <typename T>
SystemVisitor<T>::~SystemVisitor() = default;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SystemVisitor);
