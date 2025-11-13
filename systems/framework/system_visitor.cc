#include "drake/systems/framework/system_visitor.h"

// This is an empty file to confirm that our header parses on its own.

namespace drake {
namespace systems {

template <typename T>
SystemVisitor<T>::~SystemVisitor() = default;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SystemVisitor);
