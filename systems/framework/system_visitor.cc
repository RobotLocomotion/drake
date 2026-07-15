#include "drake/systems/framework/system_visitor.h"

namespace drake {
namespace systems {

template <typename T>
SystemVisitor<T>::~SystemVisitor() = default;

template <typename T>
void SystemVisitor<T>::VisitSystemPointer(const System<T>* system) {
  DRAKE_DEMAND(system != nullptr);
  VisitSystem(*system);
}

template <typename T>
void SystemVisitor<T>::VisitDiagramPointer(const Diagram<T>* diagram) {
  DRAKE_DEMAND(diagram != nullptr);
  VisitDiagram(*diagram);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SystemVisitor);
