#include "drake/systems/framework/diagram.h"

namespace drake {
namespace systems {
namespace internal {

template class DiagramOutput<double>;

template class DiagramTimeDerivatives<double>;

template class DiagramDiscreteVariables<double>;

}  // namespace internal

template class Diagram<double>;

}  // namespace systems
}  // namespace drake
