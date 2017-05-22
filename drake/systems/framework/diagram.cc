#include "drake/systems/framework/diagram.h"

namespace drake {
namespace systems {
namespace internal {

template class DiagramOutputPort<double>;
template class DiagramOutputPort<AutoDiffXd>;
template class DiagramOutputPort<symbolic::Expression>;

template class DiagramOutput<double>;

template class DiagramTimeDerivatives<double>;

template class DiagramDiscreteVariables<double>;

}  // namespace internal

template class Diagram<double>;

}  // namespace systems
}  // namespace drake
