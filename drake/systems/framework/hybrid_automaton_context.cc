#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/hybrid_automaton_context.h"


namespace drake {
namespace systems {

template class ModalSubsystem<double>;
template class ModalSubsystem<AutoDiffXd>;
template class ModalSubsystem<symbolic::Expression>;


}  // namespace systems
}  // namespace drake
