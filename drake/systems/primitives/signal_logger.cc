#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace systems {

template <typename T>
SignalLogger<T>::SignalLogger(int input_size, int batch_allocation_size)
    : log_(input_size, batch_allocation_size) {
  this->DeclareInputPort(kVectorValued, input_size);
}

template <typename T>
void SignalLogger<T>::DoPublish(const Context<T>& context,
                                const std::vector<const PublishEvent<T>*>&)
                                const {
  log_.AddData(context.get_time(),
               this->EvalVectorInput(context, 0)->get_value());
}

template class SignalLogger<double>;
template class SignalLogger<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
