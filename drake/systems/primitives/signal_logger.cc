#include "drake/systems/primitives/signal_logger.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template <typename T>
SignalLogger<T>::SignalLogger(int input_size, int batch_allocation_size)
    : batch_allocation_size_(batch_allocation_size),
      sample_times_(batch_allocation_size_),
      data_(input_size, batch_allocation_size_) {
  DRAKE_DEMAND(input_size > 0);
  this->DeclareInputPort(kVectorValued, input_size);
  DRAKE_DEMAND(batch_allocation_size_ > 0);
}

template <typename T>
void SignalLogger<T>::DoPublish(const Context<T>& context) const {
  T current_time = context.get_time();

  if (num_samples_ == 0 || current_time >= sample_times_(num_samples_ - 1))
    ++num_samples_;

  // If num_samples exceeds the current allocation, then do a conservative
  // resize (ouch!).
  // TODO(russt): change to allocating on a std::vector here and moving into a
  // single block of contiguous memory on the (first) data access, to avoid the
  // O(n^2) complexity.
  if (num_samples_ > sample_times_.size()) {
    sample_times_.conservativeResize(sample_times_.size() +
                                     batch_allocation_size_);
    data_.conservativeResize(data_.rows(),
                             data_.cols() + batch_allocation_size_);
  }

  // Record time and input to the num_samples position.
  sample_times_(num_samples_ - 1) = current_time;
  data_.col(num_samples_ - 1) = this->EvalVectorInput(context, 0)->get_value();
}

template class SignalLogger<double>;
template class SignalLogger<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
