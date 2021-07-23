#include "drake/systems/primitives/signal_log.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace drake {
namespace systems {

template <typename T>
SignalLog<T>::SignalLog(int input_size, int batch_allocation_size)
    : batch_allocation_size_(batch_allocation_size),
      sample_times_(batch_allocation_size_),
      data_(input_size, batch_allocation_size_) {
  DRAKE_DEMAND(input_size > 0);
  DRAKE_DEMAND(batch_allocation_size_ > 0);
}

template <typename T>
void SignalLog<T>::AddData(T time, VectorX<T> sample) {
  if (num_samples_ == 0 || time >= sample_times_(num_samples_ - 1))
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
  sample_times_(num_samples_ - 1) = time;
  data_.col(num_samples_ - 1) = sample;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SignalLog)

#pragma GCC diagnostic pop
