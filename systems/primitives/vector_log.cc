#include "drake/systems/primitives/vector_log.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

template <typename T>
VectorLog<T>::VectorLog(int input_size)
    : sample_times_(kDefaultCapacity),
      data_(input_size, kDefaultCapacity) {}

template <typename T>
void VectorLog<T>::Reserve(int64_t capacity) {
  if (capacity > sample_times_.size()) {
    sample_times_.conservativeResize(capacity);
    data_.conservativeResize(Eigen::NoChange, capacity);
  }
}

template <typename T>
void VectorLog<T>::AddData(T time, const VectorX<T>& sample) {
  if (num_samples_ == 0 || time >= sample_times_(num_samples_ - 1))
    ++num_samples_;

  // If num_samples exceeds the current allocation, then do a conservative
  // resize (ouch!). Clients can avoid this if necessary by calling Reserve()
  // ahead of time.
  if (num_samples_ > sample_times_.size()) {
    Reserve(sample_times_.size() * 2);
  }

  // Record time and input to the num_samples position.
  sample_times_(num_samples_ - 1) = time;
  data_.col(num_samples_ - 1) = sample;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::VectorLog)
