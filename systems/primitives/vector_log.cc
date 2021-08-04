#include "drake/systems/primitives/vector_log.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

template <typename T>
VectorLog<T>::VectorLog(int input_size)
    : sample_times_(kDefaultCapacity),
      data_(input_size, kDefaultCapacity) {
  DRAKE_ASSERT_VOID(CheckInvariants());
}

template <typename T>
void VectorLog<T>::Reserve(int64_t capacity) {
  DRAKE_ASSERT_VOID(CheckInvariants());
  if (capacity > sample_times_.size()) {
    sample_times_.conservativeResize(capacity);
    data_.conservativeResize(Eigen::NoChange, capacity);
  }
  DRAKE_ASSERT_VOID(CheckInvariants());
}

template <typename T>
void VectorLog<T>::AddData(const T& time, const VectorX<T>& sample) {
  DRAKE_ASSERT_VOID(CheckInvariants());
  // If the new size exceeds the current allocation, then do a conservative
  // resize (ouch!). Clients can avoid this if necessary by calling Reserve()
  // ahead of time.
  if (num_samples_ + 1 > sample_times_.size()) {
    Reserve(sample_times_.size() * 2);
  }

  // Record time and input to the num_samples position.
  sample_times_(int64_t{num_samples_}) = time;
  data_.col(int64_t{num_samples_}) = sample;

  // Update the count.
  ++num_samples_;
  DRAKE_ASSERT_VOID(CheckInvariants());
}

template <typename T>
void VectorLog<T>::CheckInvariants() const {
  DRAKE_DEMAND(sample_times_.size() == data_.cols());
  DRAKE_DEMAND(num_samples_ <= sample_times_.size());
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::VectorLog)
