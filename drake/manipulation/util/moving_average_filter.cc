#include "drake/manipulation/util/moving_average_filter.h"

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace utils {
namespace {

size_t get_dimensions(double data) { return 1; }

size_t get_dimensions(VectorX<double> data) { return data.size(); }
}  // namespace

template <typename T>
MovingAverageFilter<T>::MovingAverageFilter(unsigned int window_size)
    : window_size_(window_size) {
  DRAKE_THROW_UNLESS(window_size_ > 0);
}

template <typename T>
T MovingAverageFilter<T>::Compute(const T& new_data) {
  // First intialize sum (needed when type is not a scalar)

  if (window_.size() == 0) {
    sum_ = new_data;
  } else {
    // Check if new_data has the same dimension as the pre-existing data in the
    // window.
    DRAKE_THROW_UNLESS(get_dimensions(new_data) ==
                       get_dimensions(window_.front()));
    sum_ += new_data;
  }

  window_.push(new_data);

  if (window_.size() > window_size_) {
    sum_ -= window_.front();
    window_.pop();
  }
  return (1.0 / window_.size()) * sum_;
}

template class MovingAverageFilter<double>;
template class MovingAverageFilter<VectorX<double>>;

}  // namespace utils
}  // namespace manipulation
}  // namespace drake
