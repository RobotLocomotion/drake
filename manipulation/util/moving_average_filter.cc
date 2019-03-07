#include "drake/manipulation/util/moving_average_filter.h"

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace util {
namespace {

int get_dimensions(double) { return 1; }

int get_dimensions(VectorX<double> data) { return data.size(); }
}  // namespace

template <typename T>
MovingAverageFilter<T>::MovingAverageFilter(int window_size)
    : window_size_(window_size) {
  DRAKE_THROW_UNLESS(window_size_ > 0);
}

template <typename T>
T MovingAverageFilter<T>::Update(const T& new_data) {
  // First initialize sum (needed when type is not a scalar)

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

  if (window_.size() > static_cast<size_t>(window_size_)) {
    sum_ -= window_.front();
    window_.pop();
  }
  return moving_average();
}

template class MovingAverageFilter<double>;
template class MovingAverageFilter<VectorX<double>>;

}  // namespace util
}  // namespace manipulation
}  // namespace drake
