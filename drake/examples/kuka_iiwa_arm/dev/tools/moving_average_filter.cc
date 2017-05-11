#include "drake/examples/kuka_iiwa_arm/dev/tools/moving_average_filter.h"

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace tools {

template <typename T>
MovingAverageFilter<T>::MovingAverageFilter(int window_size)
    : window_size_(window_size) {
  DRAKE_THROW_UNLESS(window_size_ > 0);
}

template <typename T>
T MovingAverageFilter<T>::compute(const T& new_data) {
  // First intialize sum (needed when type is not a scalar)
  if (window_.size() == 0) {
    sum_ = new_data;
  } else {
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
template class MovingAverageFilter<Eigen::Array3d>;

}  // namespace tools
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
