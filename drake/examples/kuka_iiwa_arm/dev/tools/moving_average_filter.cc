#include "drake/examples/kuka_iiwa_arm/dev/tools/moving_average_filter.h"

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace tools {

template <typename T>
MovingAverageFilter<T>::MovingAverageFilter(int window_size)
    : window_size_(window_size), window(window_size_) {
  DRAKE_THROW_UNLESS(window_size_ > 1);
}

template <typename T>
T MovingAverageFilter<T>::compute(T new_data) {
  T sum;
  unsigned long i = 0;
  for (i = 0; i < window.size() - 1; ++i) {
    window[i] = window[i + 1];
    sum = sum + window[i];
  }
  window[i + 1] = new_data;
  sum = sum + new_data;
  return (1 / window_size_) * sum;
}

template class MovingAverageFilter<double>;
template class MovingAverageFilter<Eigen::Vector3d>;

}  // namespace tools
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
