#pragma once

#include <stddef.h>
#include <queue>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace tools {

/**
 * The implementation of a Moving Average Filter. This discrete time filter
 * outputs the average of the last n samples i.e.
 *  y(k) = 1/n ∑ⱼ x(k-j) ∀ j = 0..n-1, when n<k-1 and,
 *       = 1/k ∑ⱼ x(j) ∀ j = 0..k otherwise;
 * where n is the window size and x being the discrete-time signal that is
 * to be filtered, y is the filtered signal and k is the index of latest
 * element in the signal time-series.
 *
 * Note that this class is meant to serve as a standalone simple utility and
 * a filter of this form in a more `drake::systems` flavour can be generated
 * from a `systems::AffineSystem` since this is a LTI filter.
 *
 * @tparam T The element type.
 * Instantiated templates for the following kinds of T's are provided:
 *  - double
 *  - Eigen::Array3d
 */
template <typename T>
class MovingAverageFilter {
 public:
  // Since this class effectively has an internal state, it cannot be copied
  // or moved.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MovingAverageFilter)
  /**
   * Constructs the filter with the specified `window_size`.
   * @param window_size The size of the window which must be greater than
   * 0.
   * @throws a std::runtime_error when window_size <= 0.
   */
  explicit MovingAverageFilter(int window_size);

  /**
   * Computes the average filter result. Every call to this method modifies
   * the internal state of this filter thus resulting in a computation of
   *
   * @param new_data
   * @return
   */
  T Compute(const T& new_data);

 private:
  std::queue<T> window_;
  const size_t window_size_{0};
  T sum_;
};

}  // namespace tools
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
