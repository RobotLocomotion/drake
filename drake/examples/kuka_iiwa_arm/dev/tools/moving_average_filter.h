#pragma once

#include <vector>

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace tools {

/**
 * The implementation of a simple Moving Average Filter. This digital filter
 * outputs the average of the last n samples i.e.
 * @f[ y(k) = \frac{1}{n}*|sum_{k}^{j = k-n} x(j), @f]
 * where `n` is the window size.
 * Note that this class is meant to serve as a standalone simple utility and
 * a filter of this form in a more `drake::systems` flavour can be generated
 * from a `systems::AffineSystem` since this is a LTI filter.
 *
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 * Instantiated templates for the following kinds of T's are provided:
 *  - double
 *  - Eigen::Vector3d
 */
template <typename T>
class MovingAverageFilter {
 public:
  MovingAverageFilter(int window_size);

  T compute(const T& new_data);

 private:
  const int window_size_{-1};
  std::vector<T> window_;
};

}  // namespace tools
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
