#include "drake/common/trajectories/discrete_time_trajectory.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace trajectories {

namespace {

// Helper method to go from the Eigen entry points to the std::vector versions.
// Copies each column of mat to an element of the returned std::vector.
template <typename T>
std::vector<MatrixX<T>> ColsToStdVector(
    const Eigen::Ref<const MatrixX<T>>& mat) {
  std::vector<MatrixX<T>> vec(mat.cols());
  for (int i = 0; i < mat.cols(); i++) {
    vec[i] = mat.col(i);
  }
  return vec;
}

}  // end namespace

template <typename T>
DiscreteTimeTrajectory<T>::DiscreteTimeTrajectory(
    const Eigen::Ref<const VectorX<T>>& times,
    const Eigen::Ref<const MatrixX<T>>& values,
    const double time_comparison_tolerance)
    : DiscreteTimeTrajectory(
          std::vector<T>(times.data(), times.data() + times.size()),
          ColsToStdVector(values), time_comparison_tolerance) {}

template <typename T>
DiscreteTimeTrajectory<T>::DiscreteTimeTrajectory(
    const std::vector<T>& times, const std::vector<MatrixX<T>>& values,
    const double time_comparison_tolerance)
    : times_(times),
      values_(values),
      time_comparison_tolerance_(time_comparison_tolerance) {
  DRAKE_DEMAND(times.size() == values.size());
  // Ensure that times are convertible to double.
  for (const auto& t : times) {
    ExtractDoubleOrThrow(t);
  }
  for (int i = 1; i < static_cast<int>(times_.size()); i++) {
    DRAKE_DEMAND(times[i] - times[i - 1] >= time_comparison_tolerance_);
    DRAKE_DEMAND(values[i].rows() == values[0].rows());
    DRAKE_DEMAND(values[i].cols() == values[0].cols());
  }
  DRAKE_DEMAND(time_comparison_tolerance_ >= 0);
}

template <typename T>
PiecewisePolynomial<T> DiscreteTimeTrajectory<T>::ToZeroOrderHold() const {
  return PiecewisePolynomial<T>::ZeroOrderHold(times_, values_);
}

template <typename T>
double DiscreteTimeTrajectory<T>::time_comparison_tolerance() const {
  return time_comparison_tolerance_;
}

template <typename T>
int DiscreteTimeTrajectory<T>::num_times() const {
  return times_.size();
}

template <typename T>
const std::vector<T>& DiscreteTimeTrajectory<T>::get_times() const {
  return times_;
}

template <typename T>
std::unique_ptr<Trajectory<T>> DiscreteTimeTrajectory<T>::Clone() const {
  return std::make_unique<DiscreteTimeTrajectory<T>>(
      times_, values_, time_comparison_tolerance_);
}

template <typename T>
MatrixX<T> DiscreteTimeTrajectory<T>::value(const T& t) const {
  using std::abs;
  const double time = ExtractDoubleOrThrow(t);
  static const char* kNoMatchingTimeStr =
      "Value requested at time {} does not match any of the trajectory times "
      "within tolerance {}.";
  for (int i = 0; i < static_cast<int>(times_.size()); ++i) {
    if (time < times_[i] - time_comparison_tolerance_) {
      throw std::runtime_error(
          fmt::format(kNoMatchingTimeStr, time, time_comparison_tolerance_));
    }
    if (abs(time - times_[i]) <= time_comparison_tolerance_) {
      return values_[i];
    }
  }
  throw std::runtime_error(
      fmt::format(kNoMatchingTimeStr, time, time_comparison_tolerance_));
}

template <typename T>
Eigen::Index DiscreteTimeTrajectory<T>::rows() const {
  DRAKE_DEMAND(times_.size() > 0);
  return values_[0].rows();
}

template <typename T>
Eigen::Index DiscreteTimeTrajectory<T>::cols() const {
  DRAKE_DEMAND(times_.size() > 0);
  return values_[0].cols();
}

template <typename T>
T DiscreteTimeTrajectory<T>::start_time() const {
  DRAKE_DEMAND(times_.size() > 0);
  return times_[0];
}

template <typename T>
T DiscreteTimeTrajectory<T>::end_time() const {
  DRAKE_DEMAND(times_.size() > 0);
  return times_[times_.size() - 1];
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::DiscreteTimeTrajectory)
