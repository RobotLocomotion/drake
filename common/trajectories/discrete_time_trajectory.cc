#include "drake/common/trajectories/discrete_time_trajectory.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/math/matrix_util.h"

namespace drake {
namespace trajectories {

using math::EigenToStdVector;

template <typename T>
DiscreteTimeTrajectory<T>::DiscreteTimeTrajectory(
    const Eigen::Ref<const VectorX<T>>& times,
    const Eigen::Ref<const MatrixX<T>>& values,
    const double time_comparison_tolerance)
    : DiscreteTimeTrajectory(
          std::vector<T>(times.data(), times.data() + times.size()),
          EigenToStdVector(values), time_comparison_tolerance) {}

template <typename T>
DiscreteTimeTrajectory<T>::DiscreteTimeTrajectory(
    std::vector<T> times, std::vector<MatrixX<T>> values,
    const double time_comparison_tolerance)
    : times_(std::move(times)),
      values_(std::move(values)),
      time_comparison_tolerance_(time_comparison_tolerance) {
  DRAKE_DEMAND(times_.size() == values_.size());
  // Ensure that times are convertible to double.
  for (const auto& t : times_) {
    ExtractDoubleOrThrow(t);
  }
  for (int i = 1; i < static_cast<int>(times_.size()); i++) {
    DRAKE_DEMAND(times_[i] - times_[i - 1] >= time_comparison_tolerance_);
    DRAKE_DEMAND(values_[i].rows() == values_[0].rows());
    DRAKE_DEMAND(values_[i].cols() == values_[0].cols());
  }
  DRAKE_DEMAND(time_comparison_tolerance_ >= 0);
}

template <typename T>
DiscreteTimeTrajectory<T>::~DiscreteTimeTrajectory() = default;

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
std::unique_ptr<Trajectory<T>> DiscreteTimeTrajectory<T>::DoClone() const {
  return std::make_unique<DiscreteTimeTrajectory<T>>(
      times_, values_, time_comparison_tolerance_);
}

template <typename T>
MatrixX<T> DiscreteTimeTrajectory<T>::do_value(const T& t) const {
  using std::abs;
  const double time = ExtractDoubleOrThrow(t);
  static constexpr const char* kNoMatchingTimeStr =
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
Eigen::Index DiscreteTimeTrajectory<T>::do_rows() const {
  DRAKE_DEMAND(times_.size() > 0);
  return values_[0].rows();
}

template <typename T>
Eigen::Index DiscreteTimeTrajectory<T>::do_cols() const {
  DRAKE_DEMAND(times_.size() > 0);
  return values_[0].cols();
}

template <typename T>
T DiscreteTimeTrajectory<T>::do_start_time() const {
  DRAKE_DEMAND(times_.size() > 0);
  return times_[0];
}

template <typename T>
T DiscreteTimeTrajectory<T>::do_end_time() const {
  DRAKE_DEMAND(times_.size() > 0);
  return times_[times_.size() - 1];
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::DiscreteTimeTrajectory);
