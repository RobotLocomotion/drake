#include "drake/common/trajectories/composite_trajectory.h"

#include <utility>

#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_trajectory.h"

namespace drake {
namespace trajectories {

namespace {

template <typename T>
std::vector<T> ExtractBreaks(
    const std::vector<copyable_unique_ptr<Trajectory<T>>>& segments) {
  std::vector<T> breaks(segments.size() + 1);
  if (segments.empty()) {
    breaks[0] = 0;
    return breaks;
  }

  for (int i = 0; i < static_cast<int>(segments.size()); ++i) {
    if (i > 0) {
      DRAKE_DEMAND(segments[i]->start_time() == segments[i - 1]->end_time());
    }
    breaks[i] = segments[i]->start_time();
  }
  breaks.back() = segments.back()->end_time();
  return breaks;
}

}  // namespace

template <typename T>
CompositeTrajectory<T>::CompositeTrajectory(
    std::vector<copyable_unique_ptr<Trajectory<T>>> segments)
    : PiecewiseTrajectory<T>(ExtractBreaks(segments)),
      segments_(std::move(segments)) {
  for (int i = 1; i < static_cast<int>(segments_.size()); ++i) {
    DRAKE_DEMAND(segments_[i]->rows() == segments_[0]->rows());
    DRAKE_DEMAND(segments_[i]->cols() == segments_[0]->cols());
  }
}

template <typename T>
std::unique_ptr<Trajectory<T>> CompositeTrajectory<T>::Clone() const {
  return std::make_unique<CompositeTrajectory<T>>(segments_);
}

template <typename T>
Eigen::Index CompositeTrajectory<T>::rows() const {
  if (segments_.size() > 0) {
    return segments_[0]->rows();
  } else {
    throw std::runtime_error(
        "CompositeTrajectory has no segments. Number of rows is undefined.");
  }
}

template <typename T>
Eigen::Index CompositeTrajectory<T>::cols() const {
  if (segments_.size() > 0) {
    return segments_[0]->cols();
  } else {
    throw std::runtime_error(
        "CompositeTrajectory has no segments. Number of cols is undefined.");
  }
}

template <typename T>
MatrixX<T> CompositeTrajectory<T>::value(const T& time) const {
  const int segment_index = this->get_segment_index(time);
  DRAKE_DEMAND(static_cast<int>(segments_.size()) > segment_index);
  return this->segments_[segment_index]->value(time);
}

template <typename T>
bool CompositeTrajectory<T>::do_has_derivative() const {
  return std::all_of(segments_.begin(), segments_.end(),
                     [](const auto& segment) {
                       return segment->has_derivative();
                     });
}

template <typename T>
MatrixX<T> CompositeTrajectory<T>::DoEvalDerivative(
    const T& time, int derivative_order) const {
  const int segment_index = this->get_segment_index(time);
  DRAKE_DEMAND(static_cast<int>(segments_.size()) > segment_index);
  return this->segments_[segment_index]->EvalDerivative(time, derivative_order);
}

template <typename T>
std::unique_ptr<Trajectory<T>> CompositeTrajectory<T>::DoMakeDerivative(
    int derivative_order) const {
  DRAKE_DEMAND(derivative_order >= 0);
  if (derivative_order == 0) {
    return this->Clone();
  }
  std::vector<copyable_unique_ptr<Trajectory<T>>> derivative_curves(
      segments_.size());
  for (int i = 0; i < static_cast<int>(segments_.size()); ++i) {
    derivative_curves[i] = segments_[i]->MakeDerivative(derivative_order);
  }
  return std::make_unique<CompositeTrajectory<T>>(std::move(derivative_curves));
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class CompositeTrajectory);

}  // namespace trajectories
}  // namespace drake
