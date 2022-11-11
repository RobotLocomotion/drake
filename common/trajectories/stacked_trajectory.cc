#include "drake/common/trajectories/stacked_trajectory.h"

#include <algorithm>
#include <functional>
#include <numeric>
#include <utility>

namespace drake {
namespace trajectories {

using Eigen::Index;

template <typename T>
StackedTrajectory<T>::StackedTrajectory(bool rowwise) : rowwise_{rowwise} {
  DRAKE_ASSERT_VOID(CheckInvariants());
}

template <typename T>
StackedTrajectory<T>::~StackedTrajectory() = default;

template <typename T>
void StackedTrajectory<T>::Append(const Trajectory<T>& traj) {
  Append(traj.Clone());
}

template <typename T>
void StackedTrajectory<T>::Append(std::unique_ptr<Trajectory<T>> traj) {
  DRAKE_DEMAND(traj != nullptr);

  // Check for valid times.
  if (!children_.empty()) {
    DRAKE_THROW_UNLESS(traj->start_time() == start_time());
    DRAKE_THROW_UNLESS(traj->end_time() == end_time());
  }

  // Check for valid sizes.
  if (rowwise_) {
    DRAKE_THROW_UNLESS(children_.empty() || traj->cols() == cols());
  } else {
    DRAKE_THROW_UNLESS(children_.empty() || traj->rows() == rows());
  }

  // Take ownership and update our internal sizes.
  if (rowwise_) {
    rows_ += traj->rows();
    if (children_.empty()) {
      cols_ = traj->cols();
    }
  } else {
    cols_ += traj->cols();
    if (children_.empty()) {
      rows_ = traj->rows();
    }
  }
  children_.emplace_back(std::move(traj));

  DRAKE_ASSERT_VOID(CheckInvariants());
}

template <typename T>
void StackedTrajectory<T>::CheckInvariants() const {
  // Sanity-check that our extent on the stacking axis matches the sum over all
  // of our children.
  const Index expected_stacked_size = rowwise_ ? rows_ : cols_;
  const Index actual_stacked_size = std::transform_reduce(
      children_.begin(), children_.end(), 0, std::plus<Index>{},
      [this](const auto& child) -> Index {
        return rowwise_ ? child->rows() : child->cols();
      });
  DRAKE_DEMAND(actual_stacked_size == expected_stacked_size);

  // Sanity-check that our extent in the non-stacking axis is the same over all
  // of our children.
  const Index expected_matched_size = rowwise_ ? cols_ : rows_;
  for (const auto& child : children_) {
    const Index actual_matched_size = rowwise_ ? child->cols() : child->rows();
    DRAKE_DEMAND(actual_matched_size == expected_matched_size);
  }

  // Sanity-check that the time span is the same for all children.
  for (const auto& child : children_) {
    DRAKE_DEMAND(child->start_time() == start_time());
    DRAKE_DEMAND(child->end_time() == end_time());
  }
}

template <typename T>
std::unique_ptr<Trajectory<T>> StackedTrajectory<T>::Clone() const {
  using Self = StackedTrajectory<T>;
  auto result = std::unique_ptr<Self>(new Self(*this));
  DRAKE_ASSERT_VOID(CheckInvariants());
  DRAKE_ASSERT_VOID(result->CheckInvariants());
  return result;
}

template <typename T>
MatrixX<T> StackedTrajectory<T>::value(const T& t) const {
  MatrixX<T> result(rows(), cols());
  Index row = 0;
  Index col = 0;
  for (const auto& child : children_) {
    const MatrixX<T> child_result = child->value(t);
    const Index child_rows = child_result.rows();
    const Index child_cols = child_result.cols();
    result.block(row, col, child_rows, child_cols) = child_result;
    if (rowwise_) {
      row += child_rows;
    } else {
      col += child_cols;
    }
  }
  return result;
}

template <typename T>
Index StackedTrajectory<T>::rows() const {
  return rows_;
}

template <typename T>
Index StackedTrajectory<T>::cols() const {
  return cols_;
}

template <typename T>
T StackedTrajectory<T>::start_time() const {
  return children_.empty() ? 0 : children_.front()->start_time();
}

template <typename T>
T StackedTrajectory<T>::end_time() const {
  return children_.empty() ? 0 : children_.front()->end_time();
}

template <typename T>
bool StackedTrajectory<T>::do_has_derivative() const {
  return std::all_of(children_.begin(), children_.end(),
                     [](const auto& child) { return child->has_derivative(); });
}

template <typename T>
MatrixX<T> StackedTrajectory<T>::DoEvalDerivative(
    const T& t, int derivative_order) const {
  MatrixX<T> result(rows(), cols());
  Index row = 0;
  Index col = 0;
  for (const auto& child : children_) {
    const MatrixX<T> child_result = child->EvalDerivative(t, derivative_order);
    const Index child_rows = child_result.rows();
    const Index child_cols = child_result.cols();
    result.block(row, col, child_rows, child_cols) = child_result;
    if (rowwise_) {
      row += child_rows;
    } else {
      col += child_cols;
    }
  }
  return result;
}

template <typename T>
std::unique_ptr<Trajectory<T>> StackedTrajectory<T>::DoMakeDerivative(
    int derivative_order) const {
  auto result = std::make_unique<StackedTrajectory<T>>(rowwise_);
  for (const auto& child : children_) {
    result->Append(child->MakeDerivative(derivative_order));
  }
  return result;
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::StackedTrajectory)
