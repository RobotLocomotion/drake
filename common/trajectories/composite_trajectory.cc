#include "drake/common/trajectories/composite_trajectory.h"

#include <algorithm>
#include <functional>
#include <numeric>
#include <utility>

namespace drake {
namespace trajectories {

using Eigen::Index;

template <typename T>
CompositeTrajectory<T>::CompositeTrajectory(bool rowwise) : rowwise_{rowwise} {}

template <typename T>
CompositeTrajectory<T>::~CompositeTrajectory() = default;

template <typename T>
void CompositeTrajectory<T>::Append(std::unique_ptr<Trajectory<T>> traj) {
  DRAKE_THROW_UNLESS(traj != nullptr);
  if (!children_.empty()) {
    if (rowwise_) {
      const Index cols_previously = children_.front()->cols();
      DRAKE_THROW_UNLESS(traj->cols() == cols_previously);
    } else {
      const Index rows_previously = children_.front()->rows();
      DRAKE_THROW_UNLESS(traj->rows() == rows_previously);
    }
  }
  children_.emplace_back(std::move(traj));
}

template <typename T>
std::unique_ptr<Trajectory<T>> CompositeTrajectory<T>::Clone() const {
  return std::unique_ptr<Trajectory<T>>(new CompositeTrajectory(*this));
}

template <typename T>
MatrixX<T> CompositeTrajectory<T>::value(const T& t) const {
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
Index CompositeTrajectory<T>::rows() const {
  if (rowwise_) {
    return std::transform_reduce(
        children_.begin(), children_.end(), Index{}, std::plus<Index>{},
        [](const auto& child) -> Index { return child->rows(); });
  } else {
    return children_.empty() ? 0 : children_.front()->rows();
  }
}

template <typename T>
Index CompositeTrajectory<T>::cols() const {
  if (rowwise_) {
    return children_.empty() ? 0 : children_.front()->cols();
  } else {
    return std::transform_reduce(
        children_.begin(), children_.end(), Index{}, std::plus<Index>{},
        [](const auto& child) -> Index { return child->cols(); });
  }
}

template <typename T>
T CompositeTrajectory<T>::start_time() const {
  return children_.empty() ? 0 : children_.front()->start_time();
}

template <typename T>
T CompositeTrajectory<T>::end_time() const {
  return children_.empty() ? 0 : children_.front()->end_time();
}

template <typename T>
bool CompositeTrajectory<T>::do_has_derivative() const {
  return std::all_of(children_.begin(), children_.end(),
                     [](const auto& child) { return child->has_derivative(); });
}

template <typename T>
MatrixX<T> CompositeTrajectory<T>::DoEvalDerivative(
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
std::unique_ptr<Trajectory<T>> CompositeTrajectory<T>::DoMakeDerivative(
    int derivative_order) const {
  auto result = std::make_unique<CompositeTrajectory<T>>(rowwise_);
  for (const auto& child : children_) {
    result->Append(child->MakeDerivative(derivative_order));
  }
  return result;
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::CompositeTrajectory)
