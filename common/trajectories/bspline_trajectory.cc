#include "drake/common/trajectories/bspline_trajectory.h"

#include <algorithm>
#include <functional>
#include <utility>

#include <fmt/format.h>

#include "drake/common/default_scalars.h"
#include "drake/common/extract_double.h"
#include "drake/common/text_logging.h"

using drake::symbolic::Expression;
using drake::symbolic::Variable;
using drake::trajectories::Trajectory;

namespace drake {
namespace trajectories {

using math::BsplineBasis;

template <typename T>
BsplineTrajectory<T>::BsplineTrajectory(BsplineBasis<T> basis,
                                        std::vector<MatrixX<T>> control_points)
    : basis_(std::move(basis)), control_points_(std::move(control_points)) {
  CheckInvariants();
}

template <typename T>
BsplineTrajectory<T>::~BsplineTrajectory() = default;

template <typename T>
Eigen::SparseMatrix<T> BsplineTrajectory<T>::AsLinearInControlPoints(
    int derivative_order) const {
  DRAKE_THROW_UNLESS(derivative_order >= 0);
  if (derivative_order == 0) {
    Eigen::SparseMatrix<T> M(num_control_points(), num_control_points());
    M.setIdentity();
    return M;
  } else if (derivative_order >= basis_.order()) {
    // In this case, MakeDerivative will return a zero trajectory using a
    // single control point.
    return Eigen::SparseMatrix<T>(num_control_points(), 1);
  } else {
    // First compute the control points of the kth derivative, pᵏ, relative
    // to the original control points, p: pᵏ = p * Mᵏ.
    Eigen::SparseMatrix<T> M_k(num_control_points(), num_control_points());
    M_k.setIdentity();
    for (int j = 1; j <= derivative_order; ++j) {
      for (int i = 0; i < num_control_points() - j; ++i) {
        // pᵢᵏ⁺¹ = αᵢ * (pᵢ₊₁ᵏ - pᵢᵏ), and pᵢᵏ = p * Mᵢᵏ, where the i subscript
        // denotes the ith column. so p * Mᵢᵏ⁺¹ = αᵢ * (p * Mᵢ₊₁ᵏ - p * Mᵢᵏ), or
        // Mᵢᵏ⁺¹ = αᵢ * (Mᵢ₊₁ᵏ - Mᵢᵏ).
        M_k.col(i) =
            (basis_.order() - j) /
            (basis_.knots()[i + basis_.order()] - basis_.knots()[i + j]) *
            (M_k.col(i + 1) - M_k.col(i));
      }
    }
    return M_k.leftCols(num_control_points() - derivative_order);
  }
}

template <typename T>
VectorX<T> BsplineTrajectory<T>::EvaluateLinearInControlPoints(
    const T& t, int derivative_order) const {
  using std::clamp;
  T clamped_time = clamp(t, this->start_time(), this->end_time());
  DRAKE_THROW_UNLESS(derivative_order >= 0);
  DRAKE_THROW_UNLESS(this->cols() == 1);
  if (derivative_order == 0) {
    return basis_.EvaluateLinearInControlPoints(clamped_time);
  } else if (derivative_order >= basis_.order()) {
    return VectorX<T>::Zero(num_control_points());
  } else {
    // First compute the control points of the kth derivative, pᵏ, relative
    // to the original control points, p: pᵏ = p * Mᵏ.
    std::vector<T> derivative_knots(basis_.knots().begin() + derivative_order,
                                    basis_.knots().end() - derivative_order);
    BsplineBasis<T> lower_order_basis =
        BsplineBasis<T>(basis_.order() - derivative_order, derivative_knots);
    MatrixX<T> M_k =
        MatrixX<T>::Identity(num_control_points(), num_control_points());
    // This is similar to the code in AsLinearInControlPoints, but here we can
    // restrict ourselves to only computing the terms for the active basis
    // functions.
    std::vector<int> base_indices =
        basis_.ComputeActiveBasisFunctionIndices(clamped_time);
    for (int j = 1; j <= derivative_order; ++j) {
      for (int i = base_indices.front(); i <= base_indices.back() - j; ++i) {
        // pᵢᵏ⁺¹ = αᵢ * (pᵢ₊₁ᵏ - pᵢᵏ), and pᵢᵏ = p * Mᵢᵏ, where the i subscript
        // denotes the ith column. so p * Mᵢᵏ⁺¹ = αᵢ * (p * Mᵢ₊₁ᵏ - p * Mᵢᵏ), or
        // Mᵢᵏ⁺¹ = αᵢ * (Mᵢ₊₁ᵏ - Mᵢᵏ).
        M_k.col(i) =
            (basis_.order() - j) /
            (basis_.knots()[i + basis_.order()] - basis_.knots()[i + j]) *
            (M_k.col(i + 1) - M_k.col(i));
      }
    }
    // Now value = p * M_to_deriv * M_deriv
    VectorX<T> M_deriv =
        lower_order_basis.EvaluateLinearInControlPoints(clamped_time);
    VectorX<T> M = VectorX<T>::Zero(num_control_points());
    for (int i :
         lower_order_basis.ComputeActiveBasisFunctionIndices(clamped_time)) {
      M += M_k.col(i) * M_deriv(i);
    }
    return M;
  }
}

template <typename T>
std::unique_ptr<Trajectory<T>> BsplineTrajectory<T>::DoClone() const {
  return std::make_unique<BsplineTrajectory<T>>(*this);
}

template <typename T>
MatrixX<T> BsplineTrajectory<T>::do_value(const T& time) const {
  using std::clamp;
  return basis().EvaluateCurve(
      control_points(), clamp(time, this->start_time(), this->end_time()));
}

template <typename T>
bool BsplineTrajectory<T>::do_has_derivative() const {
  return true;
}

template <typename T>
MatrixX<T> BsplineTrajectory<T>::DoEvalDerivative(const T& time,
                                                  int derivative_order) const {
  if (derivative_order == 0) {
    return this->value(time);
  } else if (derivative_order >= basis_.order()) {
    return MatrixX<T>::Zero(this->rows(), this->cols());
  } else if (derivative_order >= 1) {
    using std::clamp;
    T clamped_time = clamp(time, this->start_time(), this->end_time());
    // For a bspline trajectory of order n, the evaluation of k th derivative
    // should take O(k^2) time by leveraging the sparsity of basis value.
    // This differs from DoMakeDerivative, which takes O(nk) time.
    std::vector<T> derivative_knots(basis_.knots().begin() + derivative_order,
                                    basis_.knots().end() - derivative_order);
    BsplineBasis<T> lower_order_basis =
        BsplineBasis<T>(basis_.order() - derivative_order, derivative_knots);
    std::vector<MatrixX<T>> coefficients(control_points());
    std::vector<int> base_indices =
        basis_.ComputeActiveBasisFunctionIndices(clamped_time);
    for (int j = 1; j <= derivative_order; ++j) {
      for (int i = base_indices.front(); i <= base_indices.back() - j; ++i) {
        coefficients.at(i) =
            (basis_.order() - j) /
            (basis_.knots()[i + basis_.order()] - basis_.knots()[i + j]) *
            (coefficients[i + 1] - coefficients[i]);
      }
      coefficients.pop_back();
    }
    std::vector<MatrixX<T>> derivative_control_points(
        num_control_points() - derivative_order,
        MatrixX<T>::Zero(this->rows(), this->cols()));
    for (int i :
         lower_order_basis.ComputeActiveBasisFunctionIndices(clamped_time)) {
      derivative_control_points.at(i) = coefficients.at(i);
    }
    return lower_order_basis.EvaluateCurve(derivative_control_points,
                                           clamped_time);
  } else {
    throw std::invalid_argument(
        fmt::format("Invalid derivative order ({}). The derivative order must "
                    "be greater than or equal to 0.",
                    derivative_order));
  }
}

template <typename T>
std::unique_ptr<Trajectory<T>> BsplineTrajectory<T>::DoMakeDerivative(
    int derivative_order) const {
  if (derivative_order == 0) {
    return this->Clone();
  } else if (derivative_order > basis_.degree()) {
    std::vector<T> derivative_knots;
    derivative_knots.push_back(basis_.knots().front());
    derivative_knots.push_back(basis_.knots().back());
    std::vector<MatrixX<T>> control_points(
        1, MatrixX<T>::Zero(this->rows(), this->cols()));
    return std::make_unique<BsplineTrajectory<T>>(
        BsplineBasis<T>(1, derivative_knots), control_points);
  } else if (derivative_order >= 1) {
    std::vector<T> derivative_knots(basis_.knots().begin() + derivative_order,
                                    basis_.knots().end() - derivative_order);
    std::vector<MatrixX<T>> coefficients(control_points());
    for (int j = 1; j <= derivative_order; ++j) {
      for (int i = 0; i < num_control_points() - j; ++i) {
        coefficients.at(i) =
            (basis_.order() - j) /
            (basis_.knots()[i + basis_.order()] - basis_.knots()[i + j]) *
            (coefficients[i + 1] - coefficients[i]);
      }
      coefficients.pop_back();
    }
    return std::make_unique<BsplineTrajectory<T>>(
        BsplineBasis<T>(basis_.order() - derivative_order, derivative_knots),
        coefficients);
  } else {
    throw std::invalid_argument(
        fmt::format("Invalid derivative order ({}). The derivative order must "
                    "be greater than or equal to 0.",
                    derivative_order));
  }
}

template <typename T>
MatrixX<T> BsplineTrajectory<T>::InitialValue() const {
  return value(this->start_time());
}

template <typename T>
MatrixX<T> BsplineTrajectory<T>::FinalValue() const {
  return value(this->end_time());
}

template <typename T>
void BsplineTrajectory<T>::InsertKnots(const std::vector<T>& additional_knots) {
  if (additional_knots.size() != 1) {
    for (const auto& time : additional_knots) {
      InsertKnots(std::vector<T>{time});
    }
  } else {
    // Implements Boehm's Algorithm for knot insertion as described in by
    // Patrikalakis et al. [1], with a typo corrected in equation 1.76.
    //
    // [1] http://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node18.html

    // Define short-hand references to match Patrikalakis et al.:
    const std::vector<T>& t = basis_.knots();
    const T& t_bar = additional_knots.front();
    const int k = basis_.order();
    DRAKE_DEMAND(this->start_time() <= t_bar && t_bar <= this->end_time());

    /* Find the index, 𝑙, of the greatest knot that is less than or equal to
    t_bar and strictly less than end_time(). */
    const int ell = basis().FindContainingInterval(t_bar);
    std::vector<T> new_knots = t;
    new_knots.insert(std::next(new_knots.begin(), ell + 1), t_bar);
    std::vector<MatrixX<T>> new_control_points{this->control_points().front()};
    for (int i = 1; i < this->num_control_points(); ++i) {
      T a{0};
      if (i < ell - k + 2) {
        a = 1;
      } else if (i <= ell) {
        // Patrikalakis et al. have t[l + k - 1] in the denominator here ([1],
        // equation 1.76). This is contradicted by other references (e.g. [2]),
        // and does not yield the desired result (that the addition of the knot
        // leaves the values of the original trajectory unchanged). We use
        // t[i + k - 1], which agrees with [2] (modulo changes in notation)
        // and produces the desired results.
        //
        // [2] Prautzsch, Hartmut, Wolfgang Boehm, and Marco Paluszny. Bézier
        // and B-spline techniques. Springer Science & Business Media, 2013.
        a = (t_bar - t[i]) / (t[i + k - 1] - t[i]);
      }
      new_control_points.push_back((1 - a) * control_points()[i - 1] +
                                   a * control_points()[i]);
    }
    // Note that since a == 0 for i > ell in the loop above, the final control
    // point from the original trajectory is never pushed back into
    // new_control_points. Do that now.
    new_control_points.push_back(this->control_points().back());
    control_points_.swap(new_control_points);
    basis_ = BsplineBasis<T>(basis_.order(), new_knots);
  }
}

template <typename T>
BsplineTrajectory<T> BsplineTrajectory<T>::CopyWithSelector(
    const std::function<MatrixX<T>(const MatrixX<T>&)>& select) const {
  std::vector<MatrixX<T>> new_control_points{};
  new_control_points.reserve(num_control_points());
  for (const MatrixX<T>& control_point : control_points_) {
    new_control_points.push_back(select(control_point));
  }

  return {basis(), new_control_points};
}

template <typename T>
BsplineTrajectory<T> BsplineTrajectory<T>::CopyBlock(int start_row,
                                                     int start_col,
                                                     int block_rows,
                                                     int block_cols) const {
  return CopyWithSelector([&start_row, &start_col, &block_rows,
                           &block_cols](const MatrixX<T>& full) {
    return full.block(start_row, start_col, block_rows, block_cols);
  });
}

template <typename T>
BsplineTrajectory<T> BsplineTrajectory<T>::CopyHead(int n) const {
  DRAKE_DEMAND(this->cols() == 1);
  DRAKE_DEMAND(n > 0);
  return CopyBlock(0, 0, n, 1);
}

template <typename T>
boolean<T> BsplineTrajectory<T>::operator==(
    const BsplineTrajectory<T>& other) const {
  if (this->basis() == other.basis() && this->rows() == other.rows() &&
      this->cols() == other.cols()) {
    boolean<T> result{true};
    for (int i = 0; i < this->num_control_points(); ++i) {
      result = result && drake::all(this->control_points()[i].array() ==
                                    other.control_points()[i].array());
      if (std::equal_to<boolean<T>>{}(result, boolean<T>{false})) {
        break;
      }
    }
    return result;
  } else {
    return boolean<T>{false};
  }
}

template <typename T>
void BsplineTrajectory<T>::CheckInvariants() const {
  DRAKE_THROW_UNLESS(static_cast<int>(control_points_.size()) ==
                     basis_.num_basis_functions());
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class BsplineTrajectory);
}  // namespace trajectories
}  // namespace drake
