#include "drake/common/trajectories/bezier_curve.h"

#include <utility>
#include <vector>

#include "drake/common/drake_bool.h"
#include "drake/common/symbolic/polynomial.h"
#include "drake/math/binomial_coefficient.h"

namespace drake {
namespace trajectories {

using Eigen::SparseMatrix;
using math::BinomialCoefficient;

template <typename T>
BezierCurve<T>::BezierCurve(double start_time, double end_time,
                            const Eigen::Ref<const MatrixX<T>>& control_points)
    : start_time_{start_time},
      end_time_{end_time},
      control_points_{control_points} {
  DRAKE_DEMAND(end_time >= start_time);
}

template <typename T>
BezierCurve<T>::~BezierCurve() = default;

template <typename T>
T BezierCurve<T>::BernsteinBasis(int i, const T& time,
                                 std::optional<int> order) const {
  using std::pow;
  int n = order.value_or(this->order());
  int coeff = BinomialCoefficient(n, i);
  T s = (time - start_time_) / (end_time_ - start_time_);
  return coeff * pow(s, i) * pow(1 - s, n - i);
}

template <typename T>
VectorX<symbolic::Expression> BezierCurve<T>::GetExpression(
    symbolic::Variable time) const {
  if constexpr (scalar_predicate<T>::is_bool) {
    MatrixX<symbolic::Expression> control_points{control_points_.rows(),
                                                 control_points_.cols()};

    if constexpr (std::is_same_v<T, double>) {
      control_points = control_points_.template cast<symbolic::Expression>();
    } else {
      // AutoDiffXd.
      for (int r = 0; r < control_points.rows(); ++r) {
        for (int c = 0; c < control_points.cols(); ++c) {
          control_points(r, c) =
              symbolic::Expression(control_points_(r, c).value());
        }
      }
    }
    return BezierCurve<symbolic::Expression>(start_time_, end_time_,
                                             control_points)
        .GetExpression(time);
  } else {
    VectorX<symbolic::Expression> ret{EvaluateT(symbolic::Expression(time))};
    for (int i = 0; i < ret.rows(); ++i) {
      ret(i) = ret(i).Expand();
    }
    return ret;
  }
}

template <typename T>
void BezierCurve<T>::ElevateOrder() {
  if (order() < 0) {
    control_points_ = MatrixX<T>::Zero(this->rows(), this->cols());
    return;
  }
  // https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-elev.html
  const int n = order();
  MatrixX<T> Q(control_points_.rows(), n + 2);

  Q.col(0) = control_points_.col(0);
  Q.col(n + 1) = control_points_.col(n);

  for (int i = 1; i <= n; ++i) {
    Q.col(i) = control_points_.col(i - 1) * static_cast<double>(i) / (n + 1) +
               control_points_.col(i) * (1 - static_cast<double>(i) / (n + 1));
  }

  control_points_ = std::move(Q);
}

template <typename T>
SparseMatrix<double> BezierCurve<T>::AsLinearInControlPoints(
    int derivative_order) const {
  DRAKE_THROW_UNLESS(derivative_order >= 0);
  if (derivative_order > order()) {
    return SparseMatrix<double>(order() + 1, 0);  // Return the empty matrix.
  } else if (derivative_order == 0) {
    SparseMatrix<double> M(order() + 1, order() + 1);
    M.setIdentity();
    return M;
  }
  const double duration = end_time_ - start_time_;
  int n = order();
  // Note: The derivation of M here follows simply from the
  // CalcDerivativePoints implementation below.
  SparseMatrix<double> M(n + 1, n);
  double coeff = n / duration;
  std::vector<Eigen::Triplet<double>> tripletList;
  tripletList.reserve(2 * n);
  for (int i = 0; i < n; ++i) {
    tripletList.push_back(Eigen::Triplet<double>(i + 1, i, coeff));
    tripletList.push_back(Eigen::Triplet<double>(i, i, -coeff));
  }
  M.setFromTriplets(tripletList.begin(), tripletList.end());
  for (int o = 1; o < derivative_order; ++o) {
    n -= 1;
    SparseMatrix<double> deltaM(n + 1, n);
    coeff = n / duration;
    tripletList.clear();
    for (int i = 0; i < n; ++i) {
      tripletList.push_back(Eigen::Triplet<double>(i + 1, i, coeff));
      tripletList.push_back(Eigen::Triplet<double>(i, i, -coeff));
    }
    deltaM.setFromTriplets(tripletList.begin(), tripletList.end());
    // Avoid aliasing. SparseMatrix does not offer the *= operatior.
    SparseMatrix<double> Mprev = std::move(M);
    M = Mprev * deltaM;
  }

  return M;
}

template <typename T>
std::unique_ptr<Trajectory<T>> BezierCurve<T>::DoClone() const {
  return std::make_unique<BezierCurve<T>>(start_time_, end_time_,
                                          control_points_);
}

template <typename T>
MatrixX<T> BezierCurve<T>::do_value(const T& time) const {
  using std::clamp;
  return EvaluateT(clamp(time, T{start_time_}, T{end_time_}));
}

template <typename T>
MatrixX<T> BezierCurve<T>::DoEvalDerivative(const T& time,
                                            int derivative_order) const {
  DRAKE_DEMAND(derivative_order >= 0);
  if (derivative_order == 0) {
    return this->value(time);
  }
  if (derivative_order > order()) {
    return VectorX<T>::Zero(this->rows());
  }

  MatrixX<T> points = CalcDerivativePoints(derivative_order);
  using std::clamp;
  const T ctime = clamp(time, T{start_time_}, T{end_time_});

  MatrixX<T> v = VectorX<T>::Zero(this->rows());
  for (int i = 0; i < points.cols(); ++i) {
    v += BernsteinBasis(i, ctime, order() - derivative_order) * points.col(i);
  }
  return v;
}

template <typename T>
std::unique_ptr<Trajectory<T>> BezierCurve<T>::DoMakeDerivative(
    int derivative_order) const {
  DRAKE_DEMAND(derivative_order >= 0);
  if (derivative_order == 0) {
    return this->Clone();
  }
  if (derivative_order > order()) {
    // Then return the zero curve.
    return std::make_unique<BezierCurve<T>>(start_time_, end_time_,
                                            VectorX<T>::Zero(this->rows()));
  }

  return std::make_unique<BezierCurve<T>>(
      start_time_, end_time_, CalcDerivativePoints(derivative_order));
}

template <typename T>
MatrixX<T> BezierCurve<T>::CalcDerivativePoints(int derivative_order) const {
  DRAKE_DEMAND(derivative_order <= order());
  int n = order();
  MatrixX<T> points =
      (control_points_.rightCols(order()) - control_points_.leftCols(order())) *
      order() / (end_time_ - start_time_);
  for (int i = 1; i < derivative_order; ++i) {
    n -= 1;
    points = (points.rightCols(n) - points.leftCols(n)).eval() * n /
             (end_time_ - start_time_);
  }
  return points;
}

template <typename T>
VectorX<T> BezierCurve<T>::EvaluateT(const T& time) const {
  VectorX<T> v = VectorX<T>::Zero(this->rows());
  for (int i = 0; i < control_points_.cols(); ++i) {
    v += BernsteinBasis(i, time) * control_points_.col(i);
  }
  return v;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class BezierCurve);

}  // namespace trajectories
}  // namespace drake
