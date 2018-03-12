#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"

#include <memory>

#include <unsupported/Eigen/MatrixFunctions>

using Eigen::Dynamic;
using Eigen::Matrix;

namespace drake {
namespace trajectories {

template <typename T>
ExponentialPlusPiecewisePolynomial<T>::ExponentialPlusPiecewisePolynomial() {
  // empty
}

template <typename T>
ExponentialPlusPiecewisePolynomial<T>::
    ExponentialPlusPiecewisePolynomial(
        const PiecewisePolynomial<T>& piecewise_polynomial_part)
    : PiecewiseTrajectory<T>(piecewise_polynomial_part),
      K_(Matrix<T, Dynamic, Dynamic>::Zero(
          piecewise_polynomial_part.rows(), 1)),
      A_(Matrix<T, Dynamic, Dynamic>::Zero(1, 1)),
      alpha_(Matrix<T, Dynamic, Dynamic>::Zero(
          1, piecewise_polynomial_part.get_number_of_segments())),
      piecewise_polynomial_part_(piecewise_polynomial_part) {
  DRAKE_ASSERT(piecewise_polynomial_part.cols() == 1);
}

template <typename T>
std::unique_ptr<Trajectory<T>> ExponentialPlusPiecewisePolynomial<T>::Clone()
    const {
  return std::make_unique<ExponentialPlusPiecewisePolynomial<T>>(*this);
}

template <typename T>
MatrixX<T> ExponentialPlusPiecewisePolynomial<T>::value(double t) const {
  int segment_index = this->get_segment_index(t);

  Eigen::Matrix<double, Eigen::Dynamic, 1> ret =
      piecewise_polynomial_part_.value(t);
  double tj = this->start_time(segment_index);
  auto exponential = (A_ * (t - tj)).eval().exp().eval();
  ret.noalias() += K_ * exponential * alpha_.col(segment_index);
  return ret;
}

template <typename T>
ExponentialPlusPiecewisePolynomial<T>
ExponentialPlusPiecewisePolynomial<T>::derivative(
    int derivative_order) const {
  DRAKE_ASSERT(derivative_order >= 0);
  // quite inefficient, especially for high order derivatives due to all the
  // temporaries...
  MatrixX K_new = K_;
  for (int i = 0; i < derivative_order; i++) {
    K_new = K_new * A_;
  }
  return ExponentialPlusPiecewisePolynomial<T>(
      K_new, A_, alpha_,
      piecewise_polynomial_part_.derivative(derivative_order));
}

template <typename T>
Eigen::Index ExponentialPlusPiecewisePolynomial<T>::rows() const {
  return piecewise_polynomial_part_.rows();
}

template <typename T>
Eigen::Index ExponentialPlusPiecewisePolynomial<T>::cols() const {
  return piecewise_polynomial_part_.cols();
}

template <typename T>
void ExponentialPlusPiecewisePolynomial<T>::shiftRight(
    double offset) {
  std::vector<double>& breaks = this->get_mutable_breaks();
  for (auto it = breaks.begin(); it != breaks.end(); ++it) {
    *it += offset;
  }
  piecewise_polynomial_part_.shiftRight(offset);
}

template class ExponentialPlusPiecewisePolynomial<double>;

}  // namespace trajectories
}  // namespace drake
