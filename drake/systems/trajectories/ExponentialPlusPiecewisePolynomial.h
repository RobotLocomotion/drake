#pragma once

#include <Eigen/Core>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/common/drake_export.h"

/**
 * y(t) = K * exp(A * (t - t_j)) * alpha.col(j) + piecewise_polynomial_part(t)
 */

template <typename CoefficientType = double>
class DRAKE_EXPORT ExponentialPlusPiecewisePolynomial
    : public PiecewiseFunction {
 public:
  typedef Eigen::Matrix<CoefficientType, Eigen::Dynamic, Eigen::Dynamic>
      MatrixX;
  typedef Eigen::Matrix<CoefficientType, Eigen::Dynamic, 1> VectorX;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> ValueType;

 private:
  MatrixX K_;
  MatrixX A_;
  MatrixX alpha_;
  PiecewisePolynomial<CoefficientType> piecewise_polynomial_part_;

 public:
  ExponentialPlusPiecewisePolynomial();

  template <typename DerivedK, typename DerivedA, typename DerivedAlpha>
  ExponentialPlusPiecewisePolynomial(
      const Eigen::MatrixBase<DerivedK>& K,
      const Eigen::MatrixBase<DerivedA>& A,
      const Eigen::MatrixBase<DerivedAlpha>& alpha,
      const PiecewisePolynomial<CoefficientType>& piecewise_polynomial_part)
      : PiecewiseFunction(piecewise_polynomial_part),
        K_(K),
        A_(A),
        alpha_(alpha),
        piecewise_polynomial_part_(piecewise_polynomial_part) {
    DRAKE_ASSERT(K.rows() == rows());
    DRAKE_ASSERT(K.cols() == A.rows());
    DRAKE_ASSERT(A.rows() == A.cols());
    DRAKE_ASSERT(alpha.rows() == A.cols());
    DRAKE_ASSERT(alpha.cols() ==
                 piecewise_polynomial_part.getNumberOfSegments());
    DRAKE_ASSERT(piecewise_polynomial_part.rows() == rows());
    DRAKE_ASSERT(piecewise_polynomial_part.cols() == 1);
  }

  // from PiecewisePolynomial
  ExponentialPlusPiecewisePolynomial(
      const PiecewisePolynomial<CoefficientType>& piecewise_polynomial_part);

  // TODO(tkoolen): fix return type (handle complex etc.)
  ValueType value(double t) const;

  ExponentialPlusPiecewisePolynomial derivative(int derivative_order = 1) const;

  virtual Eigen::Index rows() const;

  virtual Eigen::Index cols() const;

  void shiftRight(double offset);
};
