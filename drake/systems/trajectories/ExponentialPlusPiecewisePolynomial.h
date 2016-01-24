#ifndef SYSTEMS_TRAJECTORIES_EXPONENTIALPLUSPIECEWISEPOLYNOMIAL_H_
#define SYSTEMS_TRAJECTORIES_EXPONENTIALPLUSPIECEWISEPOLYNOMIAL_H_

#include <Eigen/Core>
#include <vector>
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/drakeTrajectories_export.h"


/**
 * y(t) = K * exp(A * (t - t_j)) * alpha.col(j) + piecewise_polynomial_part(t)
 */

template<typename CoefficientType = double>
class DRAKETRAJECTORIES_EXPORT ExponentialPlusPiecewisePolynomial : public PiecewiseFunction
{
public:
  typedef Eigen::Matrix<CoefficientType, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
  typedef Eigen::Matrix<CoefficientType, Eigen::Dynamic, 1> VectorX;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> ValueType;

private:
  MatrixX K;
  MatrixX A;
  MatrixX alpha;
  PiecewisePolynomial<CoefficientType> piecewise_polynomial_part;

public:
  ExponentialPlusPiecewisePolynomial();

  template <typename DerivedK, typename DerivedA, typename DerivedAlpha>
  ExponentialPlusPiecewisePolynomial(
      const Eigen::MatrixBase<DerivedK>& K, const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedAlpha>& alpha,
      const PiecewisePolynomial<CoefficientType>& piecewise_polynomial_part) :
        PiecewiseFunction(piecewise_polynomial_part), K(K), A(A), alpha(alpha), piecewise_polynomial_part(piecewise_polynomial_part)
  {
    assert(K.rows() == rows());
    assert(K.cols() == A.rows());
    assert(A.rows() == A.cols());
    assert(alpha.rows() == A.cols());
    assert(alpha.cols() == piecewise_polynomial_part.getNumberOfSegments());
    assert(piecewise_polynomial_part.rows() == rows());
    assert(piecewise_polynomial_part.cols() == 1);
  }

  // from PiecewisePolynomial
  ExponentialPlusPiecewisePolynomial(const PiecewisePolynomial<CoefficientType>& piecewise_polynomial_part);

  ValueType value(double t) const; // TODO: fix return type (handle complex etc.)

  ExponentialPlusPiecewisePolynomial derivative(int derivative_order = 1) const;

  virtual Eigen::DenseIndex rows() const;

  virtual Eigen::DenseIndex cols() const;

  void shiftRight(double offset);
};

#endif /* SYSTEMS_TRAJECTORIES_EXPONENTIALPLUSPIECEWISEPOLYNOMIAL_H_ */
