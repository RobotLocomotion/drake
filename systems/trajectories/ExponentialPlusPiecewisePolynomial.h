#ifndef SYSTEMS_TRAJECTORIES_EXPONENTIALPLUSPIECEWISEPOLYNOMIAL_H_
#define SYSTEMS_TRAJECTORIES_EXPONENTIALPLUSPIECEWISEPOLYNOMIAL_H_

#include <Eigen/Core>
#include <vector>
#include "PiecewisePolynomial.h"

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
#if defined(drakePiecewisePolynomial_EXPORTS)
#define DLLEXPORT __declspec( dllexport )
#else
#define DLLEXPORT __declspec( dllimport )
#endif
#else
#define DLLEXPORT
#endif


/**
 * y(t) = K * exp(A * (t - t_j)) * alpha_j + piecewise_polynomial_part(t)
 */

template<typename CoefficientType = double>
class DLLEXPORT ExponentialPlusPiecewisePolynomial : public PiecewiseFunction
{
public:
  typedef Eigen::Matrix<CoefficientType, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
  typedef Eigen::Matrix<CoefficientType, Eigen::Dynamic, 1> VectorX;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> ValueType;

private:
  MatrixX K;
  MatrixX A;
  std::vector<VectorX> alpha;
  PiecewisePolynomial<CoefficientType> piecewise_polynomial_part;

public:
  ExponentialPlusPiecewisePolynomial();

  ExponentialPlusPiecewisePolynomial(
      const MatrixX& K, const MatrixX& A, const std::vector<VectorX>& alpha,
      const PiecewisePolynomial<CoefficientType>& piecewise_polynomial_part);

  ValueType value(double t); // return type should not be VectorX because the scalar type of the output should be the same as the type of t

  ExponentialPlusPiecewisePolynomial derivative(int derivative_order = 1) const;

  virtual Eigen::DenseIndex rows() const;

  virtual Eigen::DenseIndex cols() const;

  void shiftRight(double offset);
};

#endif /* SYSTEMS_TRAJECTORIES_EXPONENTIALPLUSPIECEWISEPOLYNOMIAL_H_ */
