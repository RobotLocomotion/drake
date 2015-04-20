#ifndef DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_
#define DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_

#include <Eigen/Core>
#include <complex>
#include <unsupported/Eigen/Polynomials>

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
#if defined(drakePolynomial_EXPORTS)
#define DLLEXPORT __declspec( dllexport )
#else
#define DLLEXPORT __declspec( dllimport )
#endif
#else
#define DLLEXPORT
#endif

class DLLEXPORT Polynomial
{
public:
  typedef double CoefficientType;
  typedef typename Eigen::NumTraits<CoefficientType>::Real RealScalar;
  typedef std::complex<RealScalar> RootType;
  typedef Eigen::Matrix<RootType, Eigen::Dynamic, 1> RootsType;

private:
  Eigen::Matrix<CoefficientType, Eigen::Dynamic, 1> coefficients;

public:
  Polynomial(Eigen::Ref<Eigen::VectorXd> const& coefficients);

  Polynomial(int num_coefficients);

  int getNumberOfCoefficients() const;

  int getDegree() const;

  Eigen::VectorXd const& getCoefficients() const;

  template<typename T>
  T value(const T& t) const
  {
    return poly_eval(coefficients, t);
  }

  Polynomial derivative(int derivative_order = 1) const;

  Polynomial integral(double integration_constant = 0.0) const;

  Polynomial& operator+=(const Polynomial& other);

  Polynomial& operator-=(const Polynomial& other);

  Polynomial& operator*=(const Polynomial& other);

  Polynomial& operator+=(const CoefficientType& scalar);

  Polynomial& operator-=(const CoefficientType& scalar);

  Polynomial& operator*=(const CoefficientType& scalar);

  Polynomial& operator/=(const CoefficientType& scalar);

  const Polynomial operator+(const Polynomial& other) const;

  const Polynomial operator-(const Polynomial& other) const;

  const Polynomial operator*(const Polynomial& other) const;

  const Polynomial operator+(const CoefficientType& scalar) const;

  const Polynomial operator-(const CoefficientType& scalar) const;

  const Polynomial operator*(const CoefficientType& scalar) const;

  const Polynomial operator/(const CoefficientType& scalar) const;

  RootsType roots() const;

  bool isApprox(const Polynomial& other, const CoefficientType& tol) const;

  static Polynomial zero();
};

#endif /* DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_ */
