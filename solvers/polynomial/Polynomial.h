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

// represents a scalar multi-variate polynomial
// modeled after the msspoly in spotless 

template <typename CoefficientType = double>
class DLLEXPORT Polynomial
{
public:
  typedef typename Eigen::Matrix<CoefficientType, Eigen::Dynamic, Eigen::Dynamic> CoefficientsType;
  typedef typename  PowersType;
  typedef typename Eigen::NumTraits<CoefficientType>::Real RealScalar;
  typedef std::complex<RealScalar> RootType;
  typedef Eigen::Matrix<RootType, Eigen::Dynamic, 1> RootsType;

private:
  std::vector< unsigned int > var;  // list of variable IDs with length N
  CoefficientsType coefficients;    // list of M monomial coefficients
  Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> powers; // M by N list of powers: powers(i,j) is the exponent of variable(j) in monomial i

public:
  Polynomial(Eigen::Ref<CoefficientsType> const& coefficients);

  Polynomial(int num_coefficients);

  int getNumberOfCoefficients() const;

  int getDegree() const;

  CoefficientsType const& getCoefficients() const;

  template<typename T> // can be different from both CoefficientsType and RealScalar
  T value(const T& t) const
  {
    return poly_eval(coefficients, t);
  }

  Polynomial derivative(int derivative_order = 1) const;

  Polynomial integral(const CoefficientType& integration_constant = 0.0) const;

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

  bool isApprox(const Polynomial& other, const RealScalar& tol) const;

  static Polynomial zero();
};

#endif /* DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_ */
