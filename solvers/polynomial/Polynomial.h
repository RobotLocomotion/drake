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

template <typename _CoefficientType = double>
class DLLEXPORT Polynomial
{
public:
  typedef _CoefficientType CoefficientType;
  typedef typename Eigen::Matrix<CoefficientType, Eigen::Dynamic, 1> CoefficientsType;
  typedef typename Eigen::NumTraits<CoefficientType>::Real RealScalar;
  typedef std::complex<RealScalar> RootType;
  typedef Eigen::Matrix<RootType, Eigen::Dynamic, 1> RootsType;
  template <typename Rhs, typename Lhs>
  using ProductType = decltype((Rhs) 0 * (Lhs) 0);
//  using ProductType = decltype(::std::declval<Rhs>() * ::std::declval<Lhs>()); // declval not available in MSVC2010



private:
  CoefficientsType coefficients;

public:
  Polynomial(Eigen::Ref<CoefficientsType> const& coefficients);

  Polynomial(int num_coefficients);

  Polynomial();

  int getNumberOfCoefficients() const;

  int getDegree() const;

  CoefficientsType const& getCoefficients() const;

  template<typename T> // can be different from both CoefficientsType and RealScalar
  ProductType<CoefficientType, T> value(const T& t) const
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
