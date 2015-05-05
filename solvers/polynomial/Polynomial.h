#ifndef DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_
#define DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_

#include <Eigen/Core>
#include <complex>
#include <unsupported/Eigen/Polynomials>
#include <random>

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
  
  template<typename Rhs, typename Lhs>
  struct Product {
    typedef decltype((Rhs) 0 * (Lhs) 0) type;
  };

private:
  CoefficientsType coefficients;

public:
  template <typename Derived>
  Polynomial(Eigen::MatrixBase<Derived> const& coefficients) :
      coefficients(coefficients)
  {
    assert(coefficients.rows() > 0);
  }

  Polynomial(const CoefficientType& scalar_value); // this is required for some Eigen operations when used in a polynomial matrix

  Polynomial();

  int getNumberOfCoefficients() const;

  int getDegree() const;

  CoefficientsType const& getCoefficients() const;

  template<typename T> // can be different from both CoefficientsType and RealScalar
  typename Product<CoefficientType, T>::type value(const T& t) const
  {
    // adapted from Eigen/unsupported
    typedef typename Product<CoefficientType, T>::type ProductType;
    typedef typename NumTraits<T>::Real Real;
    
    if (numext::abs2(t) <= Real(1) ){
      // horner
      ProductType val = coefficients[coefficients.size() - 1];
      for (Eigen::DenseIndex i = coefficients.size() - 2; i >= 0; --i) {
        val = val * t + coefficients[i];
      }
      return val;
    }
    else
    {
      // stabilized horner
      ProductType val = coefficients[0];
      ProductType inv_x = T(1) / t;
      for (Eigen::DenseIndex i = 1; i < coefficients.size(); ++i) {
        val = val * inv_x + coefficients[i];
      }
      return std::pow(t, (ProductType) (static_cast<int>(coefficients.size() - 1))) * val;
    }    
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


  template<Eigen::DenseIndex RowsAtCompileTime, Eigen::DenseIndex ColsAtCompileTime>
  static Eigen::Matrix<Polynomial<CoefficientType>, Eigen::Dynamic, Eigen::Dynamic> randomPolynomialMatrix(Eigen::DenseIndex num_coefficients_per_polynomial, Eigen::DenseIndex rows = RowsAtCompileTime, Eigen::DenseIndex cols = ColsAtCompileTime)
  {
    Eigen::Matrix<Polynomial<CoefficientType>, RowsAtCompileTime, ColsAtCompileTime> mat(rows, cols);
    for (Eigen::DenseIndex row = 0; row < mat.rows(); ++row) {
      for (Eigen::DenseIndex col = 0; col < mat.cols(); ++col) {
        auto coeffs = (Eigen::Matrix<CoefficientType, Eigen::Dynamic, 1>::Random(num_coefficients_per_polynomial)).eval();
        mat(row, col) = Polynomial<CoefficientType>(coeffs);
      }
    }
    return mat;
  }
};

#endif /* DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_ */
