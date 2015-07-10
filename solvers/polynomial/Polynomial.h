#ifndef DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_
#define DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_

#include <Eigen/Core>
#include <complex>
#include <unsupported/Eigen/Polynomials>
#include <string>

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
  typedef typename Eigen::Matrix<CoefficientType, Eigen::Dynamic, 1> CoefficientsType;
  typedef unsigned int VarType;
  typedef typename Eigen::Matrix<VarType, Eigen::Dynamic, 1> VarsType;
  typedef unsigned int PowerType;
  typedef typename Eigen::Matrix<PowerType, Eigen::Dynamic, Eigen::Dynamic> PowersType;
  typedef typename Eigen::NumTraits<CoefficientType>::Real RealScalar;
  typedef std::complex<RealScalar> RootType;
  typedef Eigen::Matrix<RootType, Eigen::Dynamic, 1> RootsType;

private:
  VarsType vars;  // list of variable IDs with length N
  CoefficientsType coefficients;    // list of M monomial coefficients
  PowersType powers; // M by N list of powers: powers(i,j) is the exponent of variable(j) in monomial i
 
public:
  Polynomial(Eigen::Ref<CoefficientsType> const& coefficients);

  Polynomial(int num_coefficients);

  int getNumberOfCoefficients() const;

  int getDegree() const;

  CoefficientsType const& getCoefficients() const;

  template<typename T> // can be different from both CoefficientsType and RealScalar
  T value(const T& t) const
  {
    assert(vars.size()==1);  // this method can only be used for univariate polynomials
    T value = coefficients(0)*pow(t,powers(0,0));
    for (int i=1; i<coefficients.size(); i++) 
      value += coefficients(i)*pow(t,powers(i,0));
    return value;
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
  
  friend std::ostream& operator<<(std::ostream& os, const Polynomial<CoefficientType>& poly)
  {
    for (int i=0; i<poly.getNumberOfCoefficients(); i++) {
      os << '(' << poly.coefficients(i) << ')';
      for (int j=0; j<poly.vars.size(); j++) {
        if (poly.powers(i,j)>0) {
          os << '*' << poly.idToVariableName(poly.vars(j));
          if (poly.powers(i,j)>1) {
            os << "^" << poly.powers(i,j);
          }
        }
      }
      if (i<poly.getNumberOfCoefficients()-1)
        os << '+';
    }
    return os;
  };
  

private:
  bool isValidVariableName(const std::string name) const;
  
  VarType variableNameToId(const std::string name, const unsigned int m = 1) const;
  
  std::string idToVariableName(const VarType id) const;
  
  // updates the vars element in this
  // this.vars are first N elements
  // the returned matrix is a projection matrix mapping from the indices of other.vars to the new vars
  Eigen::Matrix<VarType, Eigen::Dynamic, Eigen::Dynamic> mergeVars(const Polynomial& other);
  
  // sorts through monomial list and merges any that have the same powers
  void makeMonomialsUnique(void);
};

typedef Polynomial<double> Polynomiald;

#endif /* DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_ */
