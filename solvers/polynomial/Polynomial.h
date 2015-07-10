#ifndef DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_
#define DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_

#include <Eigen/Core>
#include <complex>
#include <unsupported/Eigen/Polynomials>
#include <string>
#include <vector>

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
  typedef unsigned int VarType;
  typedef unsigned int PowerType;
  typedef typename Eigen::NumTraits<CoefficientType>::Real RealScalar;
  typedef std::complex<RealScalar> RootType;
  typedef Eigen::Matrix<RootType, Eigen::Dynamic, 1> RootsType;
  
  typedef struct _Monomial {
    CoefficientType coeff;
    std::vector<VarType> vars;  // a list of N variable ids
    std::vector<PowerType> powers; // a list of N exponents
  } Monomial;

private:
  std::vector<Monomial> monomials;
 
public:
  // continue to support the old interface
  Polynomial(Eigen::Ref<Matrix<CoefficientType,Eigen::Dynamic,Eigen::Dynamic> const& coefficients);
  Polynomial(int num_coefficients);
  
  Polynomial(void) {}; 
  Polynomial(const VarsType& _vars, const CoefficientsType& _coefficients, const PowersType& _powers);
  Polynomial(const std::vector<VarType>& _vars, const CoefficientsType& _coefficients, const std::vector<PowerType>& _powers);
  
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
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef Polynomial<double> Polynomiald;

#endif /* DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_ */
