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
  
  class Term {
  public:
    VarType var;
    PowerType power;

    bool operator==(const Term& other) const
    {
      return (var == other.var) && (power == other.power);
    }

  };

  class Monomial {
  public:
    CoefficientType coefficient;
    std::vector<Term> terms;  // a list of N variable ids
    
    int getDegree() const;
    bool hasSameExponents(const Monomial& other);
  };

private:
  std::vector<Monomial> monomials;
  bool is_univariate = true;
 
public:
  Polynomial(void) {}; 
  Polynomial(CoefficientType coeff, const std::vector<Term>& terms);

  // continue to support the old (univariate) constructor
  Polynomial(Eigen::Ref<Eigen::Matrix<CoefficientType,Eigen::Dynamic,1> > const& coefficients);
  
  int getNumberOfCoefficients() const;

  int getDegree() const;

  const std::vector<Monomial>& getMonomials() const;

  Eigen::Matrix<CoefficientType,Eigen::Dynamic,1> getCoefficients() const;

  template<typename T> // can be different from both CoefficientsType and RealScalar
  T value(const T& x) const
  {
    assert(is_univariate);  // this method can only be used for univariate polynomials
    T value = (T) 0;
    for (typename std::vector<Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
      if (iter->terms.empty())
        value += iter->coefficient;
      else
        value += iter->coefficient*pow(x,iter->terms[0].power);
    }
    return value;
  }

  Polynomial derivative(int derivative_order = 1) const;

  Polynomial integral(const CoefficientType& integration_constant = 0.0) const;

  Polynomial operator-() const;

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

  friend std::ostream& operator<<(std::ostream& os, const Monomial& m)
  {
//    if (m.coefficient == 0) return os;

    bool print_star = false;
    if (m.coefficient == -1) { os << "-"; }
    else if (m.coefficient != 1) { os << '(' << m.coefficient << ")"; print_star=true; }

    for (typename std::vector<Term>::const_iterator iter=m.terms.begin(); iter!=m.terms.end(); iter++) {
      if (print_star) os << '*';
      else print_star = true;
      os << idToVariableName(iter->var);
      if (iter->power!=1) {
        os << "^" << iter->power;
      }
    }
    return os;
  };
  
  friend std::ostream& operator<<(std::ostream& os, const Polynomial<CoefficientType>& poly)
  {
    if (poly.monomials.empty()) {
      os << "0";
      return os;
    }
    
    for (typename std::vector<Monomial>::const_iterator iter=poly.monomials.begin(); iter!=poly.monomials.end(); iter++) {
      os << *iter;
      if (iter+1 != poly.monomials.end() && (iter+1)->coefficient!=-1)
        os << '+';
    }
    return os;
  };
  

private:
  static bool isValidVariableName(const std::string name);
  
  static VarType variableNameToId(const std::string name, const unsigned int m = 1);
  
  static std::string idToVariableName(const VarType id);
  
  // sorts through monomial list and merges any that have the same powers
  void makeMonomialsUnique(void);
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


template <typename CoefficientType, int Rows, int Cols>
std::ostream& operator<<(std::ostream& os, const Eigen::Matrix< Polynomial<CoefficientType> , Rows, Cols > & poly_mat) {
  for (int i=0; i<poly_mat.rows(); i++) {
    os << "[ ";
    for (int j=0; j<poly_mat.cols(); j++) {
      os << poly_mat(i,j);
      if (j<(poly_mat.cols()-1))
        os << " , ";
    }
    os << " ]" << std::endl;
  }
  return os;
}

typedef Polynomial<double> Polynomiald;

#endif /* DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_ */
