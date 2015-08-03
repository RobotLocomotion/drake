#ifndef DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_
#define DRAKE_SOLVERS_POLYNOMIAL_POLYNOMIAL_H_

#include <Eigen/Core>
#include <complex>
#include <unsupported/Eigen/Polynomials>
#include <string>
#include <vector>
#include <random>
#include <stdexcept>

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

template <typename _CoefficientType = double>
class DLLEXPORT Polynomial
{
public:
  typedef _CoefficientType CoefficientType;
  typedef unsigned int VarType;
  typedef unsigned int PowerType;
  typedef int MSVC_STD_POW_PowerType;  // can't believe i have to do this, but msvc considers the call to std::pow ambiguous because it won't cast unsigned int to int
  typedef typename Eigen::NumTraits<CoefficientType>::Real RealScalar;
  typedef std::complex<RealScalar> RootType;
  typedef Eigen::Matrix<RootType, Eigen::Dynamic, 1> RootsType;
  
  template<typename Rhs, typename Lhs>
  struct Product {
    typedef decltype((Rhs) 0 * (Lhs) 0) type;
  };

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
  bool is_univariate;
 
public:
  Polynomial(void) : is_univariate(true) {};
  Polynomial(const CoefficientType& scalar); // this is required for some Eigen operations when used in a polynomial matrix
  Polynomial(const CoefficientType coeff, const std::vector<Term>& terms);
  Polynomial(typename std::vector<Monomial>::const_iterator start, typename std::vector<Monomial>::const_iterator finish);
  Polynomial(const std::string varname, const unsigned int num = 1);
  Polynomial(const CoefficientType& coeff, const VarType& v);

  // continue to support the old (univariate) constructor
  template <typename Derived>
  Polynomial(Eigen::MatrixBase<Derived> const& coefficients)
  {
    VarType v = variableNameToId("t");
    for (int i=0; i<coefficients.size(); i++) {
      Monomial m;
      m.coefficient = coefficients(i);
      if (i>0) {
        Term t;
        t.var = v;
        t.power = i;
        m.terms.push_back(t);
      }
      monomials.push_back(m);
    }
    is_univariate = true;
  }
  
  int getNumberOfCoefficients() const;

  int getDegree() const;

  // getSimpleVariable()
  // if the polynomial is "simple" -- e.g. just a single term with
  // coefficient 1 -- then return that variable.  otherwise return 0
  VarType getSimpleVariable() const;

  const std::vector<Monomial>& getMonomials() const;

  Eigen::Matrix<CoefficientType,Eigen::Dynamic,1> getCoefficients() const;

  template<typename T> // can be different from both CoefficientsType and RealScalar
  typename Product<CoefficientType, T>::type value(const T& x) const
  {
    typedef typename Product<CoefficientType, T>::type ProductType;

    if (!is_univariate) throw std::runtime_error("this method can only be used for univariate polynomials");
    ProductType value = (ProductType) 0;
    for (typename std::vector<Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
      if (iter->terms.empty())
        value += iter->coefficient;
      else
        value += iter->coefficient*std::pow((ProductType)x,(MSVC_STD_POW_PowerType) iter->terms[0].power);
    }
    return value;
  }

  void subs(const VarType& orig, const VarType& replacement);

  Polynomial derivative(unsigned int derivative_order = 1) const;

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

  const Polynomial operator-() const;

  const Polynomial operator*(const Polynomial& other) const;

  friend const Polynomial operator+(const Polynomial& p, const CoefficientType& scalar) {
    Polynomial ret = p;
    ret += scalar;
    return ret;
  }

  friend const Polynomial operator+(const CoefficientType& scalar,const Polynomial& p) {
    Polynomial ret = p;
    ret += scalar;
    return ret;
  }

  friend const Polynomial operator-(const Polynomial& p, const CoefficientType& scalar) {
    Polynomial ret = p;
    ret -= scalar;
    return ret;
  }

  friend const Polynomial operator-(const CoefficientType& scalar,const Polynomial& p) {
    Polynomial ret = -p;
    ret += scalar;
    return ret;
  }

  friend const Polynomial operator*(const Polynomial& p, const CoefficientType& scalar) {
    Polynomial ret = p;
    ret *= scalar;
    return ret;
  }
  friend const Polynomial operator*(const CoefficientType& scalar,const Polynomial& p) {
    Polynomial ret = p;
    ret *= scalar;
    return ret;
  }

  const Polynomial operator/(const CoefficientType& scalar) const;

  RootsType roots() const;

  bool isApprox(const Polynomial& other, const RealScalar& tol) const;

  friend std::ostream& operator<<(std::ostream& os, const Monomial& m)
  {
//    if (m.coefficient == 0) return os;

    bool print_star = false;
    if (m.coefficient == -1) { os << "-"; }
    else if (m.coefficient != 1 || m.terms.empty()) { os << '(' << m.coefficient << ")"; print_star=true; }

    for (typename std::vector<Term>::const_iterator iter=m.terms.begin(); iter!=m.terms.end(); iter++) {
      if (print_star) os << '*';
      else print_star = true;
      os << idToVariableName(iter->var);
      if (iter->power!=1) {
        os << "^" << iter->power;
      }
    }
    return os;
  }
  
  friend std::ostream& operator<<(std::ostream& os, const Polynomial& poly)
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
  }
  
static Eigen::Matrix<Polynomial, Eigen::Dynamic, Eigen::Dynamic> randomPolynomialMatrix(Eigen::DenseIndex num_coefficients_per_polynomial, Eigen::DenseIndex rows, Eigen::DenseIndex cols)
  {
    Eigen::Matrix<Polynomial, Eigen::Dynamic, Eigen::Dynamic> mat(rows, cols);
    for (Eigen::DenseIndex row = 0; row < mat.rows(); ++row) {
      for (Eigen::DenseIndex col = 0; col < mat.cols(); ++col) {
        auto coeffs = (Eigen::Matrix<CoefficientType, Eigen::Dynamic, 1>::Random(num_coefficients_per_polynomial)).eval();
        mat(row, col) = Polynomial(coeffs);
      }
    }
    return mat;
  }

private:
  static bool isValidVariableName(const std::string name);
  
  static VarType variableNameToId(const std::string name, const unsigned int m = 1);
  
  static std::string idToVariableName(const VarType id);
  
  // sorts through monomial list and merges any that have the same powers
  void makeMonomialsUnique(void);
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
