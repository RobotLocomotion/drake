#include "Polynomial.h"
#include <algorithm>
#include <cmath>
#include <string.h>
#include <stdexcept>
#include <limits>

#include <iostream> // just for debugging

using namespace std;
using namespace Eigen;

template <typename CoefficientType>
bool Polynomial<CoefficientType>::Monomial::hasSameExponents(const Monomial& other)
{
  if (terms.size() != other.terms.size())
    return false; 

  for (typename vector<Term>::const_iterator iter=terms.begin(); iter!=terms.end(); iter++) {
    typename vector<Term>::const_iterator match = find(other.terms.begin(),other.terms.end(),*iter);
    if (match==other.terms.end()) return false;
  }
  return true;
}
        
template <typename CoefficientType>
Polynomial<CoefficientType>::Polynomial(const CoefficientType& scalar)
{
  Monomial m;
  m.coefficient = scalar;
  monomials.push_back(m);
  is_univariate=true;
}

template <typename CoefficientType>
Polynomial<CoefficientType>::Polynomial(const CoefficientType coefficient, const vector<Term>& terms)
{
  Monomial m;
  m.coefficient = coefficient;
  m.terms = terms;

  is_univariate = true;
  for (int i=(int)m.terms.size()-1; i>=0; i--) {
    if ((i>0) && (m.terms[i].var != m.terms[0].var)) {
      is_univariate = false;
    }
    for (int j=0; j<(i-1); j++) { // merge any duplicate vars
      if (m.terms[i].var==m.terms[j].var) {
        m.terms[j].power+=m.terms[i].power;
        m.terms.erase(m.terms.begin()+i);
        break;
      }
    }      
  }

  monomials.push_back(m);
} 

template <typename CoefficientType>
Polynomial<CoefficientType>::Polynomial(typename vector< typename Polynomial<CoefficientType>::Monomial >::const_iterator start, typename vector< typename Polynomial<CoefficientType>::Monomial >::const_iterator finish)
{
  is_univariate = true;
  for (typename vector< typename Polynomial<CoefficientType>::Monomial >::const_iterator iter=start; iter!=finish; iter++)
    monomials.push_back(*iter);
  makeMonomialsUnique();
}

template <typename CoefficientType>
Polynomial<CoefficientType>::Polynomial(const string varname, const unsigned int num)
{
  Monomial m;
  m.coefficient = (CoefficientType) 1;
  Term t;
  t.var = variableNameToId(varname,num);
  t.power = 1;
  m.terms.push_back(t);
  monomials.push_back(m);
  is_univariate = true;
}

template <typename CoefficientType>
Polynomial<CoefficientType>::Polynomial(const CoefficientType& coeff, const VarType& v)
{
  Monomial m;
  m.coefficient = coeff;
  Term t;
  t.var = v;
  t.power = 1;
  m.terms.push_back(t);
  monomials.push_back(m);
  is_univariate = true;
}


template <typename CoefficientType>
int Polynomial<CoefficientType>::getNumberOfCoefficients() const {
  return static_cast<int>(monomials.size());
}

template <typename CoefficientType>
int Polynomial<CoefficientType>::Monomial::getDegree() const {
  if (terms.empty()) return 0;
  int degree = terms[0].power;
  for (int i=1; i<terms.size(); i++)
    degree *= terms[i].power;
  return degree;
}

template <typename CoefficientType>
int Polynomial<CoefficientType>::getDegree() const {
  int max_degree = 0;
  for (typename vector<Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    int monomial_degree = iter->getDegree();
    if (monomial_degree > max_degree) max_degree = monomial_degree; 
  }
    
  return max_degree;
}

template <typename CoefficientType>
typename Polynomial<CoefficientType>::VarType Polynomial<CoefficientType>::getSimpleVariable() const {
  if (monomials.size()!=1) return 0;
  if (monomials[0].terms.size()!=1) return 0;
  if (monomials[0].terms[0].power!=1) return 0;
  return monomials[0].terms[0].var;
}

template <typename CoefficientType>
const std::vector<typename Polynomial<CoefficientType>::Monomial>& Polynomial<CoefficientType>::getMonomials() const {
 return monomials;
}

template <typename CoefficientType>
Matrix<CoefficientType,Dynamic,1> Polynomial<CoefficientType>::getCoefficients() const {
  if (!is_univariate)
    throw runtime_error("getCoefficients is only defined for univariate polynomials");

  int deg = getDegree();
 
  Matrix<CoefficientType,Dynamic,1> coefficients = Matrix<CoefficientType,Dynamic,1>::Zero(deg+1);
  for (typename vector<Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    if (iter->terms.empty())
      coefficients[0] = iter->coefficient;
    else
      coefficients[iter->terms[0].power] = iter->coefficient;
  }  
  return coefficients;
}

template <typename CoefficientType>
void Polynomial<CoefficientType>::subs(const VarType& orig, const VarType& replacement) {
  for (typename vector<Monomial>::iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    for (typename vector<Term>::iterator t=iter->terms.begin(); t!=iter->terms.end(); t++) {
      if (t->var == orig) t->var = replacement;
    }
  }
}

template <typename CoefficientType>
Polynomial<CoefficientType> Polynomial<CoefficientType>::derivative(unsigned int derivative_order) const {
  if (!is_univariate)
    throw runtime_error("getCoefficients is only defined for univariate polynomials");

  Polynomial<CoefficientType> ret;
          
  for (typename vector<Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    if (!iter->terms.empty() && iter->terms[0].power>=derivative_order) {
      Monomial m=*iter;
      for (unsigned int k=0; k<derivative_order; k++) { // take the remaining derivatives
        m.coefficient = m.coefficient*m.terms[0].power;
        m.terms[0].power -= 1;
      }
      if (m.terms[0].power<1) m.terms.erase(m.terms.begin());
      ret.monomials.push_back(m);
    }
  }
  ret.is_univariate = true;
  return ret;
}

template <typename CoefficientType>
Polynomial<CoefficientType> Polynomial<CoefficientType>::integral(const CoefficientType& integration_constant) const {
  if (!is_univariate)
    throw runtime_error("getCoefficients is only defined for univariate polynomials");
  Polynomial<CoefficientType> ret=*this;
  
  for (typename vector<Monomial>::iterator iter=ret.monomials.begin(); iter!=ret.monomials.end(); iter++) {
    if (iter->terms.empty()) {
      Term t;
      t.var=0;
      for (typename vector<Monomial>::iterator iterB=ret.monomials.begin(); iterB!=ret.monomials.end(); iterB++) {
        if (!iterB->terms.empty()) {
          t.var = iterB->terms[0].var;
          break;
        }
      }
      if (t.var<1) throw runtime_error("don't know the variable name");
      t.power = 1;
      iter->terms.push_back(t);
    } else {
      iter->coefficient /= (RealScalar) (iter->terms[0].power+1);
      iter->terms[0].power += (PowerType) 1;
    }
  }
  Monomial m;
  m.coefficient = integration_constant;
  ret.is_univariate = true;
  ret.monomials.push_back(m);
  return ret;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator+=(const Polynomial<CoefficientType>& other) {
  for (typename vector<Monomial>::const_iterator iter=other.monomials.begin(); iter!=other.monomials.end(); iter++) {
    monomials.push_back(*iter);
  }
  makeMonomialsUnique(); // also sets is_univariate false if necessary
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator-=(const Polynomial<CoefficientType>& other) {
  for (typename vector<Monomial>::const_iterator iter=other.monomials.begin(); iter!=other.monomials.end(); iter++) {
    monomials.push_back(*iter);
    monomials.back().coefficient *= (CoefficientType) (-1);
  }
  makeMonomialsUnique(); // also sets is_univariate false if necessary
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator*=(const Polynomial<CoefficientType>& other) {
  vector<Monomial> new_monomials;

  for (typename vector<Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    for (typename vector<Monomial>::const_iterator other_iter=other.monomials.begin(); other_iter!=other.monomials.end(); other_iter++) {
      Monomial m;
      m.coefficient = iter->coefficient*other_iter->coefficient;
      m.terms = iter->terms;
      for (int i=0; i<other_iter->terms.size(); i++) {
        bool new_var = true;
        for (int j=0; j<m.terms.size(); j++) {
          if (m.terms[j].var==other_iter->terms[i].var) {
            m.terms[j].power+=other_iter->terms[i].power;
            new_var = false;
            break;
          }
        }
        if (new_var) {
          m.terms.push_back(other_iter->terms[i]);
        }
      }
      new_monomials.push_back(m);
    }
  }
  monomials = new_monomials;
  
  makeMonomialsUnique(); // also sets is_univariate false if necessary
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator+=(const CoefficientType& scalar) {
  // add to the constant monomial if I have one
  for (typename vector<Monomial>::iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    if (iter->terms.empty()) {
      iter->coefficient += scalar;
      return *this;
    }
  }

  // otherwise create the constant monomial
  Monomial m;
  m.coefficient = scalar;
  monomials.push_back(m);
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator-=(const CoefficientType& scalar) {
  // add to the constant monomial if I have one
  for (typename vector<Monomial>::iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    if (iter->terms.empty()) {
      iter->coefficient -= scalar;
      return *this;
    }
  }

  // otherwise create the constant monomial
  Monomial m;
  m.coefficient = -scalar;
  monomials.push_back(m);
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator*=(const CoefficientType& scalar) {
  for (typename vector<Monomial>::iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    iter->coefficient *= scalar;
  }
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator/=(const CoefficientType& scalar) {
  for (typename vector<Monomial>::iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    iter->coefficient /= scalar;
  }
  return *this;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator+(const Polynomial& other) const {
  Polynomial<CoefficientType> ret = *this;
  ret += other;
  return ret;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator-(const Polynomial& other) const {
  Polynomial<CoefficientType> ret = *this;
  ret -= other;
  return ret;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator-() const {
  Polynomial<CoefficientType> ret = *this;
  for (typename vector<Monomial>::iterator iter=ret.monomials.begin(); iter!=ret.monomials.end(); iter++) {
    iter->coefficient = -iter->coefficient;
  }
  return ret;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator*(const Polynomial& other) const {
  Polynomial<CoefficientType> ret = *this;
  ret *= other;
  return ret;
}


template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator/(const CoefficientType& scalar) const {
  Polynomial<CoefficientType> ret = *this;
  ret /= scalar;
  return ret;
}

template <typename CoefficientType>
typename Polynomial<CoefficientType>::RootsType Polynomial<CoefficientType>::roots() const {
  if (!is_univariate)
    throw runtime_error("getCoefficients is only defined for univariate polynomials");
  
  auto coefficients = getCoefficients();

  // need to handle degree 0 and 1 explicitly because Eigen's polynomial solver doesn't work for these
  int degree = static_cast<int>(coefficients.size())-1;
  switch (degree) {
  case 0:
    return Polynomial<CoefficientType>::RootsType(degree);
  case 1: {
    Polynomial<CoefficientType>::RootsType ret(degree);
    ret[0] = -coefficients[0] / coefficients[1];
    return ret;
  }
  default: {
    PolynomialSolver<RealScalar, Eigen::Dynamic> solver;
    solver.compute(coefficients);
    return solver.roots();
    break;
  }
  }
}

template <typename CoefficientType>
bool Polynomial<CoefficientType>::isApprox(const Polynomial& other, const RealScalar& tol) const {
  return getCoefficients().isApprox(other.getCoefficients(), tol);
}

const string name_chars = "@#_.abcdefghijklmnopqrstuvwxyz";
const unsigned int num_name_chars = 30; // length of the string above
const unsigned int name_length = 4;
const unsigned int max_name_part = 923521; // (num_name_chars+1)^name_length;

template <typename CoefficientType>
bool Polynomial<CoefficientType>::isValidVariableName(const string name) {
  size_t len = name.length();
  if (len<1) return false;
  for (int i=0; i<len; i++)
    if (name_chars.find(name[i]) == string::npos)
      return false;
  return true;
}

template <typename CoefficientType>
typename Polynomial<CoefficientType>::VarType Polynomial<CoefficientType>::variableNameToId(const string name, const unsigned int m) {
  unsigned int exponent = 1;
  VarType name_part = 0;
  for (int i=(int)name.size()-1; i>=0; i--) {
    exponent *= num_name_chars+1;
    name_part += ((VarType)name_chars.find(name[i])+1)*exponent;
  }
  const VarType maxId = std::numeric_limits<VarType>::max() / 2 / max_name_part;
  if (m>maxId) throw runtime_error("name exceeds max ID");
  if (m<1) throw runtime_error("m must be >0");
  return (VarType) 2*(name_part + max_name_part*(m-1));
}

template <typename CoefficientType>
string Polynomial<CoefficientType>::idToVariableName(const VarType id) {
  VarType name_part = (id/2) % max_name_part;  // id/2 to be compatible w/ msspoly, even though I'm not doing the trig support here

  unsigned int m = id/2/max_name_part;
  unsigned int exponent = (unsigned int) std::pow((double)(num_name_chars+1),(int)(name_length-1));
  char name[name_length+1];
  int j=0;
  for (int i=0; i<name_length; i++) {
    unsigned int name_ind = (name_part/exponent) % (num_name_chars+1);
    if (name_ind>0) name[j++] = name_chars[name_ind-1];
    exponent /= num_name_chars+1;
  }
  if (j==0) name[j++] = name_chars[0];
  name[j] = '\0';
  return string(name) + std::to_string(static_cast<unsigned long long>(m+1)); // for msvc http://stackoverflow.com/questions/10664699/stdto-string-more-than-instance-of-overloaded-function-matches-the-argument
}

template <typename CoefficientType>
void Polynomial<CoefficientType>::makeMonomialsUnique(void)
{
  VarType unique_var = 0;    // also update the univariate flag
  for (ptrdiff_t i=monomials.size()-1; i>=0; i--) {
    Monomial& mi=monomials[i];
    if (!mi.terms.empty()) {
      if (mi.terms.size()>1) is_univariate = false;
      if (mi.terms[0].var != unique_var) {
        if (unique_var>0) {
          is_univariate = false;
        } else {
          unique_var = mi.terms[0].var;
        }
      }
    }
    for (ptrdiff_t j=0; j<(i-1); j++) {
      Monomial& mj=monomials[j];
      if (mi.hasSameExponents(mj)) {
        // it's a match, so delete monomial i
        monomials[j].coefficient += monomials[i].coefficient;
        monomials.erase(monomials.begin()+i);
        break;
      }
    }
  }
}


template class DLLEXPORT Polynomial<double>;

//template class DLLEXPORT Polynomial<std::complex<double>>; // doesn't work yet because the roots solver can't handle it
