#include "Polynomial.h"
#include <cassert>
#include <algorithm>
#include <cmath>
#include <string.h>

#include <iostream> // just for debugging

using namespace std;
using namespace Eigen;

// todo: change most asserts to runtime errors

template <typename CoefficientType>
bool Polynomial<CoefficientType>::Monomial::hasSameExponents(const Monomial& other)
{
  if (vars.size() != other.vars.size()) 
    return false; 

  int N = vars.size();
  for (int i=0; i<N; i++) {
    bool found_match = false;
    for (int j=0; j<N; j++) {
      if (vars[i] == other.vars[j]) {
        if (powers[i] != other.powers[j])
          return false;
        found_match = true;
        continue;
      }
    }
    if (!found_match) return false;
  }
  return true;
}
        

template <typename CoefficientType>
Polynomial<CoefficientType>::Polynomial(const CoefficientType coefficient, const std::vector<VarType>& vars, const std::vector<PowerType>& powers) 
{
  Monomial m;
  m.coefficient = coefficient;
  m.vars = vars;
  m.powers = powers;

  is_univariate = true;
  for (int i=m.vars.size(); i>=0; i--) {
    if ((i>0) && (m.vars[i] != m.vars[0])) {
      is_univariate = false;
    }
    for (int j=0; j<(i-1); j++) { // merge any duplicate vars
      if (m.vars[i]==m.vars[j]) {
        m.powers[j]+=m.powers[i];
        m.vars.erase(m.vars.begin()+i);
        m.powers.erase(m.powers.begin()+i);
        break;
      }
    }      
  }

  monomials.push_back(m);
} 

template <typename CoefficientType>
Polynomial<CoefficientType>::Polynomial(Eigen::Ref< Matrix<CoefficientType,Dynamic,1> > const& coefficients) 
{
  VarType v = variableNameToId("t");
  for (int i=0; i<coefficients.size(); i++) {
    Monomial m;
    m.coefficient = coefficients(i);
    m.vars.push_back(v);
    m.powers.push_back(i);
    monomials.push_back(m);
  }
  is_univariate = true;
}
  
template <typename CoefficientType>
int Polynomial<CoefficientType>::getNumberOfCoefficients() const {
  return static_cast<int>(monomials.size());
}

template <typename CoefficientType>
int Polynomial<CoefficientType>::getDegree() const {
  int max_degree = 0;
  for (typename vector<Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    int monomial_degree = iter->powers[0];
    for (int i=1; i<iter->powers.size(); i++)
      monomial_degree *= iter->powers[i];
    if (monomial_degree > max_degree) max_degree = monomial_degree; 
  }
    
  return max_degree;
}

template <typename CoefficientType>
Matrix<CoefficientType,Dynamic,1> Polynomial<CoefficientType>::getCoefficients() const {
  assert(is_univariate);
  int deg = getDegree();
 
  Matrix<CoefficientType,Dynamic,1> coefficients = Matrix<CoefficientType,Dynamic,1>::Zero(deg+1);
  for (typename vector<Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    coefficients[iter->powers[0]] = iter->coefficient;
  }  
  return coefficients;
}

template <typename CoefficientType>
Polynomial<CoefficientType> Polynomial<CoefficientType>::derivative(int derivative_order) const {
  assert(is_univariate); // only defined for univariate polynomials
  assert(derivative_order >= 0);

  Polynomial<CoefficientType> ret;
          
  for (typename vector<Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    if (!iter->powers.empty() && iter->powers[0]>=derivative_order) {
      Monomial m=*iter;
      for (int k=0; k<derivative_order; k++) { // take the remaining derivatives
        m.coefficient = m.coefficient*m.powers[0];
        m.powers[0] -= 1;
      }
      ret.monomials.push_back(m);
    }
  }
  return ret;
}

template <typename CoefficientType>
Polynomial<CoefficientType> Polynomial<CoefficientType>::integral(const CoefficientType& integration_constant) const {
  assert(is_univariate); // only defined for univariate polynomials
  Polynomial<CoefficientType> ret=*this;
  
  for (typename vector<Monomial>::iterator iter=ret.monomials.begin(); iter!=ret.monomials.end(); iter++) {
    iter->coefficient /= (RealScalar) (iter->powers[0]+1);
    iter->powers[0] += (PowerType) 1;
  }
  Monomial m;
  m.coefficient = integration_constant;
  ret.monomials.push_back(m);
  return ret;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator+=(const Polynomial<CoefficientType>& other) {
  for (typename vector<Monomial>::const_iterator iter=other.monomials.begin(); iter!=other.monomials.end(); iter++) {
    monomials.push_back(*iter);
  }
  makeMonomialsUnique();
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator-=(const Polynomial<CoefficientType>& other) {
  for (typename vector<Monomial>::const_iterator iter=other.monomials.begin(); iter!=other.monomials.end(); iter++) {
    monomials.push_back(*iter);
    monomials.back().coefficient *= (CoefficientType) (-1);
  }
  makeMonomialsUnique();
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator*=(const Polynomial<CoefficientType>& other) {
  vector<Monomial> new_monomials;

  for (typename vector<Monomial>::const_iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    for (typename vector<Monomial>::const_iterator other_iter=other.monomials.begin(); other_iter!=other.monomials.end(); other_iter++) {
      Monomial m;
      m.coefficient = iter->coefficient*other_iter->coefficient;
      m.vars = iter->vars;
      m.powers = iter->powers;
      for (int i=0; i<other_iter->vars.size(); i++) {
        bool new_var = true;
        for (int j=0; j<m.vars.size(); j++) {
          if (m.vars[j]==other_iter->vars[i]) {
            m.powers[j]+=other_iter->powers[i];
            new_var = false;
            break;
          }
        }
        if (new_var) {
          m.vars.push_back(other_iter->vars[i]);
          m.powers.push_back(other_iter->powers[i]);
        }
      }
      new_monomials.push_back(m);
    }
  }
  monomials = new_monomials;
  
  makeMonomialsUnique();
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator+=(const CoefficientType& scalar) {
  // add to the constant monomial if I have one
  for (typename vector<Monomial>::iterator iter=monomials.begin(); iter!=monomials.end(); iter++) {
    if (iter->vars.empty()) {
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
    if (iter->vars.empty()) {
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
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator*(const Polynomial& other) const {
  Polynomial<CoefficientType> ret = *this;
  ret *= other;
  return ret;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator+(const CoefficientType& scalar) const {
  Polynomial<CoefficientType> ret = *this;
  ret += scalar;
  return ret;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator-(const CoefficientType& scalar) const {
  Polynomial<CoefficientType> ret = *this;
  ret -= scalar;
  return ret;
}

template <typename CoefficientType>
const Polynomial<CoefficientType> Polynomial<CoefficientType>::operator*(const CoefficientType& scalar) const {
  Polynomial<CoefficientType> ret = *this;
  ret *= scalar;
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
  // need to handle degree 0 and 1 explicitly because Eigen's polynomial solver doesn't work for these
  assert(is_univariate);  // only works for univariate polynomials
  
  auto coefficients = getCoefficients();
  int degree = coefficients.size()-1;
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
  int len = name.length();
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
  for (int i=name.size()-1; i>=0; i--) {
    exponent *= num_name_chars+1;
    name_part += (name_chars.find(name[i])+1)*exponent;
  }
  const unsigned int maxId = (1 << 50) / max_name_part;
  assert(m<=maxId);
  assert(m>0);
  return 2*(name_part + max_name_part*(m-1));
}

template <typename CoefficientType>
string Polynomial<CoefficientType>::idToVariableName(const VarType id) {
  VarType name_part = (id/2) % max_name_part;  // id/2 to be compatible w/ msspoly, even though I'm not doing the trig support here

  unsigned int m = std::round(id/2/max_name_part);
  unsigned int exponent = pow(num_name_chars+1,name_length-1);
  char name[name_length+1];
  int j=0;
  for (int i=0; i<name_length; i++) {
    unsigned int name_ind = (name_part/exponent) % (num_name_chars+1);
    if (name_ind>0) name[j++] = name_chars[name_ind-1];
    exponent /= num_name_chars+1;
  }
  if (j==0) name[j++] = name_chars[0];
  name[j] = '\0';
  return string(name) + to_string(m+1);
}

template <typename CoefficientType>
void Polynomial<CoefficientType>::makeMonomialsUnique(void)
{
  for (int i=monomials.size()-1; i>=0; i--) {
    Monomial& mi=monomials[i];
    for (int j=0; j<(i-1); j++) {
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
