#include "Polynomial.h"
#include <cassert>
#include <algorithm>
#include <cmath>
#include <string.h>

#include <iostream> // just for debugging

using namespace std;
using namespace Eigen;

template <typename CoefficientType>
Polynomial<CoefficientType>::Polynomial(Eigen::Ref<CoefficientsType> const& coefficients) :
  coefficients(coefficients), powers(coefficients.size(),1), vars(1)
{
  assert(coefficients.rows() > 0);
  powers = Matrix<PowerType,Eigen::Dynamic,1>::LinSpaced(coefficients.size(),0,coefficients.size()-1);
  vars(0) = variableNameToId("t"); // default variable name is "t"
}

template <typename CoefficientType>
Polynomial<CoefficientType>::Polynomial(int num_coefficients) :
  coefficients(num_coefficients), powers(num_coefficients,1), vars(1)
{
  assert(coefficients.rows() > 0);
  powers = Matrix<PowerType,Eigen::Dynamic,1>::LinSpaced(coefficients.size(),0,coefficients.size()-1);
  vars(0) = variableNameToId("t"); // default variable name is "t"
}

template <typename CoefficientType>
Polynomial<CoefficientType>::Polynomial(const VarsType& _vars, const CoefficientsType& _coefficients, const PowersType& _powers) :
  vars(_vars), coefficients(_coefficients), powers(_powers)
{
  assert(vars.size()==coefficients.cols());
  assert(vars.size()==powers.cols());
  assert(coefficients.rows()==powers.rows());
} 
  
template <typename CoefficientType>
Polynomial<CoefficientType>::Polynomial(const vector<VarType>& _vars, const CoefficientsType& _coefficients, const vector<PowerType>& _powers) :
  coefficients(_coefficients)
{
  assert(_vars.size()==coefficients.cols());
  assert(_vars.size()==_powers.size());
  assert(coefficients.rows()==1);
  
  vars.resize(_vars.size());
  powers.resize(1,_powers.size());

  for (int i=0; i<vars.size(); i++)
    vars(i) = _vars[i];
  for (int i=0; i<_powers.size(); i++)
    powers(1,i) = _powers[i];
} 

template <typename CoefficientType>
int Polynomial<CoefficientType>::getNumberOfCoefficients() const {
  return static_cast<int>(coefficients.size());
}

template <typename CoefficientType>
int Polynomial<CoefficientType>::getDegree() const {
  int max_degree = 0;
  for (int i=0; i<powers.rows(); i++) {
    int monomial_degree = powers.row(i).prod();
    if (monomial_degree > max_degree) max_degree = monomial_degree; 
  }
    
  return max_degree;
}

template <typename CoefficientType>
typename Polynomial<CoefficientType>::CoefficientsType const& Polynomial<CoefficientType>::getCoefficients() const {
  return coefficients;
}

template <typename CoefficientType>
Polynomial<CoefficientType> Polynomial<CoefficientType>::derivative(int derivative_order) const {
  assert(vars.size()==1); // only defined for univariate polynomials
  assert(derivative_order >= 0);
  int derivative_num_coefficients = 0;
  for (int i=0; i < powers.rows(); i++) 
    if (powers(i,0)>=derivative_order) 
      derivative_num_coefficients++;
  
  if (derivative_num_coefficients <= 0)
    return Polynomial<CoefficientType>::zero();

  Polynomial<CoefficientType> ret(derivative_num_coefficients);
  ret.vars(0) = vars(0);
          
  int j=0;
  for (int i = 0; i < powers.rows(); i++) {
    if (powers(i,0)>=derivative_order) {
      ret.coefficients(j) = coefficients(i)*powers(i,0);
      ret.powers(j,0) = powers(i,0)-1;
      for (int k=1; k<derivative_order; k++) { // take the remaining derivatives
        ret.coefficients(j) = ret.coefficients(j)*ret.powers(j,0);
        ret.powers(j,0) -= 1;
      }
      j++;
    }
  }
  return ret;
}

template <typename CoefficientType>
Polynomial<CoefficientType> Polynomial<CoefficientType>::integral(const CoefficientType& integration_constant) const {
  assert(vars.size()==1); // only defined for univariate polynomials
  Polynomial<CoefficientType> ret(getNumberOfCoefficients() + 1);
  ret.vars(0) = vars(0);
  ret.coefficients(0) = integration_constant;
  ret.powers(0,0) = 0;
  for (int i = 0; i < getNumberOfCoefficients(); i++) {
    ret.coefficients(i+1) = coefficients(i) / (RealScalar) (powers(i,0)+1);
    ret.powers(i+1,0) = powers(i,0)+1;
  }
  return ret;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator+=(const Polynomial<CoefficientType>& other) {
  auto P = mergeVars(other);
  
  coefficients.conservativeResize(coefficients.size()+other.getNumberOfCoefficients());
  coefficients.tail(other.getNumberOfCoefficients()) = other.coefficients;
  
  int M=powers.rows(), N=powers.cols();
  powers.conservativeResize(coefficients.size(),vars.size());
  powers.leftCols(vars.size()-N).setZero();
  powers.bottomRows(powers.rows()-M) = other.powers*P.transpose();
  
  makeMonomialsUnique();
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator-=(const Polynomial<CoefficientType>& other) {
  auto P = mergeVars(other);
  
  coefficients.conservativeResize(coefficients.size()+other.getNumberOfCoefficients());
  coefficients.tail(other.getNumberOfCoefficients()) = -other.coefficients;
  
  int M=powers.rows(), N=powers.cols();
  powers.conservativeResize(coefficients.size(),vars.size());
  powers.leftCols(vars.size()-N).setZero();
  powers.bottomRows(powers.rows()-M) = other.powers*P.transpose();
  
  makeMonomialsUnique();
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator*=(const Polynomial<CoefficientType>& other) {
  auto original_coefficients = coefficients;
  auto original_powers = powers;
  
  auto P = mergeVars(other);
  coefficients.resize(original_coefficients.size()*other.getNumberOfCoefficients());
  powers.resize(original_coefficients.size()*other.getNumberOfCoefficients(),P.rows());
  
  P.transposeInPlace();
  int k=0;
  for (int i=0; i<original_coefficients.size(); i++) {
    for (int j=0; j<other.getNumberOfCoefficients(); j++) {
      coefficients(k) = original_coefficients(i)*other.coefficients(j);
      powers.row(k) = other.powers.row(j)*P;
      powers.row(k).head(original_powers.cols()) += original_powers.row(i);
      k++;
    }
  }
  
  makeMonomialsUnique();
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator+=(const CoefficientType& scalar) {
  // add to the constant monomial if I have one
  for (int i=0; i<powers.rows(); i++) {
    if (powers.row(i).isZero()) {
      coefficients(i) += scalar;
      return *this;
    }
  }

  // otherwise create the constant monomial
  coefficients.conservativeResize(coefficients.size()+1);
  coefficients(coefficients.size()-1)=scalar;
  powers.conservativeResize(coefficients.size(),powers.cols());
  powers.bottomRows(1).setZero();
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator-=(const CoefficientType& scalar) {
  // add to the constant monomial if I have one
  for (int i=0; i<powers.rows(); i++) {
    if (powers.row(i).isZero()) {
      coefficients(i) -= scalar;
      return *this;
    }
  }

  // otherwise create the constant monomial
  coefficients.conservativeResize(coefficients.size()+1);
  coefficients(coefficients.size()-1)=-scalar;
  powers.conservativeResize(coefficients.size(),powers.cols());
  powers.bottomRows(1).setZero();
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator*=(const CoefficientType& scalar) {
  coefficients *= scalar;
  return *this;
}

template <typename CoefficientType>
Polynomial<CoefficientType>& Polynomial<CoefficientType>::operator/=(const CoefficientType& scalar) {
  coefficients /= scalar;
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
  assert(vars.size()==1);  // only works for univariate polynomials
  int degree = getDegree();
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
  return coefficients.isApprox(other.coefficients, tol);
}

template <typename CoefficientType>
Polynomial<CoefficientType> Polynomial<CoefficientType>::zero() {
  Polynomial<CoefficientType> ret(1);
  ret.coefficients(0) = (CoefficientType) 0;
  return ret;
}

const string name_chars = "@#_.abcdefghijklmnopqrstuvwxyz";
const unsigned int num_name_chars = 30; // length of the string above
const unsigned int name_length = 4;
const unsigned int max_name_part = 923521; // (num_name_chars+1)^name_length;

template <typename CoefficientType>
bool Polynomial<CoefficientType>::isValidVariableName(const string name) const {
  int len = name.length();
  if (len<1) return false;
  for (int i=0; i<len; i++)
    if (name_chars.find(name[i]) == string::npos)
      return false;
  return true;
}

template <typename CoefficientType>
typename Polynomial<CoefficientType>::VarType Polynomial<CoefficientType>::variableNameToId(const string name, const unsigned int m) const {
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
string Polynomial<CoefficientType>::idToVariableName(const VarType id) const {
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
Eigen::Matrix<typename Polynomial<CoefficientType>::VarType, Eigen::Dynamic, Eigen::Dynamic> Polynomial<CoefficientType>::mergeVars(const Polynomial& other) {
  Matrix<typename Polynomial<CoefficientType>::VarType, Dynamic, Dynamic> var_projection_matrix = Matrix<VarType, Dynamic, Dynamic>::Zero(vars.size()+other.vars.size(),other.vars.size());        
  unsigned int num_unique_vars = vars.size();
  vars.conservativeResize(vars.size()+other.vars.size(),1);
  for (int i=0; i<other.vars.size(); i++) {
    bool is_unique = true;
    for (int j=0; j<powers.cols(); j++) {
      if (other.vars(i) == vars(j)) {      
        var_projection_matrix(j,i) = 1;
        is_unique = false;
        break;
      }
    }
    if (is_unique) {
      vars(num_unique_vars) = other.vars(i);
      var_projection_matrix(num_unique_vars,i) = 1;
      num_unique_vars++;
    }
  }
  // trim the fat
  var_projection_matrix.conservativeResize(num_unique_vars,var_projection_matrix.cols());
  vars.conservativeResize(num_unique_vars);
  return var_projection_matrix;
}

template <typename CoefficientType>
void Polynomial<CoefficientType>::makeMonomialsUnique(void)
{
  // build projection matrix as we iterate through the list
  Matrix<CoefficientType, Dynamic,Dynamic> cP = Matrix<CoefficientType, Dynamic,Dynamic>::Zero(powers.rows(),powers.rows());
  Matrix<PowerType, Dynamic,Dynamic> pP = Matrix<PowerType, Dynamic,Dynamic>::Zero(powers.rows(),powers.rows());
  int unique_monomials=0;
  for (unsigned int i=0; i<powers.rows(); i++) {
    bool is_unique = true;
    for (unsigned int j=i+1; j<powers.rows(); j++) {
      if ((powers.row(i)-powers.row(j)).isZero()) {
        // i is a duplicate, add it into element j and do not include it in the projection matrix
        coefficients(j)+=coefficients(i);
        is_unique = false;
        break;
      } 
    }
    if (is_unique) {
      cP(unique_monomials,i) = (CoefficientType) 1;
      pP(unique_monomials,i) = (PowerType) 1;
      unique_monomials++;
    }
  }
  cP.conservativeResize(unique_monomials,cP.cols());  // trim excess rows
  pP.conservativeResize(unique_monomials,pP.cols());
  coefficients = cP*coefficients;
  powers = pP*powers;
}


template class DLLEXPORT Polynomial<double>;

//template class DLLEXPORT Polynomial<std::complex<double>>; // doesn't work yet because the roots solver can't handle it
