#ifndef DRAKE_UTIL_TRIGPOLY_H_
#define DRAKE_UTIL_TRIGPOLY_H_

#include "Polynomial.h"
#include <map>
//#include <iostream>  // just for debugging

template<typename _CoefficientType = double>
class TrigPoly
{
public:
  typedef _CoefficientType CoefficientType;
  typedef Polynomial<CoefficientType> PolyType;
  struct SinCosVars
  {
    typename PolyType::VarType s;
    typename PolyType::VarType c;
  };
  typedef std::map<typename PolyType::VarType, SinCosVars> SinCosMap;

  TrigPoly() {}

  TrigPoly(const CoefficientType& scalar)
    : poly(scalar)
  {}

  TrigPoly(const PolyType& p, SinCosMap _sin_cos_map)
    : poly(p), sin_cos_map(_sin_cos_map)
  {}

  TrigPoly(const PolyType& q, const PolyType& s, const PolyType& c)
  {
    if ((q.getDegree() != 1) || (s.getDegree() != 1) || (c.getDegree() != 1))
      throw std::runtime_error("q, s, and c must all be simple polynomails (in the msspoly sense)");

    poly = q;
    SinCosVars sc;
    sc.s = s.getSimpleVariable();
    sc.c = c.getSimpleVariable();
    sin_cos_map[q.getSimpleVariable()] = sc;
  }

  const PolyType& getPolynomial(void) const {
    return poly;
  }

  friend TrigPoly sin(const TrigPoly& p)
  {
    if (p.poly.getDegree() > 1)
      throw std::runtime_error("sin of polynomials with degree > 1 is not supported");

    if (p.poly.getMonomials().size()>1)
      throw std::runtime_error("have to handle this case (with the chain rule, etc)");

    TrigPoly ret = p;
    for (typename SinCosMap::iterator iter=ret.sin_cos_map.begin(); iter!=ret.sin_cos_map.end(); iter++) {
      ret.poly.subs(iter->first,iter->second.s);
    }
    return ret;
  }

  friend TrigPoly cos(const TrigPoly& p)
  {
    if (p.poly.getDegree() > 1)
      throw std::runtime_error("cos of polynomials with degree > 1 is not supported");

    if (p.poly.getMonomials().size()>1)
      throw std::runtime_error("have to handle this case (with the chain rule, etc)");

    TrigPoly ret = p;
    for (typename SinCosMap::iterator iter=ret.sin_cos_map.begin(); iter!=ret.sin_cos_map.end(); iter++)
      ret.poly.subs(iter->first,iter->second.c);
    return ret;
  }

  TrigPoly& operator+=(const TrigPoly& other)
  {
    poly += other.poly;
    sin_cos_map.insert(other.sin_cos_map.begin(), other.sin_cos_map.end());
    return *this;
  }

  TrigPoly& operator-=(const TrigPoly& other)
  {
    poly -= other.poly;
    sin_cos_map.insert(other.sin_cos_map.begin(), other.sin_cos_map.end());
    return *this;
  }

  TrigPoly& operator*=(const TrigPoly& other)
  {
    poly *= other.poly;
    sin_cos_map.insert(other.sin_cos_map.begin(), other.sin_cos_map.end());
    return *this;
  }

  TrigPoly& operator+=(const CoefficientType& scalar)
  {
    poly += scalar;
    return *this;
  }

  TrigPoly& operator-=(const CoefficientType& scalar)
  {
    poly -= scalar;
    return *this;
  }

  TrigPoly& operator*=(const CoefficientType& scalar)
  {
    poly *= scalar;
    return *this;
  }

  TrigPoly& operator/=(const CoefficientType& scalar)
  {
    poly /= scalar;
    return *this;
  }

  const TrigPoly operator+(const TrigPoly& other) const
  {
    TrigPoly ret = *this;
    ret += other;
    return ret;
  }

  const TrigPoly operator-(const TrigPoly& other) const
  {
    TrigPoly ret = *this;
    ret -= other;
    return ret;
  }

  const TrigPoly operator-() const
  {
    TrigPoly ret = -(*this);
    return ret;
  }

  const TrigPoly operator*(const TrigPoly& other) const
  {
    TrigPoly ret = *this;
    ret *= other;
    return ret;
  }

  friend const TrigPoly operator+(const TrigPoly& p, const CoefficientType& scalar) {
    TrigPoly ret = p;
    ret += scalar;
    return ret;
  }

  friend const TrigPoly operator+(const CoefficientType& scalar,const TrigPoly& p) {
    TrigPoly ret = p;
    ret += scalar;
    return ret;
  }

  friend const TrigPoly operator-(const TrigPoly& p, const CoefficientType& scalar) {
    TrigPoly ret = p;
    ret -= scalar;
    return ret;
  }

  friend const TrigPoly operator-(const CoefficientType& scalar,const TrigPoly& p) {
    TrigPoly ret = -p;
    ret += scalar;
    return ret;
  }

  friend const TrigPoly operator*(const TrigPoly& p, const CoefficientType& scalar) {
    TrigPoly ret = p;
    ret *= scalar;
    return ret;
  }
  friend const TrigPoly operator*(const CoefficientType& scalar,const TrigPoly& p) {
    TrigPoly ret = p;
    ret *= scalar;
    return ret;
  }


  const TrigPoly operator/(const CoefficientType& scalar) const
  {
    TrigPoly ret = *this;
    ret /= scalar;
    return ret;
  }

  friend std::ostream& operator<<(std::ostream& os, const TrigPoly<CoefficientType>& tp)
  {
    os << tp.poly;
    return os;
  }

private:
  PolyType poly;
  SinCosMap sin_cos_map;
};

template<typename CoefficientType, int Rows, int Cols>
std::ostream& operator<<(std::ostream& os, const Eigen::Matrix<TrigPoly<CoefficientType>, Rows, Cols> & tp_mat)
{
  Eigen::Matrix<Polynomial<CoefficientType>, Rows, Cols> poly_mat(tp_mat.rows(),tp_mat.cols());
  for (int i = 0; i < poly_mat.size(); i++) {
    poly_mat(i) = tp_mat(i).getPolynomial();
  }
  os << poly_mat;
  return os;
}

typedef TrigPoly<double> TrigPolyd;

#endif
