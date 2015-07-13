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

  TrigPoly(const PolyType& p, const SinCosMap& _sin_cos_map)
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

  const SinCosMap& getSinCosMap(void) const {
    return sin_cos_map;
  }

  friend TrigPoly sin(const TrigPoly& p)
  {
    if (p.poly.getDegree() > 1)
      throw std::runtime_error("sin of polynomials with degree > 1 is not supported");

    const std::vector<typename PolyType::Monomial>& m = p.poly.getMonomials();

    if (m.size()==1) {
      TrigPoly ret = p;
      if (m[0].terms.size()==0) {  // then it's a constant
        ret.poly = Polynomial<CoefficientType>(sin(m[0].coefficient));
      } else {
        typename SinCosMap::iterator iter = ret.sin_cos_map.find(m[0].terms[0].var);
        if (iter==ret.sin_cos_map.end())
          throw std::runtime_error("tried taking the sin of a variable that does not exist in my sin_cos_map");

        if (std::abs(m[0].coefficient) != (CoefficientType) 1)
          throw std::runtime_error("Drake:TrigPoly:PleaseImplementMe.  need to handle this case (like I do in the matlab version");

        ret.poly.subs(m[0].terms[0].var,iter->second.s);
      }
      return ret;
    }

    // otherwise handle the multi-monomial case recursively
    // sin(a+b+...) = sin(a)cos(b+...) + cos(a)sin(b+...)
    Polynomial<CoefficientType> pa(m[0].coefficient,m[0].terms), pb(m.begin()+1,m.end());
    TrigPoly a(pa,p.sin_cos_map), b(pb,p.sin_cos_map);
    return sin(a)*cos(b) + cos(a)*sin(b);
  }

  friend TrigPoly cos(const TrigPoly& p)
  {
    if (p.poly.getDegree() > 1)
      throw std::runtime_error("cos of polynomials with degree > 1 is not supported");

    const std::vector<typename PolyType::Monomial>& m = p.poly.getMonomials();

    if (m.size()==1) {
      TrigPoly ret = p;
      if (m[0].terms.size()==0) {  // then it's a constant
        ret.poly = Polynomial<CoefficientType>(cos(m[0].coefficient));
      } else {
        typename SinCosMap::iterator iter = ret.sin_cos_map.find(m[0].terms[0].var);
        if (iter==ret.sin_cos_map.end())
          throw std::runtime_error("tried taking the sin of a variable that does not exist in my sin_cos_map");

        if (std::abs(m[0].coefficient) != (CoefficientType) 1)
          throw std::runtime_error("Drake:TrigPoly:PleaseImplementMe.  need to handle this case (like I do in the matlab version");

        ret.poly.subs(m[0].terms[0].var,iter->second.c);
        if (m[0].coefficient == (CoefficientType) -1) { ret *= -1; }  // cos(-q) => cos(q) => c (instead of -c)
      }
      return ret;
    }

    // otherwise handle the multi-monomial case recursively
    // cos(a+b+...) = cos(a)cos(b+...) - sin(a)sin(b+...)
    Polynomial<CoefficientType> pa(m[0].coefficient,m[0].terms), pb(m.begin()+1,m.end());
    TrigPoly a(pa,p.sin_cos_map), b(pb,p.sin_cos_map);
    return cos(a)*cos(b) - sin(a)*sin(b);
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
