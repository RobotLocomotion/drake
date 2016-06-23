#pragma once

#include <map>

#include "drake/common/drake_assert.h"
#include "drake/util/Polynomial.h"

/// A scalar multi-variate polynomial containing sines and cosines.
/**
 * TrigPoly represents a Polynomial some of whose variables actually represent
 * the sines or cosines of other variables.  Sines and cosines of first-order
 * polynomials (affine combinations of variables) are decomposed into
 * polynomials of the sines and cosines of individual variables via the
 * Prosthaphaeresis formulae.
 *
 * Any variables which will appear in the arguments to trigonometric functions
 * must be declared in the "SinCosMap" (created automatically by most TrigPoly
 * constructors); attempting to, e.g., use sin(x) without first creating a
 * SinCosMap mapping for 'x' will result in an exception.
 *
 * The same variable may not appear more than once in the sin_cos_map,
 * regardless of position.
 *
 * For example:
 * \code
 * Polynomial base_x("x"), s("s"), c("c");
 * TrigPoly x(base_x, s, c)  // This "x" knows that s = sin(x)
 *                           // and that c = cos(x)
 * cout << sin(x)                     // emits "s1"
 * cout << sin(x) * x                 // emits "x1*s1"
 * cout << sin(x + x) * x             // emits "x1*s1*c1 + x1*c1*s1"
 * \endcode
 *
 * NOTE:  Certain analyses may not succeed when individual Monomials contain
 * both x and sin(x) or cos(x) terms.  This restriction is not currently
 * enforced programmatically; TODO(ggould-tri) fix this in the future.
 */
template <typename _CoefficientType = double>
class TrigPoly {
 public:
  typedef _CoefficientType CoefficientType;
  typedef Polynomial<CoefficientType> PolyType;
  typedef typename PolyType::VarType VarType;
  struct SinCosVars {
    VarType s;
    VarType c;

    bool operator==(const struct SinCosVars& other) const {
      return (s == other.s) && (c == other.c);
    }
  };
  typedef std::map<VarType, SinCosVars> SinCosMap;

  template <typename Rhs, typename Lhs>
  struct Product {
    typedef decltype(static_cast<Rhs>(0) * static_cast<Lhs>(0)) type;
  };

  /// Constructs a vacuous TrigPoly.
  TrigPoly() {}

  /// Constructs a constant TrigPoly.
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  TrigPoly(const CoefficientType& scalar) : poly(scalar) {}

  /**
   * Constructs a TrigPoly on the associated Polynomial p, but with the
   * additional information about sin and cos relations in _sin_cos_map.
   */
  TrigPoly(const PolyType& p, const SinCosMap& _sin_cos_map)
      : poly(p), sin_cos_map(_sin_cos_map) {
    // The provided _sin_cos_map might have extraneous entries; clip them.
    std::set<VarType> vars_in_use = p.getVariables();
    std::set<VarType> vars_seen_in_map;
    for (const auto& sin_cos_entry : _sin_cos_map) {
      if (!vars_in_use.count(sin_cos_entry.first)) {
        sin_cos_map.erase(sin_cos_entry.first);
      }
      DRAKE_ASSERT(!vars_seen_in_map.count(sin_cos_entry.first));
      DRAKE_ASSERT(!vars_seen_in_map.count(sin_cos_entry.second.s));
      DRAKE_ASSERT(!vars_seen_in_map.count(sin_cos_entry.second.c));
      vars_seen_in_map.insert(sin_cos_entry.first);
      vars_seen_in_map.insert(sin_cos_entry.second.s);
      vars_seen_in_map.insert(sin_cos_entry.second.c);
    }
  }

  /**
   * Constructs a TrigPoly version of q, but with the additional information
   * that the variables s and c represent the sine and cosine of q.
   */
  TrigPoly(const PolyType& q, const PolyType& s, const PolyType& c) {
    if ((q.getDegree() != 1) || (s.getDegree() != 1) || (c.getDegree() != 1))
      throw std::runtime_error(
          "q, s, and c must all be simple polynomials (in the msspoly sense)");

    poly = q;
    SinCosVars sc;
    sc.s = s.getSimpleVariable();
    sc.c = c.getSimpleVariable();
    sin_cos_map[q.getSimpleVariable()] = sc;
  }

  /// Returns the underlying Polynomial for this TrigPoly.
  const PolyType& getPolynomial(void) const { return poly; }

  /// Returns the SinCosMap for this TrigPoly.
  const SinCosMap& getSinCosMap(void) const { return sin_cos_map; }

  /// A version of sin that handles TrigPoly arguments through ADL.
  /**
   * Implements sin(x) for a TrigPoly x.
   *
   * x must be of degree 0 or 1, and must contain only variables that have
   * entries in its SinCosMap.
   */
  friend TrigPoly sin(const TrigPoly& p) {
    if (p.poly.getDegree() > 1)
      throw std::runtime_error(
          "sin of polynomials with degree > 1 is not supported");

    const std::vector<typename PolyType::Monomial>& m = p.poly.getMonomials();

    if (m.size() == 1) {
      TrigPoly ret = p;
      if (m[0].terms.size() == 0) {  // then it's a constant
        ret.poly = Polynomial<CoefficientType>(sin(m[0].coefficient));
      } else {
        typename SinCosMap::iterator iter =
            ret.sin_cos_map.find(m[0].terms[0].var);
        if (iter == ret.sin_cos_map.end())
          throw std::runtime_error(
              "tried taking the sin of a variable that does not exist in my "
              "sin_cos_map");

        if (std::abs(m[0].coefficient) != (CoefficientType)1)
          throw std::runtime_error(
              "Drake:TrigPoly:PleaseImplementMe.  need to handle this case "
              "(like I do in the matlab version");

        ret.poly.subs(m[0].terms[0].var, iter->second.s);
      }
      return ret;
    }

    // otherwise handle the multi-monomial case recursively
    // sin(a+b+...) = sin(a)cos(b+...) + cos(a)sin(b+...)
    Polynomial<CoefficientType> pa(m[0].coefficient, m[0].terms),
        pb(m.begin() + 1, m.end());
    TrigPoly a(pa, p.sin_cos_map), b(pb, p.sin_cos_map);
    return sin(a) * cos(b) + cos(a) * sin(b);
  }

  /// A version of cos that handles TrigPoly arguments through ADL.
  /**
   * Implements cos(x) for a TrigPoly x.
   *
   * x must be of degree 0 or 1, and must contain only variables that have
   * entries in its SinCosMap.
   */
  friend TrigPoly cos(const TrigPoly& p) {
    if (p.poly.getDegree() > 1)
      throw std::runtime_error(
          "cos of polynomials with degree > 1 is not supported");

    const std::vector<typename PolyType::Monomial>& m = p.poly.getMonomials();

    if (m.size() == 1) {
      TrigPoly ret = p;
      if (m[0].terms.size() == 0) {  // then it's a constant
        ret.poly = Polynomial<CoefficientType>(cos(m[0].coefficient));
      } else {
        typename SinCosMap::iterator iter =
            ret.sin_cos_map.find(m[0].terms[0].var);
        if (iter == ret.sin_cos_map.end())
          throw std::runtime_error(
              "tried taking the sin of a variable that does not exist in my "
              "sin_cos_map");

        if (std::abs(m[0].coefficient) != (CoefficientType)1)
          throw std::runtime_error(
              "Drake:TrigPoly:PleaseImplementMe.  need to handle this case "
              "(like I do in the matlab version");

        ret.poly.subs(m[0].terms[0].var, iter->second.c);
        if (m[0].coefficient == (CoefficientType)-1) {
          ret *= -1;
        }  // cos(-q) => cos(q) => c (instead of -c)
      }
      return ret;
    }

    // otherwise handle the multi-monomial case recursively
    // cos(a+b+...) = cos(a)cos(b+...) - sin(a)sin(b+...)
    Polynomial<CoefficientType> pa(m[0].coefficient, m[0].terms),
        pb(m.begin() + 1, m.end());
    TrigPoly a(pa, p.sin_cos_map), b(pb, p.sin_cos_map);
    return cos(a) * cos(b) - sin(a) * sin(b);
  }

  /// Returns all of the base (non-sin/cos) variables in this TrigPoly.
  std::set<VarType> getVariables() const {
    std::set<VarType> vars = poly.getVariables();
    for (const auto& sin_cos_item : sin_cos_map) {
      vars.erase(sin_cos_item.second.s);
      vars.erase(sin_cos_item.second.c);
    }
    return vars;
  }

  /// Given a value for every variable in this expression, evaluates it.
  /**
   * By analogy with Polynomial::evaluateMultivariate().  Values must be
   * supplied for all base variables; supplying values for sin/cos variables
   * is an error.
   */
  template <typename T>
  typename Product<CoefficientType, T>::type evaluateMultivariate(
      const std::map<VarType, T>& var_values) const {
    std::map<VarType, T> all_var_values = var_values;
    for (const auto& sin_cos_item : sin_cos_map) {
      DRAKE_ASSERT(!var_values.count(sin_cos_item.second.s));
      DRAKE_ASSERT(!var_values.count(sin_cos_item.second.c));
      all_var_values[sin_cos_item.second.s] =
          std::sin(var_values.at(sin_cos_item.first));
      all_var_values[sin_cos_item.second.c] =
          std::cos(var_values.at(sin_cos_item.first));
    }
    return poly.evaluateMultivariate(all_var_values);
  }

  /// Partially evaluates this expression, returning the resulting expression.
  /**
   * By analogy with Polynomial::evaluatePartial.  Values must be supplied for
   * all base variables only; supplying values for sin/cos variables is an
   * error.
   */
  virtual TrigPoly<CoefficientType> evaluatePartial(
      const std::map<VarType, CoefficientType>& var_values) const {
    std::map<VarType, CoefficientType> var_values_with_sincos = var_values;
    for (const auto& sin_cos_item : sin_cos_map) {
      DRAKE_ASSERT(!var_values.count(sin_cos_item.second.s));
      DRAKE_ASSERT(!var_values.count(sin_cos_item.second.c));
      if (!var_values.count(sin_cos_item.first)) {
        continue;
      }
      var_values_with_sincos[sin_cos_item.second.s] =
          std::sin(var_values.at(sin_cos_item.first));
      var_values_with_sincos[sin_cos_item.second.c] =
          std::cos(var_values.at(sin_cos_item.first));
    }
    return TrigPoly(poly.evaluatePartial(var_values_with_sincos), sin_cos_map);
  }

  /// Compares two TrigPolys for equality.
  /**
   * Note that the question of equality of TrigPolys is a bit subtle.  It is
   * not immediately clear if two TrigPolys whose poly and sin_cos_map members
   * differ equivalently (eg, a + b (b = cos(a)) and a + c (c = cos(a))) should
   * be considered equal.
   *
   * For simplicity we only consider exactly equality rather than semantic
   * equivalence.  However that decision could reasonably revisited in the
   * future.
   */
  bool operator==(const TrigPoly& other) const {
    return (poly == other.poly) && (sin_cos_map == other.sin_cos_map);
  }

  TrigPoly& operator+=(const TrigPoly& other) {
    poly += other.poly;
    sin_cos_map.insert(other.sin_cos_map.begin(), other.sin_cos_map.end());
    return *this;
  }

  TrigPoly& operator-=(const TrigPoly& other) {
    poly -= other.poly;
    sin_cos_map.insert(other.sin_cos_map.begin(), other.sin_cos_map.end());
    return *this;
  }

  TrigPoly& operator*=(const TrigPoly& other) {
    poly *= other.poly;
    sin_cos_map.insert(other.sin_cos_map.begin(), other.sin_cos_map.end());
    return *this;
  }

  TrigPoly& operator+=(const CoefficientType& scalar) {
    poly += scalar;
    return *this;
  }

  TrigPoly& operator-=(const CoefficientType& scalar) {
    poly -= scalar;
    return *this;
  }

  TrigPoly& operator*=(const CoefficientType& scalar) {
    poly *= scalar;
    return *this;
  }

  TrigPoly& operator/=(const CoefficientType& scalar) {
    poly /= scalar;
    return *this;
  }

  const TrigPoly operator+(const TrigPoly& other) const {
    TrigPoly ret = *this;
    ret += other;
    return ret;
  }

  const TrigPoly operator-(const TrigPoly& other) const {
    TrigPoly ret = *this;
    ret -= other;
    return ret;
  }

  const TrigPoly operator-() const {
    TrigPoly ret = -(*this);
    return ret;
  }

  const TrigPoly operator*(const TrigPoly& other) const {
    TrigPoly ret = *this;
    ret *= other;
    return ret;
  }

  friend const TrigPoly operator+(const TrigPoly& p,
                                  const CoefficientType& scalar) {
    TrigPoly ret = p;
    ret += scalar;
    return ret;
  }

  friend const TrigPoly operator+(const CoefficientType& scalar,
                                  const TrigPoly& p) {
    TrigPoly ret = p;
    ret += scalar;
    return ret;
  }

  friend const TrigPoly operator-(const TrigPoly& p,
                                  const CoefficientType& scalar) {
    TrigPoly ret = p;
    ret -= scalar;
    return ret;
  }

  friend const TrigPoly operator-(const CoefficientType& scalar,
                                  const TrigPoly& p) {
    TrigPoly ret = -p;
    ret += scalar;
    return ret;
  }

  friend const TrigPoly operator*(const TrigPoly& p,
                                  const CoefficientType& scalar) {
    TrigPoly ret = p;
    ret *= scalar;
    return ret;
  }
  friend const TrigPoly operator*(const CoefficientType& scalar,
                                  const TrigPoly& p) {
    TrigPoly ret = p;
    ret *= scalar;
    return ret;
  }

  const TrigPoly operator/(const CoefficientType& scalar) const {
    TrigPoly ret = *this;
    ret /= scalar;
    return ret;
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  const TrigPoly<CoefficientType>& tp) {
    os << tp.poly;
    return os;
  }

 private:
  PolyType poly;
  SinCosMap sin_cos_map;
};

template <typename CoefficientType, int Rows, int Cols>
std::ostream& operator<<(
    std::ostream& os,
    const Eigen::Matrix<TrigPoly<CoefficientType>, Rows, Cols>& tp_mat) {
  Eigen::Matrix<Polynomial<CoefficientType>, Rows, Cols> poly_mat(
      tp_mat.rows(), tp_mat.cols());
  for (int i = 0; i < poly_mat.size(); i++) {
    poly_mat(i) = tp_mat(i).getPolynomial();
  }
  os << poly_mat;
  return os;
}

typedef TrigPoly<double> TrigPolyd;
