#include "drake/common/polynomial.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

using Eigen::Dynamic;
using Eigen::Matrix;
using std::accumulate;
using std::pair;
using std::runtime_error;
using std::string;
using std::vector;

namespace drake {
template <typename T>
bool Polynomial<T>::Monomial::HasSameExponents(
    const Monomial& other) const {
  if (terms.size() != other.terms.size()) return false;

  for (typename vector<Term>::const_iterator iter = terms.begin();
       iter != terms.end(); iter++) {
    typename vector<Term>::const_iterator match =
        find(other.terms.begin(), other.terms.end(), *iter);
    if (match == other.terms.end()) return false;
  }
  return true;
}

template <typename T>
bool Polynomial<T>::Monomial::HasVariable(const VarType& var) const {
  for (const auto& t : terms) {
    if (t.var == var) {
      return true;
    }
  }
  return false;
}

template <typename T>
Polynomial<T>::Polynomial(const T& scalar) {
  Monomial m;
  m.coefficient = scalar;
  monomials_.push_back(m);
  is_univariate_ = true;
}

template <typename T>
Polynomial<T>::Polynomial(const T coefficient,
                                        const vector<Term>& terms) {
  Monomial m;
  m.coefficient = coefficient;
  m.terms = terms;

  is_univariate_ = true;
  for (int i = static_cast<int>(m.terms.size()) - 1; i >= 0; i--) {
    if ((i > 0) && (m.terms[i].var != m.terms[0].var)) {
      is_univariate_ = false;
    }
    for (int j = 0; j < (i - 1); j++) {  // merge any duplicate vars
      if (m.terms[i].var == m.terms[j].var) {
        m.terms[j].power += m.terms[i].power;
        m.terms.erase(m.terms.begin() + i);
        break;
      }
    }
  }

  monomials_.push_back(m);
}

template <typename T>
Polynomial<T>::Polynomial(
    typename vector<
        typename Polynomial<T>::Monomial>::const_iterator start,
    typename vector<typename Polynomial<T>::Monomial>::
        const_iterator finish) {
  is_univariate_ = true;
  for (
      typename vector<
          typename Polynomial<T>::Monomial>::const_iterator iter =
          start;
      iter != finish; iter++)
    monomials_.push_back(*iter);
  MakeMonomialsUnique();
}

template <typename T>
Polynomial<T>::Polynomial(const string& varname, const unsigned int num) {
  Monomial m;
  m.coefficient = T{1};
  Term t;
  t.var = VariableNameToId(varname, num);
  t.power = 1;
  m.terms.push_back(t);
  monomials_.push_back(m);
  is_univariate_ = true;
}

template <typename T>
Polynomial<T>::Polynomial(const T& coeff,
                                        const VarType& v) {
  Monomial m;
  m.coefficient = coeff;
  Term t;
  t.var = v;
  t.power = 1;
  m.terms.push_back(t);
  monomials_.push_back(m);
  is_univariate_ = true;
}

template <typename T>
int Polynomial<T>::GetNumberOfCoefficients() const {
  return static_cast<int>(monomials_.size());
}

template <typename T>
int Polynomial<T>::Monomial::GetDegree() const {
  if (terms.empty()) return 0;
  int degree = terms[0].power;
  for (size_t i = 1; i < terms.size(); i++) degree *= terms[i].power;
  return degree;
}

template <typename T>
int Polynomial<T>::Monomial::GetDegreeOf(VarType v) const {
  for (const Term& term : terms) {
    if (term.var == v) {
      return term.power;
    }
  }
  return 0;
}

template <typename T>
typename Polynomial<T>::Monomial
Polynomial<T>::Monomial::Factor(const Monomial& divisor) const {
  Monomial error, result;
  error.coefficient = 0;
  result.coefficient = coefficient / divisor.coefficient;
  for (const Term& term : terms) {
    const PowerType divisor_power = divisor.GetDegreeOf(term.var);
    if (term.power < divisor_power) { return error; }
    Term new_term;
    new_term.var = term.var;
    new_term.power = term.power - divisor_power;
    if (new_term.power > 0) {
      result.terms.push_back(new_term);
    }
  }
  for (const Term& divisor_term : divisor.terms) {
    if (!GetDegreeOf(divisor_term.var)) { return error; }
  }
  return result;
}

template <typename T>
int Polynomial<T>::GetDegree() const {
  int max_degree = 0;
  for (typename vector<Monomial>::const_iterator iter = monomials_.begin();
       iter != monomials_.end(); iter++) {
    int monomial_degree = iter->GetDegree();
    if (monomial_degree > max_degree) max_degree = monomial_degree;
  }

  return max_degree;
}

template <typename T>
bool Polynomial<T>::IsAffine() const {
  for (const auto& monomial : monomials_) {
    if ((monomial.terms.size() > 1) || (monomial.GetDegree() > 1)) {
      return false;
    }
  }
  return true;
}

template <typename T>
typename Polynomial<T>::VarType
Polynomial<T>::GetSimpleVariable() const {
  if (monomials_.size() != 1) return 0;
  if (monomials_[0].terms.size() != 1) return 0;
  if (monomials_[0].terms[0].power != 1) return 0;
  return monomials_[0].terms[0].var;
}

template <typename T>
const std::vector<typename Polynomial<T>::Monomial>&
Polynomial<T>::GetMonomials() const {
  return monomials_;
}

template <typename T>
Matrix<T, Dynamic, 1>
Polynomial<T>::GetCoefficients() const {
  if (!is_univariate_)
    throw runtime_error(
        "getCoefficients is only defined for univariate polynomials");

  int deg = GetDegree();

  Matrix<T, Dynamic, 1> coefficients =
      Matrix<T, Dynamic, 1>::Zero(deg + 1);
  for (typename vector<Monomial>::const_iterator iter = monomials_.begin();
       iter != monomials_.end(); iter++) {
    if (iter->terms.empty())
      coefficients[0] = iter->coefficient;
    else
      coefficients[iter->terms[0].power] = iter->coefficient;
  }
  return coefficients;
}

template <typename T>
std::set<typename Polynomial<T>::VarType>
Polynomial<T>::GetVariables() const {
  std::set<Polynomial<T>::VarType> vars;
  for (const Monomial& monomial : monomials_) {
    for (const Term& term : monomial.terms) {
      vars.insert(term.var);
    }
  }
  return vars;
}

template <typename T>
Polynomial<T> Polynomial<T>::EvaluatePartial(
    const std::map<VarType, T>& var_values) const {
  using std::pow;
  std::vector<Monomial> new_monomials;
  for (const Monomial& monomial : monomials_) {
    T new_coefficient = monomial.coefficient;
    std::vector<Term> new_terms;
    for (const Term& term : monomial.terms) {
      if (var_values.count(term.var)) {
        new_coefficient *= pow(var_values.at(term.var), term.power);
      } else {
        new_terms.push_back(term);
      }
    }
    Monomial new_monomial = {new_coefficient, new_terms};
    new_monomials.push_back(new_monomial);
  }
  return Polynomial(new_monomials.begin(), new_monomials.end());
}

template <typename T>
void Polynomial<T>::Subs(const VarType& orig,
                               const VarType& replacement) {
  for (typename vector<Monomial>::iterator iter = monomials_.begin();
       iter != monomials_.end(); iter++) {
    for (typename vector<Term>::iterator t = iter->terms.begin();
         t != iter->terms.end(); t++) {
      if (t->var == orig) t->var = replacement;
    }
  }
}

template <typename T>
Polynomial<T> Polynomial<T>::Substitute(
    const VarType& orig, const Polynomial<T>& replacement) const {
  // TODO(russt): Consider making this more efficient by updating coefficients
  // in place instead of relying on the more general polynomial operators.
  Polynomial<T> p;
  for (const auto& source_monomial : monomials_) {
    if (source_monomial.HasVariable(orig)) {
      Polynomial<T> m = source_monomial.coefficient;
      for (const Term& t : source_monomial.terms) {
        if (t.var == orig) {
          m *= pow(replacement, t.power);
        } else {
          m *= Polynomial(1.0, {t});
        }
        p += m;
      }
    } else {
      // Then this monomial is not changed; add it in directly.
      p += Polynomial(source_monomial.coefficient, source_monomial.terms);
    }
  }
  return p;
}  // namespace drake

template <typename T>
Polynomial<T> Polynomial<T>::Derivative(
    int derivative_order) const {
  DRAKE_DEMAND(derivative_order >= 0);
  if (!is_univariate_)
    throw runtime_error(
        "Derivative is only defined for univariate polynomials");
  if (derivative_order == 0) {
    return *this;
  }
  Polynomial<T> ret;

  for (typename vector<Monomial>::const_iterator iter = monomials_.begin();
       iter != monomials_.end(); iter++) {
    if (!iter->terms.empty() && (
            iter->terms[0].power >= static_cast<PowerType>(derivative_order))) {
      Monomial m = *iter;
      for (int k = 0; k < derivative_order;
           k++) {  // take the remaining derivatives
        m.coefficient = m.coefficient * m.terms[0].power;
        m.terms[0].power -= 1;
      }
      if (m.terms[0].power < 1) m.terms.erase(m.terms.begin());
      ret.monomials_.push_back(m);
    }
  }
  ret.is_univariate_ = true;
  return ret;
}

template <typename T>
Polynomial<T> Polynomial<T>::Integral(
    const T& integration_constant) const {
  if (!is_univariate_)
    throw runtime_error(
        "Integral is only defined for univariate polynomials");
  Polynomial<T> ret = *this;

  for (typename vector<Monomial>::iterator iter = ret.monomials_.begin();
       iter != ret.monomials_.end(); iter++) {
    if (iter->terms.empty()) {
      Term t;
      t.var = 0;
      for (typename vector<Monomial>::iterator iterB = ret.monomials_.begin();
           iterB != ret.monomials_.end(); iterB++) {
        if (!iterB->terms.empty()) {
          t.var = iterB->terms[0].var;
          break;
        }
      }
      if (t.var < 1) throw runtime_error("don't know the variable name");
      t.power = 1;
      iter->terms.push_back(t);
    } else {
      iter->coefficient /= static_cast<RealScalar>(iter->terms[0].power + 1);
      iter->terms[0].power += PowerType{1};
    }
  }
  Monomial m;
  m.coefficient = integration_constant;
  ret.is_univariate_ = true;
  ret.monomials_.push_back(m);
  return ret;
}

template <typename T>
bool Polynomial<T>::operator==(
    const Polynomial<T>& other) const {
  // Comparison of unsorted vectors is faster copying them into std::set
  // btrees rather than using std::is_permutation().
  // TODO(#2216) switch from multiset to set for further performance gains.
  const std::multiset<Monomial> this_monomials(monomials_.begin(),
                                               monomials_.end());
  const std::multiset<Monomial> other_monomials(other.monomials_.begin(),
                                                other.monomials_.end());
  return this_monomials == other_monomials;
}

template <typename T>
Polynomial<T>& Polynomial<T>::operator+=(
    const Polynomial<T>& other) {
  for (const auto& iter : other.monomials_) {
    monomials_.push_back(iter);
  }
  MakeMonomialsUnique();  // also sets is_univariate false if necessary
  return *this;
}

template <typename T>
Polynomial<T>& Polynomial<T>::operator-=(
    const Polynomial<T>& other) {
  for (const auto& iter : other.monomials_) {
    monomials_.push_back(iter);
    monomials_.back().coefficient *= T{-1};
  }
  MakeMonomialsUnique();  // also sets is_univariate false if necessary
  return *this;
}

template <typename T>
Polynomial<T>& Polynomial<T>::operator*=(
    const Polynomial<T>& other) {
  vector<Monomial> new_monomials;

  for (const auto& iter : monomials_) {
    for (const auto& other_iter : other.monomials_) {
      Monomial m;
      m.coefficient = iter.coefficient * other_iter.coefficient;
      m.terms = iter.terms;
      for (size_t i = 0; i < other_iter.terms.size(); i++) {
        bool new_var = true;
        for (size_t j = 0; j < m.terms.size(); j++) {
          if (m.terms[j].var == other_iter.terms[i].var) {
            m.terms[j].power += other_iter.terms[i].power;
            new_var = false;
            break;
          }
        }
        if (new_var) {
          m.terms.push_back(other_iter.terms[i]);
        }
      }
      new_monomials.push_back(m);
    }
  }
  monomials_ = new_monomials;

  MakeMonomialsUnique();  // also sets is_univariate false if necessary
  return *this;
}

template <typename T>
Polynomial<T>& Polynomial<T>::operator+=(
    const T& scalar) {
  // add to the constant monomial if I have one
  for (typename vector<Monomial>::iterator iter = monomials_.begin();
       iter != monomials_.end(); iter++) {
    if (iter->terms.empty()) {
      iter->coefficient += scalar;
      return *this;
    }
  }

  // otherwise create the constant monomial
  Monomial m;
  m.coefficient = scalar;
  monomials_.push_back(m);
  return *this;
}

template <typename T>
Polynomial<T>& Polynomial<T>::operator-=(
    const T& scalar) {
  // add to the constant monomial if I have one
  for (typename vector<Monomial>::iterator iter = monomials_.begin();
       iter != monomials_.end(); iter++) {
    if (iter->terms.empty()) {
      iter->coefficient -= scalar;
      return *this;
    }
  }

  // otherwise create the constant monomial
  Monomial m;
  m.coefficient = -scalar;
  monomials_.push_back(m);
  return *this;
}

template <typename T>
Polynomial<T>& Polynomial<T>::operator*=(
    const T& scalar) {
  for (typename vector<Monomial>::iterator iter = monomials_.begin();
       iter != monomials_.end(); iter++) {
    iter->coefficient *= scalar;
  }
  return *this;
}

template <typename T>
Polynomial<T>& Polynomial<T>::operator/=(
    const T& scalar) {
  for (typename vector<Monomial>::iterator iter = monomials_.begin();
       iter != monomials_.end(); iter++) {
    iter->coefficient /= scalar;
  }
  return *this;
}

template <typename T>
const Polynomial<T> Polynomial<T>::operator+(
    const Polynomial& other) const {
  Polynomial<T> ret = *this;
  ret += other;
  return ret;
}

template <typename T>
const Polynomial<T> Polynomial<T>::operator-(
    const Polynomial& other) const {
  Polynomial<T> ret = *this;
  ret -= other;
  return ret;
}

template <typename T>
const Polynomial<T> Polynomial<T>::operator-()
    const {
  Polynomial<T> ret = *this;
  for (typename vector<Monomial>::iterator iter = ret.monomials_.begin();
       iter != ret.monomials_.end(); iter++) {
    iter->coefficient = -iter->coefficient;
  }
  return ret;
}

template <typename T>
const Polynomial<T> Polynomial<T>::operator*(
    const Polynomial<T>& other) const {
  Polynomial<T> ret = *this;
  ret *= other;
  return ret;
}

template <typename T>
const Polynomial<T> Polynomial<T>::operator/(
    const T& scalar) const {
  Polynomial<T> ret = *this;
  ret /= scalar;
  return ret;
}

template <typename T>
typename Polynomial<T>::RootsType Polynomial<T>::Roots() const {
  if (!is_univariate_)
    throw runtime_error("Roots is only defined for univariate polynomials");

  // RootsType (std::complex<T>) does not currently work for AutoDiffXd nor for
  // Expression, which leaves only double.  We could, in principle, try to
  // support more types here.
  if constexpr (std::is_same_v<T, double>) {
    auto coefficients = GetCoefficients();

    // need to handle degree 0 and 1 explicitly because Eigen's polynomial
    // solver doesn't work for these
    int degree = static_cast<int>(coefficients.size()) - 1;
    switch (degree) {
      case 0:
        return Polynomial<T>::RootsType(degree);
      case 1: {
        Polynomial<T>::RootsType ret(degree);
        ret[0] = -coefficients[0] / coefficients[1];
        return ret;
      }
      default: {
        Eigen::PolynomialSolver<RealScalar, Eigen::Dynamic> solver;
        solver.compute(coefficients);
        return solver.roots();
      }
    }
  } else {
    throw std::runtime_error(
        "Polynomial<T>::Roots() is only supports T=double.");
  }
}

template <typename T>
boolean<T> Polynomial<T>::CoefficientsAlmostEqual(
    const Polynomial<T>& other, const Polynomial<T>::RealScalar& tol,
    const ToleranceType& tol_type) const {
  using std::abs;
  using std::min;
  std::vector<bool> monomial_has_match(monomials_.size(), false);
  boolean<T> comparison{true};
  for (const auto& m : other.GetMonomials()) {
    bool found_matching_term = false;
    for (size_t i = 0; i < monomials_.size(); i++) {
      if (monomial_has_match[i]) continue;
      if (m.terms == monomials_[i].terms) {
        found_matching_term = true;
        if (tol_type == ToleranceType::kAbsolute) {
          comparison = comparison &&
                       abs(m.coefficient - monomials_[i].coefficient) <= tol;
        } else {
          comparison =
              comparison &&
              abs(m.coefficient - monomials_[i].coefficient) <=
                  tol * min(abs(m.coefficient), abs(monomials_[i].coefficient));
        }
        monomial_has_match[i] = true;
        break;
      }
    }
    if (!found_matching_term) {
      if (tol_type == ToleranceType::kAbsolute) {
        // then I can still succeed, if my coefficient is close to zero.
        comparison = comparison && abs(m.coefficient) <= tol;
      } else {
        return boolean<T>{false};
      }
    }
  }
  // Finally, check any monomials in this that did not have a match in other.
  for (size_t i = 0; i < monomials_.size(); i++) {
    if (monomial_has_match[i]) continue;
    if (tol_type == ToleranceType::kAbsolute) {
      comparison = comparison && abs(monomials_[i].coefficient) <= tol;
    } else {
      return boolean<T>{false};
    }
  }
  return comparison;
}

constexpr char kNameChars[] = "@#_.abcdefghijklmnopqrstuvwxyz";
const unsigned int kNumNameChars = sizeof(kNameChars) - 1;
const unsigned int kNameLength = 4;
const unsigned int kMaxNamePart = 923521;  // (kNumNameChars+1)^kNameLength;

template <typename T>
bool Polynomial<T>::IsValidVariableName(const string name) {
  size_t len = name.length();
  if (len < 1) return false;
  for (size_t i = 0; i < len; i++)
    if (!strchr(kNameChars, name[i])) return false;
  return true;
}

template <typename T>
typename Polynomial<T>::VarType
Polynomial<T>::VariableNameToId(const string name,
                                              const unsigned int m) {
  DRAKE_THROW_UNLESS(IsValidVariableName(name));
  unsigned int multiplier = 1;
  VarType name_part = 0;
  for (int i = static_cast<int>(name.size()) - 1; i >= 0; i--) {
    const char* const character_match = strchr(kNameChars, name[i]);
    DRAKE_ASSERT(character_match != nullptr);
    VarType offset = static_cast<VarType>(character_match - kNameChars);
    name_part += (offset + 1) * multiplier;
    multiplier *= kNumNameChars + 1;
  }
  if (name_part > kMaxNamePart) {
    throw runtime_error("name " + name +
                        " (" + std::to_string(name_part) +
                        ") exceeds max allowed");
  }
  const VarType maxId = std::numeric_limits<VarType>::max() / 2 / kMaxNamePart;
  if (m > maxId) throw runtime_error("name exceeds max ID");
  if (m < 1) throw runtime_error("m must be >0");
  return static_cast<VarType>(2) * (name_part + kMaxNamePart * (m - 1));
}

template <typename T>
string Polynomial<T>::IdToVariableName(const VarType id) {
  VarType name_part = (id / 2) % kMaxNamePart;  // id/2 to be compatible w/
                                                // msspoly, even though I'm not
                                                // doing the trig support here

  unsigned int m = id / 2 / kMaxNamePart;
  unsigned int multiplier = static_cast<unsigned int>(
      std::pow(static_cast<double>(kNumNameChars + 1),
               static_cast<int>(kNameLength) - 1));
  char name[kNameLength + 1];
  int j = 0;
  for (int i = 0; i < static_cast<int>(kNameLength); i++) {
    unsigned int name_ind = (name_part / multiplier) % (kNumNameChars + 1);
    if (name_ind > 0) name[j++] = kNameChars[name_ind - 1];
    multiplier /= kNumNameChars + 1;
  }
  if (j == 0) name[j++] = kNameChars[0];
  name[j] = '\0';
  return string(name) + std::to_string((m + 1));
}

template <typename T>
void Polynomial<T>::MakeMonomialsUnique(void) {
  VarType unique_var = 0;  // also update the univariate flag
  for (int i = static_cast<int>(monomials_.size()) - 1; i >= 0; --i) {
    if (monomials_[i].coefficient == 0) {
      monomials_.erase(monomials_.begin() + i);
      continue;
    }
    Monomial& mi = monomials_[i];
    if (!mi.terms.empty()) {
      if (mi.terms.size() > 1) is_univariate_ = false;
      if (mi.terms[0].var != unique_var) {
        if (unique_var > 0) {
          is_univariate_ = false;
        } else {
          unique_var = mi.terms[0].var;
        }
      }
    }
    for (int j = 0; j <= (i - 1); j++) {
      Monomial& mj = monomials_[j];
      if (mi.HasSameExponents(mj)) {
        // it's a match, so delete monomial i
        monomials_[j].coefficient += monomials_[i].coefficient;
        monomials_.erase(monomials_.begin() + i);
        break;
      }
    }
  }
}

namespace {

using symbolic::Expression;

// Visitor class to implement FromExpression.
template <typename T>
class FromExpressionVisitor {
 public:
  Polynomial<T> Visit(const Expression& e) {
    return drake::symbolic::VisitExpression<Polynomial<T>>(this, e);
  }

 private:
  static Polynomial<T> VisitAddition(const Expression& e) {
    const auto constant = get_constant_in_addition(e);
    const auto& expr_to_coeff_map = get_expr_to_coeff_map_in_addition(e);
    return accumulate(
        expr_to_coeff_map.begin(), expr_to_coeff_map.end(),
        Polynomial<T>{constant},
        [](const Polynomial<T>& polynomial,
           const pair<const Expression, double>& p) {
          return polynomial + Polynomial<T>::FromExpression(p.first) * p.second;
        });
  }

  static Polynomial<T> VisitMultiplication(const Expression& e) {
    const auto constant = drake::symbolic::get_constant_in_multiplication(e);
    const auto& base_to_exponent_map =
        drake::symbolic::get_base_to_exponent_map_in_multiplication(e);
    return accumulate(
        base_to_exponent_map.begin(), base_to_exponent_map.end(),
        Polynomial<T>{constant},
        [](const Polynomial<T>& polynomial,
           const pair<const Expression, Expression>& p) {
          const Expression& base{p.first};
          const Expression& exponent{p.second};
          DRAKE_ASSERT(base.is_polynomial());
          DRAKE_ASSERT(is_constant(exponent));
          return polynomial *
                 pow(Polynomial<T>::FromExpression(base),
                     static_cast<int>(get_constant_value(exponent)));
        });
  }

  static Polynomial<T> VisitDivision(const Expression& e) {
    DRAKE_ASSERT(e.is_polynomial());
    const auto& first_arg{get_first_argument(e)};
    const auto& second_arg{get_second_argument(e)};
    DRAKE_ASSERT(is_constant(second_arg));
    return Polynomial<T>::FromExpression(first_arg) /
           get_constant_value(second_arg);
  }

  static Polynomial<T> VisitVariable(const Expression& e) {
    return Polynomial<T>{1.0, static_cast<Polynomial<double>::VarType>(
                                  get_variable(e).get_id())};
  }

  static Polynomial<T> VisitConstant(const Expression& e) {
    return Polynomial<T>{get_constant_value(e)};
  }

  static Polynomial<T> VisitLog(const Expression&) {
    throw runtime_error("Log expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitPow(const Expression& e) {
    DRAKE_ASSERT(e.is_polynomial());
    const int exponent{
        static_cast<int>(get_constant_value(get_second_argument(e)))};
    return pow(Polynomial<T>::FromExpression(get_first_argument(e)), exponent);
  }

  static Polynomial<T> VisitAbs(const Expression&) {
    throw runtime_error("Abs expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitExp(const Expression&) {
    throw runtime_error("Exp expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitSqrt(const Expression&) {
    throw runtime_error("Sqrt expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitSin(const Expression&) {
    throw runtime_error("Sin expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitCos(const Expression&) {
    throw runtime_error("Cos expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitTan(const Expression&) {
    throw runtime_error("Tan expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitAsin(const Expression&) {
    throw runtime_error("Asin expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitAcos(const Expression&) {
    throw runtime_error("Acos expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitAtan(const Expression&) {
    throw runtime_error("Atan expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitAtan2(const Expression&) {
    throw runtime_error("Atan2 expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitSinh(const Expression&) {
    throw runtime_error("Sinh expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitCosh(const Expression&) {
    throw runtime_error("Cosh expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitTanh(const Expression&) {
    throw runtime_error("Tanh expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitMin(const Expression&) {
    throw runtime_error("Min expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitMax(const Expression&) {
    throw runtime_error("Max expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitCeil(const Expression&) {
    throw runtime_error("Ceil expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitFloor(const Expression&) {
    throw runtime_error("Floor expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitIfThenElse(const Expression&) {
    throw runtime_error("IfThenElse expression is not polynomial-convertible.");
  }

  static Polynomial<T> VisitUninterpretedFunction(const Expression&) {
    throw runtime_error(
        "Uninterpreted-function expression is not polynomial-convertible.");
  }

  // Makes VisitExpression a friend of this class so that VisitExpression can
  // use its private methods.
  friend Polynomial<T> drake::symbolic::VisitExpression<Polynomial<T>>(
      FromExpressionVisitor*, const Expression&);
};

}  // namespace

template <typename T>
Polynomial<T> Polynomial<T>::FromExpression(const Expression& e) {
  return FromExpressionVisitor<T>{}.Visit(e);
}

// template class Polynomial<std::complex<double>>;
// doesn't work yet because the roots solver can't handle it

}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::Polynomial)
