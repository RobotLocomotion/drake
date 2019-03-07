// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.
#include <map>
#include <numeric>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic.h"
#define DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER
#include "drake/common/symbolic_expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER

namespace drake {
namespace symbolic {

using std::accumulate;
using std::logic_error;
using std::make_pair;
using std::map;
using std::ostream;
using std::ostringstream;
using std::pair;
using std::runtime_error;

namespace {
// Computes the total degree of a monomial. This method is used in a
// constructor of Monomial to set its total degree at construction.
int TotalDegree(const map<Variable, int>& powers) {
  return accumulate(powers.begin(), powers.end(), 0,
                    [](const int degree, const pair<const Variable, int>& p) {
                      return degree + p.second;
                    });
}

// Converts a symbolic expression @p e into an internal representation of
// Monomial class, a mapping from a base (Variable) to its exponent (int). This
// function is called inside of the constructor Monomial(const
// symbolic::Expression&).
map<Variable, int> ToMonomialPower(const Expression& e) {
  // TODO(soonho): Re-implement this function by using a Polynomial visitor.
  DRAKE_DEMAND(e.is_polynomial());
  map<Variable, int> powers;
  if (is_one(e)) {  // This block is deliberately left empty.
  } else if (is_constant(e)) {
    throw runtime_error("A constant not equal to 1, this is not a monomial.");
  } else if (is_variable(e)) {
    powers.emplace(get_variable(e), 1);
  } else if (is_pow(e)) {
    const Expression& base{get_first_argument(e)};
    const Expression& exponent{get_second_argument(e)};
    // The following holds because `e` is polynomial.
    DRAKE_DEMAND(is_constant(exponent));
    // The following static_cast (double -> int) does not lose information
    // because of the precondition `e.is_polynomial()`.
    const int n{static_cast<int>(get_constant_value(exponent))};
    powers = ToMonomialPower(base);
    // pow(base, n) => (∏ᵢ xᵢ)^ⁿ => ∏ᵢ (xᵢ^ⁿ)
    for (auto& p : powers) {
      p.second *= n;
    }
  } else if (is_multiplication(e)) {
    if (!is_one(get_constant_in_multiplication(e))) {
      throw runtime_error("The constant in the multiplication is not 1.");
    }
    // e = ∏ᵢ pow(baseᵢ, exponentᵢ).
    for (const auto& p : get_base_to_exponent_map_in_multiplication(e)) {
      for (const auto& q : ToMonomialPower(pow(p.first, p.second))) {
        auto it = powers.find(q.first);
        if (it == powers.end()) {
          powers.emplace(q.first, q.second);
        } else {
          it->second += q.second;
        }
      }
    }
  } else {
    throw runtime_error("This expression cannot be converted to a monomial.");
  }
  return powers;
}

// Converts a pair of variables and their integer exponents into an internal
// representation of Monomial class, a mapping from a base (Variable) to its
// exponent (int). This function is called in the constructor taking the same
// types of arguments.
map<Variable, int> ToMonomialPower(
    const Eigen::Ref<const VectorX<Variable>>& vars,
    const Eigen::Ref<const Eigen::VectorXi>& exponents) {
  DRAKE_DEMAND(vars.size() == exponents.size());
  map<Variable, int> powers;
  for (int i = 0; i < vars.size(); ++i) {
    if (exponents[i] > 0) {
      powers.emplace(vars[i], exponents[i]);
    } else if (exponents[i] < 0) {
      throw std::logic_error("The exponent is negative.");
    }
  }
  return powers;
}

}  // namespace

Monomial::Monomial(const Eigen::Ref<const VectorX<Variable>>& vars,
                   const Eigen::Ref<const Eigen::VectorXi>& exponents)
    : total_degree_{exponents.sum()},
      powers_{ToMonomialPower(vars, exponents)} {}

Monomial::Monomial(const Variable& var) : total_degree_{1}, powers_{{var, 1}} {}

Monomial::Monomial(const Variable& var, const int exponent)
    : total_degree_{exponent} {
  DRAKE_DEMAND(exponent >= 0);
  if (exponent > 0) {
    powers_.emplace(var, exponent);
  }
}

Monomial::Monomial(const map<Variable, int>& powers)
    : total_degree_{TotalDegree(powers)} {
  for (const auto& p : powers) {
    const int exponent{p.second};
    if (exponent > 0) {
      powers_.insert(p);
    } else if (exponent < 0) {
      throw std::logic_error("The exponent is negative.");
    }
    // Ignore the entry if exponent == 0.
  }
}

Monomial::Monomial(const Expression& e)
    : Monomial(ToMonomialPower(e.Expand())) {}

int Monomial::degree(const Variable& v) const {
  const auto it = powers_.find(v);
  if (it == powers_.end()) {
    return 0;
  } else {
    return it->second;
  }
}

Variables Monomial::GetVariables() const {
  Variables vars{};
  for (const pair<const Variable, int>& p : powers_) {
    vars += p.first;
  }
  return vars;
}

bool Monomial::operator==(const Monomial& m) const {
  // The first test below checks the number of factors in each monomial, e.g., x
  // * y^2 * z^3 differs from x * y^2 due to a different number of factors. x *
  // y^2 * z^3 and x * y^2 * z^7 have the same number of factors and this first
  // test is inconclusive as to whether the monomials are equal.
  if (powers_.size() != m.powers_.size()) return false;
  // The second test compares the variables and exponents on each factor, e.g.,
  // x * y^2 * z^3 and x * y^2 * z^7 returns false (different exponent on z).
  // x * y^2 * z^3 and x * y^2 * b^3 returns false (different variable z vs. b).
  // x * y^2 * z^3 and x * y^2 * z^3 returns true (equal monomials).
  for (auto it1 = powers_.begin(), it2 = m.powers_.begin();
       it1 != powers_.end(); ++it1, ++it2) {
    const Variable& var1{it1->first};
    const Variable& var2{it2->first};
    const int exponent1{it1->second};
    const int exponent2{it2->second};
    if (!var1.equal_to(var2) || exponent1 != exponent2) {
      return false;
    }
  }
  return true;
}

bool Monomial::operator!=(const Monomial& m) const { return !(*this == m); }

double Monomial::Evaluate(const Environment& env) const {
  return accumulate(
      powers_.begin(), powers_.end(), 1.0,
      [this, &env](const double v, const pair<const Variable, int>& p) {
        const Variable& var{p.first};
        const auto it = env.find(var);
        if (it == env.end()) {
          ostringstream oss;
          oss << "Monomial " << *this
              << " cannot be evaluated with the given "
                 "environment which does not provide an entry "
                 "for variable = "
              << var << ".";
          throw runtime_error(oss.str());
        } else {
          const double base{it->second};
          const int exponent{p.second};
          return v * std::pow(base, exponent);
        }
      });
}

pair<double, Monomial> Monomial::EvaluatePartial(const Environment& env) const {
  double coeff{1.0};
  map<Variable, int> new_powers;
  for (const auto& p : powers_) {
    const Variable& var{p.first};
    const int exponent{p.second};
    auto it = env.find(var);
    if (it != env.end()) {
      double base{it->second};
      coeff *= std::pow(base, exponent);
    } else {
      new_powers.insert(p);
    }
  }
  return make_pair(coeff, Monomial(new_powers));
}

Expression Monomial::ToExpression() const {
  // It builds this base_to_exponent_map and uses ExpressionMulFactory to build
  // a multiplication expression.
  map<Expression, Expression> base_to_exponent_map;
  for (const auto& p : powers_) {
    const Variable& var{p.first};
    const int exponent{p.second};
    base_to_exponent_map.emplace(Expression{var}, exponent);
  }
  return ExpressionMulFactory{1.0, base_to_exponent_map}.GetExpression();
}

Monomial& Monomial::operator*=(const Monomial& m) {
  for (const auto& p : m.get_powers()) {
    const Variable& var{p.first};
    const int exponent{p.second};
    auto it = powers_.find(var);
    if (it == powers_.end()) {
      powers_.insert(p);
    } else {
      it->second += exponent;
    }
    total_degree_ += exponent;
  }
  return *this;
}

Monomial& Monomial::pow_in_place(const int p) {
  if (p < 0) {
    ostringstream oss;
    oss << "Monomial::pow(int p) is called with a negative p = " << p;
    throw runtime_error(oss.str());
  }
  if (p == 0) {
    total_degree_ = 0;
    powers_.clear();
  } else if (p > 1) {
    for (auto& item : powers_) {
      int& exponent{item.second};
      exponent *= p;
    }
    total_degree_ *= p;
  }  // If p == 1, NO OP.
  return *this;
}

ostream& operator<<(ostream& out, const Monomial& m) {
  if (m.powers_.empty()) {
    return out << 1;
  }
  auto it = m.powers_.begin();
  out << it->first;
  if (it->second > 1) {
    out << "^" << it->second;
  }
  for (++it; it != m.powers_.end(); ++it) {
    out << " * ";
    out << it->first;
    if (it->second > 1) {
      out << "^" << it->second;
    }
  }
  return out;
}

Monomial operator*(Monomial m1, const Monomial& m2) {
  m1 *= m2;
  return m1;
}

Monomial pow(Monomial m, const int p) { return m.pow_in_place(p); }

}  // namespace symbolic
}  // namespace drake
