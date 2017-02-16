#include "drake/common/monomial.h"

#include <algorithm>
#include <map>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_expression_cell.h"
#include "drake/common/symbolic_expression_visitor.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

using std::accumulate;
using std::map;
using std::ostream;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::shared_ptr;
using std::unordered_map;

namespace internal {
Monomial::Monomial() : total_degree_{0}, powers_{} {}

Monomial::Monomial(const Variable& var, const int exponent)
    : total_degree_{exponent} {
  DRAKE_DEMAND(exponent >= 0);
  if (exponent > 0) {
    powers_.emplace(var.get_id(), exponent);
  }
}

Monomial::Monomial(const map<Variable::Id, int>& powers)
    : total_degree_{TotalDegree(powers)}, powers_{} {
  for (const auto& p : powers) {
    if (p.second > 0) {
      powers_.insert(p);
    } else if (p.second < 0) {
      throw std::runtime_error("The exponent is negative.");
    }
  }
}

// Forward declaration.
map<Variable::Id, int> ToMonomialPower(const Expression& e);

Monomial::Monomial(const Expression& e) : Monomial(ToMonomialPower(e)) {}

size_t Monomial::GetHash() const {
  // To get a hash value for a Monomial, we re-use the hash value for
  // powers_. This is suitable because powers_ is the only independent
  // data-member of Monomial class while another data-member, total_degree_ is
  // determined by a given powers_.
  return hash_value<map<Variable::Id, int>>{}(powers_);
}

bool Monomial::operator==(const Monomial& m) const {
  return powers_ == m.powers_;
}

Expression Monomial::ToExpression(
    const unordered_map<Variable::Id, Variable>& id_to_var_map) const {
  // It builds this base_to_exponent_map and uses ExpressionMulFactory to build
  // a multiplication expression.
  map<Expression, Expression> base_to_exponent_map;
  for (const auto& p : powers_) {
    const Variable::Id id{p.first};
    const int exponent{p.second};
    const auto it = id_to_var_map.find(id);
    if (it != id_to_var_map.end()) {
      const Variable& var{it->second};
      base_to_exponent_map.emplace(Expression{var}, exponent);
    } else {
      ostringstream oss;
      oss << "Variable whose ID is " << id << " appeared in a monomial "
          << *this << "." << std::endl
          << "However, Monomial::ToExpression method "
             "fails to find the corresponding Variable in a given map.";
      throw runtime_error(oss.str());
    }
  }
  return ExpressionMulFactory{1.0, base_to_exponent_map}.GetExpression();
}

int Monomial::TotalDegree(const map<Variable::Id, int>& powers) {
  return accumulate(powers.begin(), powers.end(), 0,
                    [](const int degree, const pair<Variable::Id, int>& p) {
                      return degree + p.second;
                    });
}

std::ostream& operator<<(std::ostream& out, const Monomial& m) {
  out << "{";
  for (const auto& power : m.powers_) {
    out << "(" << power.first << ", " << power.second << ") ";
  }
  return out << "}";
}

Monomial operator*(const Monomial& m1, const Monomial& m2) {
  map<Variable::Id, int> powers{m1.get_powers()};
  for (const pair<Variable::Id, int>& p : m2.get_powers()) {
    const Variable::Id var{p.first};
    const int exponent{p.second};
    auto it = powers.find(var);
    if (it == powers.end()) {
      powers.insert(p);
    } else {
      it->second += exponent;
    }
  }
  return Monomial{powers};
}

map<Variable::Id, int> ToMonomialPower(const Expression& e) {
  DRAKE_DEMAND(e.is_polynomial());
  map<Variable::Id, int> powers;
  if (is_one(e)) {  // This block is deliberately left empty.
  } else if (is_constant(e)) {
    throw runtime_error(
        "A constant not equal to to 1, this is not a monomial.");
  } else if (is_variable(e)) {
    powers.emplace(get_variable(e).get_id(), 1);
  } else if (is_pow(e)) {
    const auto& exponent = get_second_argument(e);
    if (!is_constant(exponent)) {
      throw runtime_error("The exponent is not a constant.");
    }
    auto exponent_val = get_constant_value(exponent);
    powers = ToMonomialPower(get_first_argument(e));
    // pow( pow(xᵢ, kᵢ), n) = pow(xᵢ, kᵢ * n)
    for (auto& p : powers) {
      p.second *= exponent_val;
    }
  } else if (is_multiplication(e)) {
    if (!is_one(get_constant_in_multiplication(e))) {
      throw runtime_error("The constant in the multiplication is not 1.");
    }
    for (const auto& p : get_base_to_exponent_map_in_multiplication(e)) {
      map<Variable::Id, int> p_powers = ToMonomialPower(pow(p.first, p.second));
      for (const auto& q : p_powers) {
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

}  // namespace internal

class DegreeVisitor {
 public:
  int Visit(const Expression& e, const Variables& vars) const {
    return VisitPolynomial<int>(*this, e, vars);
  }

  int operator()(const shared_ptr<ExpressionVar>& e,
                 const Variables& vars) const {
    return vars.include(e->get_variable()) ? 1 : 0;
  }

  int operator()(const shared_ptr<ExpressionConstant>& e,
                 const Variables& vars) const {
    return 0;
  }

  int operator()(const shared_ptr<ExpressionAdd>& e,
                 const Variables& vars) const {
    int degree = 0;
    for (const auto& p : e->get_expr_to_coeff_map()) {
      degree = std::max(degree, Visit(p.first, vars));
    }
    return degree;
  }

  int operator()(const shared_ptr<ExpressionMul>& e,
                 const Variables& vars) const {
    return accumulate(
        e->get_base_to_exponent_map().begin(),
        e->get_base_to_exponent_map().end(), 0,
        [this, &vars](const int& degree,
                      const pair<Expression, Expression>& p) {
          const Expression& base{p.first};
          const Expression& exponent{p.second};
          return degree +
                 Visit(base, vars) *
                     static_cast<int>(get_constant_value(exponent));
        });
  }

  int operator()(const shared_ptr<ExpressionDiv>& e,
                 const Variables& vars) const {
    return Visit(e->get_first_argument(), vars) -
           Visit(e->get_second_argument(), vars);
  }

  int operator()(const shared_ptr<ExpressionPow>& e,
                 const Variables& vars) const {
    const int exponent{
        static_cast<int>(get_constant_value(e->get_second_argument()))};
    return Visit(e->get_first_argument(), vars) * exponent;
  }
};

int Degree(const Expression& e, const Variables& vars) {
  return DegreeVisitor().Visit(e, vars);
}

int Degree(const Expression& e) { return Degree(e, e.GetVariables()); }

Expression GetMonomial(const unordered_map<Variable, int, hash_value<Variable>>&
                           map_var_to_exponent) {
  map<Expression, Expression> base_to_exponent_map;
  for (const auto& p : map_var_to_exponent) {
    DRAKE_DEMAND(p.second > 0);
    base_to_exponent_map.emplace(Expression{p.first}, p.second);
  }
  return ExpressionMulFactory{1.0, base_to_exponent_map}.GetExpression();
}

Eigen::Matrix<Expression, Eigen::Dynamic, 1> MonomialBasis(
    const Variables& vars, const int degree) {
  return internal::ComputeMonomialBasis<Eigen::Dynamic>(vars, degree);
}
}  // namespace symbolic
}  // namespace drake
