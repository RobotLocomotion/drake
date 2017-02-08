#include "drake/common/monomial.h"

#include <algorithm>
#include <map>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_expression_cell.h"
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
using std::unordered_map;

namespace internal {
Monomial::Monomial(const Variable& var, const int exponent)
    : total_degree_{exponent} {
  DRAKE_DEMAND(exponent >= 0);
  if (exponent > 0) {
    powers_.emplace(var.get_id(), exponent);
  }
}

Monomial::Monomial(const map<Variable::Id, int>& powers)
    : total_degree_{TotalDegree(powers)}, powers_(powers) {}

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
}  // namespace internal

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
