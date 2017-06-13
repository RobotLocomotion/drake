#include "drake/common/monomial_util.h"

#include <algorithm>
#include <map>
#include <numeric>
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
using std::max;
using std::move;
using std::pair;
using std::runtime_error;
using std::unordered_map;

class DegreeVisitor {
 public:
  int Visit(const Expression& e, const Variables& vars) const {
    return VisitPolynomial<int>(this, e, vars);
  }

 private:
  int VisitVariable(const Expression& e, const Variables& vars) const {
    return vars.include(get_variable(e)) ? 1 : 0;
  }

  int VisitConstant(const Expression&, const Variables&) const { return 0; }

  int VisitAddition(const Expression& e, const Variables& vars) const {
    int degree = 0;
    for (const auto& p : get_expr_to_coeff_map_in_addition(e)) {
      degree = max(degree, Visit(p.first, vars));
    }
    return degree;
  }

  int VisitMultiplication(const Expression& e, const Variables& vars) const {
    const auto& base_to_exponent_map =
        get_base_to_exponent_map_in_multiplication(e);
    return accumulate(
        base_to_exponent_map.begin(), base_to_exponent_map.end(), 0,
        [this, &vars](const int& degree,
                      const pair<Expression, Expression>& p) {
          const Expression& base{p.first};
          const Expression& exponent{p.second};
          return degree + Visit(base, vars) *
                              static_cast<int>(get_constant_value(exponent));
        });
  }

  int VisitDivision(const Expression& e, const Variables& vars) const {
    return Visit(get_first_argument(e), vars) -
           Visit(get_second_argument(e), vars);
  }

  int VisitPow(const Expression& e, const Variables& vars) const {
    const int exponent{
        static_cast<int>(get_constant_value(get_second_argument(e)))};
    return Visit(get_first_argument(e), vars) * exponent;
  }

  // Makes VisitPolynomial a friend of this class so that it can use private
  // methods.
  friend int drake::symbolic::VisitPolynomial<int>(const DegreeVisitor*,
                                                   const Expression&,
                                                   const Variables&);
};

int Degree(const Expression& e, const Variables& vars) {
  return DegreeVisitor().Visit(e, vars);
}

int Degree(const Expression& e) { return Degree(e, e.GetVariables()); }

Eigen::Matrix<Monomial, Eigen::Dynamic, 1> MonomialBasis(const Variables& vars,
                                                         const int degree) {
  return internal::ComputeMonomialBasis<Eigen::Dynamic>(vars, degree);
}

}  // namespace symbolic
}  // namespace drake
