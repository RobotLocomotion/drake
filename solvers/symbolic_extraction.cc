#include "drake/solvers/symbolic_extraction.h"

#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/common/symbolic_decompose.h"
#include "drake/math/matrix_util.h"

namespace drake {
namespace solvers {
namespace internal {

using std::endl;
using std::make_pair;
using std::map;
using std::numeric_limits;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::string;
using std::unordered_map;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

string SymbolicError::make_string(const symbolic::Expression& e,
                                  double lb, double ub, const string& msg) {
  ostringstream oss;
  oss << "Constraint " << lb << " <= " << e << " <= " << ub << " is " << msg
      << ".";
  return oss.str();
}
string SymbolicError::make_string(const symbolic::Expression& e,
                                  const string& msg) {
  ostringstream oss;
  oss << "Constraint " << e << " is " << msg << ".";
  return oss.str();
}
SymbolicError::SymbolicError(const symbolic::Expression& e, const string& msg)
    : runtime_error{make_string(e, msg)} {}
SymbolicError::SymbolicError(const symbolic::Expression& e, double lb,
                             double ub, const string& msg)
    : runtime_error{make_string(e, lb, ub, msg)} {}

void ExtractAndAppendVariablesFromExpression(
    const Expression& e, VectorXDecisionVariable* vars,
    unordered_map<Variable::Id, int>* map_var_to_index) {
  return symbolic::ExtractAndAppendVariablesFromExpression(e, vars,
                                                           map_var_to_index);
}

void DecomposeLinearExpression(const Eigen::Ref<const VectorX<Expression>>& v,
                               Eigen::MatrixXd* A, Eigen::VectorXd* b,
                               VectorXDecisionVariable* vars) {
  symbolic::DecomposeAffineExpressions(v, A, b, vars);
}

pair<VectorXDecisionVariable, unordered_map<Variable::Id, int>>
ExtractVariablesFromExpression(const Expression& e) {
  return symbolic::ExtractVariablesFromExpression(e);
}

void DecomposeQuadraticPolynomial(
    const symbolic::Polynomial& poly,
    const unordered_map<Variable::Id, int>& map_var_to_index,
    Eigen::MatrixXd* Q, Eigen::VectorXd* b, double* c) {
  symbolic::DecomposeQuadraticPolynomial(poly, map_var_to_index, Q, b, c);
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
