#include "drake/solvers/dreal_solver.h"

#include <algorithm>  // To suppress cpplint.
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dreal/api/api.h"

#include "drake/common/text_logging.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

using std::accumulate;
using std::dynamic_pointer_cast;
using std::logic_error;
using std::map;
using std::pair;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::string;
using std::unordered_map;
using std::vector;

namespace {

// Visitor helper-class to implement DrealSolver::CheckSatisfiability and
// DrealSolver::Minimize.
class DrealConverter {
 public:
  // Converts a Drake's symbolic formula `f` into dReal's symbolic formula.
  dreal::Formula Convert(const symbolic::Formula& f);
  // Converts a Drake's symbolic expression `e` into dReal's symbolic
  // expression.
  dreal::Expression Convert(const symbolic::Expression& e);
  // Converts a dReal's box (a mapping from a variable to interval) into
  // DrealSolver::IntervalBox (unordered_map of drake::symbolic::Variable →
  // interval).
  DrealSolver::IntervalBox Convert(const dreal::Box& box) const;

 private:
  dreal::Variable Convert(const symbolic::Variable& var);

  dreal::Formula VisitFalse(const symbolic::Formula& f);
  dreal::Formula VisitTrue(const symbolic::Formula& f);
  dreal::Formula VisitVariable(const symbolic::Formula& f);
  dreal::Formula VisitEqualTo(const symbolic::Formula& f);
  dreal::Formula VisitNotEqualTo(const symbolic::Formula& f);
  dreal::Formula VisitGreaterThan(const symbolic::Formula& f);
  dreal::Formula VisitGreaterThanOrEqualTo(const symbolic::Formula& f);
  dreal::Formula VisitLessThan(const symbolic::Formula& f);
  dreal::Formula VisitLessThanOrEqualTo(const symbolic::Formula& f);
  dreal::Formula VisitConjunction(const symbolic::Formula& f);
  dreal::Formula VisitDisjunction(const symbolic::Formula& f);
  dreal::Formula VisitNegation(const symbolic::Formula& f);
  dreal::Formula VisitForall(const symbolic::Formula& f);
  dreal::Formula VisitIsnan(const symbolic::Formula& f);
  dreal::Formula VisitPositiveSemidefinite(const symbolic::Formula& f);

  dreal::Expression VisitAddition(const symbolic::Expression& e);
  dreal::Expression VisitMultiplication(const symbolic::Expression& e);
  dreal::Expression VisitDivision(const symbolic::Expression& e);
  dreal::Expression VisitVariable(const symbolic::Expression& e);
  dreal::Expression VisitConstant(const symbolic::Expression& e);
  dreal::Expression VisitLog(const symbolic::Expression& e);
  dreal::Expression VisitPow(const symbolic::Expression& e);
  dreal::Expression VisitAbs(const symbolic::Expression& e);
  dreal::Expression VisitExp(const symbolic::Expression& e);
  dreal::Expression VisitSqrt(const symbolic::Expression& e);
  dreal::Expression VisitSin(const symbolic::Expression& e);
  dreal::Expression VisitCos(const symbolic::Expression& e);
  dreal::Expression VisitTan(const symbolic::Expression& e);
  dreal::Expression VisitAsin(const symbolic::Expression& e);
  dreal::Expression VisitAcos(const symbolic::Expression& e);
  dreal::Expression VisitAtan(const symbolic::Expression& e);
  dreal::Expression VisitAtan2(const symbolic::Expression& e);
  dreal::Expression VisitSinh(const symbolic::Expression& e);
  dreal::Expression VisitCosh(const symbolic::Expression& e);
  dreal::Expression VisitTanh(const symbolic::Expression& e);
  dreal::Expression VisitMin(const symbolic::Expression& e);
  dreal::Expression VisitMax(const symbolic::Expression& e);
  dreal::Expression VisitCeil(const symbolic::Expression& e);
  dreal::Expression VisitFloor(const symbolic::Expression& e);
  dreal::Expression VisitIfThenElse(const symbolic::Expression& e);
  dreal::Expression VisitUninterpretedFunction(const symbolic::Expression& e);

  // Makes VisitFormula a friend of this class so that it can use private
  // methods.
  friend dreal::Formula symbolic::VisitFormula<dreal::Formula>(
      DrealConverter*, const symbolic::Formula&);

  // Makes VisitExpression a friend of this class so that VisitExpression can
  // use its private methods.
  friend dreal::Expression symbolic::VisitExpression<dreal::Expression>(
      DrealConverter*, const symbolic::Expression&);

  // Mapping: dreal::Variable → drake::symbolic::Variable.
  unordered_map<dreal::Variable, symbolic::Variable,
                dreal::hash_value<dreal::Variable>>
      dreal_to_drake_variable_map_;

  // Mapping: drake::symbolic::Variable → dreal::Variable.
  unordered_map<symbolic::Variable, dreal::Variable>
      drake_to_dreal_variable_map_;
};

dreal::Variable::Type convert_type(const symbolic::Variable::Type type) {
  switch (type) {
    case symbolic::Variable::Type::CONTINUOUS:
      return dreal::Variable::Type::CONTINUOUS;
    case symbolic::Variable::Type::INTEGER:
      return dreal::Variable::Type::INTEGER;
    case symbolic::Variable::Type::BINARY:
      return dreal::Variable::Type::BINARY;
    case symbolic::Variable::Type::BOOLEAN:
      return dreal::Variable::Type::BOOLEAN;
    case symbolic::Variable::Type::RANDOM_UNIFORM:
    case symbolic::Variable::Type::RANDOM_GAUSSIAN:
    case symbolic::Variable::Type::RANDOM_EXPONENTIAL:
      throw runtime_error(
          "Random variables are not supported in dReal solver.");
  }
  throw runtime_error("Should be unreachable.");  // LCOV_EXCL_LINE
}

dreal::Formula DrealConverter::Convert(const symbolic::Formula& f) {
  return symbolic::VisitFormula<dreal::Formula>(this, f);
}

dreal::Expression DrealConverter::Convert(const symbolic::Expression& e) {
  return symbolic::VisitExpression<dreal::Expression>(this, e);
}

DrealSolver::IntervalBox DrealConverter::Convert(const dreal::Box& box) const {
  DrealSolver::IntervalBox interval_box;
  for (const dreal::Variable& dreal_var : box.variables()) {
    const auto it = dreal_to_drake_variable_map_.find(dreal_var);
    if (it == dreal_to_drake_variable_map_.end()) {
      throw runtime_error("Not able to construct an interval box.");
    } else {
      const auto& itv{box[dreal_var]};
      interval_box.emplace(it->second,
                           DrealSolver::Interval{itv.lb(), itv.ub()});
    }
  }
  return interval_box;
}

dreal::Variable DrealConverter::Convert(const symbolic::Variable& var) {
  auto it = drake_to_dreal_variable_map_.find(var);
  if (it == drake_to_dreal_variable_map_.end()) {
    // Not found in the map. Create and add one.
    const dreal::Variable dreal_var{var.get_name(),
                                    convert_type(var.get_type())};
    DRAKE_ASSERT(dreal_to_drake_variable_map_.find(dreal_var) ==
                 dreal_to_drake_variable_map_.end());
    drake_to_dreal_variable_map_.emplace_hint(it, var, dreal_var);
    dreal_to_drake_variable_map_.emplace(dreal_var, var);
    return dreal_var;
  } else {
    // Found `var` in the map.
    return it->second;
  }
}

dreal::Formula DrealConverter::VisitFalse(const symbolic::Formula&) {
  return dreal::Formula::False();
}

dreal::Formula DrealConverter::VisitTrue(const symbolic::Formula&) {
  return dreal::Formula::True();
}

dreal::Formula DrealConverter::VisitVariable(const symbolic::Formula& f) {
  return dreal::Formula{Convert(get_variable(f))};
}

dreal::Formula DrealConverter::VisitEqualTo(const symbolic::Formula& f) {
  const symbolic::Expression& lhs{get_lhs_expression(f)};
  const symbolic::Expression& rhs{get_rhs_expression(f)};
  return Convert(lhs) == Convert(rhs);
}

dreal::Formula DrealConverter::VisitNotEqualTo(const symbolic::Formula& f) {
  const symbolic::Expression& lhs{get_lhs_expression(f)};
  const symbolic::Expression& rhs{get_rhs_expression(f)};
  return Convert(lhs) != Convert(rhs);
}

dreal::Formula DrealConverter::VisitGreaterThan(const symbolic::Formula& f) {
  const symbolic::Expression& lhs{get_lhs_expression(f)};
  const symbolic::Expression& rhs{get_rhs_expression(f)};
  return Convert(lhs) > Convert(rhs);
}

dreal::Formula DrealConverter::VisitGreaterThanOrEqualTo(
    const symbolic::Formula& f) {
  const symbolic::Expression& lhs{get_lhs_expression(f)};
  const symbolic::Expression& rhs{get_rhs_expression(f)};
  return Convert(lhs) >= Convert(rhs);
}

dreal::Formula DrealConverter::VisitLessThan(const symbolic::Formula& f) {
  const symbolic::Expression& lhs{get_lhs_expression(f)};
  const symbolic::Expression& rhs{get_rhs_expression(f)};
  return Convert(lhs) < Convert(rhs);
}

dreal::Formula DrealConverter::VisitLessThanOrEqualTo(
    const symbolic::Formula& f) {
  const symbolic::Expression& lhs{get_lhs_expression(f)};
  const symbolic::Expression& rhs{get_rhs_expression(f)};
  return Convert(lhs) <= Convert(rhs);
}

dreal::Formula DrealConverter::VisitConjunction(const symbolic::Formula& f) {
  const set<symbolic::Formula>& conjuncts{get_operands(f)};
  return accumulate(
      conjuncts.begin(), conjuncts.end(), dreal::Formula::True(),
      [this](const dreal::Formula& f1, const symbolic::Formula& f2) {
        return f1 && Convert(f2);
      });
}

dreal::Formula DrealConverter::VisitDisjunction(const symbolic::Formula& f) {
  const set<symbolic::Formula>& disjuncts{get_operands(f)};
  return accumulate(
      disjuncts.begin(), disjuncts.end(), dreal::Formula::False(),
      [this](const dreal::Formula& f1, const symbolic::Formula& f2) {
        return f1 || Convert(f2);
      });
}

dreal::Formula DrealConverter::VisitNegation(const symbolic::Formula& f) {
  return !Convert(get_operand(f));
}

dreal::Formula DrealConverter::VisitForall(const symbolic::Formula& f) {
  const symbolic::Variables& quantified_variables{get_quantified_variables(f)};
  const symbolic::Formula& quantified_formula{get_quantified_formula(f)};
  dreal::Variables dreal_quantified_variables;
  for (const symbolic::Variable& var : quantified_variables) {
    dreal_quantified_variables.insert(Convert(var));
  }
  // TODO(soonho): Remove quantified variables from the maps.
  return forall(dreal_quantified_variables, Convert(quantified_formula));
}

dreal::Formula DrealConverter::VisitIsnan(const symbolic::Formula&) {
  throw runtime_error("Not supported.");
}

dreal::Formula DrealConverter::VisitPositiveSemidefinite(
    const symbolic::Formula&) {
  throw runtime_error("Not supported.");
}

dreal::Expression DrealConverter::VisitAddition(const symbolic::Expression& e) {
  const double c{get_constant_in_addition(e)};
  const map<symbolic::Expression, double>& expr_to_coeff_map{
      get_expr_to_coeff_map_in_addition(e)};
  return accumulate(expr_to_coeff_map.begin(), expr_to_coeff_map.end(),
                    dreal::Expression{c},
                    [this](const dreal::Expression& expr,
                           const pair<symbolic::Expression, double>& p) {
                      return expr + p.second * Convert(p.first);
                    });
}

dreal::Expression DrealConverter::VisitMultiplication(
    const symbolic::Expression& e) {
  const double c{get_constant_in_multiplication(e)};
  const map<symbolic::Expression, symbolic::Expression>& base_to_exponent_map{
      get_base_to_exponent_map_in_multiplication(e)};
  return accumulate(
      base_to_exponent_map.begin(), base_to_exponent_map.end(),
      dreal::Expression{c},
      [this](const dreal::Expression& expr,
             const pair<symbolic::Expression, symbolic::Expression>& p) {
        return expr * pow(Convert(p.first), Convert(p.second));
      });
}

dreal::Expression DrealConverter::VisitDivision(const symbolic::Expression& e) {
  return Convert(get_first_argument(e)) / Convert(get_second_argument(e));
}

dreal::Expression DrealConverter::VisitVariable(const symbolic::Expression& e) {
  return dreal::Expression{Convert(get_variable(e))};
}

dreal::Expression DrealConverter::VisitConstant(const symbolic::Expression& e) {
  return dreal::Expression{get_constant_value(e)};
}

dreal::Expression DrealConverter::VisitLog(const symbolic::Expression& e) {
  return log(Convert(get_argument(e)));
}

dreal::Expression DrealConverter::VisitPow(const symbolic::Expression& e) {
  return pow(Convert(get_first_argument(e)), Convert(get_second_argument(e)));
}

dreal::Expression DrealConverter::VisitAbs(const symbolic::Expression& e) {
  return abs(Convert(get_argument(e)));
}

dreal::Expression DrealConverter::VisitExp(const symbolic::Expression& e) {
  return exp(Convert(get_argument(e)));
}

dreal::Expression DrealConverter::VisitSqrt(const symbolic::Expression& e) {
  return sqrt(Convert(get_argument(e)));
}

dreal::Expression DrealConverter::VisitSin(const symbolic::Expression& e) {
  return sin(Convert(get_argument(e)));
}

dreal::Expression DrealConverter::VisitCos(const symbolic::Expression& e) {
  return cos(Convert(get_argument(e)));
}

dreal::Expression DrealConverter::VisitTan(const symbolic::Expression& e) {
  return tan(Convert(get_argument(e)));
}

dreal::Expression DrealConverter::VisitAsin(const symbolic::Expression& e) {
  return asin(Convert(get_argument(e)));
}

dreal::Expression DrealConverter::VisitAcos(const symbolic::Expression& e) {
  return acos(Convert(get_argument(e)));
}

dreal::Expression DrealConverter::VisitAtan(const symbolic::Expression& e) {
  return atan(Convert(get_argument(e)));
}

dreal::Expression DrealConverter::VisitAtan2(const symbolic::Expression& e) {
  return atan2(Convert(get_first_argument(e)), Convert(get_second_argument(e)));
}

dreal::Expression DrealConverter::VisitSinh(const symbolic::Expression& e) {
  return sinh(Convert(get_argument(e)));
}

dreal::Expression DrealConverter::VisitCosh(const symbolic::Expression& e) {
  return cosh(Convert(get_argument(e)));
}

dreal::Expression DrealConverter::VisitTanh(const symbolic::Expression& e) {
  return tanh(Convert(get_argument(e)));
}

dreal::Expression DrealConverter::VisitMin(const symbolic::Expression& e) {
  return min(Convert(get_first_argument(e)), Convert(get_second_argument(e)));
}

dreal::Expression DrealConverter::VisitMax(const symbolic::Expression& e) {
  return max(Convert(get_first_argument(e)), Convert(get_second_argument(e)));
}

dreal::Expression DrealConverter::VisitCeil(const symbolic::Expression&) {
  throw runtime_error("Not supported.");
}

dreal::Expression DrealConverter::VisitFloor(const symbolic::Expression&) {
  throw runtime_error("Not supported.");
}

dreal::Expression DrealConverter::VisitIfThenElse(
    const symbolic::Expression& e) {
  return if_then_else(Convert(get_conditional_formula(e)),
                      Convert(get_then_expression(e)),
                      Convert(get_else_expression(e)));
}

dreal::Expression DrealConverter::VisitUninterpretedFunction(
    const symbolic::Expression&) {
  throw runtime_error("Not supported.");
}
}  // namespace

// -----------------------------
// Implementation of DrealSolver
// -----------------------------
std::optional<DrealSolver::IntervalBox> DrealSolver::CheckSatisfiability(
    const symbolic::Formula& f, const double delta) {
  DrealConverter dreal_converter;
  dreal::Box model;
  const bool result{
      dreal::CheckSatisfiability(dreal_converter.Convert(f), delta, &model)};
  if (result) {
    return dreal_converter.Convert(model);
  } else {
    return std::nullopt;
  }
}

std::optional<DrealSolver::IntervalBox> DrealSolver::Minimize(
    const symbolic::Expression& objective, const symbolic::Formula& constraint,
    const double delta, const LocalOptimization local_optimization) {
  DrealConverter dreal_converter;
  dreal::Box model;
  dreal::Config config;
  config.mutable_precision() = delta;
  config.mutable_use_local_optimization() =
      local_optimization == LocalOptimization::kUse;
  const bool result{dreal::Minimize(dreal_converter.Convert(objective),
                                    dreal_converter.Convert(constraint), config,
                                    &model)};
  if (result) {
    return dreal_converter.Convert(model);
  } else {
    return std::nullopt;
  }
}

bool DrealSolver::is_available() { return true; }

namespace {
// Extracts value of @p key from @p solver_options. If not specified,
// returns @p default_value.
template <typename T>
T GetOptionWithDefaultValue(const SolverOptions& solver_options,
                            const std::string& key, const T& default_value) {
  const auto& options = solver_options.GetOptions<T>(DrealSolver::id());
  const auto it = options.find(key);
  if (it != options.end()) {
    return it->second;
  }
  return default_value;
}

template <typename T>
symbolic::Formula ExtractConstraints(const vector<Binding<T>>& bindings) {
  symbolic::Formula f{symbolic::Formula::True()};
  for (const Binding<T>& binding : bindings) {
    f = f && binding.evaluator()->CheckSatisfied(binding.variables());
  }
  return f;
}

// Extracts bounding box constraints in @p prog into a symbolic formula.
// This is a helper function used in DrealSolver::Solve.
symbolic::Formula ExtractBoundingBoxConstraints(
    const MathematicalProgram& prog) {
  return ExtractConstraints(prog.bounding_box_constraints());
}

// Extracts linear constraints in @p prog into a symbolic formula.
// This is a helper function used in DrealSolver::Solve.
symbolic::Formula ExtractLinearConstraints(const MathematicalProgram& prog) {
  return ExtractConstraints(prog.linear_constraints());
}

// Extracts linear-equality constraints in @p prog into a symbolic formula.
// This is a helper function used in DrealSolver::Solve.
symbolic::Formula ExtractLinearEqualityConstraints(
    const MathematicalProgram& prog) {
  return ExtractConstraints(prog.linear_equality_constraints());
}

// Extracts Lorentz-cone constraints in @p prog into a symbolic formula.
// This is a helper function used in DrealSolver::Solve.
symbolic::Formula ExtractLorentzConeConstraints(
    const MathematicalProgram& prog) {
  return ExtractConstraints(prog.lorentz_cone_constraints());
}

// Extracts rotated Lorentz-cone constraints in @p prog into a symbolic formula.
// This is a helper function used in DrealSolver::Solve.
symbolic::Formula ExtractRotatedLorentzConeConstraints(
    const MathematicalProgram& prog) {
  return ExtractConstraints(prog.rotated_lorentz_cone_constraints());
}

// Extracts linear-complementarity constraints in @p prog into a symbolic
// formula. This is a helper function used in DrealSolver::Solve.
symbolic::Formula ExtractLinearComplementarityConstraints(
    const MathematicalProgram& prog) {
  return ExtractConstraints(prog.linear_complementarity_constraints());
}

// Extracts generic constraints in @p prog into a symbolic formula. This is a
// helper function used in DrealSolver::Solve.
//
// @throws std::exception if there is a generic-constraint which does not
// provide symbolic evaluation.
symbolic::Formula ExtractGenericConstraints(const MathematicalProgram& prog) {
  return ExtractConstraints(prog.generic_constraints());
}

// Extracts linear costs in @p prog and push them into @p costs vector. This is
// a helper function used in DrealSolver::Solve.
void ExtractLinearCosts(const MathematicalProgram& prog,
                        vector<symbolic::Expression>* const costs) {
  for (const Binding<LinearCost>& binding : prog.linear_costs()) {
    const VectorXDecisionVariable& x{binding.variables()};
    const shared_ptr<LinearCost>& cost{binding.evaluator()};
    VectorX<symbolic::Expression> y;
    cost->Eval(x, &y);
    costs->push_back(y[0]);
  }
}

// Extracts quadratic costs in @p prog and push them into @p costs vector.
void ExtractQuadraticCosts(const MathematicalProgram& prog,
                           vector<symbolic::Expression>* const costs) {
  for (const Binding<QuadraticCost>& binding : prog.quadratic_costs()) {
    const VectorXDecisionVariable& x{binding.variables()};
    const shared_ptr<QuadraticCost>& cost{binding.evaluator()};
    VectorX<symbolic::Expression> y;
    cost->Eval(x, &y);
    costs->push_back(y[0]);
  }
}

}  // namespace

void DrealSolver::DoSolve(
    const MathematicalProgram& prog,
    const Eigen::VectorXd& initial_guess,
    const SolverOptions& merged_options,
    MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
      "DrealSolver doesn't support the feature of variable scaling.");
  }

  unused(initial_guess);

  // 1. Extracts the constraints from @p prog and constructs an equivalent
  // symbolic formula.
  symbolic::Formula constraints{ExtractBoundingBoxConstraints(prog) &&
                                ExtractLinearConstraints(prog) &&
                                ExtractLinearEqualityConstraints(prog) &&
                                ExtractLorentzConeConstraints(prog) &&
                                ExtractRotatedLorentzConeConstraints(prog) &&
                                ExtractLinearComplementarityConstraints(prog) &&
                                ExtractGenericConstraints(prog)};
  // 2. Extracts the costs from @p prog and constructs equivalent symbolic
  // Expressions.
  vector<symbolic::Expression> costs;
  ExtractLinearCosts(prog, &costs);
  ExtractQuadraticCosts(prog, &costs);
  // TODO(soonho): Add ExtractGenericCosts.

  // 3. Call dReal to check the delta-satisfiability of the problem.

  // TODO(soonho): Support other dReal options. For now, we only support
  // "--precision" and "--local-optimization".

  const double precision{GetOptionWithDefaultValue(
      merged_options, "precision", 0.001 /* default */)};
  const LocalOptimization local_optimization{
      GetOptionWithDefaultValue<int>(
          merged_options, "use_local_optimization", 1 /* default */) > 0
          ? LocalOptimization::kUse
          : LocalOptimization::kNotUse};
  std::optional<IntervalBox> dreal_result;
  if (costs.size() == 0) {
    // No cost functions in the problem. Call Checksatisfiability.
    dreal_result = CheckSatisfiability(constraints, precision);
  } else {
    // Call Minimize with cost = ∑ᵢ costs(i).
    dreal_result = Minimize(
        accumulate(costs.begin(), costs.end(), symbolic::Expression::Zero(),
                   std::plus<symbolic::Expression>{}),
        constraints, precision, local_optimization);
  }

  // 4. Sets MathematicalProgramResult.
  result->set_solution_result(SolutionResult::kUnknownError);
  if (dreal_result) {
    // 4.1. delta-SAT case.
    const int num_vars{prog.num_vars()};
    Eigen::VectorXd solution_vector(num_vars);
    // dReal returns an interval box instead of a point as a solution. We pick
    // the mid-point from a returned box and assign it to the result.
    for (const auto& item : *dreal_result) {
      const symbolic::Variable& var{item.first};
      const double v{item.second.mid()};
      solution_vector(prog.FindDecisionVariableIndex(var)) = v;
    }
    result->set_solution_result(SolutionResult::kSolutionFound);
    result->set_x_val(solution_vector);
  } else {
    // 4.2. UNSAT case
    result->set_solution_result(SolutionResult::kInfeasibleConstraints);
  }
}

}  // namespace solvers
}  // namespace drake
