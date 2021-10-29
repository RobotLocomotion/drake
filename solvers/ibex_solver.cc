#include "drake/solvers/ibex_solver.h"

#include <memory>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <ibex.h>

#include "drake/common/text_logging.h"
#include "drake/solvers/ibex_converter.h"

namespace drake {
namespace solvers {

using Eigen::VectorXd;

using std::shared_ptr;
using std::vector;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

namespace {

struct IbexOptions {
  // Relative precision on the objective. See
  // http://www.ibex-lib.org/doc/optim.html?highlight=eps#objective-precision-criteria
  // for details.
  double rel_eps_f{ibex::DefaultOptimizerConfig::default_rel_eps_f};

  // Absolute precision on the objective. See
  // http://www.ibex-lib.org/doc/optim.html?highlight=eps#objective-precision-criteria
  // for details.
  double abs_eps_f{ibex::DefaultOptimizerConfig::default_abs_eps_f};

  // Equality relaxation value. See
  // http://www.ibex-lib.org/doc/optim.html?highlight=eps#the-output-of-ibexopt
  // for details.
  double eps_h{ibex::NormalizedSystem::default_eps_h};

  // Activate rigor mode (certify feasibility of equations). See
  // http://www.ibex-lib.org/doc/optim.html?highlight=rigor#return-status for
  // details.
  bool rigor{false};

  // Random seed (useful for reproducibility).
  double random_seed{ibex::DefaultOptimizerConfig::default_random_seed};

  // Precision on the variable.
  double eps_x{ibex::DefaultOptimizerConfig::default_eps_x};

  // Activate trace. Updates of loup/uplo are printed while minimizing.
  // - 0 (by default): nothing is printed.
  // - 1:              prints every loup/uplo update.
  // - 2:              also prints each handled node.
  int trace{0};

  // Timeout (time in seconds). 0.0 indicates +∞.
  double timeout{0.0};
};

IbexOptions ParseOptions(const SolverOptions& merged_options) {
  IbexOptions options;

  for (const auto& it : merged_options.GetOptionsDouble(IbexSolver::id())) {
    if (it.first == "rel_eps_f") {
      if (it.second <= 0) {
        throw std::runtime_error{fmt::format(
            "IbexSolver: '{option}' option should have a positive value but "
            "'{value}' was provided.",
            fmt::arg("option", it.first), fmt::arg("value", it.second))};
      }
      options.rel_eps_f = it.second;
      continue;
    }
    if (it.first == "abs_eps_f") {
      if (it.second <= 0) {
        throw std::runtime_error{fmt::format(
            "IbexSolver: '{option}' option should have a positive value but "
            "'{value}' was provided.",
            fmt::arg("option", it.first), fmt::arg("value", it.second))};
      }
      options.abs_eps_f = it.second;
      continue;
    }
    if (it.first == "eps_h") {
      if (it.second <= 0) {
        throw std::runtime_error{fmt::format(
            "IbexSolver: '{option}' option should have a positive value but "
            "'{value}' was provided.",
            fmt::arg("option", it.first), fmt::arg("value", it.second))};
      }
      options.eps_h = it.second;
      continue;
    }
    if (it.first == "random_seed") {
      options.random_seed = it.second;
      continue;
    }
    if (it.first == "eps_x") {
      if (it.second <= 0) {
        throw std::runtime_error{fmt::format(
            "IbexSolver: '{option}' option should have a positive value but "
            "'{value}' was provided.",
            fmt::arg("option", it.first), fmt::arg("value", it.second))};
      }
      options.eps_x = it.second;
      continue;
    }
    if (it.first == "timeout") {
      if (it.second < 0) {
        throw std::runtime_error{fmt::format(
            "IbexSolver: '{option}' option should have a non-negative value "
            "but '{value}' was provided.",
            fmt::arg("option", it.first), fmt::arg("value", it.second))};
      }
      options.timeout = it.second;
      continue;
    }
    throw std::runtime_error{fmt::format(
        "IbexSolver: unsupported option '{option}' with double value "
        "'{value}'.",
        fmt::arg("option", it.first), fmt::arg("value", it.second))};
  }

  // Handles kPrintToConsole.
  if (merged_options.get_print_to_console()) {
    options.trace = 1;
  }

  for (const auto& it : merged_options.GetOptionsInt(IbexSolver::id())) {
    if (it.first == "rigor") {
      options.rigor = it.second != 0;
      continue;
    }
    if (it.first == "trace") {
      const int value{it.second};
      if (value < 0 || value > 2) {
        throw std::runtime_error{
            fmt::format("IbexSolver: cannot set IBEX Solver option 'trace' "
                        "to value '{value}'. It should be 0, 1, or, 2.",
                        fmt::arg("value", value))};
      }
      options.trace = value;
      continue;
    }
    throw std::runtime_error{fmt::format(
        "IbexSolver: unsupported option '{option}' with int value '{value}'.",
        fmt::arg("option", it.first), fmt::arg("value", it.second))};
  }

  for (const auto& it : merged_options.GetOptionsStr(IbexSolver::id())) {
    throw std::runtime_error{fmt::format(
        "IbexSolver: unsupported option '{option}' with string value "
        "'{value}'.",
        fmt::arg("option", it.first), fmt::arg("value", it.second))};
  }

  return options;
}

// Adds `f` into `factory` using `ibex_converter` and `expr_ctrs`.
//
// @throw if `f` is non-atomic.
void DoAddFormula(const Formula& f,
                  internal::IbexConverter* const ibex_converter,
                  ibex::SystemFactory* const factory,
                  vector<internal::UniquePtrToExprCtr>* const expr_ctrs) {
  internal::UniquePtrToExprCtr expr_ctr{ibex_converter->Convert(f)};
  factory->add_ctr(*expr_ctr);
  expr_ctrs->push_back(std::move(expr_ctr));
}

// Adds `f` into `factory` using `ibex_converter` and `expr_ctrs`.
void AddFormula(const Formula& f, internal::IbexConverter* const ibex_converter,
                ibex::SystemFactory* const factory,
                vector<internal::UniquePtrToExprCtr>* const expr_ctrs) {
  if (symbolic::is_conjunction(f)) {
    for (const auto& f_i : get_operands(f)) {
      DoAddFormula(f_i, ibex_converter, factory, expr_ctrs);
    }
  } else {
    DoAddFormula(f, ibex_converter, factory, expr_ctrs);
  }
}

VectorXd ExtractMidpoints(const ibex::IntervalVector& iv) {
  VectorXd v{iv.size()};
  for (int i = 0; i < iv.size(); ++i) {
    v[i] = iv[i].mid();
  }
  return v;
}

void DoOptimize(const ibex::System& sys, const IbexOptions& options,
                MathematicalProgramResult* const result) {
  const bool in_hc4 = !(sys.nb_ctr > 0 && sys.nb_ctr < sys.f_ctrs.image_dim());
  const bool kkt = (sys.nb_ctr == 0);
  ibex::DefaultOptimizerConfig config(sys, options.rel_eps_f, options.abs_eps_f,
                                      options.eps_h, options.rigor, in_hc4, kkt,
                                      options.random_seed, options.eps_x);

  config.set_trace(options.trace);
  config.set_timeout(options.timeout);
  ibex::Optimizer o(config);
  const ibex::Optimizer::Status status = o.optimize(sys.box);

  switch (status) {
    case ibex::Optimizer::INFEASIBLE:
      // If no feasible point exist less than obj_init_bound. In particular,
      // the function returns INFEASIBLE if the initial bound "obj_init_bound"
      // is LESS than the true minimum (this case is only possible if
      // goal_abs_prec and goal_rel_prec are 0). In the latter case, there may
      // exist feasible points.
      result->set_solution_result(SolutionResult::kInfeasibleConstraints);
      break;
    case ibex::Optimizer::NO_FEASIBLE_FOUND:
      // If no feasible point could be found less than obj_init_bound.
      // Contrary to INFEASIBLE, infeasibility is not proven here.
      //
      // Warning: this return value is sensitive to the abs_eps_f and
      // rel_eps_f parameters. The upperbounding makes the optimizer only
      // looking for points less than min { (1-rel_eps_f)*obj_init_bound,
      // obj_init_bound - abs_eps_f }.
      result->set_solution_result(SolutionResult::kInfeasibleConstraints);
      break;
    case ibex::Optimizer::UNBOUNDED_OBJ:
      // If the objective function seems unbounded (tends to -oo).
      result->set_solution_result(SolutionResult::kUnbounded);
      break;
    case ibex::Optimizer::TIME_OUT:
      // Timeout.
      result->set_solution_result(SolutionResult::kUnknownError);
      break;
    case ibex::Optimizer::UNREACHED_PREC: {
      // If the search is over but the resulting interval [uplo,loup] does not
      // satisfy the precision requirements. There are several possible
      // reasons: the goal function may be too pessimistic or the constraints
      // function may be too pessimistic with respect to the precision
      // requirement (which can be too stringent). This results in tiny boxes
      // that can neither be contracted nor used as new loup
      // candidates. Finally, the eps_x parameter may be too large.
      result->set_solution_result(SolutionResult::kUnknownError);
      break;
    }
    case ibex::Optimizer::SUCCESS: {
      // If the global minimum (with respect to the precision required) has
      // been found.  In particular, at least one feasible point has been
      // found, less than obj_init_bound, and in the time limit.
      result->set_x_val(ExtractMidpoints(o.get_loup_point()));
      result->set_optimal_cost(o.get_uplo() / 2.0 +
                               o.get_loup() / 2.0 /* midpoint */);
      result->set_solution_result(SolutionResult::kSolutionFound);
      break;
    }
  }
}

}  // namespace

bool IbexSolver::is_available() { return true; }

void IbexSolver::DoSolve(const MathematicalProgram& prog,
                         const VectorXd& initial_guess,
                         const SolverOptions& merged_options,
                         MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
        "IbexSolver doesn't support the feature of variable scaling.");
  }

  unused(initial_guess);
  const IbexOptions options{ParseOptions(merged_options)};

  // Creates a converter. Internally, it creates ibex variables.
  internal::IbexConverter ibex_converter(prog.decision_variables());

  ibex::SystemFactory factory;

  // Prepares bounds for variables.
  ibex::IntervalVector bounds(prog.num_vars());
  for (const auto& b : prog.bounding_box_constraints()) {
    const auto& c = b.evaluator();
    const auto& lb = c->lower_bound();
    const auto& ub = c->upper_bound();

    for (int k = 0; k < static_cast<int>(b.GetNumElements()); ++k) {
      const int index = prog.FindDecisionVariableIndex(b.variables()[k]);
      bounds[index] &= ibex::Interval(lb[k], ub[k]);
    }
  }

  // Adds ibex variables + bounds into the factory.
  factory.add_var(ibex_converter.variables(), bounds);

  // This expr_ctrs holds the translated ibex::ExprCtr* objects. After adding
  // the ibex::Ctrs to the factory, we should not destruct them at the end of
  // the loop where they are created because they are used in the system factory
  // and the system.
  vector<internal::UniquePtrToExprCtr> expr_ctrs;

  // Add constraints.
  //
  // Note: Bounding-box constraints were already handled above when we
  // computed bounds for variables.
  for (const auto& b : prog.generic_constraints()) {
    AddFormula(b.evaluator()->CheckSatisfied(b.variables()), &ibex_converter,
               &factory, &expr_ctrs);
  }
  for (const auto& b : prog.linear_equality_constraints()) {
    AddFormula(b.evaluator()->CheckSatisfied(b.variables()), &ibex_converter,
               &factory, &expr_ctrs);
  }
  for (const auto& b : prog.linear_constraints()) {
    AddFormula(b.evaluator()->CheckSatisfied(b.variables()), &ibex_converter,
               &factory, &expr_ctrs);
  }
  for (const auto& b : prog.lorentz_cone_constraints()) {
    AddFormula(b.evaluator()->CheckSatisfied(b.variables()), &ibex_converter,
               &factory, &expr_ctrs);
  }
  for (const auto& b : prog.rotated_lorentz_cone_constraints()) {
    AddFormula(b.evaluator()->CheckSatisfied(b.variables()), &ibex_converter,
               &factory, &expr_ctrs);
  }
  for (const auto& b : prog.exponential_cone_constraints()) {
    AddFormula(b.evaluator()->CheckSatisfied(b.variables()), &ibex_converter,
               &factory, &expr_ctrs);
  }
  for (const auto& b : prog.linear_complementarity_constraints()) {
    AddFormula(b.evaluator()->CheckSatisfied(b.variables()), &ibex_converter,
               &factory, &expr_ctrs);
  }

  // Note: PositiveSemidefiniteConstraint and LinearMatrixInequalityConstraint
  // are not supported because their symbolic evaluation is not defined.
  DRAKE_DEMAND(prog.positive_semidefinite_constraints().empty());
  DRAKE_DEMAND(prog.linear_matrix_inequality_constraints().empty());

  // Extract costs.
  vector<Expression> costs;
  {
    VectorX<Expression> y;
    for (const auto& binding : prog.GetAllCosts()) {
      const VectorXDecisionVariable& x{binding.variables()};
      const auto& cost{binding.evaluator()};
      cost->Eval(x, &y);
      costs.push_back(y[0]);
    }
  }

  if (costs.empty()) {
    // A segmentation fault occurs when IBEX is given a problem with no cost
    // functions. We add a dummy `0` cost to prevent it.
    costs.push_back(Expression::Zero());
  }

  // Add costs into the factory.
  vector<internal::UniquePtrToExprNode> expr_nodes;
  for (const Expression& cost : costs) {
    internal::UniquePtrToExprNode expr_node{ibex_converter.Convert(cost)};
    factory.add_goal(*expr_node);
    // We need to postpone the destruction of expr_node as it is
    // still used inside of system_factory.
    expr_nodes.push_back(std::move(expr_node));
  }

  ibex::System sys(factory);
  DoOptimize(sys, options, result);
}

}  // namespace solvers
}  // namespace drake
