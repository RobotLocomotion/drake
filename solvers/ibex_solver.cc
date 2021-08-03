#include "drake/solvers/ibex_solver.h"

#include <memory>
#include <utility>
#include <vector>

#include "./ibex.h"

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

// Adds `f` into `factor` using `ibex_converter` and `expr_ctrs`.
void AddFormula(const Formula& f, internal::IbexConverter* const ibex_converter,
                ibex::SystemFactory* const factory,
                vector<internal::UniquePtrToExprCtr>* const expr_ctrs) {
  internal::UniquePtrToExprCtr expr_ctr{ibex_converter->Convert(f)};
  factory->add_ctr(*expr_ctr);
  expr_ctrs->push_back(std::move(expr_ctr));
}

VectorXd ExtractMidpoints(const ibex::IntervalVector& iv) {
  VectorXd v{iv.size()};
  for (int i = 0; i < iv.size(); ++i) {
    v[i] = iv[i].mid();
  }
  return v;
}

void DoOptimize(const ibex::System& sys,
                MathematicalProgramResult* const result) {
  // TODO(soonho): Make the followings as solver options.
  const double rel_eps_f = ibex::Optimizer::default_rel_eps_f;
  const double abs_eps_f = ibex::Optimizer::default_abs_eps_f;
  const double eps_h = ibex::NormalizedSystem::default_eps_h;
  const bool rigor = false;
  const bool in_hc4 = !(sys.nb_ctr > 0 && sys.nb_ctr < sys.f_ctrs.image_dim());
  const double random_seed = ibex::DefaultOptimizer::default_random_seed;
  const double eps_x = ibex::Optimizer::default_eps_x;
  const double timeout = 30.0; /* sec */
  // - 0 (by default): nothing is printed
  // - 1:              prints every loup/uplo update.
  // - 2:              prints also each handled node
  const int trace = 0;

  ibex::DefaultOptimizer o(sys, rel_eps_f, abs_eps_f, eps_h, rigor, in_hc4,
                           random_seed, eps_x);
  o.trace = trace;
  o.timeout = timeout;
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
  unused(merged_options);

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
  for (const auto& b : prog.GetAllConstraints()) {
    Formula f{b.evaluator()->CheckSatisfied(b.variables())};
    if (symbolic::is_conjunction(f)) {
      for (const auto& f_i : symbolic::get_operands(f)) {
        AddFormula(f_i, &ibex_converter, &factory, &expr_ctrs);
      }
    } else {
      AddFormula(f, &ibex_converter, &factory, &expr_ctrs);
    }
  }

  // Extract costs
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
    // IbexOpt crashes when there is no cost functions. We add a dummy `0` cost
    // to prevent it.
    costs.push_back(Expression::Zero());
  }

  // Add costs into the factory.
  vector<internal::UniquePtrToExprNode> expr_nodes;
  for (const Expression& cost : costs) {
    internal::UniquePtrToExprNode expr_node{ibex_converter.Convert(cost)};
    factory.add_goal(*expr_node);
    // We need to postpone the destruction of expr_ctr as it is
    // still used inside of system_factory.
    expr_nodes.push_back(std::move(expr_node));
  }

  ibex::System sys(factory);
  DoOptimize(sys, result);
}

}  // namespace solvers
}  // namespace drake
