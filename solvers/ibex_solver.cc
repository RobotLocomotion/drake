#include "drake/solvers/ibex_solver.h"

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

#include "./ibex.h"

#include "drake/common/text_logging.h"
#include "drake/solvers/ibex_converter.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

using Eigen::VectorXd;

using std::shared_ptr;
using std::vector;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

namespace {
// Custom deleter for ibex::ExprCtr. It deletes the internal
// ibex::ExprNode while keeping the ExprSymbols intact. Note that the
// ExprSymbols will be deleted separately in
// ~ContractorIbexPolytope().
struct ExprCtrDeleter {
  void operator()(const ibex::ExprCtr* const p) const {
    if (p) {
      ibex::cleanup(p->e, false);
      delete p;
    }
  }
};

// Extracts linear costs in @p prog and push them into @p costs vector.
void ExtractLinearCosts(const MathematicalProgram& prog,
                        vector<Expression>* const costs) {
  for (const Binding<LinearCost>& binding : prog.linear_costs()) {
    const VectorXDecisionVariable& x{binding.variables()};
    const shared_ptr<LinearCost>& cost{binding.evaluator()};
    VectorX<Expression> y;
    cost->Eval(x, &y);
    costs->push_back(y[0]);
  }
}

// Extracts quadratic costs in @p prog and push them into @p costs vector.
void ExtractQuadraticCosts(const MathematicalProgram& prog,
                           vector<Expression>* const costs) {
  for (const Binding<QuadraticCost>& binding : prog.quadratic_costs()) {
    const VectorXDecisionVariable& x{binding.variables()};
    const shared_ptr<QuadraticCost>& cost{binding.evaluator()};
    VectorX<Expression> y;
    cost->Eval(x, &y);
    costs->push_back(y[0]);
  }
}

// Adds @p f into @p factor using @p ibex_converter and @p expr_ctrs.
void AddFormula(const Formula& f, internal::IbexConverter* const ibex_converter,
                ibex::SystemFactory* const factory,
                std::vector<internal::UniquePtrToExprCtr>* const expr_ctrs) {
  internal::UniquePtrToExprCtr expr_ctr{ibex_converter->Convert(f)};
  if (expr_ctr) {
    factory->add_ctr(*expr_ctr);
    expr_ctrs->push_back(std::move(expr_ctr));
  }
}

Eigen::VectorXd ExtractMidpoints(const ibex::IntervalVector& iv) {
  Eigen::VectorXd v{iv.size()};
  for (int i = 0; i < iv.size(); ++i) {
    v[i] = iv[i].mid();
  }
  return v;
}

void DoOptimize(const ibex::System& sys,
                MathematicalProgramResult* const result) {
  const double rel_eps_f = ibex::Optimizer::default_rel_eps_f;
  const double abs_eps_f = ibex::Optimizer::default_abs_eps_f;
  const double eps_h = ibex::NormalizedSystem::default_eps_h;
  const bool rigor = false;
  const bool inHC4 = !(sys.nb_ctr > 0 && sys.nb_ctr < sys.f_ctrs.image_dim());
  const double random_seed = ibex::DefaultOptimizer::default_random_seed;
  const double eps_x = ibex::Optimizer::default_eps_x;
  const double timeout = 30.0; /* sec */
  const bool trace = false;

  ibex::DefaultOptimizer o(sys, rel_eps_f, abs_eps_f, eps_h, rigor, inHC4,
                           random_seed, eps_x);
  o.trace = trace;
  o.timeout = timeout;
  const ibex::Optimizer::Status status = o.optimize(sys.box);

  switch (status) {
    case ibex::Optimizer::INFEASIBLE:
      // if no feasible point exist less than obj_init_bound. In particular,
      // the function returns INFEASIBLE if the initial bound "obj_init_bound"
      // is LESS than the true minimum (this case is only possible if
      // goal_abs_prec and goal_rel_prec are 0). In the latter case, there may
      // exist feasible points.
      result->set_solution_result(SolutionResult::kInfeasibleConstraints);
      break;
    case ibex::Optimizer::NO_FEASIBLE_FOUND:
      // if no feasible point could be found less than obj_init_bound.
      // Contrary to INFEASIBLE, infeasibility is not proven here.
      //
      // Warning: this return value is sensitive to the abs_eps_f and
      // rel_eps_f parameters. The upperbounding makes the optimizer only
      // looking for points less than min { (1-rel_eps_f)*obj_init_bound,
      // obj_init_bound - abs_eps_f }.
      result->set_solution_result(SolutionResult::kInfeasibleConstraints);
      break;
    case ibex::Optimizer::UNBOUNDED_OBJ:
      // if the objective function seems unbounded (tends to -oo).
      result->set_solution_result(SolutionResult::kUnbounded);
      break;
    case ibex::Optimizer::TIME_OUT:
      // Timeout.
      result->set_solution_result(SolutionResult::kUnknownError);
      break;
    case ibex::Optimizer::UNREACHED_PREC: {
      // if the search is over but the resulting interval [uplo,loup] does not
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
      // if the global minimum (with respect to the precision required) has
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
                         const Eigen::VectorXd& initial_guess,
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
      const size_t index = prog.FindDecisionVariableIndex(b.variables()[k]);
      bounds[index] &= ibex::Interval(lb[k], ub[k]);
    }
  }

  // Adds ibex variables + bounds into the factory.
  factory.add_var(ibex_converter.variables(), bounds);

  // This expr_ctrs holds the translated ibex::ExprCtr* objects. After adding
  // the ibex::Ctrs to the factory, we should not destruct them at the end of
  // the loop where they are created because they are used in the system factory
  // and the system.
  std::vector<internal::UniquePtrToExprCtr> expr_ctrs;

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
  ExtractLinearCosts(prog, &costs);
  ExtractQuadraticCosts(prog, &costs);

  // Add costs into the factory.
  std::vector<internal::UniquePtrToExprNode> expr_nodes;
  for (const Expression& cost : costs) {
    internal::UniquePtrToExprNode expr_node{ibex_converter.Convert(cost)};
    factory.add_goal(*expr_node);
    // We need to postpone the destruction of expr_ctr as it is
    // still used inside of system_factory.
    expr_nodes.push_back(std::move(expr_node));
  }

  // IbexOpt crashes when there is no cost functions. We add a dummy `0` cost
  // here to prevent it.
  if (costs.empty()) {
    internal::UniquePtrToExprNode expr_node{
        ibex_converter.Convert(Expression::Zero())};
    factory.add_goal(*expr_node);
    expr_nodes.push_back(std::move(expr_node));
  }

  ibex::System sys(factory);
  DoOptimize(sys, result);
}

}  // namespace solvers
}  // namespace drake
