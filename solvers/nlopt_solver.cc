#include "drake/solvers/nlopt_solver.h"

#include <algorithm>
#include <limits>
#include <list>
#include <set>
#include <stdexcept>
#include <vector>

#include <nlopt.hpp>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/unused.h"
#include "drake/math/autodiff.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

namespace {
Eigen::VectorXd MakeEigenVector(const std::vector<double>& x) {
  Eigen::VectorXd xvec(x.size());
  for (size_t i = 0; i < x.size(); i++) {
    xvec[i] = x[i];
  }
  return xvec;
}

AutoDiffVecXd MakeInputAutoDiffVec(const MathematicalProgram& prog,
                               const Eigen::VectorXd& xvec,
                               const VectorXDecisionVariable& vars) {
  const int num_vars = vars.rows();

  auto tx = math::initializeAutoDiff(xvec);
  AutoDiffVecXd this_x(num_vars);

  for (int i = 0; i < num_vars; ++i) {
    this_x(i) = tx(prog.FindDecisionVariableIndex(vars(i)));
  }

  return this_x;
}

// This function meets the signature requirements for nlopt::vfunc as
// described in
// http://ab-initio.mit.edu/wiki/index.php/NLopt_C-plus-plus_Reference#Objective_function
// Note : NLopt uses the term "Objective" which corresponds to the Drake usage
// of "Cost".
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
double EvaluateCosts(const std::vector<double>& x, std::vector<double>& grad,
                     void* f_data) {
  const MathematicalProgram* prog =
      reinterpret_cast<const MathematicalProgram*>(f_data);

  double cost = 0;
  Eigen::VectorXd xvec = MakeEigenVector(x);

  auto tx = math::initializeAutoDiff(xvec);
  AutoDiffVecXd ty(1);
  AutoDiffVecXd this_x;

  if (!grad.empty()) {
    grad.assign(grad.size(), 0);
  }

  for (auto const& binding : prog->GetAllCosts()) {
    int num_vars = binding.GetNumElements();
    this_x.resize(num_vars);
    for (int i = 0; i < num_vars; ++i) {
      this_x(i) = tx(prog->FindDecisionVariableIndex(binding.variables()(i)));
    }

    binding.constraint()->Eval(this_x, ty);

    cost += ty(0).value();
    if (!grad.empty()) {
      for (int j = 0; j < num_vars; ++j) {
        size_t vj_index =
            prog->FindDecisionVariableIndex(binding.variables()(j));
        grad[vj_index] += ty(0).derivatives()(vj_index);
      }
    }
  }

  return cost;
}

/// Structure to marshall data into the NLopt callback functions,
/// which take only a single pointer argument.
struct WrappedConstraint {
  WrappedConstraint(const Constraint* constraint_in,
                    const VectorXDecisionVariable* vars_in,
                    const MathematicalProgram* prog_in)
      : constraint(constraint_in),
        vars(vars_in),
        prog(prog_in),
        force_bounds(false),
        force_upper(false) {}

  const Constraint* constraint;
  const VectorXDecisionVariable* vars;
  const MathematicalProgram* prog;
  bool force_bounds;  ///< force usage of only upper or lower bounds
  bool force_upper;   ///< Only used if force_bounds is set.  Selects
                      ///< which bounds are being tested (lower bound
                      ///< vs. upper bound).
  // TODO(sam.creasey) It might be desirable to have a cache for the
  // result of evaluating the constraints if NLopt were being used in
  // a situation where constraints were frequently being wrapped in
  // such a way as to result in multiple evaluations.  As this is a
  // speculative case, and since NLopt's roundoff issues with
  // duplicate constraints preclude it from being used in some
  // scenarios, I'm not implementing such a cache at this time.
  std::set<size_t> active_constraints;
};

double ApplyConstraintBounds(double result, double lb, double ub) {
  // Our (Drake's) constraints are expressed in the form lb <= f(x) <=
  // ub.  NLopt always wants the value of a constraint expressed as
  // f(x) <= 0.
  //
  // For upper bounds rewrite as: f(x) - ub <= 0
  // For lower bounds rewrite as: -f(x) + lb <= 0
  //
  // If both upper and lower bounds are set, default to evaluating the
  // upper bound, and switch to the lower bound only if it's being
  // exceeded.
  //
  // See
  // http://ab-initio.mit.edu/wiki/index.php/NLopt_Reference#Nonlinear_constraints
  // for more detail on how NLopt interprets return values.

  if ((ub != std::numeric_limits<double>::infinity()) &&
      ((result >= lb) || (lb == ub))) {
    result -= ub;
  } else {
    if (lb == -std::numeric_limits<double>::infinity()) {
      throw std::runtime_error("Unable to handle constraint with no bounds.");
    }
    result *= -1;
    result += lb;
  }
  return result;
}

// This function meets the signature of nlopt_mfunc as described in
// http://ab-initio.mit.edu/wiki/index.php/NLopt_Reference#Vector-valued_constraints
void EvaluateVectorConstraint(unsigned m, double* result, unsigned n,
                              const double* x, double* grad, void* f_data) {
  const WrappedConstraint* wrapped =
      reinterpret_cast<WrappedConstraint*>(f_data);

  Eigen::VectorXd xvec(n);
  for (size_t i = 0; i < n; i++) {
    xvec[i] = x[i];
  }

  // http://ab-initio.mit.edu/wiki/index.php/NLopt_Reference#Vector-valued_constraints
  // explicity tells us that it's allocated m * n array elements
  // before invoking this function.  It does not seem to have been
  // zeroed, and not all constraints will store gradients for all
  // decision variables (so don't leave junk in the other array
  // elements).
  if (grad) {
    memset(grad, 0, sizeof(double) * m * n);
  }

  const Constraint* c = wrapped->constraint;
  const size_t num_constraints = c->num_constraints();
  DRAKE_ASSERT(num_constraints >= m);
  DRAKE_ASSERT(wrapped->active_constraints.size() == m);

  AutoDiffVecXd ty(num_constraints);
  AutoDiffVecXd this_x =
      MakeInputAutoDiffVec(*(wrapped->prog), xvec, *(wrapped->vars));
  c->Eval(this_x, ty);

  const Eigen::VectorXd& lower_bound = c->lower_bound();
  const Eigen::VectorXd& upper_bound = c->upper_bound();
  size_t result_idx = 0;
  for (size_t i = 0; i < num_constraints; i++) {
    if (!wrapped->active_constraints.count(i)) {
      continue;
    }
    if (wrapped->force_bounds && wrapped->force_upper &&
        (upper_bound(i) != std::numeric_limits<double>::infinity())) {
      result[result_idx] = ApplyConstraintBounds(
          ty(i).value(), -std::numeric_limits<double>::infinity(),
          upper_bound(i));
    } else if (wrapped->force_bounds && !wrapped->force_upper &&
               (lower_bound(i) != -std::numeric_limits<double>::infinity())) {
      result[result_idx] =
          ApplyConstraintBounds(ty(i).value(), lower_bound(i),
                                std::numeric_limits<double>::infinity());
    } else {
      result[result_idx] =
          ApplyConstraintBounds(ty(i).value(), lower_bound(i), upper_bound(i));
    }
    result_idx++;
    DRAKE_ASSERT(result_idx <= m);
  }

  if (grad) {
    result_idx = 0;
    const int num_v_variable = wrapped->vars->rows();
    std::vector<size_t> v_index(num_v_variable);
    for (int i = 0; i < num_v_variable; ++i) {
      v_index[i] =
          wrapped->prog->FindDecisionVariableIndex((*wrapped->vars)(i));
    }
    for (size_t i = 0; i < num_constraints; i++) {
      if (!wrapped->active_constraints.count(i)) {
        continue;
      }
      double grad_sign = 1;
      if (c->upper_bound()(i) == std::numeric_limits<double>::infinity()) {
        grad_sign = -1;
      } else if (wrapped->force_bounds && !wrapped->force_upper) {
        grad_sign = -1;
      }
      DRAKE_ASSERT(wrapped->vars->cols() == 1);
      for (int j = 0; j < wrapped->vars->rows(); ++j) {
        grad[(result_idx * n) + v_index[j]] =
            ty(i).derivatives()(v_index[j]) * grad_sign;
      }
      result_idx++;
      DRAKE_ASSERT(result_idx <= m);
    }
    DRAKE_ASSERT(result_idx == m);
  }
}

template <typename C>
void WrapConstraint(const MathematicalProgram& prog, const Binding<C>& binding,
                    double constraint_tol, nlopt::opt* opt,
                    std::list<WrappedConstraint>* wrapped_list) {
  // Version of the wrapped constraint which refers only to equality
  // constraints (if any), and will be used with
  // add_equality_mconstraint.
  WrappedConstraint wrapped_eq(binding.constraint().get(), &binding.variables(),
                               &prog);

  // Version of the wrapped constraint which refers only to inequality
  // constraints (if any), and will be used with
  // add_equality_mconstraint.
  WrappedConstraint wrapped_in(binding.constraint().get(), &binding.variables(),
                               &prog);

  bool is_pure_inequality = true;
  const Eigen::VectorXd& lower_bound = binding.constraint()->lower_bound();
  const Eigen::VectorXd& upper_bound = binding.constraint()->upper_bound();
  DRAKE_ASSERT(lower_bound.size() == upper_bound.size());
  for (size_t i = 0; i < static_cast<size_t>(lower_bound.size()); i++) {
    if (lower_bound(i) == upper_bound(i)) {
      wrapped_eq.active_constraints.insert(i);
    } else {
      if ((lower_bound(i) != -std::numeric_limits<double>::infinity()) &&
          (upper_bound(i) != std::numeric_limits<double>::infinity())) {
        is_pure_inequality = false;
      }
      wrapped_in.active_constraints.insert(i);
    }
  }

  if (wrapped_eq.active_constraints.size()) {
    wrapped_list->push_back(wrapped_eq);
    std::vector<double> tol(wrapped_eq.active_constraints.size(),
                            constraint_tol);
    opt->add_equality_mconstraint(EvaluateVectorConstraint,
                                  &wrapped_list->back(), tol);
  }

  if (wrapped_in.active_constraints.size()) {
    std::vector<double> tol(wrapped_in.active_constraints.size(),
                            constraint_tol);
    wrapped_list->push_back(wrapped_in);
    if (is_pure_inequality) {
      opt->add_inequality_mconstraint(EvaluateVectorConstraint,
                                      &wrapped_list->back(), tol);
    } else {
      wrapped_list->back().force_bounds = true;
      wrapped_list->back().force_upper = true;
      opt->add_inequality_mconstraint(EvaluateVectorConstraint,
                                      &wrapped_list->back(), tol);

      wrapped_list->push_back(wrapped_in);
      wrapped_list->back().force_bounds = true;
      wrapped_list->back().force_upper = false;
      opt->add_inequality_mconstraint(EvaluateVectorConstraint,
                                      &wrapped_list->back(), tol);
    }
  }
}

}  // anonymous namespace

bool NloptSolver::available() const { return true; }

SolutionResult NloptSolver::Solve(MathematicalProgram& prog) const {
  const int nx = prog.num_vars();

  // Load the algo to use and the size.
  nlopt::opt opt(nlopt::LD_SLSQP, nx);

  const Eigen::VectorXd& initial_guess = prog.initial_guess();
  std::vector<double> x(initial_guess.size());
  for (size_t i = 0; i < x.size(); i++) {
    if (!std::isnan(initial_guess[i])) {
      x[i] = initial_guess[i];
    } else {
      x[i] = 0.0;
    }
  }

  std::vector<double> xlow(nx, -std::numeric_limits<double>::infinity());
  std::vector<double> xupp(nx, std::numeric_limits<double>::infinity());

  for (auto const& binding : prog.bounding_box_constraints()) {
    const auto& c = binding.constraint();
    const auto& lower_bound = c->lower_bound();
    const auto& upper_bound = c->upper_bound();

    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      const size_t idx = prog.FindDecisionVariableIndex(binding.variables()(k));
      xlow[idx] = std::max(lower_bound(k), xlow[idx]);
      xupp[idx] = std::min(upper_bound(k), xupp[idx]);
      if (x[idx] < xlow[idx]) {
        x[idx] = xlow[idx];
      }
      if (x[idx] > xupp[idx]) {
        x[idx] = xupp[idx];
      }
    }
  }

  opt.set_lower_bounds(xlow);
  opt.set_upper_bounds(xupp);

  opt.set_min_objective(EvaluateCosts, &prog);

  // TODO(sam.creasey): All hardcoded tolerances in this function
  // should be made configurable when #1879 is fixed.
  const double constraint_tol = 1e-6;
  const double xtol_rel = 1e-6;
  const double xtol_abs = 1e-6;

  std::list<WrappedConstraint> wrapped_vector;

  // TODO(sam.creasey): Missing test coverage for generic constraints
  // with >1 output.
  for (const auto& c : prog.generic_constraints()) {
    WrapConstraint(prog, c, constraint_tol, &opt, &wrapped_vector);
  }

  for (const auto& c : prog.lorentz_cone_constraints()) {
    WrapConstraint(prog, c, constraint_tol, &opt, &wrapped_vector);
  }

  for (const auto& c : prog.rotated_lorentz_cone_constraints()) {
    WrapConstraint(prog, c, constraint_tol, &opt, &wrapped_vector);
  }

  for (const auto& c : prog.linear_equality_constraints()) {
    WrapConstraint(prog, c, constraint_tol, &opt, &wrapped_vector);
  }

  // TODO(sam.creasey): Missing test coverage for linear constraints
  // with >1 output.
  for (const auto& c : prog.linear_constraints()) {
    WrapConstraint(prog, c, constraint_tol, &opt, &wrapped_vector);
  }

  opt.set_xtol_rel(xtol_rel);
  opt.set_xtol_abs(xtol_abs);

  SolutionResult result = SolutionResult::kSolutionFound;

  double minf = 0;
  try {
    const nlopt::result nlopt_result = opt.optimize(x, minf);
    unused(nlopt_result);
  } catch (std::invalid_argument&) {
    result = SolutionResult::kInvalidInput;
  } catch (std::bad_alloc&) {
    result = SolutionResult::kUnknownError;
  } catch (nlopt::roundoff_limited) {
    result = SolutionResult::kUnknownError;
  } catch (nlopt::forced_stop) {
    result = SolutionResult::kUnknownError;
  } catch (std::runtime_error&) {
    result = SolutionResult::kUnknownError;
  }

  Eigen::VectorXd sol(x.size());
  for (int i = 0; i < nx; i++) {
    sol(i) = x[i];
  }

  prog.SetDecisionVariableValues(sol);
  prog.SetOptimalCost(minf);
  prog.SetSolverId(id());
  return result;
}

}  // namespace solvers
}  // namespace drake
