#include "drake/solvers/ipopt_solver.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/math/autodiff.h"
#include "drake/solvers/ipopt_solver_internal.h"
#include "drake/solvers/mathematical_program.h"

using Ipopt::SolverReturn;

namespace drake {
namespace solvers {
namespace {

void SetAppOptions(const SolverOptions& options, Ipopt::IpoptApplication* app) {
  // Wrap our calls to IPOPT to check for errors (i.e., unknown options).
  const auto set_double_option = [&app](const std::string& name, double value) {
    const bool success = app->Options()->SetNumericValue(name, value);
    if (!success) {
      throw std::logic_error(fmt::format(
          "Error setting IPOPT floating-point option {}={}", name, value));
    }
  };
  const auto set_int_option = [&app](const std::string& name, int value) {
    const bool success = app->Options()->SetIntegerValue(name, value);
    if (!success) {
      throw std::logic_error(
          fmt::format("Error setting IPOPT integer option {}={}", name, value));
    }
  };
  const auto set_string_option = [&app](const std::string& name,
                                        const std::string& value) {
    const bool success = app->Options()->SetStringValue(name, value);
    if (!success) {
      throw std::logic_error(fmt::format(
          "Error setting IPOPT string option {}='{}'", name, value));
    }
  };

  // Turn off the banner.
  set_string_option("sb", "yes");

  // The default linear solver is MA27, but it is not freely redistributable so
  // we cannot use it. MUMPS is the only compatible linear solver guaranteed to
  // be available on both macOS and Ubuntu. In versions of IPOPT prior to 3.13,
  // it would correctly determine that MUMPS was the only available solver, but
  // its behavior changed to instead error having unsuccessfully tried to dlopen
  // a nonexistent hsl library that would contain MA27.
  set_string_option("linear_solver", "mumps");

  // The default tolerance.
  const double tol = 1.05e-10;  // Note: SNOPT is only 1e-6, but in #3712 we
  // diagnosed that the CompareMatrices tolerance needed to be the sqrt of the
  // constr_viol_tol
  set_double_option("tol", tol);
  set_double_option("constr_viol_tol", tol);
  set_double_option("acceptable_tol", tol);
  set_double_option("acceptable_constr_viol_tol", tol);
  set_string_option("hessian_approximation", "limited-memory");

  // Note: 0 <= print_level <= 12, with higher numbers more verbose; 4 is very
  // useful for debugging. Otherwise, we default to printing nothing. The user
  // can always select an arbitrary print level, by setting the ipopt-specific
  // option name directly.
  const int verbose_level = 4;
  const int print_level = options.get_print_to_console() ? verbose_level : 0;
  set_int_option("print_level", print_level);

  const std::string& output_file = options.get_print_file_name();
  if (!output_file.empty()) {
    set_int_option("file_print_level", verbose_level);
    set_string_option("output_file", output_file);
  }

  // IPOPT does not support setting the number of threads so we ignore
  // the kMaxNumThreads option.

  // The solver-specific options will trump our defaults.
  const SolverId self = IpoptSolver::id();
  for (const auto& [name, value] : options.GetOptionsDouble(self)) {
    set_double_option(name, value);
  }
  for (const auto& [name, value] : options.GetOptionsInt(self)) {
    set_int_option(name, value);
  }
  for (const auto& [name, value] : options.GetOptionsStr(self)) {
    set_string_option(name, value);
  }
}

}  // namespace

const char* IpoptSolverDetails::ConvertStatusToString() const {
  switch (status) {
    case Ipopt::SolverReturn::SUCCESS: {
      return "Success";
    }
    case Ipopt::SolverReturn::MAXITER_EXCEEDED: {
      return "Max iteration exceeded";
    }
    case Ipopt::SolverReturn::CPUTIME_EXCEEDED: {
      return "CPU time exceeded";
    }
    case Ipopt::SolverReturn::STOP_AT_TINY_STEP: {
      return "Stop at tiny step";
    }
    case Ipopt::SolverReturn::STOP_AT_ACCEPTABLE_POINT: {
      return "Stop at acceptable point";
    }
    case Ipopt::SolverReturn::LOCAL_INFEASIBILITY: {
      return "Local infeasibility";
    }
    case Ipopt::SolverReturn::USER_REQUESTED_STOP: {
      return "User requested stop";
    }
    case Ipopt::SolverReturn::FEASIBLE_POINT_FOUND: {
      return "Feasible point found";
    }
    case Ipopt::SolverReturn::DIVERGING_ITERATES: {
      return "Divergent iterates";
    }
    case Ipopt::SolverReturn::RESTORATION_FAILURE: {
      return "Restoration failure";
    }
    case Ipopt::SolverReturn::ERROR_IN_STEP_COMPUTATION: {
      return "Error in step computation";
    }
    case Ipopt::SolverReturn::INVALID_NUMBER_DETECTED: {
      return "Invalid number detected";
    }
    case Ipopt::SolverReturn::TOO_FEW_DEGREES_OF_FREEDOM: {
      return "Too few degrees of freedom";
    }
    case Ipopt::SolverReturn::INVALID_OPTION: {
      return "Invalid option";
    }
    case Ipopt::SolverReturn::OUT_OF_MEMORY: {
      return "Out of memory";
    }
    case Ipopt::SolverReturn::INTERNAL_ERROR: {
      return "Internal error";
    }
    case Ipopt::SolverReturn::UNASSIGNED: {
      return "Unassigned";
    }
  }
  return "Unknown enumerated SolverReturn value.";
}

bool IpoptSolver::is_available() {
  return true;
}

void IpoptSolver::DoSolve(const MathematicalProgram& prog,
                          const Eigen::VectorXd& initial_guess,
                          const SolverOptions& merged_options,
                          MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
        "IpoptSolver doesn't support the feature of variable scaling.");
  }

  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->RethrowNonIpoptException(true);

  SetAppOptions(merged_options, &(*app));

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    result->set_solution_result(SolutionResult::kInvalidInput);
    return;
  }

  Ipopt::SmartPtr<internal::IpoptSolver_NLP> nlp =
      new internal::IpoptSolver_NLP(prog, initial_guess, result);
  status = app->OptimizeTNLP(nlp);
  // Set result.solver_details.
  IpoptSolverDetails& solver_details =
      result->SetSolverDetailsType<IpoptSolverDetails>();
  solver_details.status = nlp->status();
  solver_details.z_L = nlp->z_L();
  solver_details.z_U = nlp->z_U();
  solver_details.g = nlp->g();
  solver_details.lambda = nlp->lambda();
}

}  // namespace solvers
}  // namespace drake
