#include "drake/solvers/ipopt_solver.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <string>
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

// @param[in] options The options to copy into our task. It is mutable so that
// we can mutate it for efficiency; the data may be invalid afterwards, so the
// caller should not use it for anything after we return.
// @param[in,out] app The application to copy the options into.
void SetAppOptions(internal::SpecificOptions* options,
                   Ipopt::IpoptApplication* app) {
  // Wrap our calls to IPOPT to check for errors (i.e., unknown options).
  const auto set_double_option = [&app](const std::string& name, double value) {
    const bool success = app->Options()->SetNumericValue(name, value);
    if (!success) {
      throw std::logic_error(fmt::format(
          "Error setting IPOPT floating-point option {}={}", name, value));
    }
  };
  const auto set_int_option = [&app](const std::string& name, int value) {
    bool success{false};
    // Sometimes the option needs a double value, but the user sets the option
    // with an integer value. We will check the value type based on the
    // registered option name, and promote the value if necessary.
    auto reg_option = app->RegOptions()->GetOption(name);
    if (Ipopt::IsValid(reg_option)) {
      const auto option_type = reg_option->Type();
      if (option_type == Ipopt::RegisteredOptionType::OT_Number) {
        success =
            app->Options()->SetNumericValue(name, static_cast<double>(value));
      } else if (option_type == Ipopt::RegisteredOptionType::OT_Integer) {
        success = app->Options()->SetIntegerValue(name, value);
      }
    }
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

  set_string_option("hessian_approximation", "limited-memory");

  // Any user-supplied options handled below will overwrite the above
  // defaults.
  options->Respell([](const auto& common, auto* respelled) {
    // Note: 0 <= print_level <= 12, with higher numbers more verbose; 4 is
    // very useful for debugging. Otherwise, we default to printing nothing.
    // The user can always select an arbitrary print level, by setting the
    // ipopt-specific option name directly.
    const int verbose = 4;
    respelled->emplace("print_level", common.print_to_console ? verbose : 0);
    if (!common.print_file_name.empty()) {
      respelled->emplace("output_file", common.print_file_name);
      respelled->emplace("file_print_level", verbose);
    }
    // IPOPT does not support setting the number of threads so we ignore the
    // kMaxThreads option.
  });
  options->CopyToCallbacks(set_double_option, set_int_option,
                           set_string_option);
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
    case Ipopt::SolverReturn::WALLTIME_EXCEEDED: {
      return "Wall time exceeded";
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

IpoptSolver::IpoptSolver()
    : SolverBase(id(), &is_available, &is_enabled,
                 &ProgramAttributesSatisfied) {}

bool IpoptSolver::is_available() {
  return true;
}

void IpoptSolver::DoSolve2(const MathematicalProgram& prog,
                           const Eigen::VectorXd& initial_guess,
                           internal::SpecificOptions* options,
                           MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
        "IpoptSolver doesn't support the feature of variable scaling.");
  }

  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->RethrowNonIpoptException(true);

  SetAppOptions(options, &(*app));

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
