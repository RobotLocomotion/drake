#include "drake/solvers/mosek_solver.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <list>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <fmt/format.h>
#include <mosek.h>

#include "drake/common/scoped_singleton.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver_internal.h"

namespace drake {
namespace solvers {
namespace {
// This function is used to print information for each iteration to the console,
// it will show PRSTATUS, PFEAS, DFEAS, etc. For more information, check out
// https://docs.mosek.com/10.0/capi/solver-io.html. This printstr is copied
// directly from https://docs.mosek.com/10.0/capi/solver-io.html#stream-logging.
void MSKAPI printstr(void*, const char str[]) { printf("%s", str); }

}  // anonymous namespace

/*
 * Implements RAII for a Mosek license / environment.
 */
class MosekSolver::License {
 public:
  License() {
    if (!MosekSolver::is_enabled()) {
      throw std::runtime_error(
          "Could not locate MOSEK license file because MOSEKLM_LICENSE_FILE "
          "environment variable was not set.");
    }

    MSKrescodee rescode = MSK_makeenv(&mosek_env_, nullptr);
    if (rescode != MSK_RES_OK) {
      throw std::runtime_error("Could not create MOSEK environment.");
    }
    DRAKE_DEMAND(mosek_env_ != nullptr);

    const int num_tries = 3;
    rescode = MSK_RES_TRM_INTERNAL;
    for (int i = 0; i < num_tries && rescode != MSK_RES_OK; ++i) {
      // Acquire the license for the base MOSEK system so that we can
      // fail fast if the license file is missing or the server is
      // unavailable. Any additional features should be checked out
      // later by MSK_optimizetrm if needed (so there's still the
      // possibility of later failure at that stage if the desired
      // feature is unavailable or another error occurs).
      rescode = MSK_checkoutlicense(mosek_env_, MSK_FEATURE_PTS);
    }

    if (rescode != MSK_RES_OK) {
      throw std::runtime_error("Could not acquire MOSEK license.");
    }
  }

  ~License() {
    MSK_deleteenv(&mosek_env_);
    mosek_env_ = nullptr;  // Fail-fast if accidentally used after destruction.
  }

  MSKenv_t mosek_env() const { return mosek_env_; }

 private:
  MSKenv_t mosek_env_{nullptr};
};

std::shared_ptr<MosekSolver::License> MosekSolver::AcquireLicense() {
  // According to
  // https://docs.mosek.com/8.1/cxxfusion/solving-parallel.html sharing
  // an env used between threads is safe (not mentioned in 10.0 documentation),
  // but nothing mentions thread-safety when allocating the environment. We can
  // safeguard against this ambiguity by using GetScopedSingleton for basic
  // thread-safety when acquiring / releasing the license.
  return GetScopedSingleton<MosekSolver::License>();
}

bool MosekSolver::is_available() { return true; }

void MosekSolver::DoSolve(const MathematicalProgram& prog,
                          const Eigen::VectorXd& initial_guess,
                          const SolverOptions& merged_options,
                          MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
        "MosekSolver doesn't support the feature of variable scaling.");
  }

  // num_decision_vars is the total number of decision variables in @p prog. It
  // includes both the matrix variables (for psd matrix variables) and
  // non-matrix variables.
  const int num_decision_vars = prog.num_vars();
  MSKrescodee rescode{MSK_RES_OK};

  if (!license_) {
    license_ = AcquireLicense();
  }
  MSKenv_t env = license_->mosek_env();

  internal::MosekSolverProgram mosek_solver_prog(prog, env);
  // The number of non-matrix variables in @p prog.
  const int num_nonmatrix_vars_in_prog =
      mosek_solver_prog.decision_variable_index_to_mosek_nonmatrix_variable()
          .size();

  // Set the options (parameters).
  std::optional<std::string> msk_writedata;
  if (rescode == MSK_RES_OK) {
    rescode =
        mosek_solver_prog.UpdateOptions(merged_options, id(), &msk_writedata);
  }

  // Always check if rescode is MSK_RES_OK before we call any Mosek functions.
  // If it is not MSK_RES_OK, then bypasses everything and exits.
  if (rescode == MSK_RES_OK) {
    rescode =
        MSK_appendvars(mosek_solver_prog.task(), num_nonmatrix_vars_in_prog);
  }
  // Add positive semidefinite constraint. This also adds Mosek matrix
  // variables.
  // psd_barvar_indices records the index of the bar matrix for this positive
  // semidefinite constraint. We will use this bar matrix index later, for
  // example when retrieving the dual solution.
  std::unordered_map<Binding<PositiveSemidefiniteConstraint>, MSKint32t>
      psd_barvar_indices;
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddPositiveSemidefiniteConstraints(
        prog, &psd_barvar_indices);
  }
  // Add the constraint that Mosek matrix variable entries corresponding to the
  // same decision variables should all be equal.
  if (rescode == MSK_RES_OK) {
    rescode =
        mosek_solver_prog
            .AddEqualityConstraintBetweenMatrixVariablesForSameDecisionVariable();  // NOLINT
  }
  // Add costs
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddCosts(prog);
  }

  // We store the dual variable indices for each bounding box constraint.
  // bb_con_dual_indices[constraint] returns the pair (lower_bound_indices,
  // upper_bound_indices), where lower_bound_indices are the indices of the
  // lower bound dual variables, and upper_bound_indices are the indices of the
  // upper bound dual variables.
  std::unordered_map<Binding<BoundingBoxConstraint>,
                     std::pair<internal::ConstraintDualIndices,
                               internal::ConstraintDualIndices>>
      bb_con_dual_indices;
  // Add bounding box constraints on decision variables.
  if (rescode == MSK_RES_OK) {
    rescode =
        mosek_solver_prog.AddBoundingBoxConstraints(prog, &bb_con_dual_indices);
  }
  // Specify binary variables.
  bool with_integer_or_binary_variable = false;
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.SpecifyVariableType(
        prog, &with_integer_or_binary_variable);
  }
  // Add linear constraints.
  std::unordered_map<Binding<LinearConstraint>, internal::ConstraintDualIndices>
      linear_con_dual_indices;
  std::unordered_map<Binding<LinearEqualityConstraint>,
                     internal::ConstraintDualIndices>
      lin_eq_con_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddLinearConstraints(
        prog, &linear_con_dual_indices, &lin_eq_con_dual_indices);
  }

  // Add Lorentz cone constraints.
  std::unordered_map<Binding<LorentzConeConstraint>,
                     internal::ConstraintDualIndices>
      lorentz_cone_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddConeConstraints(
        prog, prog.lorentz_cone_constraints(), &lorentz_cone_dual_indices);
  }

  // Add rotated Lorentz cone constraints.
  std::unordered_map<Binding<RotatedLorentzConeConstraint>,
                     internal::ConstraintDualIndices>
      rotated_lorentz_cone_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddConeConstraints(
        prog, prog.rotated_lorentz_cone_constraints(),
        &rotated_lorentz_cone_dual_indices);
  }

  // Add linear matrix inequality constraints.
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddLinearMatrixInequalityConstraint(prog);
  }

  // Add exponential cone constraints.
  std::unordered_map<Binding<ExponentialConeConstraint>,
                     internal::ConstraintDualIndices>
      exp_cone_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddConeConstraints(
        prog, prog.exponential_cone_constraints(), &exp_cone_dual_indices);
  }

  // log file.
  const bool print_to_console = merged_options.get_print_to_console();
  const std::string print_file_name = merged_options.get_print_file_name();
  // Refer to https://docs.mosek.com/10.0/capi/solver-io.html#stream-logging
  // for Mosek stream logging.
  // First we check if the user wants to print to both the console and the file.
  // If true, throw an error BEFORE we create the log file through
  // MSK_linkfiletotaskstream. Otherwise we might create the log file but cannot
  // close it.
  if (print_to_console && !print_file_name.empty()) {
    throw std::runtime_error(
        "MosekSolver::Solve(): cannot print to both the console and the log "
        "file.");
  } else if (print_to_console) {
    if (rescode == MSK_RES_OK) {
      rescode = MSK_linkfunctotaskstream(mosek_solver_prog.task(),
                                         MSK_STREAM_LOG, nullptr, printstr);
    }
  } else if (!print_file_name.empty()) {
    if (rescode == MSK_RES_OK) {
      rescode = MSK_linkfiletotaskstream(
          mosek_solver_prog.task(), MSK_STREAM_LOG, print_file_name.c_str(), 0);
    }
  }

  // Determines the solution type.
  MSKsoltypee solution_type;
  if (with_integer_or_binary_variable) {
    solution_type = MSK_SOL_ITG;
  } else if (prog.quadratic_costs().empty() &&
             prog.lorentz_cone_constraints().empty() &&
             prog.rotated_lorentz_cone_constraints().empty() &&
             prog.positive_semidefinite_constraints().empty() &&
             prog.linear_matrix_inequality_constraints().empty() &&
             prog.exponential_cone_constraints().empty()) {
    solution_type = MSK_SOL_BAS;
  } else {
    solution_type = MSK_SOL_ITR;
  }

  // Since Mosek 10, it allows setting the initial guess for both continuous and
  // integer/binary variables. See
  // https://docs.mosek.com/latest/rmosek/tutorial-mio-shared.html#specifying-an-initial-solution
  // for more details.
  const bool has_any_finite_initial_guess =
      initial_guess.unaryExpr([](double g) { return std::isfinite(g); }).any();
  if (has_any_finite_initial_guess) {
    DRAKE_ASSERT(initial_guess.size() == prog.num_vars());
    for (int i = 0; i < prog.num_vars(); ++i) {
      auto it = mosek_solver_prog
                    .decision_variable_index_to_mosek_nonmatrix_variable()
                    .find(i);
      if (it != mosek_solver_prog
                    .decision_variable_index_to_mosek_nonmatrix_variable()
                    .end()) {
        if (rescode == MSK_RES_OK) {
          const MSKrealt initial_guess_i = initial_guess(i);
          if (!std::isnan(initial_guess_i)) {
            rescode =
                MSK_putxxslice(mosek_solver_prog.task(), solution_type,
                               it->second, it->second + 1, &initial_guess_i);
          }
        }
      }
    }
  }

  result->set_solution_result(SolutionResult::kUnknownError);
  // Run optimizer.
  if (rescode == MSK_RES_OK) {
    // TODO(hongkai.dai@tri.global): add trmcode to the returned struct.
    MSKrescodee trmcode;  // termination code
    rescode = MSK_optimizetrm(mosek_solver_prog.task(), &trmcode);
    // Refer to
    // https://docs.mosek.com/latest/capi/debugging-tutorials.html#debugging-tutorials
    // on printing the solution summary.
    if (print_to_console || !print_file_name.empty()) {
      if (rescode == MSK_RES_OK) {
        rescode = MSK_solutionsummary(mosek_solver_prog.task(), MSK_STREAM_LOG);
      }
    }
  }

  if (rescode == MSK_RES_OK && msk_writedata.has_value()) {
    rescode =
        MSK_writedata(mosek_solver_prog.task(), msk_writedata.value().c_str());
  }

  MSKsolstae solution_status{MSK_SOL_STA_UNKNOWN};
  if (rescode == MSK_RES_OK) {
    rescode = MSK_getsolsta(mosek_solver_prog.task(), solution_type,
                            &solution_status);
  }
  if (rescode == MSK_RES_OK) {
    switch (solution_status) {
      case MSK_SOL_STA_OPTIMAL:
      case MSK_SOL_STA_INTEGER_OPTIMAL:
      case MSK_SOL_STA_PRIM_FEAS: {
        result->set_solution_result(SolutionResult::kSolutionFound);
        break;
      }
      case MSK_SOL_STA_DUAL_INFEAS_CER: {
        result->set_solution_result(SolutionResult::kDualInfeasible);
        break;
      }
      case MSK_SOL_STA_PRIM_INFEAS_CER: {
        result->set_solution_result(SolutionResult::kInfeasibleConstraints);
        break;
      }
      default: {
        result->set_solution_result(SolutionResult::kUnknownError);
        break;
      }
    }
    MSKint32t num_mosek_vars;
    rescode = MSK_getnumvar(mosek_solver_prog.task(), &num_mosek_vars);
    DRAKE_ASSERT(rescode == MSK_RES_OK);
    Eigen::VectorXd mosek_sol_vector(num_mosek_vars);
    rescode = MSK_getxx(mosek_solver_prog.task(), solution_type,
                        mosek_sol_vector.data());
    MSKint32t num_bar_x;
    rescode = MSK_getnumbarvar(mosek_solver_prog.task(), &num_bar_x);
    DRAKE_ASSERT(rescode == MSK_RES_OK);
    std::vector<Eigen::VectorXd> mosek_bar_x_sol(num_bar_x);
    for (int i = 0; i < num_bar_x; ++i) {
      MSKint32t bar_xi_dim;
      rescode = MSK_getdimbarvarj(mosek_solver_prog.task(), i, &bar_xi_dim);
      DRAKE_ASSERT(rescode == MSK_RES_OK);
      mosek_bar_x_sol[i].resize(bar_xi_dim * (bar_xi_dim + 1) / 2);
      rescode = MSK_getbarxj(mosek_solver_prog.task(), solution_type, i,
                             mosek_bar_x_sol[i].data());
    }
    DRAKE_ASSERT(rescode == MSK_RES_OK);
    Eigen::VectorXd sol_vector(num_decision_vars);
    for (int i = 0; i < num_decision_vars; ++i) {
      auto it1 = mosek_solver_prog
                     .decision_variable_index_to_mosek_nonmatrix_variable()
                     .find(i);
      if (it1 != mosek_solver_prog
                     .decision_variable_index_to_mosek_nonmatrix_variable()
                     .end()) {
        sol_vector(i) = mosek_sol_vector(it1->second);
      } else {
        auto it2 =
            mosek_solver_prog.decision_variable_index_to_mosek_matrix_variable()
                .find(i);
        sol_vector(i) = mosek_bar_x_sol[it2->second.bar_matrix_index()](
            it2->second.IndexInLowerTrianglePart());
      }
    }
    if (rescode == MSK_RES_OK) {
      result->set_x_val(sol_vector);
    }
    MSKrealt optimal_cost;
    rescode = MSK_getprimalobj(mosek_solver_prog.task(), solution_type,
                               &optimal_cost);
    DRAKE_ASSERT(rescode == MSK_RES_OK);
    if (rescode == MSK_RES_OK) {
      result->set_optimal_cost(optimal_cost);
    }
    rescode = mosek_solver_prog.SetDualSolution(
        solution_type, prog, bb_con_dual_indices, linear_con_dual_indices,
        lin_eq_con_dual_indices, lorentz_cone_dual_indices,
        rotated_lorentz_cone_dual_indices, exp_cone_dual_indices,
        psd_barvar_indices, result);
    DRAKE_ASSERT(rescode == MSK_RES_OK);
  }

  MosekSolverDetails& solver_details =
      result->SetSolverDetailsType<MosekSolverDetails>();
  solver_details.rescode = rescode;
  solver_details.solution_status = solution_status;
  if (rescode == MSK_RES_OK) {
    rescode = MSK_getdouinf(mosek_solver_prog.task(), MSK_DINF_OPTIMIZER_TIME,
                            &(solver_details.optimizer_time));
  }
  // rescode is not used after this. If in the future, the user wants to call
  // more MSK functions after this line, then he/she needs to check if rescode
  // is OK. But do not modify result->solution_result_ if rescode is not OK
  // after this line.
  unused(rescode);
}

}  // namespace solvers
}  // namespace drake
