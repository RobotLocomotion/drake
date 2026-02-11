#include "drake/solvers/mosek_solver.h"

#include <cmath>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <mosek.h>

#include "drake/common/scoped_singleton.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver_internal.h"

namespace drake {
namespace solvers {

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
      throw std::runtime_error(
          fmt::format("Could not acquire MOSEK license: {}. See "
                      "https://docs.mosek.com/11.1/capi/"
                      "response-codes.html#mosek.rescode for details.",
                      fmt_streamed(rescode)));
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

bool MosekSolver::is_available() {
  return true;
}

void MosekSolver::DoSolve2(const MathematicalProgram& prog,
                           const Eigen::VectorXd& initial_guess,
                           internal::SpecificOptions* options,
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

  internal::MosekSolverProgram impl(prog, env);
  // The number of non-matrix variables in @p prog.
  const int num_nonmatrix_vars_in_prog =
      impl.decision_variable_to_mosek_nonmatrix_variable().size();

  // Set the options (parameters).
  bool is_printing{};
  std::optional<std::string> msk_writedata;
  if (rescode == MSK_RES_OK) {
    impl.UpdateOptions(options, &is_printing, &msk_writedata);
  }

  // Always check if rescode is MSK_RES_OK before we call any Mosek functions.
  // If it is not MSK_RES_OK, then bypasses everything and exits.
  if (rescode == MSK_RES_OK) {
    rescode = MSK_appendvars(impl.task(), num_nonmatrix_vars_in_prog);
  }
  // Add positive semidefinite constraint. This also adds Mosek matrix
  // variables.
  // psd_barvar_indices records the index of the bar matrix for this positive
  // semidefinite constraint. We will use this bar matrix index later, for
  // example when retrieving the dual solution.
  std::unordered_map<Binding<PositiveSemidefiniteConstraint>, MSKint32t>
      psd_barvar_indices;
  if (rescode == MSK_RES_OK) {
    rescode =
        impl.AddPositiveSemidefiniteConstraints(prog, &psd_barvar_indices);
  }
  // Add the constraint that Mosek matrix variable entries corresponding to the
  // same decision variables should all be equal.
  if (rescode == MSK_RES_OK) {
    rescode =
        impl.AddEqualityConstraintBetweenMatrixVariablesForSameDecisionVariable();  // NOLINT
  }
  // Add costs
  if (rescode == MSK_RES_OK) {
    rescode = impl.AddCosts(prog);
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
  // When a PositiveSemidefiniteConstraint is imposed on a scalar, it is
  // equivalent to constraining that scalar variable with a bounding box
  // constraint. We store the dual variable indices for that
  // PositiveSemidefiniteConstraint in scalar_psd_con_dual_indices.
  std::unordered_map<
      Binding<PositiveSemidefiniteConstraint>,
      std::pair<internal::ConstraintDualIndex, internal::ConstraintDualIndex>>
      scalar_psd_con_dual_indices;
  // Add bounding box constraints on decision variables.
  if (rescode == MSK_RES_OK) {
    rescode = impl.AddVariableBounds(prog, &bb_con_dual_indices,
                                     &scalar_psd_con_dual_indices);
  }
  // Specify binary variables.
  bool with_integer_or_binary_variable = false;
  if (rescode == MSK_RES_OK) {
    rescode = impl.SpecifyVariableType(prog, &with_integer_or_binary_variable);
  }
  // Add linear constraints.
  std::unordered_map<Binding<LinearConstraint>, internal::ConstraintDualIndices>
      linear_con_dual_indices;
  std::unordered_map<Binding<LinearEqualityConstraint>,
                     internal::ConstraintDualIndices>
      lin_eq_con_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode = impl.AddLinearConstraints(prog, &linear_con_dual_indices,
                                        &lin_eq_con_dual_indices);
  }

  // Add Lorentz cone constraints.
  std::unordered_map<Binding<LorentzConeConstraint>, MSKint64t>
      lorentz_cone_acc_indices;
  if (rescode == MSK_RES_OK) {
    rescode = impl.AddConeConstraints(prog, prog.lorentz_cone_constraints(),
                                      &lorentz_cone_acc_indices);
  }

  // Add rotated Lorentz cone constraints.
  std::unordered_map<Binding<RotatedLorentzConeConstraint>, MSKint64t>
      rotated_lorentz_cone_acc_indices;
  if (rescode == MSK_RES_OK) {
    rescode =
        impl.AddConeConstraints(prog, prog.rotated_lorentz_cone_constraints(),
                                &rotated_lorentz_cone_acc_indices);
  }

  // Add quadratic constraints.
  std::unordered_map<Binding<QuadraticConstraint>, MSKint64t>
      quadratic_constraint_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode =
        impl.AddQuadraticConstraints(prog, &quadratic_constraint_dual_indices);
  }

  // Add 2x2 PSD constraints as rotated lorentz cones.
  std::unordered_map<Binding<PositiveSemidefiniteConstraint>, MSKint64t>
      twobytwo_psd_constraint_cone_acc_indices;
  if (rescode == MSK_RES_OK) {
    rescode = impl.Add2x2PositiveSemidefiniteConstraints(
        prog, &twobytwo_psd_constraint_cone_acc_indices);
  }

  // Add linear matrix inequality constraints.
  std::unordered_map<Binding<LinearMatrixInequalityConstraint>, MSKint64t>
      lmi_acc_indices;
  if (rescode == MSK_RES_OK) {
    rescode = impl.AddLinearMatrixInequalityConstraint(prog, &lmi_acc_indices);
  }

  // Add exponential cone constraints.
  std::unordered_map<Binding<ExponentialConeConstraint>, MSKint64t>
      exp_cone_acc_indices;
  if (rescode == MSK_RES_OK) {
    rescode = impl.AddConeConstraints(prog, prog.exponential_cone_constraints(),
                                      &exp_cone_acc_indices);
  }

  // Determines the solution type.
  MSKsoltypee solution_type;
  if (with_integer_or_binary_variable) {
    solution_type = MSK_SOL_ITG;
  } else if (prog.quadratic_costs().empty() && prog.l2norm_costs().empty() &&
             prog.quadratic_constraints().empty() &&
             prog.lorentz_cone_constraints().empty() &&
             prog.rotated_lorentz_cone_constraints().empty() &&
             prog.positive_semidefinite_constraints().empty() &&
             prog.linear_matrix_inequality_constraints().empty() &&
             prog.exponential_cone_constraints().empty()) {
    // The program is LP.
    int ipm_basis{};
    if (rescode == MSK_RES_OK) {
      rescode = MSK_getintparam(impl.task(), MSK_IPAR_INTPNT_BASIS, &ipm_basis);
    }
    // By default ipm_basis > 0 and Mosek will do a basis identification to
    // clean up the solution after the interior point method (IPM), then we can
    // query the basis solution. Otherwise we will only query the IPM solution.
    if (ipm_basis > 0) {
      solution_type = MSK_SOL_BAS;
    } else {
      solution_type = MSK_SOL_ITR;
    }
  } else {
    solution_type = MSK_SOL_ITR;
  }

  // Since Mosek 10, it allows setting the initial guess for both continuous and
  // integer/binary variables. See
  // https://docs.mosek.com/11.1/rmosek/tutorial-mio-shared.html#specifying-an-initial-solution
  // for more details.
  if (initial_guess.array().isFinite().any()) {
    DRAKE_ASSERT(initial_guess.size() == prog.num_vars());
    for (int i = 0; i < prog.num_vars(); ++i) {
      const auto& map_to_mosek =
          impl.decision_variable_to_mosek_nonmatrix_variable();
      if (auto it = map_to_mosek.find(i); it != map_to_mosek.end()) {
        if (rescode == MSK_RES_OK) {
          const MSKrealt initial_guess_i = initial_guess(i);
          if (!std::isnan(initial_guess_i)) {
            rescode = MSK_putxxslice(impl.task(), solution_type, it->second,
                                     it->second + 1, &initial_guess_i);
          }
        }
      }
    }
  }

  result->set_solution_result(SolutionResult::kSolverSpecificError);
  // Run optimizer.
  if (rescode == MSK_RES_OK) {
    // TODO(hongkai.dai@tri.global): add trmcode to the returned struct.
    MSKrescodee trmcode;  // termination code
    rescode = MSK_optimizetrm(impl.task(), &trmcode);
    // Refer to
    // https://docs.mosek.com/11.1/capi/debugging-tutorials.html#debugging-tutorials
    // on printing the solution summary.
    if (is_printing) {
      if (rescode == MSK_RES_OK) {
        rescode = MSK_solutionsummary(impl.task(), MSK_STREAM_LOG);
      }
    }
  }

  if (rescode == MSK_RES_OK && msk_writedata.has_value()) {
    rescode = MSK_writedata(impl.task(), msk_writedata.value().c_str());
  }

  MSKsolstae solution_status{MSK_SOL_STA_UNKNOWN};
  if (rescode == MSK_RES_OK) {
    rescode = MSK_getsolsta(impl.task(), solution_type, &solution_status);
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
        result->set_solution_result(SolutionResult::kSolverSpecificError);
        break;
      }
    }
    MSKint32t num_mosek_vars;
    rescode = MSK_getnumvar(impl.task(), &num_mosek_vars);
    DRAKE_ASSERT(rescode == MSK_RES_OK);
    Eigen::VectorXd mosek_sol_vector(num_mosek_vars);
    rescode = MSK_getxx(impl.task(), solution_type, mosek_sol_vector.data());
    MSKint32t num_bar_x;
    rescode = MSK_getnumbarvar(impl.task(), &num_bar_x);
    DRAKE_ASSERT(rescode == MSK_RES_OK);
    std::vector<Eigen::VectorXd> mosek_bar_x_sol(num_bar_x);
    for (int i = 0; i < num_bar_x; ++i) {
      MSKint32t bar_xi_dim;
      rescode = MSK_getdimbarvarj(impl.task(), i, &bar_xi_dim);
      DRAKE_ASSERT(rescode == MSK_RES_OK);
      mosek_bar_x_sol[i].resize(bar_xi_dim * (bar_xi_dim + 1) / 2);
      rescode = MSK_getbarxj(impl.task(), solution_type, i,
                             mosek_bar_x_sol[i].data());
    }
    DRAKE_ASSERT(rescode == MSK_RES_OK);
    Eigen::VectorXd sol_vector(num_decision_vars);
    for (int i = 0; i < num_decision_vars; ++i) {
      auto it1 = impl.decision_variable_to_mosek_nonmatrix_variable().find(i);
      if (it1 != impl.decision_variable_to_mosek_nonmatrix_variable().end()) {
        sol_vector(i) = mosek_sol_vector(it1->second);
      } else {
        auto it2 = impl.decision_variable_to_mosek_matrix_variable().find(i);
        sol_vector(i) = mosek_bar_x_sol[it2->second.bar_matrix_index()](
            it2->second.IndexInLowerTrianglePart());
      }
    }
    if (rescode == MSK_RES_OK) {
      result->set_x_val(sol_vector);
    }
    MSKrealt optimal_cost;
    switch (solution_status) {
      case MSK_SOL_STA_PRIM_INFEAS_CER: {
        result->set_optimal_cost(MathematicalProgram::kGlobalInfeasibleCost);
        break;
      }
      default: {
        rescode = MSK_getprimalobj(impl.task(), solution_type, &optimal_cost);
        DRAKE_ASSERT(rescode == MSK_RES_OK);
        if (rescode == MSK_RES_OK) {
          result->set_optimal_cost(optimal_cost);
        }
      }
    }
    rescode = impl.SetDualSolution(
        solution_type, prog, bb_con_dual_indices, linear_con_dual_indices,
        lin_eq_con_dual_indices, quadratic_constraint_dual_indices,
        lorentz_cone_acc_indices, rotated_lorentz_cone_acc_indices,
        lmi_acc_indices, exp_cone_acc_indices, psd_barvar_indices,
        scalar_psd_con_dual_indices, twobytwo_psd_constraint_cone_acc_indices,
        result);
    DRAKE_ASSERT(rescode == MSK_RES_OK);
  }

  MosekSolverDetails& solver_details =
      result->SetSolverDetailsType<MosekSolverDetails>();
  solver_details.rescode = rescode;
  solver_details.solution_status = solution_status;
  if (rescode == MSK_RES_OK) {
    rescode = MSK_getdouinf(impl.task(), MSK_DINF_OPTIMIZER_TIME,
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
