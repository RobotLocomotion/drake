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

// @param slx Mosek dual variables for variable lower bound. See
// https://docs.mosek.com/10.0/capi/alphabetic-functionalities.html#mosek.task.getslx
// @param sux Mosek dual variables for variable upper bound. See
// https://docs.mosek.com/10.0/capi/alphabetic-functionalities.html#mosek.task.getsux
// @param slc Mosek dual variables for linear constraint lower bound. See
// https://docs.mosek.com/10.0/capi/alphabetic-functionalities.html#mosek.task.getslc
// @param suc Mosek dual variables for linear constraint upper bound. See
// https://docs.mosek.com/10.0/capi/alphabetic-functionalities.html#mosek.task.getsuc
void SetBoundingBoxDualSolution(
    const std::vector<Binding<BoundingBoxConstraint>>& constraints,
    const std::vector<MSKrealt>& slx, const std::vector<MSKrealt>& sux,
    const std::vector<MSKrealt>& slc, const std::vector<MSKrealt>& suc,
    const std::unordered_map<
        Binding<BoundingBoxConstraint>,
        std::pair<internal::MosekSolverProgram::ConstraintDualIndices,
                  internal::MosekSolverProgram::ConstraintDualIndices>>&
        bb_con_dual_indices,
    MathematicalProgramResult* result) {
  auto set_dual_sol = [](int mosek_dual_lower_index, int mosek_dual_upper_index,
                         const std::vector<MSKrealt>& mosek_dual_lower,
                         const std::vector<MSKrealt>& mosek_dual_upper,
                         double* dual_sol) {
    const double dual_lower = mosek_dual_lower_index == -1
                                  ? 0.
                                  : mosek_dual_lower[mosek_dual_lower_index];
    const double dual_upper = mosek_dual_upper_index == -1
                                  ? 0.
                                  : mosek_dual_upper[mosek_dual_upper_index];
    // Mosek defines all dual solutions as non-negative. However we use
    // "reduced cost" as the dual solution, so the dual solution for a
    // lower bound should be non-negative, while the dual solution for
    // an upper bound should be non-positive.
    if (dual_lower > dual_upper) {
      // We use the larger dual as the active one.
      *dual_sol = dual_lower;
    } else {
      *dual_sol = -dual_upper;
    }
  };
  for (const auto& binding : constraints) {
    internal::MosekSolverProgram::ConstraintDualIndices lower_bound_duals,
        upper_bound_duals;
    std::tie(lower_bound_duals, upper_bound_duals) =
        bb_con_dual_indices.at(binding);
    Eigen::VectorXd dual_sol =
        Eigen::VectorXd::Zero(binding.evaluator()->num_vars());
    for (int i = 0; i < binding.variables().rows(); ++i) {
      switch (lower_bound_duals[i].type) {
        case internal::MosekSolverProgram::DualVarType::kVariableBound: {
          set_dual_sol(lower_bound_duals[i].index, upper_bound_duals[i].index,
                       slx, sux, &(dual_sol(i)));
          break;
        }
        case internal::MosekSolverProgram::DualVarType::kLinearConstraint: {
          set_dual_sol(lower_bound_duals[i].index, upper_bound_duals[i].index,
                       slc, suc, &(dual_sol(i)));
          break;
        }
        default: {
          throw std::runtime_error(
              "The dual variable for a BoundingBoxConstraint lower bound can "
              "only be slx or slc.");
        }
      }
    }
    result->set_dual_solution(binding, dual_sol);
  }
}

template <typename C>
void SetLinearConstraintDualSolution(
    const std::vector<Binding<C>>& bindings, const std::vector<MSKrealt>& slc,
    const std::vector<MSKrealt>& suc,
    const std::unordered_map<
        Binding<C>, internal::MosekSolverProgram::ConstraintDualIndices>&
        linear_con_dual_indices,
    MathematicalProgramResult* result) {
  for (const auto& binding : bindings) {
    const internal::MosekSolverProgram::ConstraintDualIndices duals =
        linear_con_dual_indices.at(binding);
    Eigen::VectorXd dual_sol =
        Eigen::VectorXd::Zero(binding.evaluator()->num_constraints());
    for (int i = 0; i < dual_sol.rows(); ++i) {
      DRAKE_DEMAND(
          duals[i].type ==
          internal::MosekSolverProgram::DualVarType::kLinearConstraint);
      // Mosek defines all dual solutions as non-negative. However we use
      // "reduced cost" as the dual solution, so the dual solution for a
      // lower bound should be non-negative, while the dual solution for
      // an upper bound should be non-positive.
      if (slc[duals[i].index] > suc[duals[i].index]) {
        dual_sol[i] = slc[duals[i].index];
      } else {
        dual_sol[i] = -suc[duals[i].index];
      }
    }
    result->set_dual_solution(binding, dual_sol);
  }
}

template <typename C>
void SetNonlinearConstraintDualSolution(
    const std::vector<Binding<C>>& bindings, const std::vector<MSKrealt>& snx,
    const std::unordered_map<
        Binding<C>, internal::MosekSolverProgram::ConstraintDualIndices>&
        dual_indices,
    MathematicalProgramResult* result) {
  for (const auto& binding : bindings) {
    const internal::MosekSolverProgram::ConstraintDualIndices duals =
        dual_indices.at(binding);
    Eigen::VectorXd dual_sol = Eigen::VectorXd::Zero(duals.size());
    for (int i = 0; i < dual_sol.rows(); ++i) {
      DRAKE_DEMAND(duals[i].type ==
                   internal::MosekSolverProgram::DualVarType::kNonlinearConic);
      dual_sol[i] = snx[duals[i].index];
    }
    result->set_dual_solution(binding, dual_sol);
  }
}

MSKrescodee SetPositiveSemidefiniteConstraintDualSolution(
    const MathematicalProgram& prog,
    const std::unordered_map<Binding<PositiveSemidefiniteConstraint>,
                             MSKint32t>& psd_barvar_indices,
    MSKtask_t task, MSKsoltypee whichsol, MathematicalProgramResult* result) {
  MSKrescodee rescode = MSK_RES_OK;
  for (const auto& psd_constraint : prog.positive_semidefinite_constraints()) {
    // Get the bar index of the Mosek matrix var.
    const auto it = psd_barvar_indices.find(psd_constraint);
    if (it == psd_barvar_indices.end()) {
      throw std::runtime_error(
          "SetPositiveSemidefiniteConstraintDualSolution: this positive "
          "semidefinite constraint has not been "
          "registered in Mosek as a matrix variable. This should not happen, "
          "please post an issue on Drake: "
          "https://github.com/RobotLocomotion/drake/issues/new.");
    }
    const auto bar_index = it->second;
    // barsj stores the lower triangular values of the psd matrix (as the dual
    // solution).
    std::vector<MSKrealt> barsj(
        psd_constraint.evaluator()->matrix_rows() *
        (psd_constraint.evaluator()->matrix_rows() + 1) / 2);
    rescode = MSK_getbarsj(task, whichsol, bar_index, barsj.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    // Copy barsj to dual_lower. We don't use barsj directly since MSKrealt
    // might be a different data type from double.
    Eigen::VectorXd dual_lower(barsj.size());
    for (int i = 0; i < dual_lower.rows(); ++i) {
      dual_lower(i) = barsj[i];
    }
    result->set_dual_solution(psd_constraint, dual_lower);
  }
  return rescode;
}

MSKrescodee SetDualSolution(
    const MSKtask_t& task, MSKsoltypee which_sol,
    const MathematicalProgram& prog,
    const std::unordered_map<
        Binding<BoundingBoxConstraint>,
        std::pair<internal::MosekSolverProgram::ConstraintDualIndices,
                  internal::MosekSolverProgram::ConstraintDualIndices>>&
        bb_con_dual_indices,
    const std::unordered_map<
        Binding<LinearConstraint>,
        internal::MosekSolverProgram::ConstraintDualIndices>&
        linear_con_dual_indices,
    const std::unordered_map<
        Binding<LinearEqualityConstraint>,
        internal::MosekSolverProgram::ConstraintDualIndices>&
        lin_eq_con_dual_indices,
    const std::unordered_map<
        Binding<LorentzConeConstraint>,
        internal::MosekSolverProgram::ConstraintDualIndices>&
        lorentz_cone_dual_indices,
    const std::unordered_map<
        Binding<RotatedLorentzConeConstraint>,
        internal::MosekSolverProgram::ConstraintDualIndices>&
        rotated_lorentz_cone_dual_indices,
    const std::unordered_map<
        Binding<ExponentialConeConstraint>,
        internal::MosekSolverProgram::ConstraintDualIndices>&
        exp_cone_dual_indices,
    const std::unordered_map<Binding<PositiveSemidefiniteConstraint>,
                             MSKint32t>& psd_barvar_indices,
    MathematicalProgramResult* result) {
  // TODO(hongkai.dai): support other types of constraints, like linear
  // constraint, second order cone constraint, etc.
  MSKrescodee rescode{MSK_RES_OK};
  if (which_sol != MSK_SOL_ITG) {
    // Mosek cannot return dual solution if the solution type is MSK_SOL_ITG
    // (which stands for mixed integer optimizer), see
    // https://docs.mosek.com/10.0/capi/accessing-solution.html#available-solutions
    int num_mosek_vars{0};
    rescode = MSK_getnumvar(task, &num_mosek_vars);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    // Mosek dual variables for variable lower bounds (slx) and upper bounds
    // (sux). Refer to
    // https://docs.mosek.com/10.0/capi/alphabetic-functionalities.html#mosek.task.getsolution
    // for more explanation.
    std::vector<MSKrealt> slx(num_mosek_vars);
    std::vector<MSKrealt> sux(num_mosek_vars);
    rescode = MSK_getslx(task, which_sol, slx.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = MSK_getsux(task, which_sol, sux.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    int num_linear_constraints{0};
    rescode = MSK_getnumcon(task, &num_linear_constraints);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    // Mosek dual variables for linear constraints lower bounds (slc) and upper
    // bounds (suc). Refer to
    // https://docs.mosek.com/10.0/capi/alphabetic-functionalities.html#mosek.task.getsolution
    // for more explanation.
    std::vector<MSKrealt> slc(num_linear_constraints);
    std::vector<MSKrealt> suc(num_linear_constraints);
    rescode = MSK_getslc(task, which_sol, slc.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = MSK_getsuc(task, which_sol, suc.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    // Set the duals for the bounding box constraint.
    SetBoundingBoxDualSolution(prog.bounding_box_constraints(), slx, sux, slc,
                               suc, bb_con_dual_indices, result);
    // Set the duals for the linear constraint.
    SetLinearConstraintDualSolution(prog.linear_constraints(), slc, suc,
                                    linear_con_dual_indices, result);
    // Set the duals for the linear equality constraint.
    SetLinearConstraintDualSolution(prog.linear_equality_constraints(), slc,
                                    suc, lin_eq_con_dual_indices, result);
    // Set the duals for the nonlinear conic constraint.
    // Mosek provides the dual solution for nonlinear conic constraints only if
    // the program is solved through interior point approach.
    if (which_sol == MSK_SOL_ITR) {
      std::vector<MSKrealt> snx(num_mosek_vars);
      rescode = MSK_getsnx(task, which_sol, snx.data());
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
      SetNonlinearConstraintDualSolution(prog.lorentz_cone_constraints(), snx,
                                         lorentz_cone_dual_indices, result);
      SetNonlinearConstraintDualSolution(
          prog.rotated_lorentz_cone_constraints(), snx,
          rotated_lorentz_cone_dual_indices, result);
      SetNonlinearConstraintDualSolution(prog.exponential_cone_constraints(),
                                         snx, exp_cone_dual_indices, result);
    }
    rescode = SetPositiveSemidefiniteConstraintDualSolution(
        prog, psd_barvar_indices, task, which_sol, result);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
  }
  return rescode;
}

// Throws a runtime error if the mosek option is set incorrectly.
template <typename T>
void ThrowForInvalidOption(MSKrescodee rescode, const std::string& option,
                           const T& val) {
  if (rescode != MSK_RES_OK) {
    const std::string mosek_version =
        fmt::format("{}.{}", MSK_VERSION_MAJOR, MSK_VERSION_MINOR);
    throw std::runtime_error(fmt::format(
        "MosekSolver(): cannot set Mosek option '{option}' to value '{value}', "
        "response code {code}, check "
        "https://docs.mosek.com/{version}/capi/response-codes.html for the "
        "meaning of the response code, check "
        "https://docs.mosek.com/{version}/capi/param-groups.html for allowable "
        "values in C++, or "
        "https://docs.mosek.com/{version}/pythonapi/param-groups.html "
        "for allowable values in python.",
        fmt::arg("option", option), fmt::arg("value", val),
        fmt::arg("code", rescode), fmt::arg("version", mosek_version)));
  }
}

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

  // Mosek treats matrix variable (variables in a psd matrix) in a special
  // manner, but MathematicalProgram doesn't. Hence we need to pick out the
  // matrix and non-matrix variables in @p prog, and record how they will be
  // stored in Mosek.
  std::unordered_map<int, internal::MosekSolverProgram::MatrixVariableEntry>
      decision_variable_index_to_mosek_matrix_variable;
  std::unordered_map<int, int>
      decision_variable_index_to_mosek_nonmatrix_variable;
  // Multiple entries in Mosek matrix variables could correspond to the same
  // decision variable. We will need to add linear equality constraints to
  // equate these entries.
  std::unordered_map<
      int, std::vector<internal::MosekSolverProgram::MatrixVariableEntry>>
      matrix_variable_entries_for_same_decision_variable;
  internal::MapProgramDecisionVariableToMosekVariable(
      prog, &decision_variable_index_to_mosek_matrix_variable,
      &decision_variable_index_to_mosek_nonmatrix_variable,
      &matrix_variable_entries_for_same_decision_variable);
  DRAKE_ASSERT(
      static_cast<int>(
          decision_variable_index_to_mosek_matrix_variable.size() +
          decision_variable_index_to_mosek_nonmatrix_variable.size()) ==
      prog.num_vars());
  // The number of non-matrix variables in @p prog.
  const int num_nonmatrix_vars_in_prog =
      decision_variable_index_to_mosek_nonmatrix_variable.size();
  // For each entry of the matrix variable X̅ᵢ(m,n), if this entry is also used
  // in the cost or constraint, then we will use the term <E̅ₘₙ,X̅ᵢ>, where
  // the inner product <E̅ₘₙ, X̅ᵢ> = X̅ᵢ(m,n). The "selection matrix" E̅ₘₙ helps
  // to select the (m, n)'th entry of the psd matrix variable. E̅ₘₙ is stored
  // inside Mosek with a unique ID.
  // matrix_variable_entry_to_selection_matrix_id maps the matrix variable
  // entry to the ID of E̅ₘₙ.
  // TODO(hongkai.dai): do not map the matrix variable entry to the selection
  // matrix, instead maps the tuple (matrix_size, row_index, column_index) to
  // the selection matrix. This will create fewer selection matrices, when
  // multiple matrix variables have the same size.
  std::unordered_map<internal::MosekSolverProgram::MatrixVariableEntry::Id,
                     MSKint64t>
      matrix_variable_entry_to_selection_matrix_id;

  internal::MosekSolverProgram mosek_solver_prog(env,
                                                 num_nonmatrix_vars_in_prog);

  // Set the options (parameters).
  for (const auto& double_options : merged_options.GetOptionsDouble(id())) {
    if (rescode == MSK_RES_OK) {
      rescode = MSK_putnadouparam(mosek_solver_prog.task(),
                                  double_options.first.c_str(),
                                  double_options.second);
      ThrowForInvalidOption(rescode, double_options.first,
                            double_options.second);
    }
  }
  for (const auto& int_options : merged_options.GetOptionsInt(id())) {
    if (rescode == MSK_RES_OK) {
      rescode =
          MSK_putnaintparam(mosek_solver_prog.task(), int_options.first.c_str(),
                            int_options.second);
      ThrowForInvalidOption(rescode, int_options.first, int_options.second);
    }
  }
  std::optional<std::string> msk_writedata;
  for (const auto& str_options : merged_options.GetOptionsStr(id())) {
    if (rescode == MSK_RES_OK) {
      if (str_options.first == "writedata") {
        if (str_options.second != "") {
          msk_writedata = str_options.second;
        }
      } else {
        rescode = MSK_putnastrparam(mosek_solver_prog.task(),
                                    str_options.first.c_str(),
                                    str_options.second.c_str());
        ThrowForInvalidOption(rescode, str_options.first, str_options.second);
      }
    }
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
            .AddEqualityConstraintBetweenMatrixVariablesForSameDecisionVariable(
                matrix_variable_entries_for_same_decision_variable,
                &matrix_variable_entry_to_selection_matrix_id);
  }
  // Add costs
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddCosts(
        prog, decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        &matrix_variable_entry_to_selection_matrix_id);
  }

  // We store the dual variable indices for each bounding box constraint.
  // bb_con_dual_indices[constraint] returns the pair (lower_bound_indices,
  // upper_bound_indices), where lower_bound_indices are the indices of the
  // lower bound dual variables, and upper_bound_indices are the indices of the
  // upper bound dual variables.
  std::unordered_map<
      Binding<BoundingBoxConstraint>,
      std::pair<internal::MosekSolverProgram::ConstraintDualIndices,
                internal::MosekSolverProgram::ConstraintDualIndices>>
      bb_con_dual_indices;
  // Add bounding box constraints on decision variables.
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddBoundingBoxConstraints(
        prog, decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        &matrix_variable_entry_to_selection_matrix_id, &bb_con_dual_indices);
  }
  // Specify binary variables.
  bool with_integer_or_binary_variable = false;
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.SpecifyVariableType(
        prog, decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        &with_integer_or_binary_variable);
  }
  // Add linear constraints.
  std::unordered_map<Binding<LinearConstraint>,
                     internal::MosekSolverProgram::ConstraintDualIndices>
      linear_con_dual_indices;
  std::unordered_map<Binding<LinearEqualityConstraint>,
                     internal::MosekSolverProgram::ConstraintDualIndices>
      lin_eq_con_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddLinearConstraints(
        prog, decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        &matrix_variable_entry_to_selection_matrix_id, &linear_con_dual_indices,
        &lin_eq_con_dual_indices);
  }

  // Add Lorentz cone constraints.
  std::unordered_map<Binding<LorentzConeConstraint>,
                     internal::MosekSolverProgram::ConstraintDualIndices>
      lorentz_cone_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddConeConstraints(
        prog, prog.lorentz_cone_constraints(),
        decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        &matrix_variable_entry_to_selection_matrix_id,
        &lorentz_cone_dual_indices);
  }

  // Add rotated Lorentz cone constraints.
  std::unordered_map<Binding<RotatedLorentzConeConstraint>,
                     internal::MosekSolverProgram::ConstraintDualIndices>
      rotated_lorentz_cone_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddConeConstraints(
        prog, prog.rotated_lorentz_cone_constraints(),
        decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        &matrix_variable_entry_to_selection_matrix_id,
        &rotated_lorentz_cone_dual_indices);
  }

  // Add linear matrix inequality constraints.
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddLinearMatrixInequalityConstraint(
        prog, decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        &matrix_variable_entry_to_selection_matrix_id);
  }

  // Add exponential cone constraints.
  std::unordered_map<Binding<ExponentialConeConstraint>,
                     internal::MosekSolverProgram::ConstraintDualIndices>
      exp_cone_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode = mosek_solver_prog.AddConeConstraints(
        prog, prog.exponential_cone_constraints(),
        decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        &matrix_variable_entry_to_selection_matrix_id, &exp_cone_dual_indices);
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
      auto it = decision_variable_index_to_mosek_nonmatrix_variable.find(i);
      if (it != decision_variable_index_to_mosek_nonmatrix_variable.end()) {
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
      auto it1 = decision_variable_index_to_mosek_nonmatrix_variable.find(i);
      if (it1 != decision_variable_index_to_mosek_nonmatrix_variable.end()) {
        sol_vector(i) = mosek_sol_vector(it1->second);
      } else {
        auto it2 = decision_variable_index_to_mosek_matrix_variable.find(i);
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
    rescode = SetDualSolution(
        mosek_solver_prog.task(), solution_type, prog, bb_con_dual_indices,
        linear_con_dual_indices, lin_eq_con_dual_indices,
        lorentz_cone_dual_indices, rotated_lorentz_cone_dual_indices,
        exp_cone_dual_indices, psd_barvar_indices, result);
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
