#include "drake/solvers/gurobi_solver.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <fmt/format.h>

// TODO(jwnimmer-tri) Eventually resolve these warnings.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// NOLINTNEXTLINE(build/include) False positive due to weird include style.
#include "gurobi_c.h"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/parallelism.h"
#include "drake/common/scope_exit.h"
#include "drake/common/scoped_singleton.h"
#include "drake/common/text_logging.h"
#include "drake/math/eigen_sparse_triplet.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/gurobi_solver_internal.h"
#include "drake/solvers/mathematical_program.h"

// TODO(hongkai.dai): GurobiSolver class should store data member such as
// GRB_model, GRB_env, is_new_variables, etc.
namespace drake {
namespace solvers {
namespace {

// Returns the base URL for Gurobi's online reference manual.
std::string refman() {
  return fmt::format(
      "https://www.docs.gurobi.com/projects/optimizer/en/{}.{}/reference",
      GRB_VERSION_MAJOR, GRB_VERSION_MINOR);
}

// Information to be passed through a Gurobi C callback to
// grant it information about its problem (the host
// MathematicalProgram prog, and which decision variables
// are not represented in prog), and what user functions
// are present for handling the callback.
// TODO(gizatt) This struct can be replaced with a ptr to
// the GurobiSolver class (or the callback can shell to a
// method on that class) once the above TODO(hongkai.dai) is
// completed. It might be able to be further reduced if
// GurobiSolver subclasses GRBCallback in the Gurobi C++ API.
struct GurobiCallbackInformation {
  const MathematicalProgram* prog{};
  std::vector<bool> is_new_variable;
  // Used in callbacks to store raw Gurobi variable values.
  std::vector<double> solver_sol_vector;
  // Used in callbacks to store variable values that appear
  // in the MathematicalProgram (which are a subset of the
  // Gurobi variable values).
  Eigen::VectorXd prog_sol_vector;
  GurobiSolver::MipNodeCallbackFunction mip_node_callback;
  GurobiSolver::MipSolCallbackFunction mip_sol_callback;
  MathematicalProgramResult* result{};
};

// Utility that, given a raw Gurobi solution vector, a container
// in which to populate the Mathematical Program solution vector,
// and a mapping of which elements should be accepted from the Gurobi
// solution vector, sets a MathematicalProgram's solution to the
// Gurobi solution.
void SetProgramSolutionVector(const std::vector<bool>& is_new_variable,
                              const std::vector<double>& solver_sol_vector,
                              Eigen::VectorXd* prog_sol_vector) {
  int k = 0;
  for (size_t i = 0; i < is_new_variable.size(); ++i) {
    if (!is_new_variable[i]) {
      (*prog_sol_vector)(k) = solver_sol_vector[i];
      k++;
    }
  }
}

//  @param gurobi_dual_solutions gurobi_dual_solutions(i) is the dual solution
//  for the variable bound lower <= gurobi_var(i) <= upper. This is extracted
//  from "RC" (stands for reduced cost) field from gurobi model.
//  @param bb_con_dual_indices Maps each bounding box constraint to the indices
//  of its dual variables for both lower and upper bounds.
void SetBoundingBoxDualSolution(
    const MathematicalProgram& prog,
    const std::vector<double>& gurobi_dual_solutions,
    const std::unordered_map<Binding<BoundingBoxConstraint>,
                             std::pair<std::vector<int>, std::vector<int>>>&
        bb_con_dual_indices,
    MathematicalProgramResult* result) {
  for (const auto& binding : prog.bounding_box_constraints()) {
    Eigen::VectorXd dual_sol =
        Eigen::VectorXd::Zero(binding.evaluator()->num_vars());
    std::vector<int> lower_dual_indices, upper_dual_indices;
    std::tie(lower_dual_indices, upper_dual_indices) =
        bb_con_dual_indices.at(binding);
    for (int i = 0; i < binding.evaluator()->num_vars(); ++i) {
      if (lower_dual_indices[i] != -1 &&
          gurobi_dual_solutions[lower_dual_indices[i]] >= 0) {
        // This lower bound is active since the reduced cost is non-negative.
        dual_sol(i) = gurobi_dual_solutions[lower_dual_indices[i]];
      } else if (upper_dual_indices[i] != -1 &&
                 gurobi_dual_solutions[upper_dual_indices[i]] <= 0) {
        // This upper bound is active since the reduced cost is non-positive.
        dual_sol(i) = gurobi_dual_solutions[upper_dual_indices[i]];
      }
    }
    result->set_dual_solution(binding, dual_sol);
  }
}

/**
 * Set the dual solution for each linear inequality and equality constraint.
 * @param gurobi_dual_solutions The dual solutions for each linear
 * inequality/equality constraint. This is extracted from "Pi" field from gurobi
 * model.
 * @param constraint_dual_start_row constraint_dual_start_row[constraint] maps
 * the linear inequality/equality constraint to the starting index of the
 * corresponding dual variable.
 */
void SetLinearConstraintDualSolutions(
    const MathematicalProgram& prog,
    const Eigen::VectorXd& gurobi_dual_solutions,
    const std::unordered_map<Binding<Constraint>, int>&
        constraint_dual_start_row,
    MathematicalProgramResult* result) {
  for (const auto& binding : prog.linear_equality_constraints()) {
    result->set_dual_solution(
        binding,
        gurobi_dual_solutions.segment(constraint_dual_start_row.at(binding),
                                      binding.evaluator()->num_constraints()));
  }

  for (const auto& binding : prog.linear_constraints()) {
    Eigen::VectorXd dual_solution =
        Eigen::VectorXd::Zero(binding.evaluator()->num_constraints());
    int gurobi_constraint_index = constraint_dual_start_row.at(binding);
    const auto& lb = binding.evaluator()->lower_bound();
    const auto& ub = binding.evaluator()->upper_bound();
    for (int i = 0; i < binding.evaluator()->num_constraints(); ++i) {
      if (!std::isinf(ub(i)) || !std::isinf(lb(i))) {
        if (!std::isinf(lb(i)) && std::isinf(ub(i))) {
          dual_solution(i) = gurobi_dual_solutions(gurobi_constraint_index);
          gurobi_constraint_index++;
        } else if (!std::isinf(ub(i)) && std::isinf(lb(i))) {
          dual_solution(i) = gurobi_dual_solutions(gurobi_constraint_index);
          gurobi_constraint_index++;
        } else if (!std::isinf(ub(i)) && !std::isinf(lb(i))) {
          // When the constraint has both lower and upper bound, we know that if
          // the lower bound is active, then the dual solution >= 0. If the
          // upper bound is active, then the dual solution <= 0.
          const double lower_bound_dual =
              gurobi_dual_solutions(gurobi_constraint_index);
          const double upper_bound_dual =
              gurobi_dual_solutions(gurobi_constraint_index + 1);
          // Due to small numerical error, even if the bound is not active,
          // gurobi still reports that the dual variable has non-zero value. So
          // we compare the absolute value of the lower and upper bound, and
          // choose the one with the larger absolute value.
          dual_solution(i) =
              std::abs(lower_bound_dual) > std::abs(upper_bound_dual)
                  ? lower_bound_dual
                  : upper_bound_dual;
          gurobi_constraint_index += 2;
        }
      }
    }
    result->set_dual_solution(binding, dual_solution);
  }
}

template <typename C>
void SetSecondOrderConeDualSolution(
    const std::vector<Binding<C>>& constraints,
    const Eigen::VectorXd& gurobi_qcp_dual_solutions,
    MathematicalProgramResult* result, int* soc_count) {
  for (const auto& binding : constraints) {
    const Vector1d dual_solution(gurobi_qcp_dual_solutions(*soc_count));
    (*soc_count)++;
    result->set_dual_solution(binding, dual_solution);
  }
}

void SetAllSecondOrderConeDualSolution(const MathematicalProgram& prog,
                                       GRBmodel* model,
                                       MathematicalProgramResult* result) {
  const int num_soc = prog.lorentz_cone_constraints().size() +
                      prog.rotated_lorentz_cone_constraints().size();
  Eigen::VectorXd gurobi_qcp_dual_solutions(num_soc);
  GRBgetdblattrarray(model, GRB_DBL_ATTR_QCPI, 0, num_soc,
                     gurobi_qcp_dual_solutions.data());

  int soc_count = 0;
  SetSecondOrderConeDualSolution(prog.lorentz_cone_constraints(),
                                 gurobi_qcp_dual_solutions, result, &soc_count);
  SetSecondOrderConeDualSolution(prog.rotated_lorentz_cone_constraints(),
                                 gurobi_qcp_dual_solutions, result, &soc_count);
}

// Utility to extract Gurobi solve status information into
// a struct to communicate to user callbacks.
GurobiSolver::SolveStatusInfo GetGurobiSolveStatus(void* cbdata, int where) {
  GurobiSolver::SolveStatusInfo solve_status;
  GRBcbget(cbdata, where, GRB_CB_RUNTIME, &(solve_status.reported_runtime));
  solve_status.current_objective = -1.0;
  GRBcbget(cbdata, where, GRB_CB_MIPNODE_OBJBST,
           &(solve_status.best_objective));
  GRBcbget(cbdata, where, GRB_CB_MIPNODE_OBJBND, &(solve_status.best_bound));
  GRBcbget(cbdata, where, GRB_CB_MIPNODE_SOLCNT,
           &(solve_status.feasible_solutions_count));
  double explored_node_count_double{};
  GRBcbget(cbdata, where, GRB_CB_MIPNODE_NODCNT, &explored_node_count_double);
  solve_status.explored_node_count = explored_node_count_double;
  return solve_status;
}

// Handles the Gurobi C callback function. The where values should match the
// value of the Gurobi where callback codes at
// https://docs.gurobi.com/projects/optimizer/en/12.0/reference/numericcodes/callbacks.html.
// When handling a particular callback where code, be very careful to handle the
// associated what callback codes correctly as this has caused bugs in the past.
int gurobi_callback(GRBmodel* model, void* cbdata, int where, void* usrdata) {
  GurobiCallbackInformation* callback_info =
      reinterpret_cast<GurobiCallbackInformation*>(usrdata);
  if (callback_info->mip_sol_callback == nullptr) {
    // Skip if no callback is provided.
    return 0;
  }
  switch (where) {
    case GRB_CB_POLLING: {
      return 0;
    }
    case GRB_CB_PRESOLVE: {
      return 0;
    }
    case GRB_CB_SIMPLEX: {
      return 0;
    }
    case GRB_CB_MIP: {
      return 0;
    }
    case GRB_CB_MIPSOL: {
      // Extract variable values from Gurobi, and set the current
      // solution of the MathematicalProgram to these values.
      int error = GRBcbget(cbdata, where, GRB_CB_MIPSOL_SOL,
                           callback_info->solver_sol_vector.data());
      if (error) {
        drake::log()->error("GRB error {} in MIPSol callback cbget: {}\n",
                            error, GRBgeterrormsg(GRBgetenv(model)));
        return 1;
      }
      SetProgramSolutionVector(callback_info->is_new_variable,
                               callback_info->solver_sol_vector,
                               &(callback_info->prog_sol_vector));
      callback_info->result->set_x_val(callback_info->prog_sol_vector);

      GurobiSolver::SolveStatusInfo solve_status =
          GetGurobiSolveStatus(cbdata, where);

      callback_info->mip_sol_callback(*(callback_info->prog), solve_status);
      return 0;
    }
    case GRB_CB_MIPNODE: {
      int sol_status;
      int error = GRBcbget(cbdata, where, GRB_CB_MIPNODE_STATUS, &sol_status);
      if (error) {
        drake::log()->error(
            "GRB error {} in MIPNode callback getting sol status: {}\n", error,
            GRBgeterrormsg(GRBgetenv(model)));
        return 1;
      }
      if (sol_status == GRB_OPTIMAL) {
        // Fetch the current node relaxation solution values.
        error = GRBcbget(cbdata, where, GRB_CB_MIPNODE_REL,
                         callback_info->solver_sol_vector.data());
        if (error) {
          drake::log()->error("GRB error {} in MIPNode callback cbget: {}\n",
                              error, GRBgeterrormsg(GRBgetenv(model)));
          return 1;
        }
        SetProgramSolutionVector(callback_info->is_new_variable,
                                 callback_info->solver_sol_vector,
                                 &(callback_info->prog_sol_vector));
        callback_info->result->set_x_val(callback_info->prog_sol_vector);

        GurobiSolver::SolveStatusInfo solve_status =
            GetGurobiSolveStatus(cbdata, where);

        Eigen::VectorXd vals;
        VectorXDecisionVariable vars;
        callback_info->mip_node_callback(*(callback_info->prog), solve_status,
                                         &vals, &vars);

        // The callback may return an assignment of some number of variables
        // as a new heuristic solution seed. If so, feed those back to Gurobi.
        if (vals.size() > 0) {
          std::vector<double> new_sol(callback_info->prog->num_vars(),
                                      GRB_UNDEFINED);
          for (int i = 0; i < vals.size(); i++) {
            double val = vals[i];
            int k = callback_info->prog->FindDecisionVariableIndex(vars[i]);
            new_sol[k] = val;
          }
          double objective_solution;
          error = GRBcbsolution(cbdata, new_sol.data(), &objective_solution);
          if (error) {
            drake::log()->error("GRB error {} in injection: {}\n", error,
                                GRBgeterrormsg(GRBgetenv(model)));
            return 1;
          }
        }
      }
      return 0;
    }
    case GRB_CB_MESSAGE: {
      return 0;
    }
    case GRB_CB_BARRIER: {
      return 0;
    }
    case GRB_CB_MULTIOBJ: {
      return 0;
    }
    case GRB_CB_IIS: {
      return 0;
    }
  }
  // This switch statement should be an exhaustive reference to the Gurobi what
  // codes.
  DRAKE_UNREACHABLE();
}

// Checks if the number of variables in the Gurobi model is as expected. This
// operation can be EXPENSIVE, since it requires calling GRBupdatemodel
// (Gurobi typically adopts lazy update, where it does not update the model
// until calling the optimize function).
// This function should only be used in DEBUG mode as a sanity check.
__attribute__((unused)) bool HasCorrectNumberOfVariables(
    GRBmodel* model, int num_vars_expected) {
  int error = GRBupdatemodel(model);
  if (error) return false;
  int num_vars{};
  error = GRBgetintattr(model, "NumVars", &num_vars);
  if (error) return false;
  return (num_vars == num_vars_expected);
}

/*
 * Add quadratic or linear costs to the optimization problem.
 */
int AddLinearAndQuadraticCosts(GRBmodel* model, double* pconstant_cost,
                               const MathematicalProgram& prog) {
  // Aggregates the quadratic costs and linear costs in the form
  // 0.5 * x' * Q_all * x + linear_term' * x.
  using std::abs;
  // record the non-zero entries in the cost 0.5*x'*Q*x + b'*x.
  std::vector<Eigen::Triplet<double>> Q_nonzero_coefs;
  std::vector<Eigen::Triplet<double>> b_nonzero_coefs;
  double& constant_cost = *pconstant_cost;
  constant_cost = 0;
  for (const auto& binding : prog.quadratic_costs()) {
    const auto& constraint = binding.evaluator();
    const int constraint_variable_dimension = binding.GetNumElements();
    const Eigen::MatrixXd& Q = constraint->Q();
    const Eigen::VectorXd& b = constraint->b();
    constant_cost += constraint->c();

    DRAKE_ASSERT(Q.rows() == constraint_variable_dimension);

    // constraint_variable_index[i] is the index of the i'th decision variable
    // binding.GetFlattendSolution(i).
    std::vector<int> constraint_variable_index(constraint_variable_dimension);

    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      constraint_variable_index[i] =
          prog.FindDecisionVariableIndex(binding.variables()(i));
    }

    for (int i = 0; i < Q.rows(); i++) {
      const double Qii = 0.5 * Q(i, i);
      if (Qii != 0) {
        Q_nonzero_coefs.push_back(Eigen::Triplet<double>(
            constraint_variable_index[i], constraint_variable_index[i], Qii));
      }
      for (int j = i + 1; j < Q.cols(); j++) {
        const double Qij = 0.5 * (Q(i, j) + Q(j, i));
        if (Qij != 0) {
          Q_nonzero_coefs.push_back(Eigen::Triplet<double>(
              constraint_variable_index[i], constraint_variable_index[j], Qij));
        }
      }
    }

    for (int i = 0; i < b.size(); i++) {
      if (b(i) != 0) {
        b_nonzero_coefs.push_back(
            Eigen::Triplet<double>(constraint_variable_index[i], 0, b(i)));
      }
    }
  }

  // Add linear cost in prog.linear_costs() to the aggregated cost.
  for (const auto& binding : prog.linear_costs()) {
    const auto& constraint = binding.evaluator();
    const auto& a = constraint->a();
    constant_cost += constraint->b();

    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      b_nonzero_coefs.push_back(Eigen::Triplet<double>(
          prog.FindDecisionVariableIndex(binding.variables()(i)), 0, a(i)));
    }
  }

  Eigen::SparseMatrix<double> Q_all(prog.num_vars(), prog.num_vars());
  Eigen::SparseMatrix<double> linear_terms(prog.num_vars(), 1);
  Q_all.setFromTriplets(Q_nonzero_coefs.begin(), Q_nonzero_coefs.end());
  linear_terms.setFromTriplets(b_nonzero_coefs.begin(), b_nonzero_coefs.end());

  std::vector<Eigen::Index> Q_all_row;
  std::vector<Eigen::Index> Q_all_col;
  std::vector<double> Q_all_val;
  drake::math::SparseMatrixToRowColumnValueVectors(Q_all, Q_all_row, Q_all_col,
                                                   Q_all_val);

  std::vector<int> Q_all_row_indices_int(Q_all_row.size());
  std::vector<int> Q_all_col_indices_int(Q_all_col.size());
  for (int i = 0; i < static_cast<int>(Q_all_row_indices_int.size()); i++) {
    Q_all_row_indices_int[i] = static_cast<int>(Q_all_row[i]);
    Q_all_col_indices_int[i] = static_cast<int>(Q_all_col[i]);
  }

  std::vector<Eigen::Index> linear_row;
  std::vector<Eigen::Index> linear_col;
  std::vector<double> linear_val;
  drake::math::SparseMatrixToRowColumnValueVectors(linear_terms, linear_row,
                                                   linear_col, linear_val);

  std::vector<int> linear_row_indices_int(linear_row.size());
  for (int i = 0; i < static_cast<int>(linear_row_indices_int.size()); i++) {
    linear_row_indices_int[i] = static_cast<int>(linear_row[i]);
  }

  const int QPtermsError = GRBaddqpterms(
      model, static_cast<int>(Q_all_row.size()), Q_all_row_indices_int.data(),
      Q_all_col_indices_int.data(), Q_all_val.data());
  if (QPtermsError) {
    return QPtermsError;
  }
  for (int i = 0; i < static_cast<int>(linear_row.size()); i++) {
    const int LinearTermError = GRBsetdblattrarray(
        model, "Obj", linear_row_indices_int[i], 1, linear_val.data() + i);
    if (LinearTermError) {
      return LinearTermError;
    }
  }
  // If loop completes, no errors exist so the value '0' must be returned.
  return 0;
}

// Add both LinearConstraints and LinearEqualityConstraints to gurobi
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
int ProcessLinearConstraints(
    GRBmodel* model, const MathematicalProgram& prog,
    int* num_gurobi_linear_constraints,
    std::unordered_map<Binding<Constraint>, int>* constraint_dual_start_row) {
  for (const auto& binding : prog.linear_equality_constraints()) {
    const auto& constraint = binding.evaluator();

    constraint_dual_start_row->emplace(binding, *num_gurobi_linear_constraints);

    const int error = internal::AddLinearConstraint(
        prog, model, constraint->get_sparse_A(), constraint->lower_bound(),
        constraint->upper_bound(), binding.variables(), true,
        num_gurobi_linear_constraints);
    if (error) {
      return error;
    }
  }

  for (const auto& binding : prog.linear_constraints()) {
    const auto& constraint = binding.evaluator();

    constraint_dual_start_row->emplace(binding, *num_gurobi_linear_constraints);

    const int error = internal::AddLinearConstraint(
        prog, model, constraint->get_sparse_A(), constraint->lower_bound(),
        constraint->upper_bound(), binding.variables(), false,
        num_gurobi_linear_constraints);
    if (error) {
      return error;
    }
  }

  // If loop completes, no errors exist so the value '0' must be returned.
  return 0;
}

template <typename T>
void SetOptionOrThrow(GRBenv* model_env, const std::string& option,
                      const T& val) {
  static_assert(std::is_same_v<T, int> || std::is_same_v<T, double> ||
                    std::is_same_v<T, std::string>,
                "Option values must be int, double, or string");

  // Set the parameter as requested, returning immediately in case of success.
  const char* actual_type;
  int error = 0;
  if constexpr (std::is_same_v<T, int>) {
    actual_type = "integer";
    error = GRBsetintparam(model_env, option.c_str(), val);
  } else if constexpr (std::is_same_v<T, double>) {
    actual_type = "floating-point";
    error = GRBsetdblparam(model_env, option.c_str(), val);
  } else if constexpr (std::is_same_v<T, std::string>) {
    actual_type = "string";
    error = GRBsetstrparam(model_env, option.c_str(), val.c_str());
  }
  if (!error) {
    return;
  }

  // Report range errors (i.e., the parameter name is known, but `val` is bad).
  if (error == GRB_ERROR_VALUE_OUT_OF_RANGE) {
    throw std::runtime_error(fmt::format(
        "GurobiSolver(): '{}' is outside the parameter {}'s valid range", val,
        option));
  }

  // In case of "unknown", it could either be truly unknown or else just the
  // wrong data type.
  if (error == GRB_ERROR_UNKNOWN_PARAMETER) {
    // For the expected param_type, we have:
    //   1: INT param
    //   2: DBL param
    //   3: STR param
    const int param_type = GRBgetparamtype(model_env, option.c_str());

    // If the user provided an int for a double param, treat it as a double
    // without any complaint. This is especially helpful for Python users.
    if constexpr (std::is_same_v<T, int>) {
      if (param_type == 2) {
        SetOptionOrThrow<double>(model_env, option, val);
        return;
      }
    }

    // Otherwise, identify all other cases of type-mismatches.
    const char* expected_type = nullptr;
    switch (param_type) {
      case 1: {
        expected_type = "integer";
        break;
      }
      case 2: {
        expected_type = "floating-point";
        break;
      }
      case 3: {
        expected_type = "string";
        break;
      }
    }
    if (expected_type != nullptr) {
      throw std::runtime_error(
          fmt::format("GurobiSolver(): parameter {} should be a {} not a {}",
                      option, expected_type, actual_type));
    }

    // Otherwise, it was truly unknown not just wrongly-typed.
    throw std::runtime_error(fmt::format(
        "GurobiSolver(): '{}' is an unknown parameter in Gurobi, check "
        "{}/parameters.html for allowable parameters",
        option, refman()));
  }

  // The error code should always be UNKNOWN_PARAMETER or VALUE_OUT_OF_RANGE,
  // but just in case we'll handle other errors with a fallback. This is
  // untested because it's thought to be unreachable in practice.
  throw std::runtime_error(fmt::format(
      "GurobiSolver(): error code {}, cannot set option '{}' to value '{}', "
      "check {}/parameters.html for all allowable options and values.",
      error, option, val, refman()));
}

void SetSolution(
    GRBmodel* model, GRBenv* model_env, const MathematicalProgram& prog,
    const std::vector<bool>& is_new_variable, int num_prog_vars, bool is_mip,
    int num_gurobi_linear_constraints, double constant_cost,
    const std::unordered_map<Binding<Constraint>, int>&
        constraint_dual_start_row,
    const std::unordered_map<Binding<BoundingBoxConstraint>,
                             std::pair<std::vector<int>, std::vector<int>>>&
        bb_con_dual_indices,
    MathematicalProgramResult* result, GurobiSolverDetails* solver_details) {
  int num_total_variables = is_new_variable.size();
  // Gurobi has solved not only for the decision variables in
  // MathematicalProgram prog, but also for any extra decision variables
  // that this GurobiSolver injected to craft certain constraints, such as
  // Lorentz cones.  We therefore filter out the optimized values for
  // injected variables, and report back values for the MathematicalProgram
  // variables only.
  // solver_sol_vector includes the potentially newly added variables, i.e.,
  // variables not in MathematicalProgram prog, but added to Gurobi by
  // GurobiSolver.
  // prog_sol_vector only includes the original variables in
  // MathematicalProgram prog.
  std::vector<double> solver_sol_vector(num_total_variables);
  GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, num_total_variables,
                     solver_sol_vector.data());
  Eigen::VectorXd prog_sol_vector(num_prog_vars);
  SetProgramSolutionVector(is_new_variable, solver_sol_vector,
                           &prog_sol_vector);
  result->set_x_val(prog_sol_vector);

  // If QCPDual is 0 and the program has quadratic constraints (including
  // both Lorentz cone and rotated Lorentz cone constraints), then the dual
  // variables are not computed.
  int qcp_dual;
  int error = GRBgetintparam(model_env, "QCPDual", &qcp_dual);
  DRAKE_DEMAND(!error);

  int num_q_constrs = 0;
  error = GRBgetintattr(model, "NumQConstrs", &num_q_constrs);
  DRAKE_DEMAND(!error);

  const bool compute_dual = !(num_q_constrs > 0 && qcp_dual == 0);

  // Set dual solutions.
  if (!is_mip && compute_dual) {
    // Gurobi only provides dual solution for continuous models.
    // Gurobi stores its dual solution for each variable bounds in "reduced
    // cost".
    std::vector<double> reduced_cost(num_total_variables);
    GRBgetdblattrarray(model, GRB_DBL_ATTR_RC, 0, num_total_variables,
                       reduced_cost.data());
    SetBoundingBoxDualSolution(prog, reduced_cost, bb_con_dual_indices, result);

    Eigen::VectorXd gurobi_dual_solutions =
        Eigen::VectorXd::Zero(num_gurobi_linear_constraints);
    GRBgetdblattrarray(model, GRB_DBL_ATTR_PI, 0, num_gurobi_linear_constraints,
                       gurobi_dual_solutions.data());
    SetLinearConstraintDualSolutions(prog, gurobi_dual_solutions,
                                     constraint_dual_start_row, result);

    SetAllSecondOrderConeDualSolution(prog, model, result);
  }

  // Obtain optimal cost.
  double optimal_cost = std::numeric_limits<double>::quiet_NaN();
  GRBgetdblattr(model, GRB_DBL_ATTR_OBJVAL, &optimal_cost);

  // Provide Gurobi's computed cost in addition to the constant cost.
  result->set_optimal_cost(optimal_cost + constant_cost);

  if (is_mip) {
    // The program wants to retrieve sub-optimal solutions
    int sol_count{0};
    GRBgetintattr(model, "SolCount", &sol_count);
    for (int solution_number = 0; solution_number < sol_count;
         ++solution_number) {
      error = GRBsetintparam(model_env, "SolutionNumber", solution_number);
      DRAKE_DEMAND(!error);
      double suboptimal_obj{1.0};
      error = GRBgetdblattrarray(model, "Xn", 0, num_total_variables,
                                 solver_sol_vector.data());
      DRAKE_DEMAND(!error);
      error = GRBgetdblattr(model, "PoolObjVal", &suboptimal_obj);
      DRAKE_DEMAND(!error);
      SetProgramSolutionVector(is_new_variable, solver_sol_vector,
                               &prog_sol_vector);
      result->AddSuboptimalSolution(suboptimal_obj, prog_sol_vector);
    }
    // If the problem is a mixed-integer optimization program, provide
    // Gurobi's lower bound.
    double lower_bound;
    error = GRBgetdblattr(model, GRB_DBL_ATTR_OBJBOUND, &lower_bound);
    if (error) {
      drake::log()->error("GRB error {} getting lower bound: {}\n", error,
                          GRBgeterrormsg(GRBgetenv(model)));
      solver_details->error_code = error;
    } else {
      solver_details->objective_bound = lower_bound;
    }
  }
}

std::optional<int> ParseInt(std::string_view s) {
  int result{};
  const char* begin = s.data();
  const char* end = s.data() + s.size();
  auto [past, ec] = std::from_chars(begin, end, result);
  if ((ec == std::errc()) && (past == end)) {
    return result;
  }
  return std::nullopt;
}
}  // namespace

bool GurobiSolver::is_available() {
  return true;
}

/*
 * Implements RAII for a Gurobi license / environment.
 */
class GurobiSolver::License {
 public:
  License() {
    if (!GurobiSolver::is_enabled()) {
      throw std::runtime_error(
          "Could not locate Gurobi license key file because GRB_LICENSE_FILE "
          "environment variable was not set.");
    }
    if (const char* filename = std::getenv("GRB_LICENSE_FILE")) {
      // For unit testing, we employ a hack to keep env_ uninitialized so that
      // we don't need a valid license file.
      if (std::string_view{filename}.find("DRAKE_UNIT_TEST_NO_LICENSE") !=
          std::string_view::npos) {
        return;
      }
    }
    const int num_tries = 3;
    int grb_load_env_error = 1;
    for (int i = 0; grb_load_env_error && i < num_tries; ++i) {
      grb_load_env_error = GRBloadenv(&env_, nullptr);
    }
    if (grb_load_env_error) {
      const char* grb_msg = GRBgeterrormsg(env_);
      throw std::runtime_error(
          "Could not create Gurobi environment because "
          "Gurobi returned code " +
          std::to_string(grb_load_env_error) + " with message \"" + grb_msg +
          "\".");
    }
    DRAKE_DEMAND(env_ != nullptr);
  }

  ~License() {
    GRBfreeenv(env_);
    env_ = nullptr;
  }

  GRBenv* GurobiEnv() { return env_; }

 private:
  GRBenv* env_ = nullptr;
};

namespace {
bool IsGrbLicenseFileLocalHost() {
  // We use the existence of the string HOSTID in the license file as
  // confirmation that the license is associated with the local host.
  const char* grb_license_file = std::getenv("GRB_LICENSE_FILE");
  if (grb_license_file == nullptr) {
    return false;
  }
  const std::optional<std::string> contents = ReadFile(grb_license_file);
  if (!contents) {
    return false;
  }
  return contents->find("HOSTID") != std::string::npos;
}
}  // namespace

std::shared_ptr<GurobiSolver::License> GurobiSolver::AcquireLicense() {
  // Gurobi recommends acquiring the license only once per program to avoid
  // overhead from acquiring the license (and console spew for academic license
  // users; see #19657). However, if users are using a shared network license
  // from a limited pool, then we risk them checking out the license and not
  // giving it back (e.g., if they are working in a jupyter notebook). As a
  // compromise, we extend license beyond the lifetime of the GurobiSolver iff
  // we can confirm that the license is associated with the local host.
  //
  // The first time the anyone calls GurobiSolver::AcquireLicense, we check
  // whether the license is local. If yes, the local_host_holder keeps the
  // license's use_count lower bounded to 1. If no, the local_hold_holder is
  // null and the usual GetScopedSingleton workflow applies.
  static never_destroyed<std::shared_ptr<void>> local_host_holder{
      IsGrbLicenseFileLocalHost() ? GetScopedSingleton<GurobiSolver::License>()
                                  : nullptr};
  return GetScopedSingleton<GurobiSolver::License>();
}

// TODO(hongkai.dai@tri.global): break this large DoSolve function to smaller
// ones.
void GurobiSolver::DoSolve2(const MathematicalProgram& prog,
                            const Eigen::VectorXd& initial_guess,
                            internal::SpecificOptions* options,
                            MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
        "GurobiSolver doesn't support the feature of variable scaling.");
  }

  if (!license_) {
    license_ = AcquireLicense();
  }
  GRBenv* env = license_->GurobiEnv();

  const int num_prog_vars = prog.num_vars();
  int num_gurobi_vars = num_prog_vars;
  // Potentially Gurobi can add variables on top of the variables in
  // MathematicalProgram prog.
  // is_new_variable[i] is true if the i'th variable in Gurobi environment is
  // not stored in MathematicalProgram, but added by the GurobiSolver.
  // For example, for Lorentz cone and rotated Lorentz cone constraint,to impose
  // that A*x+b lies in the (rotated) Lorentz cone, we add decision variable z
  // to Gurobi, defined as z = A*x + b.
  // The size of is_new_variable should increase if we add new decision
  // variables to Gurobi model.
  // The invariant is
  // EXPECT_TRUE(HasCorrectNumberOfVariables(model, is_new_variables.size()))
  std::vector<bool> is_new_variable(num_prog_vars, false);

  std::vector<char> gurobi_var_type(num_prog_vars);
  bool is_mip{false};
  for (int i = 0; i < num_prog_vars; ++i) {
    switch (prog.decision_variable(i).get_type()) {
      case MathematicalProgram::VarType::CONTINUOUS:
        gurobi_var_type[i] = GRB_CONTINUOUS;
        break;
      case MathematicalProgram::VarType::BINARY:
        gurobi_var_type[i] = GRB_BINARY;
        is_mip = true;
        break;
      case MathematicalProgram::VarType::INTEGER:
        gurobi_var_type[i] = GRB_INTEGER;
        is_mip = true;
        break;
      case MathematicalProgram::VarType::BOOLEAN:
        throw std::runtime_error(
            "Boolean variables should not be used with Gurobi solver.");
      case MathematicalProgram::VarType::RANDOM_UNIFORM:
      case MathematicalProgram::VarType::RANDOM_GAUSSIAN:
      case MathematicalProgram::VarType::RANDOM_EXPONENTIAL:
        throw std::runtime_error(
            "Random variables should not be used with Gurobi solver.");
    }
  }

  std::vector<double> xlow;
  std::vector<double> xupp;
  AggregateBoundingBoxConstraints(prog, &xlow, &xupp);

  // bb_con_dual_indices[constraint] returns the pair (lower_dual_indices,
  // upper_dual_indices), where lower_dual_indices are the indices of the dual
  // variables associated with the lower bound side (x >= lower) of the bounding
  // box constraint; upper_dual_indices are the indices of the dual variables
  // associated with the upper bound side (x <= upper) of the bounding box
  // constraint. If the index is -1, then it means there is not an associated
  // dual variable (because that row in the bounding box constraint can never
  // be active, as there are other bounding box constraint that imposes tighter
  // bounds on that variable).
  std::unordered_map<Binding<BoundingBoxConstraint>,
                     std::pair<std::vector<int>, std::vector<int>>>
      bb_con_dual_indices;
  // Now loop over all of the bounding box constraints again, if a bounding box
  // constraint has its lower or upper bound equals to xlow or xupp, then that
  // bounding box constraint has an associated dual variable.
  for (const auto& binding : prog.bounding_box_constraints()) {
    const auto& constraint = binding.evaluator();
    const Eigen::VectorXd& lower_bound = constraint->lower_bound();
    const Eigen::VectorXd& upper_bound = constraint->upper_bound();

    std::vector<int> upper_dual_indices(constraint->num_vars(), -1);
    std::vector<int> lower_dual_indices(constraint->num_vars(), -1);
    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      const int idx = prog.FindDecisionVariableIndex(binding.variables()(k));
      if (xlow[idx] == lower_bound(k)) {
        lower_dual_indices[k] = idx;
      }
      if (xupp[idx] == upper_bound(k)) {
        upper_dual_indices[k] = idx;
      }
    }
    bb_con_dual_indices.emplace(
        binding, std::make_pair(lower_dual_indices, upper_dual_indices));
  }

  // constraint_dual_start_row[constraint] returns the starting index of the
  // dual variable corresponding to this constraint
  std::unordered_map<Binding<Constraint>, int> constraint_dual_start_row;

  // Our second order cone constraints imposes A*x+b lies within the (rotated)
  // Lorentz cone. Unfortunately Gurobi only supports a vector z lying within
  // the (rotated) Lorentz cone. So we create new variable z, with the
  // constraint z - A*x = b and z being within the (rotated) Lorentz cone.
  // Here lorentz_cone_new_varaible_indices and
  // rotated_lorentz_cone_new_variable_indices
  // record the indices of the newly created variable z in the Gurobi program.
  std::vector<std::vector<int>> lorentz_cone_new_variable_indices;
  internal::AddSecondOrderConeVariables(
      prog.lorentz_cone_constraints(), &is_new_variable, &num_gurobi_vars,
      &lorentz_cone_new_variable_indices, &gurobi_var_type, &xlow, &xupp);

  std::vector<std::vector<int>> rotated_lorentz_cone_new_variable_indices;
  internal::AddSecondOrderConeVariables(
      prog.rotated_lorentz_cone_constraints(), &is_new_variable,
      &num_gurobi_vars, &rotated_lorentz_cone_new_variable_indices,
      &gurobi_var_type, &xlow, &xupp);

  std::vector<std::vector<int>> l2norm_costs_lorentz_cone_variable_indices;
  internal::AddL2NormCostVariables(prog.l2norm_costs(), &is_new_variable,
                                   &num_gurobi_vars,
                                   &l2norm_costs_lorentz_cone_variable_indices,
                                   &gurobi_var_type, &xlow, &xupp);

  GRBmodel* model = nullptr;
  GRBnewmodel(env, &model, "gurobi_model", num_gurobi_vars, nullptr, &xlow[0],
              &xupp[0], gurobi_var_type.data(), nullptr);
  ScopeExit guard([model]() {
    GRBfreemodel(model);
  });

  int error = 0;
  double constant_cost = 0;
  if (!error) {
    error = AddLinearAndQuadraticCosts(model, &constant_cost, prog);
  }

  int num_gurobi_linear_constraints = 0;
  if (!error) {
    error = internal::AddL2NormCosts(prog,
                                     l2norm_costs_lorentz_cone_variable_indices,
                                     model, &num_gurobi_linear_constraints);
  }

  if (!error) {
    error =
        ProcessLinearConstraints(model, prog, &num_gurobi_linear_constraints,
                                 &constraint_dual_start_row);
  }

  // Add Lorentz cone constraints.
  if (!error) {
    error = internal::AddSecondOrderConeConstraints(
        prog, prog.lorentz_cone_constraints(),
        lorentz_cone_new_variable_indices, model,
        &num_gurobi_linear_constraints);
  }

  // Add rotated Lorentz cone constraints.
  if (!error) {
    error = internal::AddSecondOrderConeConstraints(
        prog, prog.rotated_lorentz_cone_constraints(),
        rotated_lorentz_cone_new_variable_indices, model,
        &num_gurobi_linear_constraints);
  }

  DRAKE_ASSERT(HasCorrectNumberOfVariables(model, is_new_variable.size()));

  // The new model gets a copy of the Gurobi environment, so when we set
  // parameters, we have to be sure to set them on the model's environment,
  // not the global Gurobi environment.
  // See: FAQ #11: https://www.gurobi.com/support/faqs
  // Note that it is not necessary to free this environment; rather,
  // we just have to call GRBfreemodel(model).
  GRBenv* model_env = GRBgetenv(model);
  DRAKE_DEMAND(model_env != nullptr);

  // A couple options don't use the standard GRBset{...}param API.
  const bool compute_iis = [&options]() {
    const int value = options->Pop<int>("GRBcomputeIIS").value_or(0);
    if (!(value == 0 || value == 1)) {
      throw std::runtime_error(fmt::format(
          "GurobiSolver(): option GRBcomputeIIS should be either 0 or 1, but "
          "is incorrectly set to {}",
          value));
    }
    return value;
  }();
  const std::optional<std::string> grb_write =
      options->template Pop<std::string>("GRBwrite");

  // Convert the common options into their Gurobi flavor.
  options->Respell([](const auto& common, auto* respelled) {
    respelled->emplace("LogToConsole", common.print_to_console ? 1 : 0);
    if (!common.print_file_name.empty()) {
      respelled->emplace("LogFile", common.print_file_name);
    }
    // Here's our priority order for selecting the number of threads:
    // - Gurobi-specific solver option "Threads" (already taken care of by the
    ///  trumping logic inside SpecificOptions).
    // - The value of CommonSolverOptions::kMaxThreads if set.
    // - GUROBI_NUM_THREADS environment variable.
    // - Drake's maximum parallelism.
    std::optional<int> num_threads = common.max_threads;
    if (!num_threads.has_value()) {
      // If unset, use GUROBI_NUM_THREADS. We attempt to read the value of
      // GUROBI_NUM_THREADS and warn the user if it is not parseable.
      if (char* num_threads_str = std::getenv("GUROBI_NUM_THREADS")) {
        num_threads = ParseInt(num_threads_str);
        if (num_threads.has_value()) {
          log()->debug("Using GUROBI_NUM_THREADS={}", *num_threads);
        } else {
          static const logging::Warn log_once(
              "Ignoring unparseable value '{}' for GUROBI_NUM_THREADS",
              num_threads_str);
        }
      }
    }
    if (!num_threads.has_value()) {
      // If unset, use max parallelism.
      num_threads = Parallelism::Max().num_threads();
    }
    DRAKE_DEMAND(num_threads.has_value());
    respelled->emplace("Threads", *num_threads);
  });

  // Copy the remaining options into model_env. Set the logging level first, so
  // that changes to any of the other options are uniformly logged (or not).
  SetOptionOrThrow(model_env, "LogToConsole",
                   options->Pop<int>("LogToConsole").value_or(0));
  options->CopyToCallbacks(
      [&model_env](const std::string& key, double value) {
        SetOptionOrThrow(model_env, key, value);
      },
      [&model_env](const std::string& key, int value) {
        SetOptionOrThrow(model_env, key, value);
      },
      [&model_env](const std::string& key, const std::string& value) {
        SetOptionOrThrow(model_env, key, value);
      });

  for (int i = 0; i < static_cast<int>(prog.num_vars()); ++i) {
    if (!error && !std::isnan(initial_guess(i))) {
      error = GRBsetdblattrelement(model, "Start", i, initial_guess(i));
    }
  }

  GRBupdatemodel(model);

  int num_gurobi_linear_constraints_expected;
  GRBgetintattr(model, GRB_INT_ATTR_NUMCONSTRS,
                &num_gurobi_linear_constraints_expected);
  DRAKE_DEMAND(num_gurobi_linear_constraints ==
               num_gurobi_linear_constraints_expected);

  // If we have been supplied a callback,
  // register it with Gurobi.
  // We initialize callback_info outside of the if() scope
  // so that it persists until after GRBoptimize() has been
  // called and completed. We need this struct to survive
  // throughout the solve process.
  GurobiCallbackInformation callback_info;
  if (mip_node_callback_ != nullptr || mip_sol_callback_ != nullptr) {
    callback_info.prog = &prog;
    callback_info.is_new_variable = is_new_variable;
    callback_info.solver_sol_vector.resize(is_new_variable.size());
    callback_info.prog_sol_vector.resize(num_prog_vars);
    callback_info.mip_node_callback = mip_node_callback_;
    callback_info.mip_sol_callback = mip_sol_callback_;
    callback_info.result = result;
    if (!error) {
      error = GRBsetcallbackfunc(model, &gurobi_callback, &callback_info);
    }
  }

  if (!error) {
    error = GRBoptimize(model);
  }

  if (!error) {
    if (compute_iis) {
      int optimstatus = 0;
      GRBgetintattr(model, GRB_INT_ATTR_STATUS, &optimstatus);
      if (optimstatus == GRB_INF_OR_UNBD || optimstatus == GRB_INFEASIBLE) {
        // Only compute IIS when the problem is infeasible.
        error = GRBcomputeIIS(model);
      }
    }
  }
  if (!error) {
    if (grb_write.has_value() && (grb_write->size() > 0)) {
      error = GRBwrite(model, grb_write.value().c_str());
      if (error) {
        const std::string gurobi_version =
            fmt::format("{}.{}", GRB_VERSION_MAJOR, GRB_VERSION_MINOR);
        throw std::runtime_error(fmt::format(
            "GurobiSolver(): setting GRBwrite to {}, this is not supported. "
            "Check {}/python/model.html#Write for more details.",
            grb_write.value(), refman()));
      }
    }
  }

  SolutionResult solution_result = SolutionResult::kSolverSpecificError;

  GurobiSolverDetails& solver_details =
      result->SetSolverDetailsType<GurobiSolverDetails>();

  if (error) {
    solution_result = SolutionResult::kInvalidInput;
    drake::log()->info("Gurobi returns code {}, with message \"{}\".\n", error,
                       GRBgeterrormsg(env));
    solver_details.error_code = error;
  } else {
    // Always set the primal and dual solution for any non-error gurobi
    // status.
    SetSolution(model, model_env, prog, is_new_variable, num_prog_vars, is_mip,
                num_gurobi_linear_constraints, constant_cost,
                constraint_dual_start_row, bb_con_dual_indices, result,
                &solver_details);
    int optimstatus = 0;
    GRBgetintattr(model, GRB_INT_ATTR_STATUS, &optimstatus);

    solver_details.optimization_status = optimstatus;

    if (optimstatus != GRB_OPTIMAL && optimstatus != GRB_SUBOPTIMAL) {
      switch (optimstatus) {
        case GRB_INF_OR_UNBD: {
          solution_result = SolutionResult::kInfeasibleOrUnbounded;
          break;
        }
        case GRB_UNBOUNDED: {
          result->set_optimal_cost(MathematicalProgram::kUnboundedCost);
          solution_result = SolutionResult::kUnbounded;
          break;
        }
        case GRB_INFEASIBLE: {
          result->set_optimal_cost(MathematicalProgram::kGlobalInfeasibleCost);
          solution_result = SolutionResult::kInfeasibleConstraints;
          break;
        }
      }
    } else {
      solution_result = SolutionResult::kSolutionFound;
    }
  }

  error = GRBgetdblattr(model, GRB_DBL_ATTR_RUNTIME,
                        &(solver_details.optimizer_time));
  if (error && !solver_details.error_code) {
    // Only overwrite the error code if no error happened before getting the
    // runtime.
    solver_details.error_code = error;
  }

  result->set_solution_result(solution_result);
}

}  // namespace solvers
}  // namespace drake

#pragma GCC diagnostic pop  // "-Wunused-parameter"
