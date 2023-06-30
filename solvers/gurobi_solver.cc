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
#include "drake/common/scope_exit.h"
#include "drake/common/scoped_singleton.h"
#include "drake/common/text_logging.h"
#include "drake/math/eigen_sparse_triplet.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

// TODO(hongkai.dai): GurobiSolver class should store data member such as
// GRB_model, GRB_env, is_new_variables, etc.
namespace drake {
namespace solvers {
namespace {

// Returns the (base) URL for Gurobi's online reference manual.
std::string refman() {
  return fmt::format("https://www.gurobi.com/documentation/{}.{}/refman",
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

int gurobi_callback(GRBmodel* model, void* cbdata, int where, void* usrdata) {
  GurobiCallbackInformation* callback_info =
      reinterpret_cast<GurobiCallbackInformation*>(usrdata);

  if (where == GRB_CB_POLLING) {
  } else if (where == GRB_CB_PRESOLVE) {
  } else if (where == GRB_CB_SIMPLEX) {
  } else if (where == GRB_CB_MIP) {
  } else if (where == GRB_CB_MIPSOL &&
             callback_info->mip_sol_callback != nullptr) {
    // Extract variable values from Gurobi, and set the current
    // solution of the MathematicalProgram to these values.
    int error = GRBcbget(cbdata, where, GRB_CB_MIPSOL_SOL,
                         callback_info->solver_sol_vector.data());
    if (error) {
      drake::log()->error("GRB error {} in MIPSol callback cbget: {}\n", error,
                          GRBgeterrormsg(GRBgetenv(model)));
      return 0;
    }
    SetProgramSolutionVector(callback_info->is_new_variable,
                             callback_info->solver_sol_vector,
                             &(callback_info->prog_sol_vector));
    callback_info->result->set_x_val(callback_info->prog_sol_vector);

    GurobiSolver::SolveStatusInfo solve_status =
        GetGurobiSolveStatus(cbdata, where);

    callback_info->mip_sol_callback(*(callback_info->prog), solve_status);

  } else if (where == GRB_CB_MIPNODE &&
             callback_info->mip_node_callback != nullptr) {
    int sol_status;
    int error = GRBcbget(cbdata, where, GRB_CB_MIPNODE_STATUS, &sol_status);
    if (error) {
      drake::log()->error(
          "GRB error {} in MIPNode callback getting sol status: {}\n", error,
          GRBgeterrormsg(GRBgetenv(model)));
      return 0;
    } else if (sol_status == GRB_OPTIMAL) {
      // Extract variable values from Gurobi, and set the current
      // solution of the MathematicalProgram to these values.
      error = GRBcbget(cbdata, where, GRB_CB_MIPSOL_SOL,
                       callback_info->solver_sol_vector.data());
      if (error) {
        drake::log()->error("GRB error {} in MIPSol callback cbget: {}\n",
                            error, GRBgeterrormsg(GRBgetenv(model)));
        return 0;
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
        }
      }
    }
  } else if (where == GRB_CB_BARRIER) {
  } else if (where == GRB_CB_MESSAGE) {
  }
  return 0;
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

// Adds a constraint of one of the following forms :
// lb ≤ A*x ≤ ub
// or
// A*x == lb
//
// @param is_equality True if the imposed constraint is
// A*x == lb, false otherwise.
// @param[in, out] num_gurobi_linear_constraints The number of linear
// constraints stored in the gurobi model.
// @return error as an integer. The full set of error values are
// described here :
// https://www.gurobi.com/documentation/9.5/refman/error_codes.html
// This function assumes `vars` doesn't contain duplicate variables.
int AddLinearConstraintNoDuplication(
    const MathematicalProgram& prog, GRBmodel* model,
    const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub, const VectorXDecisionVariable& vars,
    bool is_equality, int* num_gurobi_linear_constraints) {
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row_major = A;

  const std::vector<int> var_index = prog.FindDecisionVariableIndices(vars);

  // If this linear constraint is an equality constraint, we know that we can
  // pass in A * vars = lb directly to Gurobi.
  if (is_equality) {
    std::vector<int> nonzero_col_index;
    nonzero_col_index.reserve(A_row_major.nonZeros());
    std::vector<char> sense(A.rows(), GRB_EQUAL);
    for (int i = 0; i < A.nonZeros(); ++i) {
      nonzero_col_index.push_back(
          var_index[*(A_row_major.innerIndexPtr() + i)]);
    }
    *num_gurobi_linear_constraints += A.rows();
    int error =
        GRBaddconstrs(model, A_row_major.rows(), A_row_major.nonZeros(),
                      A_row_major.outerIndexPtr(), nonzero_col_index.data(),
                      A_row_major.valuePtr(), sense.data(),
                      const_cast<double*>(lb.data()), nullptr);
    return error;
  }

  // Now handle the inequality constraints.
  // Each row of linear constraint in Gurobi is in the form
  // aᵀx ≤ b or aᵀx ≥ b or aᵀx=b, namely it doesn't accept imposing both the
  // lower and the upper bound for a linear expression in one row. So for
  // the constraint lb(i) <= A.row(i).dot(vars) <= ub(i), there are 4 situations
  // 1. If both lb(i) and ub(i) are infinity, then we don't add any constraints
  // to Gurobi.
  // 2. If lb(i) is finite and ub(i) is infinity, then we add one constraint
  // A.row(i).dot(vars) >= lb(i) to Gurobi.
  // 3. If ub(i) is finite but lb(i) is -infinity, then we add one constraint
  // A.row(i).dot(vars) <= ub(i) to Gurobi.
  // 4. If both lb(i) and ub(i) are finite, then we add two constraints
  // A.row(i).dot(vars) >= lb(i) and A.row(i).dot(vars) <= ub(i) to Gurobi.
  // As a result, we add the constraint A_gurobi * vars <= rhs to Gurobi.

  // Each row of A introduces at most two constraints in Gurobi, so we reserve 2
  // * A.rows().
  std::vector<double> rhs;
  rhs.reserve(A.rows() * 2);
  std::vector<char> sense;
  sense.reserve(A.rows() * 2);

  // The matrix A_gurobi is stored in Compressed Sparse Row (CSR) format, using
  // three vectors cbeg, cind and cval. Please refer to
  // https://www.gurobi.com/documentation/9.5/refman/c_addconstrs.html for the
  // meaning of these three vectors. The non-zero entries in the i'th row of
  // A_gurobi is stored in the chunk cind[cbeg[i]:cbeg[i+1]] and
  // cval[cbeg[i]:cbeg[i+1]]
  std::vector<int> cbeg;
  cbeg.reserve(A.rows() * 2 + 1);
  cbeg.push_back(0);
  std::vector<int> cind;
  cind.reserve(A.nonZeros() * 2);
  std::vector<double> cval;
  cval.reserve(A.nonZeros() * 2);

  int A_gurobi_rows = 0;

  // Add A_row_major.row(i) * vars ≥ bound (or ≤ bound) to the CSR format cbeg,
  // cind, cva, also update rhs, sense and A_gurobi_rows.
  auto add_gurobi_row = [&A_row_major, &var_index, &cbeg, &rhs, &sense, &cind,
                         &cval,
                         &A_gurobi_rows](int i, double bound, char row_sense) {
    cbeg.push_back(cbeg.back() + *(A_row_major.outerIndexPtr() + i + 1) -
                   *(A_row_major.outerIndexPtr() + i));
    rhs.push_back(bound);
    sense.push_back(row_sense);
    for (int j = *(A_row_major.outerIndexPtr() + i);
         j < *(A_row_major.outerIndexPtr() + i + 1); ++j) {
      cind.push_back(var_index[*(A_row_major.innerIndexPtr() + j)]);
      cval.push_back(*(A_row_major.valuePtr() + j));
    }
    A_gurobi_rows++;
  };

  for (int i = 0; i < A_row_major.rows(); ++i) {
    if (!std::isinf(lb(i))) {
      // Add A_row_major.row(i) * vars >= lb(i)
      add_gurobi_row(i, lb(i), GRB_GREATER_EQUAL);
    }
    if (!std::isinf(ub(i))) {
      // Add A_row_major.row(i) * vars <= ub(i)
      add_gurobi_row(i, ub(i), GRB_LESS_EQUAL);
    }
  }
  *num_gurobi_linear_constraints += A_gurobi_rows;
  int error =
      GRBaddconstrs(model, A_gurobi_rows, cbeg.back(), cbeg.data(), cind.data(),
                    cval.data(), sense.data(), rhs.data(), nullptr);

  return error;
}

int AddLinearConstraint(const MathematicalProgram& prog, GRBmodel* model,
                        const Eigen::SparseMatrix<double>& A,
                        const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                        const VectorXDecisionVariable& vars, bool is_equality,
                        int* num_gurobi_linear_constraints) {
  const symbolic::Variables vars_set(vars);
  if (static_cast<int>(vars_set.size()) == vars.rows()) {
    return AddLinearConstraintNoDuplication(prog, model, A, lb, ub, vars,
                                            is_equality,
                                            num_gurobi_linear_constraints);
  } else {
    Eigen::SparseMatrix<double> A_new;
    VectorX<symbolic::Variable> vars_new;
    AggregateDuplicateVariables(A, vars, &A_new, &vars_new);
    return AddLinearConstraintNoDuplication(prog, model, A_new, lb, ub,
                                            vars_new, is_equality,
                                            num_gurobi_linear_constraints);
  }
}

/*
 * Add (rotated) Lorentz cone constraints, that z = A*x+b is in the (rotated)
 * Lorentz cone.
 * A vector z is in the Lorentz cone, if
 * z(0) >= sqrt(z(1)^2 + ... + z(N-1)^2)
 * A vector z is in the rotated Lorentz cone, if
 * z(0)*z(1) >= z(2)^2 + ... + z(N-1)^2
 * z(0) >= 0, z(1) >= 0
 * @tparam C  A constraint type, either LorentzConeConstraint or
 * RotatedLorentzConeConstraint.
 * @param second_order_cone_constraints  A vector of Binding objects, containing
 * either Lorentz cone constraints, or rotated Lorentz cone constraints.
 * @param second_order_cone_new_variable_indices. The indices of variable z in
 * the Gurobi model.
 * @param model The Gurobi model.
 * @param[in, out] num_gurobi_linear_constraints The number of linear
 * constraints stored in the gurobi model.
 */
template <typename C>
int AddSecondOrderConeConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& second_order_cone_constraints,
    const std::vector<std::vector<int>>& second_order_cone_new_variable_indices,
    GRBmodel* model, int* num_gurobi_linear_constraints) {
  static_assert(
      std::is_same_v<C, LorentzConeConstraint> ||
          std::is_same_v<C, RotatedLorentzConeConstraint>,
      "Expects either LorentzConeConstraint or RotatedLorentzConeConstraint");
  bool is_rotated_cone = std::is_same_v<C, RotatedLorentzConeConstraint>;

  DRAKE_ASSERT(second_order_cone_constraints.size() ==
               second_order_cone_new_variable_indices.size());
  int second_order_cone_count = 0;
  int num_gurobi_vars;
  int error = GRBgetintattr(model, "NumVars", &num_gurobi_vars);
  DRAKE_ASSERT(!error);
  for (const auto& binding : second_order_cone_constraints) {
    const auto& A = binding.evaluator()->A();
    const auto& b = binding.evaluator()->b();

    int num_x = A.cols();
    int num_z = A.rows();

    // Add the constraint z - A*x = b
    // xz_indices records the indices of [x; z] in Gurobi.
    std::vector<int> xz_indices(num_x + num_z, 0);

    for (int i = 0; i < num_x; ++i) {
      xz_indices[i] = prog.FindDecisionVariableIndex(binding.variables()(i));
    }
    for (int i = 0; i < num_z; ++i) {
      xz_indices[num_x + i] =
          second_order_cone_new_variable_indices[second_order_cone_count][i];
    }
    // z - A*x will be written as M * [x; z], where M = [-A I].
    // Gurobi expects M in compressed sparse row format, so we will first find
    // out the non-zero entries in each row of M.
    std::vector<Eigen::Triplet<double>> M_triplets;
    M_triplets.reserve(A.nonZeros() + num_z);
    for (int i = 0; i < A.outerSize(); ++i) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
        M_triplets.emplace_back(it.row(), xz_indices[it.col()], -it.value());
      }
    }
    for (int i = 0; i < num_z; ++i) {
      M_triplets.emplace_back(i, xz_indices[num_x + i], 1);
    }

    Eigen::SparseMatrix<double, Eigen::RowMajor> M(num_z, num_gurobi_vars);
    // Eigen::SparseMatrix::setFromTriplets will automatically group the sum of
    // the values in M_triplets that correspond to the same entry in the sparse
    // matrix.
    M.setFromTriplets(M_triplets.begin(), M_triplets.end());

    std::vector<char> sense(num_z, GRB_EQUAL);

    error = GRBaddconstrs(model, num_z, M.nonZeros(), M.outerIndexPtr(),
                          M.innerIndexPtr(), M.valuePtr(), sense.data(),
                          const_cast<double*>(b.data()), nullptr);
    DRAKE_ASSERT(!error);
    *num_gurobi_linear_constraints += num_z;

    // Gurobi uses a matrix Q to differentiate Lorentz cone and rotated Lorentz
    // cone constraint.
    // https://www.gurobi.com/documentation/9.5/refman/c_grbaddqconstr.html
    // For Lorentz cone constraint,
    // Q = [-1 0 0 ... 0]
    //     [ 0 1 0 ... 0]
    //     [ 0 0 1 ... 0]
    //          ...
    //     [ 0 0 0 ... 1]
    // namely Q = diag([-1; 1; 1; ...; 1], so
    // z' * Q * z = z(1)^2 + ... + z(n-1)^2 - z(0)^2.
    // For rotated Lorentz cone constraint
    // Q = [0 -1 0 0 ... 0]
    //     [0  0 0 0 ... 0]
    //     [0  0 1 0 ... 0]
    //     [0  0 0 1 ... 0]
    //           ...
    //     [0  0 0 0 ... 1]
    // so z' * Q * z = z(2)^2 + ... + z(n-1)^2 - z(0) * z(1).
    // We will store Q in a sparse format.
    // qrow stores the row    indices of the non-zero entries of Q.
    // qcol stores the column indices of the non-zero entries of Q.
    // qval stores the value          of the non-zero entries of Q.
    size_t num_Q_nonzero = is_rotated_cone ? num_z - 1 : num_z;
    std::vector<int> qrow(num_Q_nonzero);
    std::vector<int> qcol(num_Q_nonzero);
    std::vector<double> qval(num_Q_nonzero);
    for (int i = 0; i < num_z - 2; ++i) {
      int zi_index =
          second_order_cone_new_variable_indices[second_order_cone_count]
                                                [i + 2];
      qrow[i] = zi_index;
      qcol[i] = zi_index;
      qval[i] = 1.0;
    }
    int z0_index =
        second_order_cone_new_variable_indices[second_order_cone_count][0];
    int z1_index =
        second_order_cone_new_variable_indices[second_order_cone_count][1];
    if (is_rotated_cone) {
      qrow[num_z - 2] = z0_index;
      qcol[num_z - 2] = z1_index;
      qval[num_z - 2] = -1;
    } else {
      qrow[num_z - 2] = z0_index;
      qcol[num_z - 2] = z0_index;
      qval[num_z - 2] = -1;
      qrow[num_z - 1] = z1_index;
      qcol[num_z - 1] = z1_index;
      qval[num_z - 1] = 1;
    }
    error =
        GRBaddqconstr(model, 0, nullptr, nullptr, num_Q_nonzero, qrow.data(),
                      qcol.data(), qval.data(), GRB_LESS_EQUAL, 0.0, NULL);
    if (error) {
      return error;
    }
    ++second_order_cone_count;
  }
  return 0;
}

/*
 * Add quadratic or linear costs to the optimization problem.
 */
int AddCosts(GRBmodel* model, double* pconstant_cost,
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

    const int error = AddLinearConstraint(
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

    const int error = AddLinearConstraint(
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

// For Lorentz and rotated Lorentz cone constraints
// Ax + b in (rotated) Lorentz cone, we will introduce new variables z as
// z = Ax+b
// z in (rotated) Lorentz cone.
// So add the new variable z before constructing the Gurobi model, as
// recommended by the Gurobi manual, to add all decision variables at once
// when constructing the problem.
// @param second_order_cone_variable_indices
// second_order_cone_variable_indices[i]
// contains the indices of the newly added variable z for the i'th second order
// cone in @p second_order_cones[i].
// @p tparam C Either LorentzConeConstraint or RotatedLorentzConeConstraint.
// TODO(hongkai.dai): rewrite this function not templated on Binding, when
// Binding class is moved out from MathematicalProgram as a public class.
// @param second_order_cones A vector of bindings, containing either Lorentz
// cone constraint, or rotated Lorentz cone constraint.
// @param is_new_variable is_new_variable[i] is true if the i'th variable in
// Gurobi model is not included in MathematicalProgram.
// @param num_gurobi_vars Number of variables in Gurobi model.
// @param second_order_cone_variable_indices
// second_order_cone_variable_indices[i]
// contains the indices of variable z stored in Gurobi model, in \p
// second_order_cones[i].
// @param gurobi_var_type. The type of the Gurobi variables.
// @param xlow The lower bound of the Gurobi variables.
// @param xupp The upper bound of the Gurobi variables.
template <typename C>
void AddSecondOrderConeVariables(
    const std::vector<Binding<C>>& second_order_cones,
    std::vector<bool>* is_new_variable, int* num_gurobi_vars,
    std::vector<std::vector<int>>* second_order_cone_variable_indices,
    std::vector<char>* gurobi_var_type, std::vector<double>* xlow,
    std::vector<double>* xupp) {
  static_assert(
      std::is_same_v<C, LorentzConeConstraint> ||
          std::is_same_v<C, RotatedLorentzConeConstraint>,
      "Expects LorentzConeConstraint and RotatedLorentzConeConstraint.");
  bool is_rotated_cone = std::is_same_v<C, RotatedLorentzConeConstraint>;

  int num_new_second_order_cone_var = 0;
  second_order_cone_variable_indices->resize(second_order_cones.size());

  // The newly added variable z for the Lorentz cone constraint is appended
  // to the existing variables. So increment the variable indices
  // accordingly.
  int lorentz_cone_count = 0;
  for (const auto& binding : second_order_cones) {
    int num_new_lorentz_cone_var_i = binding.evaluator()->A().rows();
    (*second_order_cone_variable_indices)[lorentz_cone_count].resize(
        num_new_lorentz_cone_var_i);
    for (int i = 0; i < num_new_lorentz_cone_var_i; ++i) {
      (*second_order_cone_variable_indices)[lorentz_cone_count][i] =
          *num_gurobi_vars + num_new_second_order_cone_var + i;
    }
    num_new_second_order_cone_var += num_new_lorentz_cone_var_i;
    ++lorentz_cone_count;
  }
  *num_gurobi_vars += num_new_second_order_cone_var;
  is_new_variable->resize(*num_gurobi_vars, true);

  // Newly added variable z is continuous variable.
  gurobi_var_type->resize(*num_gurobi_vars, GRB_CONTINUOUS);

  // For Lorentz cone constraint, z(0) >= 0.
  // For rotated Lorentz cone constraint, z(0) >= 0, z(1) >= 0.
  xlow->resize(*num_gurobi_vars, -std::numeric_limits<double>::infinity());
  xupp->resize(*num_gurobi_vars, std::numeric_limits<double>::infinity());
  for (int i = 0; i < static_cast<int>(second_order_cones.size()); ++i) {
    xlow->at((*second_order_cone_variable_indices)[i][0]) = 0;
    if (is_rotated_cone) {
      xlow->at((*second_order_cone_variable_indices)[i][1]) = 0;
    }
  }
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
  std::ifstream stream{grb_license_file};
  const std::string contents{std::istreambuf_iterator<char>{stream},
                             std::istreambuf_iterator<char>{}};
  if (stream.fail()) {
    return false;
  }
  return contents.find("HOSTID") != std::string::npos;
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
  static never_destroyed<std::shared_ptr<void>> local_host_holder{[]() {
    return IsGrbLicenseFileLocalHost()
               ? GetScopedSingleton<GurobiSolver::License>()
               : nullptr;
  }()};
  return GetScopedSingleton<GurobiSolver::License>();
}

// TODO(hongkai.dai@tri.global): break this large DoSolve function to smaller
// ones.
void GurobiSolver::DoSolve(const MathematicalProgram& prog,
                           const Eigen::VectorXd& initial_guess,
                           const SolverOptions& merged_options,
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

  // Bound constraints.
  std::vector<double> xlow(num_prog_vars,
                           -std::numeric_limits<double>::infinity());
  std::vector<double> xupp(num_prog_vars,
                           std::numeric_limits<double>::infinity());

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

  for (const auto& binding : prog.bounding_box_constraints()) {
    const auto& constraint = binding.evaluator();
    const Eigen::VectorXd& lower_bound = constraint->lower_bound();
    const Eigen::VectorXd& upper_bound = constraint->upper_bound();

    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      const int idx = prog.FindDecisionVariableIndex(binding.variables()(k));
      xlow[idx] = std::max(lower_bound(k), xlow[idx]);
      xupp[idx] = std::min(upper_bound(k), xupp[idx]);
    }
  }
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
  AddSecondOrderConeVariables(
      prog.lorentz_cone_constraints(), &is_new_variable, &num_gurobi_vars,
      &lorentz_cone_new_variable_indices, &gurobi_var_type, &xlow, &xupp);

  std::vector<std::vector<int>> rotated_lorentz_cone_new_variable_indices;
  AddSecondOrderConeVariables(prog.rotated_lorentz_cone_constraints(),
                              &is_new_variable, &num_gurobi_vars,
                              &rotated_lorentz_cone_new_variable_indices,
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
    error = AddCosts(model, &constant_cost, prog);
  }

  int num_gurobi_linear_constraints = 0;
  if (!error) {
    error =
        ProcessLinearConstraints(model, prog, &num_gurobi_linear_constraints,
                                 &constraint_dual_start_row);
  }

  // Add Lorentz cone constraints.
  if (!error) {
    error =
        AddSecondOrderConeConstraints(prog, prog.lorentz_cone_constraints(),
                                      lorentz_cone_new_variable_indices, model,
                                      &num_gurobi_linear_constraints);
  }

  // Add rotated Lorentz cone constraints.
  if (!error) {
    error = AddSecondOrderConeConstraints(
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

  // Handle common solver options before gurobi-specific options stored in
  // merged_options, so that gurobi-specific options can overwrite common solver
  // options.
  // Gurobi creates a new log file every time we set "LogFile" parameter through
  // GRBsetstrparam(). So in order to avoid creating log files repeatedly, we
  // store the log file name in @p log_file variable, and only call
  // GRBsetstrparam(model_env, "LogFile", log_file) for once.
  std::string log_file = merged_options.get_print_file_name();
  if (!error) {
    SetOptionOrThrow(model_env, "LogToConsole",
                     static_cast<int>(merged_options.get_print_to_console()));
  }

  // Default the option for number of threads based on an environment variable
  // (but only if the user hasn't set the option directly already).
  if (merged_options.GetOptionsInt(id()).count("Threads") == 0) {
    if (char* num_threads_str = std::getenv("GUROBI_NUM_THREADS")) {
      const std::optional<int> num_threads = ParseInt(num_threads_str);
      if (num_threads.has_value()) {
        SetOptionOrThrow(model_env, "Threads", *num_threads);
        log()->debug("Using GUROBI_NUM_THREADS={}", *num_threads);
      } else {
        static const logging::Warn log_once(
            "Ignoring unparseable value '{}' for GUROBI_NUM_THREADS",
            num_threads_str);
      }
    }
  }

  for (const auto& it : merged_options.GetOptionsDouble(id())) {
    if (!error) {
      SetOptionOrThrow(model_env, it.first, it.second);
    }
  }
  bool compute_iis = false;
  for (const auto& it : merged_options.GetOptionsInt(id())) {
    if (!error) {
      if (it.first == "GRBcomputeIIS") {
        compute_iis = static_cast<bool>(it.second);
        if (!(it.second == 0 || it.second == 1)) {
          throw std::runtime_error(fmt::format(
              "GurobiSolver(): option GRBcomputeIIS should be either "
              "0 or 1, but is incorrectly set to {}",
              it.second));
        }
      } else {
        SetOptionOrThrow(model_env, it.first, it.second);
      }
    }
  }
  std::optional<std::string> grb_write;
  for (const auto& it : merged_options.GetOptionsStr(id())) {
    if (!error) {
      if (it.first == "GRBwrite") {
        if (it.second != "") {
          grb_write = it.second;
        }
      } else if (it.first == "LogFile") {
        log_file = it.second;
      } else {
        SetOptionOrThrow(model_env, it.first, it.second);
      }
    }
  }
  SetOptionOrThrow(model_env, "LogFile", log_file);

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
    if (grb_write.has_value()) {
      error = GRBwrite(model, grb_write.value().c_str());
      if (error) {
        const std::string gurobi_version =
            fmt::format("{}.{}", GRB_VERSION_MAJOR, GRB_VERSION_MINOR);
        throw std::runtime_error(fmt::format(
            "GurobiSolver(): setting GRBwrite to {}, this is not supported. "
            "Check {}/py_model_write.html for more details.",
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
    // Always set the primal and dual solution for any non-error gurobi status.
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
