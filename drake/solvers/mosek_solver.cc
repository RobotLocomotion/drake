// A wrapper file for MosekWrapper and mosekQP that handles constraint and
// objective marshalling

#include "drake/solvers/mosek_solver.h"

#include <mosek/mosek.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace drake {
namespace solvers {
namespace {
// Add LinearConstraints and LinearEqualityConstraints to the Mosek task.
template<typename _Binding>
MSKrescodee AddLinearConstraintsFromBindings(MSKtask_t* task,
                                             const std::list<_Binding>& constraint_list,
                                             bool is_equality_constraint) {
  for (const auto &binding : constraint_list) {
    auto constraint = binding.constraint();
    std::vector<int> var_indices = binding.variable_indices();
    const Eigen::MatrixXd& A = constraint->A();
    const Eigen::VectorXd& lb = constraint->lower_bound();
    const Eigen::VectorXd& ub = constraint->upper_bound();
    MSKint32t constraint_idx;
    MSKrescodee rescode = MSK_getnumcon(*task, &constraint_idx);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = MSK_appendcons(*task, A.rows());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    // Loop through each row of the constraint, and determine the sense of
    // each constraint. The sense can be equality constraint, less than,
    // greater than, or bounded on both side.
    for (int i = 0; i < A.rows(); ++i) {
      if (is_equality_constraint) {
        rescode =
            MSK_putconbound(*task, constraint_idx + i, MSK_BK_FX, lb(i), ub(i));
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
      } else {
        if (isinf(lb(i)) && isinf(ub(i))) {
          rescode = MSK_putconbound(*task, constraint_idx + i, MSK_BK_FR, -MSK_INFINITY, MSK_INFINITY);
        } else if (isinf(lb(i)) && !isinf(ub(i))) {
          rescode = MSK_putconbound(*task,
                                    constraint_idx + i,
                                    MSK_BK_UP,
                                    -MSK_INFINITY,
                                    ub(i));
        } else if (!isinf(lb(i)) && isinf(ub(i))) {
          rescode = MSK_putconbound(*task,
                                    constraint_idx + i,
                                    MSK_BK_LO,
                                    lb(i),
                                    MSK_INFINITY);
        } else {
          rescode = MSK_putconbound(*task,
                                    constraint_idx + i,
                                    MSK_BK_RA,
                                    lb(i),
                                    ub(i));
        }
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
      }
      std::vector<MSKint32t> A_nnz_col_idx;
      std::vector<double> A_nnz_val;
      A_nnz_col_idx.reserve(A.cols());
      A_nnz_val.reserve(A.cols());
      for (int j = 0; j < A.cols(); ++j) {
        if (std::abs(A(i, j)) > std::numeric_limits<double>::epsilon()) {
          A_nnz_col_idx.push_back(var_indices[j]);
          A_nnz_val.push_back(A(i, j));
        }
      }
      rescode = MSK_putarow(*task,
                            constraint_idx + i,
                            A_nnz_val.size(),
                            A_nnz_col_idx.data(),
                            A_nnz_val.data());
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    }
  }
  return MSK_RES_OK;
}
MSKrescodee AddLinearConstraints(MSKtask_t* task,
                                 const MathematicalProgram& prog) {
  MSKrescodee rescode = AddLinearConstraintsFromBindings(task,
                                                         prog.linear_equality_constraints(),
                                                         true);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode =
      AddLinearConstraintsFromBindings(task, prog.linear_constraints(), false);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }

  return rescode;
}

MSKrescodee AddBoundingBoxConstraints(MSKtask_t* task, const MathematicalProgram& prog) {
  int num_vars = prog.num_vars();
  std::vector<double> x_lb(num_vars, -std::numeric_limits<double>::infinity());
  std::vector<double> x_ub(num_vars, std::numeric_limits<double>::infinity());
  for(const auto& binding : prog.bounding_box_constraints()) {
    const auto& constraint = binding.constraint();
    const Eigen::VectorXd& lower_bound = constraint->lower_bound();
    const Eigen::VectorXd& upper_bound = constraint->upper_bound();
    int var_count = 0;
    for(const DecisionVariableView& var : binding.variable_list()) {
      for(int i = 0; i < static_cast<int>(var.size()); ++i) {
        int x_idx = var.index() + i;
        x_lb[x_idx] = std::max(x_lb[x_idx], lower_bound[var_count]);
        x_ub[x_idx] = std::min(x_ub[x_idx], upper_bound[var_count]);
        var_count++;
      }
    }
  }

  MSKrescodee rescode= MSK_RES_OK;
  for(int i = 0; i< num_vars; i++) {
    if(isinf(x_lb[i]) && isinf(x_ub[i])) {
      rescode = MSK_putvarbound(*task, i, MSK_BK_FR, -MSK_INFINITY, MSK_INFINITY);
    }
    else if (isinf(x_lb[i]) && !isinf(x_ub[i])) {
      rescode = MSK_putvarbound(*task, i, MSK_BK_UP, -MSK_INFINITY, x_ub[i]);
    }
    else if (!isinf(x_lb[i]) && isinf(x_ub[i])) {
      rescode = MSK_putvarbound(*task, i, MSK_BK_LO, x_lb[i], MSK_INFINITY);
    }
    else {
      rescode = MSK_putvarbound(*task, i, MSK_BK_RA, x_lb[i], x_ub[i]);
    }
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
  }
  return rescode;
}

MSKrescodee AddCosts(MSKtask_t* task, const MathematicalProgram &prog) {
  // Add the cost in the form 0.5 * x' * Q_all * x + linear_terms' * x
  MSKrescodee rescode = MSK_RES_OK;
  int xDim = prog.num_vars();
  std::vector<Eigen::Triplet<double>> linear_term_triplets;
  for(const auto& binding : prog.linear_costs()) {
    int var_count = 0;
    const auto& c = binding.constraint()->A();
    for(const DecisionVariableView &var : binding.variable_list()) {
      for(int i = 0; i < static_cast<int>(var.size()); ++i) {
        if(std::abs(c(var_count)) > std::numeric_limits<double>::epsilon()) {
          linear_term_triplets.push_back(Eigen::Triplet<double>(var.index() + i, 0, c(var_count)));
        }
        var_count++;
      }
    }
  }
  Eigen::SparseMatrix<double, Eigen::ColMajor> linear_terms(xDim, 1);
  linear_terms.setFromTriplets(linear_term_triplets.begin(), linear_term_triplets.end());
  for(Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it(linear_terms, 0); it; ++it) {
    rescode = MSK_putcj(*task, static_cast<MSKint32t>(it.row()), it.value());
    if(rescode != MSK_RES_OK) {
      return rescode;
    }
  }
  return rescode;
}
} // end namespace


bool MosekSolver::available() const { return true; }

SolutionResult MosekSolver::Solve(MathematicalProgram& prog) const {
  int num_vars = prog.num_vars();
  MSKenv_t env = NULL;
  MSKtask_t task = NULL;
  MSKrescodee rescode;

  // Create the Mosek environment.
  rescode = MSK_makeenv(&env, NULL);
  if (rescode == MSK_RES_OK) {
    // Create the optimization task.
    rescode = MSK_maketask(env, 0, num_vars, &task);
  }
  if (rescode == MSK_RES_OK) {
    rescode = MSK_appendvars(task, num_vars);
  }
  // Add costs
  if(rescode == MSK_RES_OK) {
    rescode = AddCosts(&task, prog);
  }
  // Add bounding box constraints on decision variables.
  if (rescode == MSK_RES_OK) {
    rescode = AddBoundingBoxConstraints(&task, prog);
  }
  // Add linear constraints.
  if(rescode == MSK_RES_OK) {
    rescode = AddLinearConstraints(&task, prog);
  }

  SolutionResult result = SolutionResult::kUnknownError;
  // Run optimizer.
  if(rescode == MSK_RES_OK) {
    MSKrescodee trmcode; // termination code
    rescode = MSK_optimizetrm(task, &trmcode);
  }

  if(rescode == MSK_RES_OK) {
    MSKsolstae solsta;
    if(rescode == MSK_RES_OK) {
      rescode = MSK_getsolsta(task, MSK_SOL_BAS, &solsta);
    }
    if(rescode == MSK_RES_OK) {
      switch (solsta) {
        case MSK_SOL_STA_OPTIMAL:
        case MSK_SOL_STA_NEAR_OPTIMAL: {
          result = SolutionResult::kSolutionFound;
          Eigen::VectorXd sol_vector(num_vars);
          MSK_getxx(task, MSK_SOL_BAS, sol_vector.data());
          prog.SetDecisionVariableValues(sol_vector);
          break;
        }
        case MSK_SOL_STA_DUAL_INFEAS_CER:
        case MSK_SOL_STA_PRIM_INFEAS_CER:
        case MSK_SOL_STA_NEAR_DUAL_FEAS:
        case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER: {
          result = SolutionResult::kInfeasibleConstraints;
          break;
        }
        default: {
          result = SolutionResult::kUnknownError;
          break;
        }
      }
    }
  }

  if(rescode != MSK_RES_OK) {
    result = SolutionResult ::kUnknownError;
  }

  MSK_deletetask(&task);
  MSK_deleteenv(&env);
  return result;
}

}
}
