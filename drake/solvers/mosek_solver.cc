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
MSKrescodee AddLinearConstraintsFromBindings(MSKtask_t &task,
                                             const std::list<_Binding> &constraint_list,
                                             bool is_equality_constraint) {
  for (const auto &binding : constraint_list) {
    auto constraint = binding.constraint();
    std::vector<int> var_indices = binding.variable_indices();
    Eigen::MatrixXd A = constraint->A();
    Eigen::VectorXd lb = constraint->lower_bound();
    Eigen::VectorXd ub = constraint->upper_bound();
    MSKint32t constraint_idx;
    MSKrescodee rescode = MSK_getnumcon(task, &constraint_idx);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    // Loop through each row of the constraint, and determine the sense of
    // each constraint. The sense can be equality constraint, less than,
    // greater than, or bounded on both side.
    for (int i = 0; i < A.rows(); ++i) {
      rescode = MSK_appendcons(task, 1);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
      if (is_equality_constraint) {
        rescode =
            MSK_putconbound(task, constraint_idx + i, MSK_BK_FX, lb(i), ub(i));
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
      } else {
        if (isinf(lb(i)) && isinf(ub(i))) {
          continue; // Both lower and upper bounds are infinity, bypass the row.
        } else if (isinf(lb(i)) && !isinf(ub(i))) {
          rescode = MSK_putconbound(task,
                                    constraint_idx + i,
                                    MSK_BK_UP,
                                    -MSK_INFINITY,
                                    ub(i));
        } else if (!isinf(lb(i)) && isinf(ub(i))) {
          rescode = MSK_putconbound(task,
                                    constraint_idx + i,
                                    MSK_BK_LO,
                                    lb(i),
                                    MSK_INFINITY);
        } else {
          rescode = MSK_putconbound(task,
                                    constraint_idx + i,
                                    MSK_BK_RA,
                                    lb(i),
                                    ub(i));
        }
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
      }
      std::vector<MSKint32t> A_nnz_col_idx(A.rows());
      std::vector<double> A_nnz_val(A.rows());
      A_nnz_col_idx.reserve(A.cols());
      A_nnz_val.reserve(A.cols());
      for (int j = 0; j < A.cols(); ++j) {
        if (std::abs(A(i, j)) > std::numeric_limits<double>::epsilon()) {
          A_nnz_col_idx.push_back(var_indices[j]);
          A_nnz_val.push_back(A(i, j));
        }
      }
      rescode = MSK_putarow(task,
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
MSKrescodee AddLinearConstraints(MSKtask_t &task,
                                 const MathematicalProgram &prog) {
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
}

MSKrescodee AddCosts(MSKtask_t &task, const MathematicalProgram &prog) {

}
} // end namespace


bool MosekSolver::available() const { return true; }

SolutionResult MosekSolver::Solve(MathematicalProgram& prog) const {
  MSKenv_t env = NULL;
  MSKtask_t task = NULL;
  MSKrescodee rescode;

  // Create the Mosek environment.
  rescode = MSK_makeenv(&env, NULL);
  if (rescode == MSK_RES_OK) {
    // Create the optimization task.
    rescode = MSK_maketask(env, 0, prog.num_vars(), &task);
  }
  // Add linear constraints
  rescode = AddLinearConstraints(env, task, prog);
}

}
}
