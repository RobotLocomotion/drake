#include "drake/solvers/gurobi_solver_internal.h"

#include <limits>

#include "drake/solvers/aggregate_costs_constraints.h"

namespace drake {
namespace solvers {
namespace internal {
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
  // https://www.gurobi.com/documentation/10.0/refman/c_addconstrs.html for the
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
}  // namespace internal
}  // namespace solvers
}  // namespace drake
