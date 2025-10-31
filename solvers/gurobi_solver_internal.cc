#include "drake/solvers/gurobi_solver_internal.h"

#include <limits>

#include "drake/solvers/aggregate_costs_constraints.h"

namespace drake {
namespace solvers {
namespace internal {
namespace {
const double kInf = std::numeric_limits<double>::infinity();

// Gurobi will throw a warning if the coeffient is less than 1E-13, so we filter
// out these small entries.
const double kGurobiZeroTol = 1E-13;
}  // namespace

int AddLinearConstraintNoDuplication(
    const MathematicalProgram& prog, GRBmodel* model,
    const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub, const VectorXDecisionVariable& vars,
    bool is_equality, int* num_gurobi_linear_constraints) {
  // Gurobi add the linear constraint row by row, so we will need a row-major
  // sparse matrix.
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row_major =
      A.pruned(1, kGurobiZeroTol);

  const std::vector<int> var_index = prog.FindDecisionVariableIndices(vars);

  // If this linear constraint is an equality constraint, we know that we can
  // pass in A * vars = lb directly to Gurobi.
  if (is_equality) {
    std::vector<int> nonzero_col_index;
    nonzero_col_index.reserve(A_row_major.nonZeros());
    std::vector<char> sense(A.rows(), GRB_EQUAL);
    for (int i = 0; i < A_row_major.nonZeros(); ++i) {
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
  // https://docs.gurobi.com/projects/optimizer/en/12.0/reference/c/model.html#c.GRBaddqconstr
  // for the meaning of these three vectors. The non-zero entries in the i'th
  // row of A_gurobi is stored in the chunk cind[cbeg[i]:cbeg[i+1]] and
  // cval[cbeg[i]:cbeg[i+1]]
  std::vector<int> cbeg;
  cbeg.reserve(A.rows() * 2 + 1);
  cbeg.push_back(0);
  std::vector<int> cind;
  cind.reserve(A_row_major.nonZeros() * 2);
  std::vector<double> cval;
  cval.reserve(A_row_major.nonZeros() * 2);

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

namespace {
// Drake's MathematicalProgram imposes the second order cone constraint in the
// form "A*x+b is in cone". On the other hand, Gurobi imposes the second order
// cone in this form "z is in cone". So we consider to add the linear equality
// constraint z-A*x=b. We write z-A*x in the form of M * [x;z], where
// M = [-A I].
void ConvertSecondOrderConeLinearConstraint(
    const Eigen::SparseMatrix<double>& A, const std::vector<int>& xz_indices,
    std::vector<Eigen::Triplet<double>>* M_triplets) {
  M_triplets->clear();
  M_triplets->reserve(A.nonZeros() + A.rows());
  for (int i = 0; i < A.outerSize(); ++i) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
      if (std::abs(it.value()) > kGurobiZeroTol) {
        M_triplets->emplace_back(it.row(), xz_indices[it.col()], -it.value());
      }
    }
  }
  for (int i = 0; i < A.rows(); ++i) {
    M_triplets->emplace_back(i, xz_indices[A.cols() + i], 1);
  }
}

// Gurobi uses a matrix Q to differentiate Lorentz cone and rotated Lorentz
// cone constraint.
// https://docs.gurobi.com/projects/optimizer/en/current/reference/c/model.html#c.GRBaddqconstr
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
// Note that Q in the rotated Lorentz cone case is not symmetric (following the
// example https://www.gurobi.com/documentation/12.0/examples/qcp_c_c.html).
// We will store Q in a sparse format.
// qrow stores the row    indices of the non-zero entries of Q.
// qcol stores the column indices of the non-zero entries of Q.
// qval stores the value          of the non-zero entries of Q.
// GRBaddqconstr expects this qrow/qcol/qval format instead of
// Eigen::SparseMatrix format.
void CalcSecondOrderConeQ(bool is_rotated_cone,
                          const std::vector<int>& z_indices,
                          std::vector<int>* qrow, std::vector<int>* qcol,
                          std::vector<double>* qval) {
  const int num_z = z_indices.size();
  const size_t num_Q_nonzero = is_rotated_cone ? num_z - 1 : num_z;
  qrow->clear();
  qcol->clear();
  qval->clear();
  qrow->reserve(num_Q_nonzero);
  qcol->reserve(num_Q_nonzero);
  qval->reserve(num_Q_nonzero);
  for (int i = 0; i < num_z - 2; ++i) {
    const int zi_index = z_indices[i + 2];
    qrow->push_back(zi_index);
    qcol->push_back(zi_index);
    qval->push_back(1.0);
  }
  const int z0_index = z_indices[0];
  const int z1_index = z_indices[1];
  if (is_rotated_cone) {
    qrow->push_back(z0_index);
    qcol->push_back(z1_index);
    qval->push_back(-1);
  } else {
    qrow->push_back(z0_index);
    qcol->push_back(z0_index);
    qval->push_back(-1);
    qrow->push_back(z1_index);
    qcol->push_back(z1_index);
    qval->push_back(1);
  }
}
}  // namespace

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
    ConvertSecondOrderConeLinearConstraint(A, xz_indices, &M_triplets);
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
    // https://docs.gurobi.com/projects/optimizer/en/12.0/reference/c/model.html#c.GRBaddqconstr
    std::vector<int> qrow;
    std::vector<int> qcol;
    std::vector<double> qval;
    CalcSecondOrderConeQ(
        is_rotated_cone,
        second_order_cone_new_variable_indices[second_order_cone_count], &qrow,
        &qcol, &qval);
    const size_t num_Q_nonzero = qrow.size();
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

void AddL2NormCostVariables(
    const std::vector<Binding<L2NormCost>>& l2_norm_costs,
    std::vector<bool>* is_new_variable, int* num_gurobi_vars,
    std::vector<std::vector<int>>* lorentz_cone_variable_indices,
    std::vector<char>* gurobi_var_type, std::vector<double>* xlow,
    std::vector<double>* xupp) {
  for (const auto& l2_norm_cost : l2_norm_costs) {
    // z is of size l2_norm_cost.A().rows() + 1
    for (int i = 0; i < 1 + l2_norm_cost.evaluator()->get_sparse_A().rows();
         ++i) {
      is_new_variable->push_back(true);
      gurobi_var_type->push_back(GRB_CONTINUOUS);
    }
    lorentz_cone_variable_indices->emplace_back(
        1 + l2_norm_cost.evaluator()->get_sparse_A().rows(), 0);
    for (int i = 0; i < 1 + l2_norm_cost.evaluator()->get_sparse_A().rows();
         ++i) {
      lorentz_cone_variable_indices->back()[i] = *num_gurobi_vars + i;
    }
    // For Lorentz cone constraint z(0) >= 0.
    xlow->push_back(0);
    xupp->push_back(kInf);
    for (int i = 0; i < l2_norm_cost.evaluator()->get_sparse_A().rows(); ++i) {
      xlow->push_back(-kInf);
      xupp->push_back(kInf);
    }
    *num_gurobi_vars += 1 + l2_norm_cost.evaluator()->get_sparse_A().rows();
  }
}

int AddL2NormCosts(const MathematicalProgram& prog,
                   const std::vector<std::vector<int>>&
                       l2norm_costs_lorentz_cone_variable_indices,
                   GRBmodel* model, int* num_gurobi_linear_constraints) {
  DRAKE_ASSERT(prog.l2norm_costs().size() ==
               l2norm_costs_lorentz_cone_variable_indices.size());
  int num_gurobi_vars;
  int error = GRBgetintattr(model, "NumVars", &num_gurobi_vars);
  DRAKE_ASSERT(!error);
  // We declare the vectors here outside of the for loop, so that we can re-use
  // the heap memory of these vectors in each iteration of the for loop.
  // We will impose the linear equality constraint z[1:] - Cx = d.
  // xz_indices records the indices of [x;z[1:]] in Gurobi.
  // M_triplets records the non-zero entries in M = [-C I].
  // sense records the sign (=, <= or >=) in Gurobi model's linear constraints.
  // qrow, qcol, qval stores the row, column and value of the Q matrix in the
  // second order cone constraint.
  std::vector<int> xz_indices;
  std::vector<Eigen::Triplet<double>> M_triplets;
  // This is used when adding linear constraints to the Gurobi model to indicate
  // that we are adding equality constraints.
  std::vector<char> sense;
  std::vector<int> qrow;
  std::vector<int> qcol;
  std::vector<double> qval;
  for (int i = 0; i < ssize(prog.l2norm_costs()); ++i) {
    const Eigen::SparseMatrix<double>& C =
        prog.l2norm_costs()[i].evaluator()->get_sparse_A();
    const auto& d = prog.l2norm_costs()[i].evaluator()->b();
    const int num_x = C.cols();
    const int num_z = C.rows() + 1;
    // Add the constraint z[1:]-C*x=d
    xz_indices.clear();
    xz_indices.reserve(num_x + num_z - 1);
    for (int j = 0; j < num_x; ++j) {
      xz_indices.push_back(prog.FindDecisionVariableIndex(
          prog.l2norm_costs()[i].variables()(j)));
    }
    for (int j = 1; j < num_z; ++j) {
      xz_indices.push_back(l2norm_costs_lorentz_cone_variable_indices[i][j]);
    }
    // z[1:]-Cx will be written as M * [x;z[1:]], where M=[-C I].
    ConvertSecondOrderConeLinearConstraint(C, xz_indices, &M_triplets);
    // Gurobi expects the matrix in the sparse row format.
    Eigen::SparseMatrix<double, Eigen::RowMajor> M(num_z - 1, num_gurobi_vars);
    M.setFromTriplets(M_triplets.begin(), M_triplets.end());

    sense.clear();
    sense.reserve(num_z - 1);
    for (int j = 0; j < num_z - 1; ++j) {
      sense.push_back(GRB_EQUAL);
    }

    error = GRBaddconstrs(model, num_z - 1, M.nonZeros(), M.outerIndexPtr(),
                          M.innerIndexPtr(), M.valuePtr(), sense.data(),
                          const_cast<double*>(d.data()), nullptr);
    DRAKE_ASSERT(!error);
    *num_gurobi_linear_constraints += num_z - 1;
    CalcSecondOrderConeQ(false /*is_rotated_cone=false*/,
                         l2norm_costs_lorentz_cone_variable_indices[i], &qrow,
                         &qcol, &qval);
    const size_t num_Q_nonzero = qrow.size();
    error =
        GRBaddqconstr(model, 0, nullptr, nullptr, num_Q_nonzero, qrow.data(),
                      qcol.data(), qval.data(), GRB_LESS_EQUAL, 0.0, NULL);
    if (error) {
      return error;
    }
    // Add the cost min z[0]
    error = GRBsetdblattrelement(
        model, "Obj", l2norm_costs_lorentz_cone_variable_indices[i][0], 1.0);
    if (error) {
      return error;
    }
  }
  return 0;
}

// Explicit instantiation.
template int AddSecondOrderConeConstraints<LorentzConeConstraint>(
    const MathematicalProgram& prog,
    const std::vector<Binding<LorentzConeConstraint>>&
        second_order_cone_constraints,
    const std::vector<std::vector<int>>& second_order_cone_new_variable_indices,
    GRBmodel* model, int* num_gurobi_linear_constraints);
template int AddSecondOrderConeConstraints<RotatedLorentzConeConstraint>(
    const MathematicalProgram& prog,
    const std::vector<Binding<RotatedLorentzConeConstraint>>&
        second_order_cone_constraints,
    const std::vector<std::vector<int>>& second_order_cone_new_variable_indices,
    GRBmodel* model, int* num_gurobi_linear_constraints);

template void AddSecondOrderConeVariables<LorentzConeConstraint>(
    const std::vector<Binding<LorentzConeConstraint>>& second_order_cones,
    std::vector<bool>* is_new_variable, int* num_gurobi_vars,
    std::vector<std::vector<int>>* second_order_cone_variable_indices,
    std::vector<char>* gurobi_var_type, std::vector<double>* xlow,
    std::vector<double>* xupp);
template void AddSecondOrderConeVariables<RotatedLorentzConeConstraint>(
    const std::vector<Binding<RotatedLorentzConeConstraint>>&
        second_order_cones,
    std::vector<bool>* is_new_variable, int* num_gurobi_vars,
    std::vector<std::vector<int>>* second_order_cone_variable_indices,
    std::vector<char>* gurobi_var_type, std::vector<double>* xlow,
    std::vector<double>* xupp);
}  // namespace internal
}  // namespace solvers
}  // namespace drake
