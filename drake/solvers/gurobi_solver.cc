#include "gurobi_solver.h"

#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "gurobi_c++.h"

#include "drake/common/drake_assert.h"
#include "drake/math/eigen_sparse_triplet.h"

namespace drake {
namespace solvers {
namespace {

// TODO(naveenoid): This is currently largely copy-pasta from the deprecated
// Gurobi wrapper in solvers. Utilise sparsity in the constraint matrices,
// i.e. Something like :
//    Eigen::SparseMatrix<double, Eigen::RowMajor> sparseA(A.sparseView());
//    sparseA.makeCompressed();
//    error =  GRBaddconstrs(model, A.rows(), sparseA.nonZeros(),
//                           sparseA.InnerIndices(), sparseA.OuterStarts(),
//                           sparseA.Values(),sense, b.data(), nullptr);
//    return(error);

/**
 * Adds a constraint of one of the following forms :
 * Ax>=b, Ax<=b, or Ax==b,
 * where the character variable @p constraint_sense specifies the type.
 * x in this case is the full dimensional variable being optimised.
 *
 * @param[in] constraint_sense a character variable specifying the
 * sense on the constraint. The Gurobi macros maybe used to specify the
 * constraint sense.
 *  i.e.
 *  GRB_LESS_EQUAL    : '<'
 *  GRB_GREATER_EQUAL : '>'
 *  GRB_EQUAL         : '='
 *
 * @return error as an integer. The full set of error values are
 * described here :
 * http://www.gurobi.com/documentation/6.5/refman/error_codes.html#sec:ErrorCodes
 */
template <typename DerivedA, typename DerivedB>
int AddConstraints(GRBmodel* model, const Eigen::MatrixBase<DerivedA>& A,
                   const Eigen::MatrixBase<DerivedB>& b, char constraint_sense,
                   double sparseness_threshold) {
  for (int i = 0; i < A.rows(); i++) {
    int non_zeros_index = 0;
    std::vector<int> constraint_index(A.cols(), 0);
    std::vector<double> constraint_value(A.cols(), 0.0);

    for (int j = 0; j < A.cols(); j++) {
      if (std::abs(A(i, j)) > sparseness_threshold) {
        constraint_value[non_zeros_index] = A(i, j);
        constraint_index[non_zeros_index++] = j;
      }
    }
    int error =
        GRBaddconstr(model, non_zeros_index, &constraint_index[0],
                     &constraint_value[0], constraint_sense, b(i), nullptr);
    if (error) return error;
  }
  // If loop completes, no errors exist so the value '0' must be returned.
  return 0;
}

/*
 * Add quadratic or linear costs to the optimization problem.
 */
int AddCosts(GRBmodel* model, MathematicalProgram& prog,
             double sparseness_threshold) {
  // Aggregates the quadratic costs and linear costs in the form
  // 0.5 * x' * Q_all * x + linear_term' * x
  using std::abs;
  // record the non-zero entries in the cost 0.5*x'*Q*x + b'*x
  std::vector<Eigen::Triplet<double>> Q_nonzero_coefs;
  std::vector<Eigen::Triplet<double>> b_nonzero_coefs;
  for (const auto& binding : prog.quadratic_costs()) {
    const auto& constraint = binding.constraint();
    const int constraint_variable_dimension = binding.GetNumElements();
    Eigen::MatrixXd Q = constraint->Q();
    Eigen::VectorXd b = constraint->b();

    DRAKE_ASSERT(Q.rows() == constraint_variable_dimension);

    // constraint_variable_index[i] is the index of the i'th decision variable
    // binding.VariableListToVectorXd(i)
    std::vector<int> constraint_variable_index(constraint_variable_dimension);
    int constraint_variable_count = 0;
    for (const DecisionVariableView& var : binding.variable_list()) {
      for (int i = 0; i < static_cast<int>(var.size()); i++) {
        constraint_variable_index[constraint_variable_count] = var.index() + i;
        constraint_variable_count++;
      }
    }
    for (int i = 0; i < Q.rows(); i++) {
      const double Qii = 0.5 * Q(i, i);
      if (abs(Qii) > sparseness_threshold) {
        Q_nonzero_coefs.push_back(Eigen::Triplet<double>(
            constraint_variable_index[i], constraint_variable_index[i], Qii));
      }
      for (int j = i + 1; j < Q.cols(); j++) {
        const double Qij = 0.5 * (Q(i, j) + Q(j, i));
        if (abs(Qij) > sparseness_threshold) {
          Q_nonzero_coefs.push_back(
              Eigen::Triplet<double>(constraint_variable_index[i],
                                     constraint_variable_index[j], Qij));
        }
      }
    }

    for (int i = 0; i < b.size(); i++) {
      if (abs(b(i)) > sparseness_threshold) {
        b_nonzero_coefs.push_back(
            Eigen::Triplet<double>(constraint_variable_index[i], 0, b(i)));
      }
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

/// Splits out the equality and inequality constraints and makes call to
/// add any non-inf constraints.
int ProcessConstraints(GRBmodel* model, MathematicalProgram& prog,
                       double sparseness_threshold) {
  // TODO(naveenoid) : needs test coverage.
  for (const auto& binding : prog.linear_equality_constraints()) {
    const auto& constraint = binding.constraint();
    const int error =
        AddConstraints(model, constraint->A(), constraint->lower_bound(),
                       GRB_EQUAL, sparseness_threshold);
    if (error) {
      return error;
    }
  }

  for (const auto& binding : prog.linear_constraints()) {
    const auto& constraint = binding.constraint();

    if (constraint->lower_bound() !=
        -Eigen::MatrixXd::Constant((constraint->lower_bound()).rows(), 1,
                                   std::numeric_limits<double>::infinity())) {
      const int error =
          AddConstraints(model, constraint->A(), constraint->lower_bound(),
                         GRB_GREATER_EQUAL, sparseness_threshold);
      if (error) {
        return error;
      }
    }
    if (constraint->upper_bound() !=
        Eigen::MatrixXd::Constant((constraint->upper_bound()).rows(), 1,
                                  std::numeric_limits<double>::infinity())) {
      const int error =
          AddConstraints(model, constraint->A(), constraint->upper_bound(),
                         GRB_LESS_EQUAL, sparseness_threshold);
      if (error) {
        return error;
      }
    }
  }
  // If loop completes, no errors exist so the value '0' must be returned.
  return 0;
}
}  // close namespace

bool GurobiSolver::available() const { return true; }

SolutionResult GurobiSolver::Solve(MathematicalProgram& prog) const {
  // We only process quadratic costs and linear / bounding box
  // constraints.

  GRBenv* env = nullptr;
  GRBloadenv(&env, nullptr);
  // Corresponds to no console or file logging.
  GRBsetintparam(env, GRB_INT_PAR_OUTPUTFLAG, 0);

  DRAKE_ASSERT(prog.generic_costs().empty());
  DRAKE_ASSERT(prog.generic_constraints().empty());

  const int num_vars = prog.num_vars();

  // bound constraints
  std::vector<double> xlow(num_vars, -std::numeric_limits<double>::infinity());
  std::vector<double> xupp(num_vars, std::numeric_limits<double>::infinity());

  for (const auto& binding : prog.bounding_box_constraints()) {
    const auto& constraint = binding.constraint();
    const Eigen::VectorXd& lower_bound = constraint->lower_bound();
    const Eigen::VectorXd& upper_bound = constraint->upper_bound();
    for (const DecisionVariableView& decision_variable_view :
         binding.variable_list()) {
      for (size_t k = 0; k < decision_variable_view.size(); k++) {
        const int idx = decision_variable_view.index() + k;
        xlow[idx] = std::max(lower_bound(k), xlow[idx]);
        xupp[idx] = std::min(upper_bound(k), xupp[idx]);
      }
    }
  }

  GRBmodel* model = nullptr;
  GRBnewmodel(env, &model, "QP", num_vars, nullptr, &xlow[0], &xupp[0], nullptr,
              nullptr);

  int error = 0;
  // TODO(naveenoid) : This needs access externally.
  double sparseness_threshold = 1e-14;
  error = AddCosts(model, prog, sparseness_threshold);
  if (!error) {
    error = ProcessConstraints(model, prog, sparseness_threshold);
  }
  SolutionResult result = SolutionResult::kUnknownError;

  if (!error) {
    error = GRBoptimize(model);
  }

  // If any error exists so far, its either from invalid input or
  // from unknown errors.
  // TODO(naveenoid) : Properly handle gurobi specific error.
  // message.
  if (error) {
    // TODO(naveenoid) : log error message using GRBgeterrormsg(env).
    result = SolutionResult::kInvalidInput;
  } else {
    int optimstatus = 0;
    GRBgetintattr(model, GRB_INT_ATTR_STATUS, &optimstatus);

    if (optimstatus != GRB_OPTIMAL) {
      if (optimstatus == GRB_INF_OR_UNBD) {
        result = SolutionResult::kInfeasibleConstraints;
      }
    } else {
      result = SolutionResult::kSolutionFound;
      Eigen::VectorXd sol_vector = Eigen::VectorXd::Zero(num_vars);
      GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, num_vars, sol_vector.data());
      prog.SetDecisionVariableValues(sol_vector);
    }
  }

  prog.SetSolverResult("Gurobi", error);

  GRBfreemodel(model);
  GRBfreeenv(env);

  return result;
}

}  // namespace drake
}  // namespace solvers
