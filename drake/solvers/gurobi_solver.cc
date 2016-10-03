#include "gurobi_solver.h"

#include <Eigen/Core>
#include "gurobi_c++.h"

#include "drake/common/drake_assert.h"

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

/// Splits out the quadratic costs and makes calls to add them individually.
int AddCosts(GRBmodel* model, MathematicalProgram& prog,
             double sparseness_threshold) {
  int start_row = 0;
  for (const auto& binding : prog.quadratic_costs()) {
    const auto& constraint = binding.constraint();
    const int constraint_variable_dimension = binding.GetNumElements();

    Eigen::MatrixXd Q = 0.5 * (constraint->Q());
    Eigen::VectorXd b = constraint->b();

    // Check for square matrices.
    DRAKE_ASSERT(Q.rows() == Q.cols());
    // Check for symmetric matrices.
    DRAKE_ASSERT(Q.transpose() == Q);
    // Check for Quadratic and Linear Cost dimensions.
    DRAKE_ASSERT(Q.rows() == constraint_variable_dimension);
    DRAKE_ASSERT(b.cols() == 1);
    DRAKE_ASSERT(b.rows() == constraint_variable_dimension);

    // adding each Q term (only upper triangular).
    for (int i = 0; i < constraint_variable_dimension; i++) {
      for (int j = i; j < constraint_variable_dimension; j++) {
        if (std::abs(Q(i, j)) > sparseness_threshold) {
          int row_ind = i + start_row;
          int col_ind = j + start_row;
          // TODO(naveenoid) : Port to batch addition mode of this function
          // by utilising the Upper right (or lower left) triangular matrix.
          // The single element addition method used below is recommended
          // initially by Gurobi since it has a low cost.
          double individual_quadratic_cost_value = Q(i, j);
          const int error = GRBaddqpterms(model, 1, &row_ind, &col_ind,
                                          &individual_quadratic_cost_value);
          if (error) {
            return (error);
          }
        }
      }
    }
    const int error = GRBsetdblattrarray(
        model, "Obj", start_row, constraint_variable_dimension, b.data());
    if (error) {
      return error;
    }
    start_row += Q.rows();
    // Verify that the start_row does not exceed the total possible
    // dimension of the decision variable.
    DRAKE_ASSERT(start_row <= static_cast<int>(prog.num_vars()));
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
