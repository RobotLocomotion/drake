#include "GurobiSolver.h"

#include "drake/common/drake_assert.h"
#include "Optimization.h"

#include <Eigen/Core>

#include "gurobi_c++.h"


namespace drake {
namespace solvers {

using namespace std;

bool GurobiSolver::available() const {
  return true;
}

// TODO(naveenoid): This is currently largely copy-pasta from the deprecated
// Gurobi wrapper in solvers. Utilise sparsity in the constraint matrices,
// i.e. Something like :
//    Eigen::SparseMatrix<double, Eigen::RowMajor> sparseA(A.sparseView());
//    sparseA.makeCompressed();
//    error =  GRBaddconstrs(model, A.rows(), sparseA.nonZeros(),
//                           sparseA.InnerIndices(), sparseA.OuterStarts(),
//                           sparseA.Values(),sense, b.data(), NULL);
//    return(error);

template<typename DerivedA, typename DerivedB>
int myGRBaddconstrs(GRBmodel *model, Eigen::MatrixBase <DerivedA> const &A,
                    Eigen::MatrixBase <DerivedB> const &b, char sense,
                    double sparseness_threshold = 1e-14) {
  int error = 0, i, j, nnz;

  std::vector<int> cind(A.cols(), 0);
  std::vector<double> cval(A.cols(), 0.0);

  for (i = 0; i < A.rows(); i++) {
    nnz = 0;
    for (j = 0; j < A.cols(); j++) {
      if (abs(A(i, j)) > sparseness_threshold) {
        cval[nnz] = A(i, j);
        cind[nnz++] = j;
      }
    }
    error = GRBaddconstr(model, nnz, &cind[0], &cval[0], sense, b(i), NULL);
    if (error) break;
  }
  return error;
}


SolutionResult GurobiSolver::Solve(OptimizationProblem & prog) const {

  // We only process quadratic costs and linear / bounding box
  // constraints

  GRBenv *env = NULL;
  GRBloadenv(&env, NULL);
  // Corresponds to no console or file logging.
  GRBsetintparam(env, GRB_INT_PAR_OUTPUTFLAG, 0);

  DRAKE_ASSERT(prog.generic_costs().empty());
  DRAKE_ASSERT(prog.generic_constraints().empty());

  Eigen::VectorXd solution(prog.num_vars());
  int num_vars = prog.num_vars();


  // bound constraints
  std::vector<double> xlow(num_vars, -std::numeric_limits<double>::infinity());
  std::vector<double> xupp(num_vars, std::numeric_limits<double>::infinity());

  for (auto const &binding : prog.bounding_box_constraints()) {
    auto const &c = binding.constraint();
    const Eigen::VectorXd &lower_bound = c->lower_bound();
    const Eigen::VectorXd &upper_bound = c->upper_bound();
    for (const DecisionVariableView &v : binding.variable_list()) {
      for (size_t k = 0; k < v.size(); k++) {
        const int idx = v.index() + k;
        xlow[idx] = std::max(lower_bound(k), xlow[idx]);
        xupp[idx] = std::min(upper_bound(k), xupp[idx]);
      }
    }
  }

  int error;
  GRBmodel *model = NULL;

  GRBnewmodel(env, &model, "QP", num_vars, NULL, &xlow[0], &xupp[0], NULL, NULL);
  // TODO(naveenoid) : needs coverage for multiple quadratic costs
  int start_row = 0;
  for (auto const &binding : prog.quadratic_costs()) {
    auto const &c = binding.constraint();

    Eigen::MatrixXd Q = 0.5 * (c->Q());
    Eigen::VectorXd b = c->b();

    DRAKE_ASSERT(Q.rows() == Q.cols());
    DRAKE_ASSERT(Q.rows() == num_vars);
    DRAKE_ASSERT(b.cols() == 1);
    DRAKE_ASSERT(b.rows() == num_vars);

    int row_ind=0, col_ind=0;
    // adding each Q term (only upper triangular)
    for (int i = 0; i < num_vars; i++) {
      for (int j = i; j < num_vars; j++) {
        if (*(Q.data() + (i * num_vars) + j) != 0) {
          row_ind = i + start_row;
          col_ind = j + start_row;
          error = GRBaddqpterms(model, 1, &row_ind, &col_ind, (Q.data() + (i * num_vars) + j));
        }
      }
    }
    GRBsetdblattrarray(model, "Obj", start_row, num_vars, b.data());
    start_row = start_row + Q.rows();
  }

  // TODO(naveenoid) : needs test coverage
  for (auto const &binding :  prog.linear_equality_constraints()) {
    auto const &c = binding.constraint();
    myGRBaddconstrs(model, c->A(), c->lower_bound(), GRB_EQUAL, 1e-18);
  }

  for (auto const &binding :  prog.linear_constraints()) {
    auto const &c = binding.constraint();

    if (c->lower_bound() != -Eigen::MatrixXd::Constant((c->lower_bound()).rows(), 1,
                                                       std::numeric_limits<double>::infinity())) {
      myGRBaddconstrs(model, c->A(), c->lower_bound(), GRB_GREATER_EQUAL, 1e-18);
    }
    if (c->upper_bound() != Eigen::MatrixXd::Constant((c->upper_bound()).rows(), 1,
                                                      std::numeric_limits<double>::infinity())) {
      myGRBaddconstrs(model, c->A(), c->upper_bound(), GRB_LESS_EQUAL, 1e-18);
    }
  }

  SolutionResult result = SolutionResult::kSolutionFound;

  error = GRBoptimize(model);

  int optimstatus;
  double objval;

  if (error) {
    // TODO(naveenoid) : log error message using GRBgeterrormsg(env)
    result = SolutionResult::kInvalidInput;
  }
  else {

    error = GRBgetintattr(model, GRB_INT_ATTR_STATUS, &optimstatus);
    error = GRBgetdblattr(model, GRB_DBL_ATTR_OBJVAL, &objval);

    if (optimstatus != GRB_OPTIMAL) {
      if (optimstatus == GRB_INF_OR_UNBD) {
        result = SolutionResult::kInfeasibleConstraints;
      } else {
        result = SolutionResult::kUnknownError;
      }
    }
  }

  Eigen::VectorXd sol_vector = Eigen::VectorXd::Zero(num_vars);

  GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, num_vars, sol_vector.data());
  prog.SetDecisionVariableValues(sol_vector);

  prog.SetSolverResult("Gurobi", 0);

  GRBfreemodel(model);
  GRBfreeenv(env);

  return result;
}

} // namespace drake
} // namespace solvers