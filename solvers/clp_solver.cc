#include "drake/solvers/clp_solver.h"

#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ClpSimplex.hpp"

#include "drake/common/text_logging.h"
#include "drake/solvers/aggregate_costs_constraints.h"

namespace drake {
namespace solvers {
constexpr double kInf = std::numeric_limits<double>::infinity();

namespace {
void ConstructClpModel(
    const std::vector<Eigen::Triplet<double>>& constraint_coeffs,
    const std::vector<double>& constraint_lower,
    const std::vector<double>& constraint_upper, const Eigen::VectorXd& xlow,
    const Eigen::VectorXd& xupp,
    const Eigen::SparseMatrix<double>& quadratic_matrix,
    const Eigen::VectorXd& objective_coeff, double constant_cost,
    ClpModel* model) {
  Eigen::SparseMatrix<double> constraint_mat(constraint_lower.size(),
                                             xlow.rows());
  constraint_mat.setFromTriplets(constraint_coeffs.begin(),
                                 constraint_coeffs.end());
  // Now convert the sparse matrix to Compressed Column Storage format.
  model->loadProblem(constraint_mat.cols(), constraint_mat.rows(),
                     constraint_mat.outerIndexPtr(),
                     constraint_mat.innerIndexPtr(), constraint_mat.valuePtr(),
                     xlow.data(), xupp.data(), objective_coeff.data(),
                     constraint_lower.data(), constraint_upper.data(),
                     nullptr /* rowObjective=nullptr */);
  if (quadratic_matrix.nonZeros() > 0) {
    static const logging::Warn log_once(
        "Currently CLP solver has a memory issue when solving a QP. The user "
        "should be aware of this risk.");
    model->loadQuadraticObjective(
        quadratic_matrix.cols(), quadratic_matrix.outerIndexPtr(),
        quadratic_matrix.innerIndexPtr(), quadratic_matrix.valuePtr());
  }
  model->setObjectiveOffset(-constant_cost);
}

template <typename C>
void AddLinearConstraint(
    const MathematicalProgram& prog, const Binding<C>& linear_constraint,
    std::vector<Eigen::Triplet<double>>* constraint_coeffs,
    std::vector<double>* constraint_lower,
    std::vector<double>* constraint_upper, int* constraint_count,
    std::unordered_map<Binding<Constraint>, int>* constraint_dual_start_index) {
  const Binding<Constraint> binding_cast =
      internal::BindingDynamicCast<Constraint>(linear_constraint);
  constraint_dual_start_index->emplace(binding_cast, *constraint_count);
  const std::vector<int> variable_indices =
      prog.FindDecisionVariableIndices(linear_constraint.variables());
  for (int i = 0; i < linear_constraint.evaluator()->num_constraints(); ++i) {
    for (int j = 0; j < linear_constraint.variables().rows(); ++j) {
      const int var_index = variable_indices[j];
      if (linear_constraint.evaluator()->A()(i, j) != 0) {
        constraint_coeffs->emplace_back(
            i + *constraint_count, var_index,
            linear_constraint.evaluator()->A()(i, j));
      }
    }
    (*constraint_lower)[*constraint_count + i] =
        linear_constraint.evaluator()->lower_bound()(i);
    (*constraint_upper)[*constraint_count + i] =
        linear_constraint.evaluator()->upper_bound()(i);
  }
  *constraint_count += linear_constraint.evaluator()->num_constraints();
}

void AddLinearConstraints(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<double>>* constraint_coeffs,
    std::vector<double>* constraint_lower,
    std::vector<double>* constraint_upper,
    std::unordered_map<Binding<Constraint>, int>* constraint_dual_start_index) {
  int num_constraints = 0;
  int num_nonzero_coeff_max = 0;
  for (const auto& linear_constraint : prog.linear_constraints()) {
    num_constraints += linear_constraint.evaluator()->num_constraints();
    num_nonzero_coeff_max += linear_constraint.evaluator()->num_constraints() *
                             linear_constraint.variables().rows();
  }
  for (const auto& linear_eq_constraint : prog.linear_equality_constraints()) {
    num_constraints += linear_eq_constraint.evaluator()->num_constraints();
    num_nonzero_coeff_max +=
        linear_eq_constraint.evaluator()->num_constraints() *
        linear_eq_constraint.variables().rows();
  }
  constraint_lower->resize(num_constraints);
  constraint_upper->resize(num_constraints);
  constraint_coeffs->reserve(num_nonzero_coeff_max);
  int constraint_count = 0;
  for (const auto& linear_constraint : prog.linear_constraints()) {
    AddLinearConstraint<LinearConstraint>(
        prog, linear_constraint, constraint_coeffs, constraint_lower,
        constraint_upper, &constraint_count, constraint_dual_start_index);
  }
  for (const auto& linear_equal_constraint :
       prog.linear_equality_constraints()) {
    AddLinearConstraint<LinearEqualityConstraint>(
        prog, linear_equal_constraint, constraint_coeffs, constraint_lower,
        constraint_upper, &constraint_count, constraint_dual_start_index);
  }
}

// @param lambda The dual solution for the whole problem. This can be obtained
// through CLP by calling getRowPrice() or getReducedCost() function.
void SetConstraintDualSolution(
    const Eigen::Map<const Eigen::VectorXd>& lambda,
    const std::unordered_map<Binding<Constraint>, int>&
        constraint_dual_start_index,
    MathematicalProgramResult* result) {
  for (const auto& [constraint, dual_start_index] :
       constraint_dual_start_index) {
    result->set_dual_solution(
        constraint, lambda.segment(dual_start_index,
                                   constraint.evaluator()->num_constraints()));
  }
}

void SetBoundingBoxConstraintDualSolution(
    const Eigen::Map<const Eigen::VectorXd>& column_dual_sol,
    const std::unordered_map<Binding<BoundingBoxConstraint>,
                             std::pair<std::vector<int>, std::vector<int>>>&
        bb_con_dual_variable_indices,
    MathematicalProgramResult* result) {
  for (const auto& [bb_con, dual_var_indices] : bb_con_dual_variable_indices) {
    Eigen::VectorXd bb_con_dual_sol =
        Eigen::VectorXd::Zero(bb_con.evaluator()->num_constraints());
    const std::vector<int>& lower_dual_indices = dual_var_indices.first;
    const std::vector<int>& upper_dual_indices = dual_var_indices.second;
    for (int k = 0; k < bb_con.evaluator()->num_constraints(); ++k) {
      // At most one of the bound is active (If both the lower and upper bounds
      // are active, then these two bounds have to be equal, then the dual
      // solution for both bounds are the same).
      if (lower_dual_indices[k] != -1 &&
          column_dual_sol(lower_dual_indices[k]) > 0) {
        // If the dual solution is positive, then the lower bound is active
        // (according to the definition of reduced cost).
        bb_con_dual_sol[k] = column_dual_sol[lower_dual_indices[k]];
      } else if (upper_dual_indices[k] != -1 &&
                 column_dual_sol(upper_dual_indices[k]) < 0) {
        // If the dual solution is negative, then the upper bound is active
        // (according to the definition of reduced cost).
        bb_con_dual_sol[k] = column_dual_sol[upper_dual_indices[k]];
      }
    }

    result->set_dual_solution(bb_con, bb_con_dual_sol);
  }
}

void SetSolution(
    const MathematicalProgram& prog, const ClpModel& model,
    const std::unordered_map<Binding<Constraint>, int>&
        constraint_dual_start_index,
    const std::unordered_map<Binding<BoundingBoxConstraint>,
                             std::pair<std::vector<int>, std::vector<int>>>&
        bb_con_dual_variable_indices,
    MathematicalProgramResult* result) {
  ClpSolverDetails& solver_details =
      result->SetSolverDetailsType<ClpSolverDetails>();
  result->set_x_val(Eigen::Map<const Eigen::VectorXd>(model.getColSolution(),
                                                      prog.num_vars()));
  Eigen::Map<const Eigen::VectorXd> lambda(model.dualRowSolution(),
                                           model.getNumRows());
  SetConstraintDualSolution(lambda, constraint_dual_start_index, result);
  Eigen::Map<const Eigen::VectorXd> column_dual_sol(model.dualColumnSolution(),
                                                    model.getNumCols());
  SetBoundingBoxConstraintDualSolution(column_dual_sol,
                                       bb_con_dual_variable_indices, result);
  solver_details.status = model.status();
  SolutionResult solution_result{SolutionResult::kUnknownError};
  switch (solver_details.status) {
    case -1: {
      solution_result = SolutionResult::kUnknownError;
      break;
    }
    case 0: {
      solution_result = SolutionResult::kSolutionFound;
      break;
    }
    case 1: {
      solution_result = SolutionResult::kInfeasibleConstraints;
      break;
    }
    case 2: {
      solution_result = SolutionResult::kUnbounded;
      break;
    }
    case 3: {
      solution_result = SolutionResult::kIterationLimit;
      break;
    }
    default: {
      // Merging multiple CLP status code into one Drake SolutionResult code.
      solution_result = SolutionResult::kUnknownError;
    }
  }
  double objective_val{-kInf};
  if (solution_result == SolutionResult::kInfeasibleConstraints) {
    objective_val = kInf;
  } else if (solution_result == SolutionResult::kUnbounded) {
    objective_val = -kInf;
  } else {
    objective_val = model.getObjValue();
  }
  result->set_solution_result(solution_result);
  result->set_optimal_cost(objective_val);
}

// bb_con_dual_indices[bb_con] returns the indices of the dual variable for the
// lower/upper bound of this bounding box constraint. If this bounding box
// constraint cannot be active (some other bounding box constraint has a tighter
// bound than this one), then its dual variable index is -1.
// @param xlow The aggregated lower bound for all decision variables across all
// bounding box constraints.
// @param xupp The aggregated upper bound for all decision variables across all
// bounding box constraints.
void SetBoundingBoxConstraintDualIndices(
    const MathematicalProgram& prog,
    const Binding<BoundingBoxConstraint>& bb_con, const Eigen::VectorXd& xlow,
    const Eigen::VectorXd& xupp,
    std::unordered_map<Binding<BoundingBoxConstraint>,
                       std::pair<std::vector<int>, std::vector<int>>>*
        bbcon_dual_indices) {
  const int num_vars = bb_con.evaluator()->num_constraints();
  std::vector<int> lower_dual_indices(num_vars, -1);
  std::vector<int> upper_dual_indices(num_vars, -1);
  for (int k = 0; k < num_vars; ++k) {
    const int idx = prog.FindDecisionVariableIndex(bb_con.variables()(k));
    if (xlow[idx] == bb_con.evaluator()->lower_bound()(k)) {
      lower_dual_indices[k] = idx;
    }
    if (xupp[idx] == bb_con.evaluator()->upper_bound()(k)) {
      upper_dual_indices[k] = idx;
    }
  }
  bbcon_dual_indices->emplace(
      bb_con, std::make_pair(lower_dual_indices, upper_dual_indices));
}

void ParseModelExceptLinearConstraints(
    const MathematicalProgram& prog, Eigen::VectorXd* xlow,
    Eigen::VectorXd* xupp, Eigen::SparseMatrix<double>* quadratic_matrix,
    Eigen::VectorXd* objective_coeff, double* constant_cost) {
  // Construct model using loadProblem function.
  DRAKE_ASSERT(xlow->rows() == prog.num_vars());
  DRAKE_ASSERT(xupp->rows() == prog.num_vars());
  AggregateBoundingBoxConstraints(prog, xlow, xupp);
  Eigen::SparseVector<double> linear_coeff;
  VectorX<symbolic::Variable> quadratic_vars, linear_vars;
  Eigen::SparseMatrix<double> Q_lower;
  AggregateQuadraticAndLinearCosts(prog.quadratic_costs(), prog.linear_costs(),
                                   &Q_lower, &quadratic_vars, &linear_coeff,
                                   &linear_vars, constant_cost);
  // We need to convert 0.5 * quadratic_var.dot(Q_lower * quadratic_var) to
  // 0.5 * x.dot(quadratic_matrix * x), where x is the vector of all the
  // decision variables. To do so, we first map each entry of quadratic_var to
  // its corresponding entry in x.
  std::unordered_map<symbolic::Variable::Id, int> quadratic_var_to_index;
  for (int i = 0; i < quadratic_vars.rows(); ++i) {
    quadratic_var_to_index.emplace(
        quadratic_vars(i).get_id(),
        prog.FindDecisionVariableIndex(quadratic_vars(i)));
  }
  std::vector<Eigen::Triplet<double>> quadratic_matrix_triplets;
  quadratic_matrix_triplets.reserve(Q_lower.nonZeros());
  for (int col = 0; col < Q_lower.outerSize(); ++col) {
    const int x_col_index =
        quadratic_var_to_index.at(quadratic_vars(col).get_id());
    for (Eigen::SparseMatrix<double>::InnerIterator it(Q_lower, col); it;
         ++it) {
      const int x_row_index =
          quadratic_var_to_index.at(quadratic_vars(it.row()).get_id());
      quadratic_matrix_triplets.emplace_back(x_row_index, x_col_index,
                                             it.value());
    }
  }
  quadratic_matrix->setFromTriplets(quadratic_matrix_triplets.begin(),
                                    quadratic_matrix_triplets.end());

  objective_coeff->resize(prog.num_vars());
  for (Eigen::SparseVector<double>::InnerIterator it(linear_coeff); it; ++it) {
    (*objective_coeff)(prog.FindDecisionVariableIndex(linear_vars(it.row()))) =
        it.value();
  }
}

int ChooseLogLevel(const SolverOptions& options) {
  if (options.get_print_to_console()) {
    // Documented as "factorizations plus a bit more" in ClpModel.hpp.
    return 3;
  }
  return 0;
}
}  // namespace

bool ClpSolver::is_available() { return true; }

void ClpSolver::DoSolve(const MathematicalProgram& prog,
                        const Eigen::VectorXd& initial_guess,
                        const SolverOptions& merged_options,
                        MathematicalProgramResult* result) const {
  // TODO(hongkai.dai): use initial guess and more of the merged options.
  unused(initial_guess);
  ClpSimplex model;
  model.setLogLevel(ChooseLogLevel(merged_options));
  Eigen::VectorXd xlow(prog.num_vars());
  Eigen::VectorXd xupp(prog.num_vars());
  Eigen::VectorXd objective_coeff = Eigen::VectorXd::Zero(prog.num_vars());
  Eigen::SparseMatrix<double> quadratic_matrix(prog.num_vars(),
                                               prog.num_vars());
  double constant_cost{0.};
  ParseModelExceptLinearConstraints(prog, &xlow, &xupp, &quadratic_matrix,
                                    &objective_coeff, &constant_cost);

  std::vector<Eigen::Triplet<double>> constraint_coeffs;
  std::vector<double> constraint_lower, constraint_upper;
  std::unordered_map<Binding<Constraint>, int> constraint_dual_start_index;
  if (!prog.linear_constraints().empty() ||
      !prog.linear_equality_constraints().empty()) {
    AddLinearConstraints(prog, &constraint_coeffs, &constraint_lower,
                         &constraint_upper, &constraint_dual_start_index);
  }
  ConstructClpModel(constraint_coeffs, constraint_lower, constraint_upper, xlow,
                    xupp, quadratic_matrix, objective_coeff, constant_cost,
                    &model);
  std::unordered_map<Binding<BoundingBoxConstraint>,
                     std::pair<std::vector<int>, std::vector<int>>>
      bb_con_dual_variable_indices;
  for (const auto& bb_con : prog.bounding_box_constraints()) {
    SetBoundingBoxConstraintDualIndices(prog, bb_con, xlow, xupp,
                                        &bb_con_dual_variable_indices);
  }

  // Solve
  model.primal();

  // Set the solution
  SetSolution(prog, model, constraint_dual_start_index,
              bb_con_dual_variable_indices, result);
}
}  // namespace solvers
}  // namespace drake
