#include "drake/solvers/clp_solver.h"

#include <limits>
#include <vector>

#include "ClpSimplex.hpp"

#include "drake/solvers/aggregate_costs_constraints.h"

namespace drake {
namespace solvers {
constexpr double kInf = std::numeric_limits<double>::infinity();

namespace {
void ConstructClpModel(
    const std::vector<Eigen::Triplet<double>>& constraint_coeffs,
    const std::vector<double>& constraint_lower,
    const std::vector<double>& constraint_upper, const Eigen::VectorXd& xlow,
    const Eigen::VectorXd& xupp, const Eigen::VectorXd& objective_coeff,
    double constant_cost, ClpModel* model) {
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
  model->setObjectiveOffset(-constant_cost);
}

template <typename C>
void AddLinearConstraint(const MathematicalProgram& prog,
                         const Binding<C>& linear_constraint,
                         std::vector<Eigen::Triplet<double>>* constraint_coeffs,
                         std::vector<double>* constraint_lower,
                         std::vector<double>* constraint_upper,
                         int* constraint_count) {
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
    std::vector<double>* constraint_upper) {
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
    AddLinearConstraint<LinearConstraint>(prog, linear_constraint,
                                          constraint_coeffs, constraint_lower,
                                          constraint_upper, &constraint_count);
  }
  for (const auto& linear_equal_constraint :
       prog.linear_equality_constraints()) {
    AddLinearConstraint<LinearEqualityConstraint>(
        prog, linear_equal_constraint, constraint_coeffs, constraint_lower,
        constraint_upper, &constraint_count);
  }
}

void SetSolution(const MathematicalProgram& prog, const ClpModel& model,
                 MathematicalProgramResult* result) {
  ClpSolverDetails& solver_details =
      result->SetSolverDetailsType<ClpSolverDetails>();
  result->set_x_val(Eigen::Map<const Eigen::VectorXd>(model.getColSolution(),
                                                      prog.num_vars()));
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

void ParseModelExceptLinearConstraints(const MathematicalProgram& prog,
                                       Eigen::VectorXd* xlow,
                                       Eigen::VectorXd* xupp,
                                       Eigen::VectorXd* objective_coeff,
                                       double* constant_cost) {
  // Construct model using loadProblem function.
  DRAKE_ASSERT(xlow->rows() == prog.num_vars());
  DRAKE_ASSERT(xupp->rows() == prog.num_vars());
  AggregateBoundingBoxConstraints(prog, xlow, xupp);
  Eigen::SparseVector<double> linear_coeff;
  VectorX<symbolic::Variable> vars;
  AggregateLinearCosts(prog.linear_costs(), &linear_coeff, &vars,
                       constant_cost);
  DRAKE_ASSERT(objective_coeff->rows() == prog.num_vars());
  for (Eigen::SparseVector<double>::InnerIterator it(linear_coeff); it; ++it) {
    (*objective_coeff)(prog.FindDecisionVariableIndex(vars(it.row()))) =
        it.value();
  }
}
}  // namespace

bool ClpSolver::is_available() { return true; }

void ClpSolver::DoSolve(const MathematicalProgram& prog,
                        const Eigen::VectorXd& initial_guess,
                        const SolverOptions& merged_options,
                        MathematicalProgramResult* result) const {
  // TODO(hongkai.dai): use initial guess and merged options.
  unused(initial_guess);
  unused(merged_options);
  ClpSimplex model;
  Eigen::VectorXd xlow(prog.num_vars());
  Eigen::VectorXd xupp(prog.num_vars());
  Eigen::VectorXd objective_coeff = Eigen::VectorXd::Zero(prog.num_vars());
  double constant_cost{0.};
  ParseModelExceptLinearConstraints(prog, &xlow, &xupp, &objective_coeff,
                                    &constant_cost);

  std::vector<Eigen::Triplet<double>> constraint_coeffs;
  std::vector<double> constraint_lower, constraint_upper;
  if (!prog.linear_constraints().empty() ||
      !prog.linear_equality_constraints().empty()) {
    AddLinearConstraints(prog, &constraint_coeffs, &constraint_lower,
                         &constraint_upper);
  }
  // TODO(hongkai.dai@tri.global): set the dual solution indices also.
  ConstructClpModel(constraint_coeffs, constraint_lower, constraint_upper, xlow,
                    xupp, objective_coeff, constant_cost, &model);

  // Solve
  model.primal();

  // Set the solution
  SetSolution(prog, model, result);
}
}  // namespace solvers
}  // namespace drake
