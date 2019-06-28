#include "drake/solvers/csdp_solver.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>

#include "drake/common/text_logging.h"
#include "drake/solvers/csdp_solver_internal.h"

namespace drake {
namespace solvers {
namespace {
using internal::ConvertCsdpBlockMatrixtoEigen;
using internal::SdpaFreeFormat;
using internal::DecisionVariableInSdpaX;
using internal::CsdpMatrixIndex;


void SetCsdpSolverDetails(int csdp_ret, double pobj, double dobj, int y_size,
                          double* y, const csdp::blockmatrix& Z,
                          CsdpSolverDetails* solver_details) {
  solver_details->return_code = csdp_ret;
  solver_details->primal_objective = pobj;
  solver_details->dual_objective = dobj;
  solver_details->y_val.resize(y_size);
  for (int i = 0; i < y_size; ++i) {
    // CSDP uses Fortran 1-index array so the array y points to will be of size
    // y_size + 1 with the zero entry ignored.
    solver_details->y_val(i) = y[i + 1];
  }
  ConvertCsdpBlockMatrixtoEigen(Z, &(solver_details->Z_val));
}

SolutionResult ConvertCsdpReturnToSolutionResult(int csdp_ret) {
  switch (csdp_ret) {
    case 0:
    case 3:
      return SolutionResult::kSolutionFound;
    case 1:
      return SolutionResult::kInfeasibleConstraints;
    case 2:
      return SolutionResult::kDualInfeasible;
    case 4:
      return SolutionResult::kIterationLimit;
    default:
      return SolutionResult::kUnknownError;
  }
}

void SetProgramSolution(const SdpaFreeFormat& sdpa_free_format,
                        const csdp::blockmatrix& X,
                        const Eigen::VectorXd& s_val,
                        Eigen::VectorXd* prog_sol) {
  for (int i = 0;
       i < static_cast<int>(sdpa_free_format.prog_var_in_sdpa().size()); ++i) {
    if (holds_alternative<DecisionVariableInSdpaX>(
            sdpa_free_format.prog_var_in_sdpa()[i])) {
      const auto& decision_var_in_X = get<DecisionVariableInSdpaX>(
          sdpa_free_format.prog_var_in_sdpa()[i]);

      double X_entry_val{};
      switch (X.blocks[decision_var_in_X.entry_in_X.block_index + 1]
                  .blockcategory) {
        case csdp::MATRIX: {
          X_entry_val =
              X.blocks[decision_var_in_X.entry_in_X.block_index + 1]
                  .data.mat[CsdpMatrixIndex(
                      decision_var_in_X.entry_in_X.row_index_in_block,
                      decision_var_in_X.entry_in_X.column_index_in_block,
                      X.blocks[decision_var_in_X.entry_in_X.block_index + 1]
                          .blocksize)];
          break;
        }
        case csdp::DIAG: {
          X_entry_val =
              X.blocks[decision_var_in_X.entry_in_X.block_index + 1]
                  .data
                  .vec[decision_var_in_X.entry_in_X.row_index_in_block + 1];
          break;
        }
        default: {
          throw std::runtime_error(
              "SetProgramSolution(): unknown X block type.");
        }
      }
      (*prog_sol)(i) =
          decision_var_in_X.offset +
          (decision_var_in_X.coeff_sign == internal::Sign::kPositive
               ? X_entry_val
               : -X_entry_val);
    } else if (holds_alternative<double>(
                   sdpa_free_format.prog_var_in_sdpa()[i])) {
      (*prog_sol)(i) = get<double>(sdpa_free_format.prog_var_in_sdpa()[i]);
    } else if (holds_alternative<SdpaFreeFormat::FreeVariableIndex>(
                   sdpa_free_format.prog_var_in_sdpa()[i])) {
      (*prog_sol)(i) = s_val(get<SdpaFreeFormat::FreeVariableIndex>(
          sdpa_free_format.prog_var_in_sdpa()[i]));
    }
  }
}

void SolveProgramWithNoFreeVariables(
    const MathematicalProgram& prog,
    const SdpaFreeFormat& sdpa_free_format,
    MathematicalProgramResult* result) {
  DRAKE_ASSERT(sdpa_free_format.num_free_variables() == 0);

  csdp::blockmatrix C;
  double* rhs;
  csdp::constraintmatrix* constraints{nullptr};
  GenerateCsdpProblemDataWithoutFreeVariables(sdpa_free_format, &C, &rhs,
                                              &constraints);

  struct csdp::blockmatrix X, Z;
  double* y;
  csdp::initsoln(sdpa_free_format.num_X_rows(), sdpa_free_format.g().rows(), C,
                 rhs, constraints, &X, &y, &Z);
  double pobj, dobj;
  const int ret = csdp::easy_sdp(
      sdpa_free_format.num_X_rows(), sdpa_free_format.g().rows(), C, rhs,
      constraints, -sdpa_free_format.constant_min_cost_term(), &X, &y, &Z,
      &pobj, &dobj);

  // Set solver details.
  CsdpSolverDetails& solver_details =
      result->SetSolverDetailsType<CsdpSolverDetails>();
  SetCsdpSolverDetails(ret, pobj, dobj, sdpa_free_format.g().rows(), y, Z,
                       &solver_details);
  // Retrieve the information back to result.
  // Since CSDP solves a maximization problem max -cost, where "cost" is the
  // minimization cost in MathematicalProgram, we need to negate the cost.
  result->set_optimal_cost(
      ret == 1 /* primal infeasible */ ? MathematicalProgram::
                                             kGlobalInfeasibleCost
                                       : -pobj);
  result->set_solution_result(ConvertCsdpReturnToSolutionResult(ret));
  Eigen::VectorXd prog_sol(prog.num_vars());
  SetProgramSolution(sdpa_free_format, X, Eigen::VectorXd::Zero(0), &prog_sol);
  result->set_x_val(prog_sol);

  csdp::free_prob(sdpa_free_format.num_X_rows(), sdpa_free_format.g().rows(), C,
                  rhs, constraints, X, y, Z);
}


void SolveProgramThroughNullspaceApproach(
    const MathematicalProgram& prog, const SdpaFreeFormat& sdpa_free_format,
    MathematicalProgramResult* result) {
  drake::log()->warn(
      "The problem has free variables, and CSDP removes the free "
      "variables by computing the null space of linear constraint in the "
      "dual space. This step can be time consuming. Consider providing a lower "
      "and/or upper bound for each decision variable.");
  Eigen::SparseMatrix<double> C_hat;
  std::vector<Eigen::SparseMatrix<double>> A_hat;
  Eigen::VectorXd rhs_hat, y_hat;
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> QR_B;
  RemoveFreeVariableByNullspaceApproach(sdpa_free_format, &C_hat, &A_hat,
                                        &rhs_hat, &y_hat, &QR_B);

  // Now try to call CSDP to solve this problem.
  csdp::blockmatrix C_csdp;
  double* rhs_csdp{nullptr};
  csdp::constraintmatrix* constraints_csdp{nullptr};
  ConvertSparseMatrixFormatToCsdpProblemData(sdpa_free_format.X_blocks(),
                                              C_hat, A_hat, rhs_hat, &C_csdp,
                                              &rhs_csdp, &constraints_csdp);
  struct csdp::blockmatrix X_csdp, Z;
  double* y{nullptr};
  csdp::initsoln(sdpa_free_format.num_X_rows(), rhs_hat.rows(), C_csdp,
                 rhs_csdp, constraints_csdp, &X_csdp, &y, &Z);
  double pobj{0};
  double dobj{0};
  const int ret = csdp::easy_sdp(sdpa_free_format.num_X_rows(), rhs_hat.rows(),
                                 C_csdp, rhs_csdp, constraints_csdp,
                                 -sdpa_free_format.constant_min_cost_term() +
                                     sdpa_free_format.g().dot(y_hat),
                                 &X_csdp, &y, &Z, &pobj, &dobj);
  Eigen::SparseMatrix<double> X_hat(sdpa_free_format.num_X_rows(),
                                    sdpa_free_format.num_X_rows());
  ConvertCsdpBlockMatrixtoEigen(X_csdp, &X_hat);
  // Now compute the free variable values.
  // AX(i) is trace(Ai, X_hat)
  Eigen::VectorXd AX(sdpa_free_format.A().size());
  for (int i = 0; i < AX.rows(); ++i) {
    AX(i) = (sdpa_free_format.A()[i].cwiseProduct(X_hat)).sum();
  }
  Eigen::VectorXd s_val;
  s_val = QR_B.solve(sdpa_free_format.g() - AX);
  // Set solver details.
  CsdpSolverDetails& solver_details =
      result->SetSolverDetailsType<CsdpSolverDetails>();
  SetCsdpSolverDetails(ret, pobj, dobj, rhs_hat.rows(), y, Z,
                       &solver_details);
  // Retrieve the information back to result
  // Since CSDP solves a maximization problem max -cost, where "cost" is the
  // minimization cost in MathematicalProgram, we need to negate the cost.
  result->set_solution_result(ConvertCsdpReturnToSolutionResult(ret));
  result->set_optimal_cost(
      ret == 1 /* primal infeasible */ ? MathematicalProgram::
                                             kGlobalInfeasibleCost
                                       : -pobj);
  Eigen::VectorXd prog_sol(prog.num_vars());
  SetProgramSolution(sdpa_free_format, X_csdp, s_val, &prog_sol);
  result->set_x_val(prog_sol);

  csdp::free_prob(sdpa_free_format.num_X_rows(), rhs_hat.rows(), C_csdp,
                  rhs_csdp, constraints_csdp, X_csdp, y, Z);
}

/**
 * For the problem
 * max tr(C * X) + dᵀs
 * s.t tr(Aᵢ*X) + bᵢᵀs = aᵢ
 *     X ≽ 0
 *     s is free.
 * Remove the free variable s by introducing two slack variables y⁺ ≥ 0 and y⁻
 * ≥ 0, and the constraint y⁺ - y⁻ = s. We get a new program without free
 * variables.
 * max tr(Ĉ * X̂)
 * s.t tr(Âᵢ*X̂) = aᵢ
 *     X̂ ≽ 0
 * where Ĉ = diag(C, diag(d), -diag(d))
 *       X̂ = diag(X, diag(y⁺), diag(y⁻))
 *       Âᵢ = diag(Aᵢ, diag(bᵢ), -diag(bᵢ))
 */
void SolveProgramThroughTwoSlackVariablesApproach(
    const MathematicalProgram& prog, const SdpaFreeFormat& sdpa_free_format,
    MathematicalProgramResult* result) {
  drake::log()->warn(
      "The problem has free variables, and CSDP removes the free "
      "variables by introducing the slack variable y_plus >=0 , y_minus >= "
      "0, and constraint y_plus - y_minus = free_variable. This can "
      "introduce numerical problems to the solver. Consider providing a lower "
      "and/or upper bound for each decision variable.");
  std::vector<internal::BlockInX> X_hat_blocks = sdpa_free_format.X_blocks();
  X_hat_blocks.emplace_back(internal::BlockType::kDiagonal,
                            2 * sdpa_free_format.num_free_variables());
  const int num_X_hat_rows =
      sdpa_free_format.num_X_rows() + 2 * sdpa_free_format.num_free_variables();
  std::vector<std::vector<Eigen::Triplet<double>>> A_hat_triplets =
      sdpa_free_format.A_triplets();
  for (int j = 0; j < sdpa_free_format.num_free_variables(); ++j) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(sdpa_free_format.B(), j);
         it; ++it) {
      const int i = it.row();
      // Add the entry in Âᵢ that multiplies with sⱼ.
      A_hat_triplets[i].emplace_back(sdpa_free_format.num_X_rows() + j,
                                     sdpa_free_format.num_X_rows() + j,
                                     it.value());
      A_hat_triplets[i].emplace_back(
          sdpa_free_format.num_X_rows() +
              sdpa_free_format.num_free_variables() + j,
          sdpa_free_format.num_X_rows() +
              sdpa_free_format.num_free_variables() + j,
          -it.value());
    }
  }
  std::vector<Eigen::SparseMatrix<double>> A_hat;
  A_hat.reserve(sdpa_free_format.A().size());
  for (int i = 0; i < static_cast<int>(sdpa_free_format.A().size()); ++i) {
    A_hat.emplace_back(num_X_hat_rows, num_X_hat_rows);
    A_hat.back().setFromTriplets(A_hat_triplets[i].begin(),
                                 A_hat_triplets[i].end());
  }
  // Add the entry in Ĉ that multiplies with sᵢ
  std::vector<Eigen::Triplet<double>> C_hat_triplets =
      sdpa_free_format.C_triplets();
  for (Eigen::SparseMatrix<double>::InnerIterator it(sdpa_free_format.d(), 0);
       it; ++it) {
    const int i = it.row();
    C_hat_triplets.emplace_back(sdpa_free_format.num_X_rows() + i,
                                sdpa_free_format.num_X_rows() + i, it.value());
    C_hat_triplets.emplace_back(sdpa_free_format.num_X_rows() +
                                    sdpa_free_format.num_free_variables() + i,
                                sdpa_free_format.num_X_rows() +
                                    sdpa_free_format.num_free_variables() + i,
                                -it.value());
  }
  Eigen::SparseMatrix<double> C_hat(num_X_hat_rows, num_X_hat_rows);
  C_hat.setFromTriplets(C_hat_triplets.begin(), C_hat_triplets.end());

  // Now try to call CSDP to solve this problem.
  csdp::blockmatrix C_csdp;
  double* rhs_csdp{nullptr};
  csdp::constraintmatrix* constraints_csdp{nullptr};
  ConvertSparseMatrixFormatToCsdpProblemData(X_hat_blocks, C_hat, A_hat,
                                              sdpa_free_format.g(), &C_csdp,
                                              &rhs_csdp, &constraints_csdp);
  struct csdp::blockmatrix X_csdp, Z;
  double* y{nullptr};
  csdp::initsoln(num_X_hat_rows, sdpa_free_format.g().rows(), C_csdp, rhs_csdp,
                 constraints_csdp, &X_csdp, &y, &Z);
  double pobj{0};
  double dobj{0};
  const int ret = csdp::easy_sdp(num_X_hat_rows, sdpa_free_format.g().rows(),
                                 C_csdp, rhs_csdp, constraints_csdp,
                                 -sdpa_free_format.constant_min_cost_term(),
                                 &X_csdp, &y, &Z, &pobj, &dobj);
  Eigen::SparseMatrix<double> X_hat(num_X_hat_rows, num_X_hat_rows);
  ConvertCsdpBlockMatrixtoEigen(X_csdp, &X_hat);
  // Now retrieve the value for the free variable s as y⁺ - y⁻.
  Eigen::VectorXd s_val(sdpa_free_format.num_free_variables());
  for (int i = 0; i < sdpa_free_format.num_free_variables(); ++i) {
    double y_plus_val{0};
    for (Eigen::SparseMatrix<double>::InnerIterator it(
             X_hat, sdpa_free_format.num_X_rows() + i);
         it; ++it) {
      y_plus_val = it.value();
    }
    double y_minus_val{0};
    for (Eigen::SparseMatrix<double>::InnerIterator it(
             X_hat, sdpa_free_format.num_X_rows() +
                        sdpa_free_format.num_free_variables() + i);
         it; ++it) {
      y_minus_val = it.value();
    }
    s_val(i) = y_plus_val - y_minus_val;
  }

  CsdpSolverDetails& solver_details =
      result->SetSolverDetailsType<CsdpSolverDetails>();
  SetCsdpSolverDetails(ret, pobj, dobj, sdpa_free_format.g().rows(), y, Z,
                       &solver_details);
  // Retrieve the information back to result
  // Since CSDP solves a maximization problem max -cost, where "cost" is the
  // minimization cost in MathematicalProgram, we need to negate the cost.
  result->set_solution_result(ConvertCsdpReturnToSolutionResult(ret));
  result->set_optimal_cost(
      ret == 1 /* primal infeasible */ ? MathematicalProgram::
                                             kGlobalInfeasibleCost
                                       : -pobj);
  Eigen::VectorXd prog_sol(prog.num_vars());
  SetProgramSolution(sdpa_free_format, X_csdp, s_val, &prog_sol);
  result->set_x_val(prog_sol);

  csdp::free_prob(num_X_hat_rows, sdpa_free_format.g().rows(), C_csdp, rhs_csdp,
                  constraints_csdp, X_csdp, y, Z);
}
}  // namespace

void CsdpSolver::DoSolve(const MathematicalProgram& prog,
                         const Eigen::VectorXd&, const SolverOptions&,
                         MathematicalProgramResult* result) const {
  result->set_solver_id(CsdpSolver::id());
  const SdpaFreeFormat sdpa_free_format(prog);
  if (sdpa_free_format.num_free_variables() == 0) {
    SolveProgramWithNoFreeVariables(prog, sdpa_free_format, result);
  } else {
    switch (method_) {
      case RemoveFreeVariableMethod::kNullspace: {
        SolveProgramThroughNullspaceApproach(prog, sdpa_free_format,
                                                       result);
        break;
      }
      case RemoveFreeVariableMethod::kTwoSlackVariables: {
         SolveProgramThroughTwoSlackVariablesApproach(
            prog, sdpa_free_format, result);
        break;
      }
    }
  }
}

bool CsdpSolver::is_available() { return true; }
}  // namespace solvers
}  // namespace drake
