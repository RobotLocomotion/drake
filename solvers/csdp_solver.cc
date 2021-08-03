#include "drake/solvers/csdp_solver.h"

#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>

#include "drake/common/filesystem.h"
#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/csdp_solver_internal.h"

// Note that in the below, the first argument to csdp::easy_sdp is a params
// filename, but that feature is a Drake-specific patch added to the CSDP
// headers and source; refer to drake/tools/workspace/csdp/repository.bzl
// for details.

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
    if (std::holds_alternative<DecisionVariableInSdpaX>(
            sdpa_free_format.prog_var_in_sdpa()[i])) {
      const auto& decision_var_in_X = std::get<DecisionVariableInSdpaX>(
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
    } else if (std::holds_alternative<double>(
                   sdpa_free_format.prog_var_in_sdpa()[i])) {
      (*prog_sol)(i) = std::get<double>(sdpa_free_format.prog_var_in_sdpa()[i]);
    } else if (std::holds_alternative<SdpaFreeFormat::FreeVariableIndex>(
                   sdpa_free_format.prog_var_in_sdpa()[i])) {
      (*prog_sol)(i) = s_val(int{std::get<SdpaFreeFormat::FreeVariableIndex>(
          sdpa_free_format.prog_var_in_sdpa()[i])});
    }
  }
}

void SolveProgramWithNoFreeVariables(
    const MathematicalProgram& prog,
    const SdpaFreeFormat& sdpa_free_format,
    const std::string& csdp_params_pathname,
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
      csdp_params_pathname.c_str(),
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
    const std::string& csdp_params_pathname,
    MathematicalProgramResult* result) {
  static const logging::Warn log_once(
      "The problem has free variables, and CSDP removes the free "
      "variables by computing the null space of linear constraint in the "
      "dual space. This step can be time consuming. Consider providing a lower "
      "and/or upper bound for each decision variable.");
  Eigen::SparseMatrix<double> C_hat;
  std::vector<Eigen::SparseMatrix<double>> A_hat;
  Eigen::VectorXd rhs_hat, y_hat;
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> QR_B;
  sdpa_free_format.RemoveFreeVariableByNullspaceApproach(
      &C_hat, &A_hat, &rhs_hat, &y_hat, &QR_B);

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
  const int ret = csdp::easy_sdp(
      csdp_params_pathname.c_str(),
      sdpa_free_format.num_X_rows(), rhs_hat.rows(), C_csdp, rhs_csdp,
      constraints_csdp, -sdpa_free_format.constant_min_cost_term()
          + sdpa_free_format.g().dot(y_hat),
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
    const std::string& csdp_params_pathname,
    MathematicalProgramResult* result) {
  static const logging::Warn log_once(
      "The problem has free variables, and CSDP removes the free "
      "variables by introducing the slack variable y_plus >=0 , y_minus >= "
      "0, and constraint y_plus - y_minus = free_variable. This can "
      "introduce numerical problems to the solver. Consider providing a lower "
      "and/or upper bound for each decision variable.");
  std::vector<internal::BlockInX> X_hat_blocks;
  std::vector<Eigen::SparseMatrix<double>> A_hat;
  Eigen::SparseMatrix<double> C_hat;
  sdpa_free_format.RemoveFreeVariableByTwoSlackVariablesApproach(
      &X_hat_blocks, &A_hat, &C_hat);
  const int num_X_hat_rows =
      sdpa_free_format.num_X_rows() + 2 * sdpa_free_format.num_free_variables();

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
  const int ret = csdp::easy_sdp(
      csdp_params_pathname.c_str(),
      num_X_hat_rows, sdpa_free_format.g().rows(), C_csdp, rhs_csdp,
      constraints_csdp, -sdpa_free_format.constant_min_cost_term(),
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

/*
 * For the problem
 * max tr(C * X) + dᵀs
 * s.t tr(Aᵢ*X) + bᵢᵀs = aᵢ
 *     X ≽ 0
 *     s is free.
 * Remove the free variable s by introducing a slack variable t with the
 * Lorentz cone constraint t ≥ sqrt(sᵀs). We get a new program without free
 * variables.
 * max tr(Ĉ * X̂)
 * s.t tr(Âᵢ*X̂) = aᵢ
 *     X̂ ≽ 0
 * Refer to RemoveFreeVariableByLorentzConeSlackApproach in sdpa_free_format.h
 * for more details.
 */
void SolveProgramThroughLorentzConeSlackApproach(
    const MathematicalProgram& prog, const SdpaFreeFormat& sdpa_free_format,
    const std::string& csdp_params_pathname,
    MathematicalProgramResult* result) {
  static const logging::Warn log_once(
      "The problem has free variables, and CSDP removes the free "
      "variables by introducing a slack variable t with the Lorentz cone "
      "constraint t>= sqrt(s'*s) This can introduce numerical problems to the "
      "solver. Consider providing a lower "
      "and/or upper bound for each decision variable.");
  std::vector<internal::BlockInX> X_hat_blocks;
  std::vector<Eigen::SparseMatrix<double>> A_hat;
  Eigen::VectorXd rhs_hat;
  Eigen::SparseMatrix<double> C_hat;
  sdpa_free_format.RemoveFreeVariableByLorentzConeSlackApproach(
      &X_hat_blocks, &A_hat, &rhs_hat, &C_hat);
  const int num_X_hat_rows =
      sdpa_free_format.num_X_rows() + sdpa_free_format.num_free_variables() + 1;

  // Now try to call CSDP to solve this problem.
  csdp::blockmatrix C_csdp;
  double* rhs_csdp{nullptr};
  csdp::constraintmatrix* constraints_csdp{nullptr};
  ConvertSparseMatrixFormatToCsdpProblemData(X_hat_blocks, C_hat, A_hat,
                                             rhs_hat, &C_csdp, &rhs_csdp,
                                             &constraints_csdp);
  struct csdp::blockmatrix X_csdp, Z;
  double* y{nullptr};
  csdp::initsoln(num_X_hat_rows, rhs_hat.rows(), C_csdp, rhs_csdp,
                 constraints_csdp, &X_csdp, &y, &Z);
  double pobj{0};
  double dobj{0};
  const int ret = csdp::easy_sdp(
      csdp_params_pathname.c_str(),
      num_X_hat_rows, rhs_hat.rows(), C_csdp, rhs_csdp, constraints_csdp,
      -sdpa_free_format.constant_min_cost_term(),
      &X_csdp, &y, &Z, &pobj, &dobj);
  Eigen::SparseMatrix<double> X_hat(num_X_hat_rows, num_X_hat_rows);
  ConvertCsdpBlockMatrixtoEigen(X_csdp, &X_hat);
  // Now retrieve the value for the free variable s from Y.
  Eigen::VectorXd s_val =
      Eigen::VectorXd::Zero(sdpa_free_format.num_free_variables());
  for (int i = 0; i < sdpa_free_format.num_free_variables(); ++i) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(
             X_hat, sdpa_free_format.num_X_rows() + i + 1);
         it; ++it) {
      s_val(i) = it.value();
      // There are two non-zero entries in this column, Y(0, i+1) = s(i) and
      // Y(i+1, i+1) = t(i). We only care about the first non-zero entry Y(0,
      // i+1), so break here.
      break;
    }
  }

  CsdpSolverDetails& solver_details =
      result->SetSolverDetailsType<CsdpSolverDetails>();
  SetCsdpSolverDetails(ret, pobj, dobj, rhs_hat.rows(), y, Z, &solver_details);
  // Retrieve the information back to result.
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

  csdp::free_prob(num_X_hat_rows, rhs_hat.rows(), C_csdp, rhs_csdp,
                  constraints_csdp, X_csdp, y, Z);
}

std::string MaybeWriteCsdpParams(const SolverOptions& options) {
  // We'll consolidate options into this config string to pass to CSDP.
  std::string all_csdp_params;

  // Map the common options into CSDP's conventions.
  if (options.get_print_to_console()) {
    all_csdp_params += "printlevel=1\n";
  }

  // TODO(jwnimmer-tri) We could pass through the other named options here,
  // if we wanted to.

  if (all_csdp_params.empty()) {
    // No need to write a temporary file.
    return {};
  }

  // Choose a temporary filename.
  const char* dir = std::getenv("TEST_TMPDIR");
  if (!dir) { dir = std::getenv("TMPDIR"); }
  if (!dir) { dir = "/tmp"; }
  filesystem::path path_template(dir);
  path_template.append("robotlocomotion_drake_XXXXXX");
  std::string result = path_template;
  int fd = ::mkstemp(result.data());
  DRAKE_THROW_UNLESS(fd != -1);
  const size_t total = all_csdp_params.size();
  ssize_t written = ::write(fd, all_csdp_params.data(), total);
  DRAKE_THROW_UNLESS(written != -1);
  DRAKE_THROW_UNLESS(written == static_cast<ssize_t>(total));
  ::close(fd);

  return result;
}

}  // namespace

void CsdpSolver::DoSolve(const MathematicalProgram& prog,
                         const Eigen::VectorXd&,
                         const SolverOptions& merged_options,
                         MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
        "CsdpSolver doesn't support the feature of variable scaling.");
  }

  // If necessary, write the custom CSDP parameters to a temporary file, which
  // we should remove when this function returns.
  const std::string csdp_params_pathname =
      MaybeWriteCsdpParams(merged_options);
  ScopeExit guard([&csdp_params_pathname]() {
    if (!csdp_params_pathname.empty()) {
      ::unlink(csdp_params_pathname.c_str());
    }
  });

  result->set_solver_id(CsdpSolver::id());
  const SdpaFreeFormat sdpa_free_format(prog);
  if (sdpa_free_format.num_free_variables() == 0) {
    SolveProgramWithNoFreeVariables(
        prog, sdpa_free_format, csdp_params_pathname, result);
  } else {
    switch (method_) {
      case RemoveFreeVariableMethod::kNullspace: {
        SolveProgramThroughNullspaceApproach(
            prog, sdpa_free_format, csdp_params_pathname, result);
        break;
      }
      case RemoveFreeVariableMethod::kTwoSlackVariables: {
        SolveProgramThroughTwoSlackVariablesApproach(
            prog, sdpa_free_format, csdp_params_pathname, result);
        break;
      }
      case RemoveFreeVariableMethod::kLorentzConeSlack: {
        SolveProgramThroughLorentzConeSlackApproach(
            prog, sdpa_free_format, csdp_params_pathname, result);
        break;
      }
    }
  }
}

bool CsdpSolver::is_available() { return true; }
}  // namespace solvers
}  // namespace drake
