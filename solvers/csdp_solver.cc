#include "drake/solvers/csdp_solver.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>

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
}  // namespace

void CsdpSolver::DoSolve(const MathematicalProgram& prog,
                         const Eigen::VectorXd&, const SolverOptions&,
                         MathematicalProgramResult* result) const {
  result->set_solver_id(CsdpSolver::id());
  const SdpaFreeFormat sdpa_free_format(prog);
  if (sdpa_free_format.num_free_variables() == 0) {
    SolveProgramWithNoFreeVariables(prog, sdpa_free_format, result);
  } else {
    throw std::runtime_error(
        "CsdpSolver::Solve(): do not support problem with free variables yet.");
  }
}

bool CsdpSolver::is_available() { return true; }
}  // namespace solvers
}  // namespace drake
