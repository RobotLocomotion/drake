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
namespace internal {

void ConvertSparseMatrixFormattToCsdpProblemData(
    const std::vector<BlockInX>& X_blocks, const Eigen::SparseMatrix<double>& C,
    const std::vector<Eigen::SparseMatrix<double>> A,
    const Eigen::VectorXd& rhs, csdp::blockmatrix* C_csdp, double** rhs_csdp,
    csdp::constraintmatrix** constraints) {
  const int num_X_rows = C.rows();
  DRAKE_ASSERT(C.cols() == num_X_rows);
  DRAKE_ASSERT(static_cast<int>(A.size()) == rhs.rows());

  // maps the row index in X to the block index. Both the row index and the
  // block index are 0-indexed.
  std::vector<int> X_row_to_block_index(num_X_rows);
  std::vector<int> block_start_rows(static_cast<int>(X_blocks.size()));
  int row_count = 0;
  for (int i = 0; i < static_cast<int>(X_blocks.size()); ++i) {
    block_start_rows[i] = row_count;
    for (int row = row_count; row < row_count + X_blocks[i].num_rows; ++row) {
      X_row_to_block_index[row] = i;
    }
    row_count += static_cast<int>(X_blocks[i].num_rows);
  }

  C_csdp->nblocks = static_cast<int>(X_blocks.size());
  C_csdp->blocks = static_cast<struct csdp::blockrec*>(
      /* We need to add 1 here because CSDP is uses Fortran 1-indexed, so the
         0'th block is wasted. */
      malloc((C_csdp->nblocks + 1) * sizeof(struct csdp::blockrec)));
  for (int m = 0; m < C_csdp->nblocks; ++m) {
    // CSDP uses Fortran index, so we need to add 1.
    C_csdp->blocks[m + 1].blockcategory =
        X_blocks[m].block_type == BlockType::kMatrix ? csdp::MATRIX
                                                     : csdp::DIAG;
    C_csdp->blocks[m + 1].blocksize = X_blocks[m].num_rows;
    if (X_blocks[m].block_type == BlockType::kMatrix) {
      C_csdp->blocks[m + 1].data.mat = static_cast<double*>(
          malloc(X_blocks[m].num_rows * X_blocks[m].num_rows * sizeof(double)));
      for (int j = 0; j < X_blocks[m].num_rows; ++j) {
        // First fill in this column with 0, and then we will go through the
        // non-zero entries (stored inside  C) to set the value of the
        // corresponding entries in C_csdp.
        for (int i = 0; i < X_blocks[m].num_rows; ++i) {
          C_csdp->blocks[m + 1]
              .data.mat[ijtok(i + 1, j + 1, X_blocks[m].num_rows)] = 0;
        }
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 C, block_start_rows[m] + j);
             it; ++it) {
          C_csdp->blocks[m + 1]
              .data.mat[ijtok(it.row() - block_start_rows[m] + 1, j + 1,
                              X_blocks[m].num_rows)] = it.value();
        }
      }
    } else if (X_blocks[m].block_type == BlockType::kDiagonal) {
      C_csdp->blocks[m + 1].data.vec = static_cast<double*>(
          // CSDP uses Fortran 1-index array, so the 0'th entry is wasted.
          malloc((X_blocks[m].num_rows + 1) * sizeof(double)));
      for (int j = 0; j < X_blocks[m].num_rows; ++j) {
        C_csdp->blocks[m + 1].data.vec[j + 1] = 0.0;
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 C, block_start_rows[m] + j);
             it; ++it) {
          DRAKE_ASSERT(it.row() == it.col());
          C_csdp->blocks[m + 1].data.vec[j + 1] = it.value();
        }
      }
    } else {
      throw std::runtime_error(
          "ConvertSparseMatrixFormattToCsdpProblemData() only support MATRIX "
          "or DIAG blocks.");
    }
  }

  // copy rhs
  *rhs_csdp = static_cast<double*>(malloc((rhs.rows() + 1) * sizeof(double)));
  for (int i = 0; i < rhs.rows(); ++i) {
    (*rhs_csdp)[i + 1] = rhs(i);
  }

  // Copy constraints.
  *constraints = static_cast<struct csdp::constraintmatrix*>(
      malloc((static_cast<int>(A.size()) + 1) *
             sizeof(struct csdp::constraintmatrix)));
  struct csdp::sparseblock* blockptr;
  for (int constraint_index = 0; constraint_index < static_cast<int>(A.size());
       ++constraint_index) {
    (*constraints)[constraint_index + 1].blocks = nullptr;
    // Start from the last block in the block-diagonal matrix
    // A[constraint_index], we add each block in the reverse order.
    for (int block_index = static_cast<int>(X_blocks.size() - 1);
         block_index >= 0; --block_index) {
      std::vector<Eigen::Triplet<double>> A_block_triplets;
      // CSDP only stores the non-zero entries in the upper-triangular part of
      // each block. Also the row and column indices in A_block_triplets are
      // the indices within THIS block, not the indices in the whole matrix
      // A[constraint_index].
      A_block_triplets.reserve((X_blocks[block_index].num_rows + 1) *
                               X_blocks[block_index].num_rows / 2);
      for (int col_index = block_start_rows[block_index];
           col_index <
           block_start_rows[block_index] + X_blocks[block_index].num_rows;
           ++col_index) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A[constraint_index],
                                                           col_index);
             it; ++it) {
          if (it.row() > it.col()) {
            break;
          }
          A_block_triplets.emplace_back(
              it.row() - block_start_rows[block_index] + 1,
              it.col() - block_start_rows[block_index] + 1, it.value());
        }
      }
      if (!A_block_triplets.empty()) {
        blockptr = static_cast<struct csdp::sparseblock*>(
            malloc(sizeof(struct csdp::sparseblock)));
        // CSDP uses Fortran 1-indexed array.
        blockptr->blocknum = block_index + 1;
        blockptr->blocksize = X_blocks[block_index].num_rows;
        // CSDP uses Fortran 1-indexed array.
        blockptr->constraintnum = constraint_index + 1;
        blockptr->next = nullptr;
        blockptr->nextbyblock = nullptr;
        blockptr->entries = static_cast<double*>(malloc(
            (1 + static_cast<int>(A_block_triplets.size())) * sizeof(double)));
        blockptr->iindices = static_cast<int*>(malloc(
            (1 + static_cast<int>(A_block_triplets.size())) * sizeof(int)));
        blockptr->jindices = static_cast<int*>(malloc(
            (1 + static_cast<int>(A_block_triplets.size())) * sizeof(int)));
        blockptr->numentries = static_cast<int>(A_block_triplets.size());
        for (int i = 0; i < blockptr->numentries; ++i) {
          blockptr->iindices[i + 1] = A_block_triplets[i].row();
          blockptr->jindices[i + 1] = A_block_triplets[i].col();
          blockptr->entries[i + 1] = A_block_triplets[i].value();
        }
        // Insert this block into the linked list of
        // constraints[constraint_index + 1] blocks.
        blockptr->next = (*constraints)[constraint_index + 1].blocks;
        (*constraints)[constraint_index + 1].blocks = blockptr;
      }
    }
  }
}

void GenerateCsdpProblemDataWithoutFreeVariables(
    const SdpaFreeFormat& sdpa_free_format, csdp::blockmatrix* C_csdp,
    double** rhs_csdp, csdp::constraintmatrix** constraints) {
  if (sdpa_free_format.num_free_variables() == 0) {
    Eigen::SparseMatrix<double> C(sdpa_free_format.num_X_rows(),
                                  sdpa_free_format.num_X_rows());
    C.setFromTriplets(sdpa_free_format.C_triplets().begin(),
                      sdpa_free_format.C_triplets().end());
    std::vector<Eigen::SparseMatrix<double>> A;
    A.reserve(sdpa_free_format.A_triplets().size());
    for (int i = 0; i < static_cast<int>(sdpa_free_format.A_triplets().size());
         ++i) {
      A.emplace_back(sdpa_free_format.num_X_rows(),
                     sdpa_free_format.num_X_rows());
      A.back().setFromTriplets(sdpa_free_format.A_triplets()[i].begin(),
                               sdpa_free_format.A_triplets()[i].end());
    }
    ConvertSparseMatrixFormattToCsdpProblemData(sdpa_free_format.X_blocks(), C,
                                                A, sdpa_free_format.g(), C_csdp,
                                                rhs_csdp, constraints);
  } else {
    throw std::runtime_error(
        "GenerateCsdpProblemDataWithoutFreeVariables(): the formulation has "
        "free variables, you should't call this method.");
  }
}

void ConvertCsdpBlockMatrixtoEigen(const csdp::blockmatrix& X_csdp,
                                   Eigen::SparseMatrix<double>* X) {
  int num_X_nonzero_entries = 0;
  for (int i = 0; i < X_csdp.nblocks; ++i) {
    if (X_csdp.blocks[i + 1].blockcategory == csdp::MATRIX) {
      num_X_nonzero_entries +=
          X_csdp.blocks[i + 1].blocksize * X_csdp.blocks[i + 1].blocksize;
    } else if (X_csdp.blocks[i + 1].blockcategory == csdp::DIAG) {
      num_X_nonzero_entries += X_csdp.blocks[i + 1].blocksize;
    }
  }
  std::vector<Eigen::Triplet<double>> X_triplets;
  X_triplets.reserve(num_X_nonzero_entries);
  int X_row_count = 0;
  for (int k = 0; k < X_csdp.nblocks; ++k) {
    if (X_csdp.blocks[k + 1].blockcategory == csdp::MATRIX) {
      for (int i = 0; i < X_csdp.blocks[k + 1].blocksize; ++i) {
        for (int j = 0; j < X_csdp.blocks[k + 1].blocksize; ++j) {
          X_triplets.emplace_back(
              X_row_count + i, X_row_count + j,
              X_csdp.blocks[k + 1].data.mat[ijtok(
                  i + 1, j + 1, X_csdp.blocks[k + 1].blocksize)]);
        }
      }
    } else if (X_csdp.blocks[k + 1].blockcategory == csdp::DIAG) {
      for (int i = 0; i < X_csdp.blocks[k + 1].blocksize; ++i) {
        X_triplets.emplace_back(X_row_count + i, X_row_count + i,
                                X_csdp.blocks[k + 1].data.vec[i + 1]);
      }
    } else {
      throw std::runtime_error(
          "ConvertCsdpBlockMatrixtoEigen(): unknown block matrix type.");
    }
    X_row_count += X_csdp.blocks[k + 1].blocksize;
  }
  X->resize(X_row_count, X_row_count);
  X->setFromTriplets(X_triplets.begin(), X_triplets.end());
}

void SetCsdpSolverDetails(int csdp_ret, double pobj, double dobj, int y_size,
                          double* y, const csdp::blockmatrix& Z,
                          CsdpSolverDetails* solver_details) {
  solver_details->return_code = csdp_ret;
  solver_details->primal_objective = pobj;
  solver_details->dual_objective = dobj;
  solver_details->y_val.resize(y_size);
  for (int i = 0; i < y_size; ++i) {
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
      const auto& decision_var_in_X =
          get<DecisionVariableInSdpaX>(sdpa_free_format.prog_var_in_sdpa()[i]);

      double X_entry_val{};
      switch (X.blocks[decision_var_in_X.entry_in_X.block_index + 1]
                  .blockcategory) {
        case csdp::MATRIX: {
          X_entry_val =
              X.blocks[decision_var_in_X.entry_in_X.block_index + 1]
                  .data.mat[ijtok(
                      decision_var_in_X.entry_in_X.row_index_in_block + 1,
                      decision_var_in_X.entry_in_X.column_index_in_block + 1,
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
          (decision_var_in_X.coeff_sign == Sign::kPositive ? X_entry_val
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

void SolveProgramWithNoFreeVariables(const MathematicalProgram& prog,
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
  // Retrieve the information back to result
  // Since CSDP solves a mazimization problem max -cost, where "cost" is the
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

void FreeCsdpProblemData(int num_constraints, csdp::blockmatrix C_csdp,
                         double* rhs_csdp,
                         csdp::constraintmatrix* constraints) {
  // This function is copied from the source code in csdp/lib/freeprob.c
  free(rhs_csdp);
  csdp::free_mat(C_csdp);
  csdp::sparseblock* ptr;
  csdp::sparseblock* oldptr;
  if (constraints != nullptr) {
    for (int i = 1; i <= num_constraints; ++i) {
      ptr = constraints[i].blocks;
      while (ptr != nullptr) {
        free(ptr->entries);
        free(ptr->iindices);
        free(ptr->jindices);
        oldptr = ptr;
        ptr = ptr->next;
        free(oldptr);
      }
    }
    free(constraints);
  }
}

}  // namespace internal

void CsdpSolver::DoSolve(const MathematicalProgram& prog,
                         const Eigen::VectorXd&, const SolverOptions&,
                         MathematicalProgramResult* result) const {
  result->set_solver_id(CsdpSolver::id());
  const internal::SdpaFreeFormat sdpa_free_format(prog);
  if (sdpa_free_format.num_free_variables() == 0) {
    internal::SolveProgramWithNoFreeVariables(prog, sdpa_free_format, result);
  } else {
    throw std::runtime_error(
        "CsdpSolver::Solve(): do not support problem with free variables yet.");
  }
}

bool CsdpSolver::is_available() { return true; }
}  // namespace solvers
}  // namespace drake
