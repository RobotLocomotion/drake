#include "drake/solvers/csdp_solver_internal.h"

namespace drake {
namespace solvers {
namespace internal {
void ConvertSparseMatrixFormatToCsdpProblemData(
    const std::vector<BlockInX>& X_blocks, const Eigen::SparseMatrix<double>& C,
    const std::vector<Eigen::SparseMatrix<double>> A,
    const Eigen::VectorXd& rhs, csdp::blockmatrix* C_csdp, double** rhs_csdp,
    csdp::constraintmatrix** constraints) {
  const int num_X_rows = C.rows();
  DRAKE_ASSERT(C.cols() == C.rows());
  DRAKE_ASSERT(static_cast<int>(A.size()) == rhs.rows());

  // Maps the row index in X to the block index. Both the row index and the
  // block index are 0-indexed.
  std::vector<int> X_row_to_block_index(num_X_rows);
  std::vector<int> block_start_rows(static_cast<int>(X_blocks.size()));
  int row_count = 0;
  for (int block_index = 0; block_index < static_cast<int>(X_blocks.size());
       ++block_index) {
    block_start_rows[block_index] = row_count;
    for (int row = row_count; row < row_count + X_blocks[block_index].num_rows;
         ++row) {
      X_row_to_block_index[row] = block_index;
    }
    row_count += static_cast<int>(X_blocks[block_index].num_rows);
  }

  C_csdp->nblocks = static_cast<int>(X_blocks.size());
  // We need to add 1 here because CSDP uses Fortran 1-indexed, so the
  // 0'th block is wasted.
  C_csdp->blocks = static_cast<struct csdp::blockrec*>(
      malloc((C_csdp->nblocks + 1) * sizeof(struct csdp::blockrec)));
  for (int block_index = 0; block_index < C_csdp->nblocks; ++block_index) {
    const BlockInX& X_block = X_blocks[block_index];
    // CSDP uses Fortran index, so we need to add 1.
    csdp::blockrec& C_block = C_csdp->blocks[block_index + 1];
    C_block.blockcategory =
        X_block.block_type == BlockType::kMatrix ? csdp::MATRIX : csdp::DIAG;
    C_block.blocksize = X_block.num_rows;
    if (X_block.block_type == BlockType::kMatrix) {
      C_block.data.mat = static_cast<double*>(
          // CSDP's data.mat is an array of size num_rows x num_rows.
          malloc(X_block.num_rows * X_block.num_rows * sizeof(double)));
      for (int j = 0; j < X_block.num_rows; ++j) {
        // First fill in this column with 0, and then we will go through the
        // non-zero entries (stored inside C) to set the value of the
        // corresponding entries in C_csdp.
        for (int i = 0; i < X_block.num_rows; ++i) {
          C_block.data.mat[CsdpMatrixIndex(i, j, X_block.num_rows)] = 0;
        }
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 C, block_start_rows[block_index] + j);
             it; ++it) {
          C_block.data.mat[CsdpMatrixIndex(
              it.row() - block_start_rows[block_index], j, X_block.num_rows)] =
              it.value();
        }
      }
    } else if (X_block.block_type == BlockType::kDiagonal) {
      // CSDP uses Fortran 1-index array, so the 0'th entry is wasted.
      C_block.data.vec =
          static_cast<double*>(malloc((X_block.num_rows + 1) * sizeof(double)));
      for (int j = 0; j < X_block.num_rows; ++j) {
        C_block.data.vec[j + 1] = 0.0;
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 C, block_start_rows[block_index] + j);
             it; ++it) {
          DRAKE_ASSERT(it.row() == it.col());
          C_block.data.vec[j + 1] = it.value();
        }
      }
    } else {
      throw std::runtime_error(
          "ConvertSparseMatrixFormatToCsdpProblemData() only supports MATRIX "
          "or DIAG blocks.");
    }
  }

  // Copy rhs.
  // CSDP stores the right-hand vector as an Fortran 1-indexed array, so we
  // need to add 1 here.
  *rhs_csdp = static_cast<double*>(malloc((rhs.rows() + 1) * sizeof(double)));
  for (int i = 0; i < rhs.rows(); ++i) {
    (*rhs_csdp)[i + 1] = rhs(i);
  }

  // Copy constraints.
  *constraints = static_cast<struct csdp::constraintmatrix*>(
      malloc((static_cast<int>(A.size()) + 1) *
             sizeof(struct csdp::constraintmatrix)));
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
        struct csdp::sparseblock* blockptr =
            static_cast<struct csdp::sparseblock*>(
                malloc(sizeof(struct csdp::sparseblock)));
        // CSDP uses Fortran 1-indexed array.
        blockptr->blocknum = block_index + 1;
        blockptr->blocksize = X_blocks[block_index].num_rows;
        // CSDP uses Fortran 1-indexed array.
        blockptr->constraintnum = constraint_index + 1;
        blockptr->next = nullptr;
        blockptr->nextbyblock = nullptr;
        blockptr->entries = static_cast<double*>(malloc(
            (static_cast<int>(A_block_triplets.size()) + 1) * sizeof(double)));
        blockptr->iindices = static_cast<int*>(malloc(
            (static_cast<int>(A_block_triplets.size()) + 1) * sizeof(int)));
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
    ConvertSparseMatrixFormatToCsdpProblemData(sdpa_free_format.X_blocks(), C,
                                               A, sdpa_free_format.g(), C_csdp,
                                               rhs_csdp, constraints);
  } else {
    throw std::runtime_error(
        "GenerateCsdpProblemDataWithoutFreeVariables(): the formulation has "
        "free variables, you shouldn't call this method.");
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
    } else {
      throw std::runtime_error(
          "ConvertCsdpBlockMatrixtoEigen(): unknown block category.");
    }
  }
  std::vector<Eigen::Triplet<double>> X_triplets;
  X_triplets.reserve(num_X_nonzero_entries);
  int X_row_count = 0;
  for (int block_index = 0; block_index < X_csdp.nblocks; ++block_index) {
    if (X_csdp.blocks[block_index + 1].blockcategory == csdp::MATRIX) {
      for (int i = 0; i < X_csdp.blocks[block_index + 1].blocksize; ++i) {
        for (int j = 0; j < X_csdp.blocks[block_index + 1].blocksize; ++j) {
          X_triplets.emplace_back(
              X_row_count + i, X_row_count + j,
              X_csdp.blocks[block_index + 1].data.mat[CsdpMatrixIndex(
                  i, j, X_csdp.blocks[block_index + 1].blocksize)]);
        }
      }
    } else if (X_csdp.blocks[block_index + 1].blockcategory == csdp::DIAG) {
      for (int i = 0; i < X_csdp.blocks[block_index + 1].blocksize; ++i) {
        X_triplets.emplace_back(X_row_count + i, X_row_count + i,
                                X_csdp.blocks[block_index + 1].data.vec[i + 1]);
      }
    } else {
      throw std::runtime_error(
          "ConvertCsdpBlockMatrixtoEigen(): unknown block matrix type.");
    }
    X_row_count += X_csdp.blocks[block_index + 1].blocksize;
  }
  X->resize(X_row_count, X_row_count);
  X->setFromTriplets(X_triplets.begin(), X_triplets.end());
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

int CsdpMatrixIndex(int row, int col, int num_rows) {
  // Internally matrix uses 1-indexed Fortran array, so we need to add 1.
  return ijtok(row + 1, col + 1, num_rows);
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
