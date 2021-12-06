#include "drake/multibody/fixed_fem/dev/petsc_symmetric_block_sparse_matrix.h"

#include <numeric>
#include <utility>

#include <petscksp.h>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

using Eigen::MatrixXd;
using std::vector;

namespace {

using MatrixXdRowMajor =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

/* Converts a PETSc matrix to an Eigen row major dense matrix. */
MatrixXdRowMajor MakeEigenRowMajorMatrix(const Mat& petsc_matrix) {
  /* Ensure that the matrix has been assembled. */
  MatAssemblyBegin(petsc_matrix, MAT_FINAL_ASSEMBLY);
  MatAssemblyEnd(petsc_matrix, MAT_FINAL_ASSEMBLY);

  int rows, cols;
  MatGetSize(petsc_matrix, &rows, &cols);

  vector<int> row_indexes(rows);
  vector<int> col_indexes(cols);
  std::iota(row_indexes.begin(), row_indexes.end(), 0);
  std::iota(col_indexes.begin(), col_indexes.end(), 0);

  MatrixXdRowMajor eigen_dense = MatrixXdRowMajor::Zero(rows, cols);
  // MatGetValues extracts values from `petsc_matrix` in row major fashion.
  MatGetValues(petsc_matrix, rows, row_indexes.data(), cols, col_indexes.data(),
               eigen_dense.data());

  return eigen_dense;
}

/* Converts a PETSc matrix to an Eigen column major dense matrix. */
MatrixX<double> MakeEigenMatrix(const Mat& petsc_matrix) {
  return MatrixX<double>(MakeEigenRowMajorMatrix(petsc_matrix));
}

}  // namespace

/* The implementation class for PetscSymmetricBlockSparseMatrix. Each of these
 functions mirrors a method on the PetscSymmetricBlockSparseMatrix. See
 PetscSymmetricBlockSparseMatrix for documentation. */
class PetscSymmetricBlockSparseMatrix::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl);

  Impl() = default;

  Impl(int size, int block_size, const vector<int>& nonzero_blocks)
      : size_(size), block_size_(block_size), num_blocks_(size / block_size) {
    PetscInitialize(PETSC_NULL, PETSC_NULL, PETSC_NULL, PETSC_NULL);
    MatCreateSeqSBAIJ(PETSC_COMM_SELF, block_size, size, size, 0,
                      nonzero_blocks.data(), &owned_matrix_);
    /* Initialize solver and preconditioner. */
    KSPCreate(PETSC_COMM_SELF, &owned_solver_);
    KSPGetPC(owned_solver_, &preconditioner_);
  }

  ~Impl() {
    KSPDestroy(&owned_solver_);
    MatDestroy(&owned_matrix_);
  }

  std::unique_ptr<Impl> Clone() const {
    auto clone = std::make_unique<Impl>();
    clone->size_ = this->size_;
    clone->block_size_ = this->block_size_;
    clone->num_blocks_ = this->num_blocks_;
    /* Ensure that the matrix has been assembled. */
    MatAssemblyBegin(this->owned_matrix_, MAT_FINAL_ASSEMBLY);
    MatAssemblyEnd(this->owned_matrix_, MAT_FINAL_ASSEMBLY);
    MatDuplicate(this->owned_matrix_, MAT_COPY_VALUES, &clone->owned_matrix_);

    /* Initialize solver and preconditioner. */
    KSPCreate(PETSC_COMM_SELF, &clone->owned_solver_);
    KSPGetPC(clone->owned_solver_, &clone->preconditioner_);

    return clone;
  }

  void SetZero() {
    /* Ensure that the matrix has been assembled. */
    MatAssemblyBegin(owned_matrix_, MAT_FINAL_ASSEMBLY);
    MatAssemblyEnd(owned_matrix_, MAT_FINAL_ASSEMBLY);
    MatZeroEntries(owned_matrix_);
  }

  VectorX<double> Solve(SolverType solver_type,
                        PreconditionerType preconditioner_type,
                        const VectorX<double>& b) {
    /* Ensure that the matrix has been assembled. */
    MatAssemblyBegin(owned_matrix_, MAT_FINAL_ASSEMBLY);
    MatAssemblyEnd(owned_matrix_, MAT_FINAL_ASSEMBLY);

    switch (solver_type) {
      case SolverType::kDirect:
        KSPSetType(owned_solver_, KSPPREONLY);
        if (preconditioner_type != PreconditionerType::kCholesky) {
          throw std::logic_error(
              "Direct solver can only be paired with Cholesky preconditioner.");
        }
        break;
      case SolverType::kMINRES:
        KSPSetType(owned_solver_, KSPMINRES);
        break;
      case SolverType::kConjugateGradient:
        KSPSetType(owned_solver_, KSPCG);
        break;
    }

    switch (preconditioner_type) {
      case PreconditionerType::kCholesky:
        PCSetType(preconditioner_, PCCHOLESKY);
        break;
      case PreconditionerType::kIncompleteCholesky:
        PCSetType(preconditioner_, PCICC);
        break;
      case PreconditionerType::kBlockJacobi:
        PCSetType(preconditioner_, PCBJACOBI);
        break;
    }

    KSPSetFromOptions(owned_solver_);

    // TODO(xuchenhan-tri): The solve involves some dynamic allocation and
    // moving data around. Measure whether they are heavy on performance and
    // eliminate them if they are.

    /* Allocate for PETSc version of x and b. */
    Vec x_petsc;
    VecCreateSeq(PETSC_COMM_SELF, size_, &x_petsc);
    Vec b_petsc;
    VecDuplicate(x_petsc, &b_petsc);

    /* Copy right hand side data into b_petsc. */
    vector<int> vector_indexes(size_);
    std::iota(vector_indexes.begin(), vector_indexes.end(), 0);
    VecSetValues(b_petsc, size_, vector_indexes.data(), b.data(),
                 INSERT_VALUES);
    VecAssemblyBegin(b_petsc);
    VecAssemblyEnd(b_petsc);

    /* Set matrix. */
    KSPSetOperators(owned_solver_, owned_matrix_, owned_matrix_);
    /* Solve! */
    KSPSolve(owned_solver_, b_petsc, x_petsc);

    /* Copy solution data to eigen vector. */
    VectorX<double> x_eigen(size_);
    VecGetValues(x_petsc, size_, vector_indexes.data(), x_eigen.data());

    /* Clean up PETSc temporary variables. */
    VecDestroy(&b_petsc);
    VecDestroy(&x_petsc);

    return x_eigen;
  }

  void AddToBlock(const VectorX<int>& block_indices,
                  const MatrixX<double>& block) {
    /* Notice that `MatSetValuesBlocked()` takes row oriented data whereas
     `block.data()` is column oriented. But since `block` is symmetric, we
     don't have to make the conversion. */
    MatSetValuesBlocked(owned_matrix_, block_indices.size(),
                        block_indices.data(), block_indices.size(),
                        block_indices.data(), block.data(), ADD_VALUES);
  }

  /* Makes a dense matrix representation of this block-sparse matrix. This
   operation is expensive and is usually only used for debugging purpose. */
  MatrixX<double> MakeDenseMatrix() const {
    MatrixX<double> eigen_dense = internal::MakeEigenMatrix(owned_matrix_);
    /* Notice that we store the matrix in PETSc as upper triangular matrix
     and the lower triangular part is ignored. To restore the full
     matrix, we need to fill in these blocks by hand. */
    for (int i = 0; i < num_blocks_; ++i) {
      for (int j = 0; j < i; ++j) {
        eigen_dense.block(i * block_size_, j * block_size_, block_size_,
                          block_size_) =
            eigen_dense
                .block(j * block_size_, i * block_size_, block_size_,
                       block_size_)
                .transpose();
      }
    }
    return eigen_dense;
  }

  void ZeroRowsAndColumns(const vector<int>& indexes, double value) {
    /* Ensure that the matrix has been assembled. */
    MatAssemblyBegin(owned_matrix_, MAT_FINAL_ASSEMBLY);
    MatAssemblyEnd(owned_matrix_, MAT_FINAL_ASSEMBLY);
    MatZeroRowsColumns(owned_matrix_, indexes.size(), indexes.data(), value,
                       PETSC_NULL, PETSC_NULL);
  }

  void SetRelativeTolerance(double tolerance) {
    KSPSetTolerances(owned_solver_, tolerance, PETSC_DEFAULT, PETSC_DEFAULT,
                     PETSC_DEFAULT);
  }

  int size() const { return size_; }

 private:
  // The underlying PETSc matrix stored as MATSEQSBAIJ.
  Mat owned_matrix_;
  int size_{0};
  int block_size_{0};
  int num_blocks_{0};
  KSP owned_solver_;
  PC preconditioner_;
};

PetscSymmetricBlockSparseMatrix::PetscSymmetricBlockSparseMatrix() = default;

PetscSymmetricBlockSparseMatrix::PetscSymmetricBlockSparseMatrix(
    int size, int block_size, const vector<int>& nonzero_blocks)
    : pimpl_(std::make_unique<Impl>(size, block_size, nonzero_blocks)) {
  DRAKE_DEMAND(size >= 0 && block_size > 0);
  DRAKE_DEMAND(size % block_size == 0);
}

PetscSymmetricBlockSparseMatrix::~PetscSymmetricBlockSparseMatrix() = default;

std::unique_ptr<PetscSymmetricBlockSparseMatrix>
PetscSymmetricBlockSparseMatrix::Clone() const {
  std::unique_ptr<PetscSymmetricBlockSparseMatrix> clone(
      new PetscSymmetricBlockSparseMatrix());
  clone->pimpl_ = pimpl_->Clone();
  return clone;
}

void PetscSymmetricBlockSparseMatrix::AddToBlock(
    const VectorX<int>& block_indices, const MatrixX<double>& block) {
  pimpl_->AddToBlock(block_indices, block);
}

VectorX<double> PetscSymmetricBlockSparseMatrix::Solve(
    SolverType solver_type, PreconditionerType preconditioner_type,
    const VectorX<double>& b) {
  return pimpl_->Solve(solver_type, preconditioner_type, b);
}

void PetscSymmetricBlockSparseMatrix::SetZero() { pimpl_->SetZero(); }

MatrixX<double> PetscSymmetricBlockSparseMatrix::MakeDenseMatrix() const {
  return pimpl_->MakeDenseMatrix();
}

void PetscSymmetricBlockSparseMatrix::ZeroRowsAndColumns(
    const vector<int>& indexes, double value) {
  pimpl_->ZeroRowsAndColumns(indexes, value);
}

void PetscSymmetricBlockSparseMatrix::SetRelativeTolerance(double tolerance) {
  pimpl_->SetRelativeTolerance(tolerance);
}

int PetscSymmetricBlockSparseMatrix::rows() const { return pimpl_->size(); }

int PetscSymmetricBlockSparseMatrix::cols() const { return pimpl_->size(); }

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
