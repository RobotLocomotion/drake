#include "drake/multibody/fixed_fem/dev/petsc_symmetric_block_sparse_matrix.h"

#include <numeric>
#include <string>
#include <utility>

#include <petscksp.h>

#include "drake/common/unused.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

using Eigen::MatrixXd;
using std::vector;

namespace {

/* Returns true if the given PETSc matrix is "assembled". */
bool is_assembled(const Mat& A) {
  PetscBool assembled = PETSC_FALSE;
  MatAssembled(A, &assembled);
  return assembled == PETSC_TRUE;
}

using MatrixXdRowMajor =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

/* Converts an assembled PETSc matrix to an Eigen row major dense matrix. */
MatrixXdRowMajor MakeEigenRowMajorMatrix(const Mat& petsc_matrix) {
  DRAKE_DEMAND(is_assembled(petsc_matrix));
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

/* Converts an assembled PETSc matrix to an Eigen column major dense matrix. */
MatrixX<double> MakeEigenMatrix(const Mat& petsc_matrix) {
  DRAKE_DEMAND(is_assembled(petsc_matrix));
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

  Impl(int size, int block_size,
       const vector<int>& num_upper_triangular_blocks_per_row)
      : size_(size), block_size_(block_size), num_blocks_(size / block_size) {
    EnsurePetscIsInitialized();
    MatCreateSeqSBAIJ(PETSC_COMM_SELF, block_size, size, size, 0,
                      num_upper_triangular_blocks_per_row.data(),
                      &owned_matrix_);
    /* Initialize solver and preconditioner. */
    KSPCreate(PETSC_COMM_SELF, &owned_solver_);
    KSPGetPC(owned_solver_, &preconditioner_);
  }

  ~Impl() {
    KSPDestroy(&owned_solver_);
    MatDestroy(&owned_matrix_);
  }

  std::unique_ptr<Impl> Clone() const {
    ThrowIfNotAssembled(__func__);
    auto clone = std::make_unique<Impl>();
    clone->size_ = this->size_;
    clone->block_size_ = this->block_size_;
    clone->num_blocks_ = this->num_blocks_;
    MatDuplicate(this->owned_matrix_, MAT_COPY_VALUES, &clone->owned_matrix_);

    /* Initialize solver and preconditioner. */
    KSPCreate(PETSC_COMM_SELF, &clone->owned_solver_);
    KSPGetPC(clone->owned_solver_, &clone->preconditioner_);

    return clone;
  }

  void SetZero() {
    AssembleIfNecessary();
    MatZeroEntries(owned_matrix_);
  }

  VectorX<double> Solve(SolverType solver_type,
                        PreconditionerType preconditioner_type,
                        const VectorX<double>& b) const {
    ThrowIfNotAssembled(__func__);
    VectorX<double> x = b;
    SolveInPlace(solver_type, preconditioner_type, &x);
    return x;
  }

  void SolveInPlace(SolverType solver_type,
                    PreconditionerType preconditioner_type,
                    EigenPtr<VectorX<double>> b) const {
    ThrowIfNotAssembled(__func__);
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
      case PreconditionerType::kJacobi:
        PCSetType(preconditioner_, PCJACOBI);
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
    VecSetValues(b_petsc, size_, vector_indexes.data(), b->data(),
                 INSERT_VALUES);
    VecAssemblyBegin(b_petsc);
    VecAssemblyEnd(b_petsc);

    // TODO(xuchenhan-tri): Currently we don't repeatedly solve the same system
    // with multiple right hand sides, so we always reset the operators. Split
    // the factorization and the solve into two distinct phases to reuse the
    // factorization if we need that in the future.
    /* Set matrix. */
    KSPSetOperators(owned_solver_, owned_matrix_, owned_matrix_);
    /* Solve! */
    KSPSolve(owned_solver_, b_petsc, x_petsc);

    /* Copy solution data to eigen vector. */
    b->setZero();
    VecGetValues(x_petsc, size_, vector_indexes.data(), b->data());

    /* Clean up PETSc temporary variables. */
    VecDestroy(&b_petsc);
    VecDestroy(&x_petsc);
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
    ThrowIfNotAssembled(__func__);
    MatrixX<double> eigen_dense = internal::MakeEigenMatrix(owned_matrix_);
    /* Notice that we store the matrix in PETSc as upper triangular matrix
     and the lower triangular part is ignored. To restore the full
     matrix, we need to fill in these blocks by hand. */
    eigen_dense.triangularView<Eigen::StrictlyLower>() =
        eigen_dense.triangularView<Eigen::StrictlyUpper>().transpose();
    return eigen_dense;
  }

  void ZeroRowsAndColumns(const vector<int>& indexes, double value) {
    /* Ensure that the matrix has been assembled. */
    AssembleIfNecessary();
    MatZeroRowsColumns(owned_matrix_, indexes.size(), indexes.data(), value,
                       PETSC_NULL, PETSC_NULL);
  }

  void SetRelativeTolerance(double tolerance) {
    KSPSetTolerances(owned_solver_, tolerance, PETSC_DEFAULT, PETSC_DEFAULT,
                     PETSC_DEFAULT);
  }

  int size() const { return size_; }

  void AssembleIfNecessary() {
    if (!is_assembled(owned_matrix_)) {
      MatAssemblyBegin(owned_matrix_, MAT_FINAL_ASSEMBLY);
      MatAssemblyEnd(owned_matrix_, MAT_FINAL_ASSEMBLY);
    }
  }

 private:
  void ThrowIfNotAssembled(const char* func) const {
    if (!is_assembled(owned_matrix_)) {
      throw std::runtime_error(
          "PetscSymmetricBlockSparseMatrix::" + std::string(func) +
          "(): matrix is not yet assembled. Call AssembleIfNecessary() first.");
    }
  }

  /* Invokes PetscInitialize() if it hasn't been invoked yet in the program.
   No-op otherwise. */
  void EnsurePetscIsInitialized() {
    static bool done = []() {
      // TODO(xuchenhan-tri): Investigate PETSc's usage of global state.
      PetscInitialize(PETSC_NULL, PETSC_NULL, PETSC_NULL, PETSC_NULL);
      return true;
    }();
    unused(done);
  }

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
    int size, int block_size,
    const vector<int>& num_upper_triangular_blocks_per_row) {
  DRAKE_DEMAND(size >= 0 && block_size > 0);
  DRAKE_DEMAND(size % block_size == 0);
  pimpl_ = std::make_unique<Impl>(size, block_size,
                                  num_upper_triangular_blocks_per_row);
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
    const VectorX<double>& b) const {
  return pimpl_->Solve(solver_type, preconditioner_type, b);
}

void PetscSymmetricBlockSparseMatrix::SolveInPlace(
    SolverType solver_type, PreconditionerType preconditioner_type,
    EigenPtr<VectorX<double>> b) const {
  pimpl_->SolveInPlace(solver_type, preconditioner_type, b);
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

void PetscSymmetricBlockSparseMatrix::AssembleIfNecessary() {
  pimpl_->AssembleIfNecessary();
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
