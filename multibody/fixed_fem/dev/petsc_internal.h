#pragma once

#include <numeric>
#include <vector>

#include <petscksp.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

// TODO(xuchenhan-tri): Move to cc file.
inline MatrixX<double> MakeDenseMatrix(const Mat& petsc_matrix) {
  using std::vector;
  // Ensure that the matrix has been assembled.
  MatAssemblyBegin(petsc_matrix, MAT_FINAL_ASSEMBLY);
  MatAssemblyEnd(petsc_matrix, MAT_FINAL_ASSEMBLY);

  int rows, cols;
  MatGetSize(petsc_matrix, &rows, &cols);

  MatrixX<double> eigen_dense(rows, cols);
  vector<int> row_indexes(rows);
  vector<int> col_indexes(cols);
  std::iota(row_indexes.begin(), row_indexes.end(), 0);
  std::iota(col_indexes.begin(), col_indexes.end(), 0);
  // PETSc uses row major and Eigen defaults to column major.
  MatGetValues(petsc_matrix, rows, row_indexes.data(), cols, col_indexes.data(),
               eigen_dense.transpose().data());
  return eigen_dense.transpose();
}

/* Wrapper around a PETSc sparse matrix */
class PetscSparseMatrix {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PetscSparseMatrix);

  /* Constructs a PetscSparseMatrix.
   @pre The input argument `petsc_matrix` must be assembled. */
  explicit PetscSparseMatrix(const Mat& petsc_matrix) {
    MatConvert(petsc_matrix, MATSEQAIJ, MAT_INITIAL_MATRIX, &A_);
    MatGetSize(petsc_matrix, &rows_, &cols_);
  }
  ~PetscSparseMatrix() { MatDestroy(&A_); }

  const Mat& get_A() const { return A_; }

  MatrixX<double> MakeDenseMatrix() const {
    return internal::MakeDenseMatrix(A_);
  }

  int rows() const { return rows_; }
  int cols() const { return cols_; }

 private:
  Mat A_;
  int rows_{};
  int cols_{};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
