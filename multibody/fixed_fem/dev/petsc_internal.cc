#include "drake/multibody/fixed_fem/dev/petsc_internal.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

using MatrixXdRowMajor =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

MatrixXdRowMajor MakeEigenRowMajorMatrix(const Mat& petsc_matrix) {
  using std::vector;
  // Ensure that the matrix has been assembled.
  MatAssemblyBegin(petsc_matrix, MAT_FINAL_ASSEMBLY);
  MatAssemblyEnd(petsc_matrix, MAT_FINAL_ASSEMBLY);

  int rows, cols;
  MatGetSize(petsc_matrix, &rows, &cols);

  vector<int> row_indexes(rows);
  vector<int> col_indexes(cols);
  std::iota(row_indexes.begin(), row_indexes.end(), 0);
  std::iota(col_indexes.begin(), col_indexes.end(), 0);

  MatrixXdRowMajor eigen_dense(rows, cols);
  MatGetValues(petsc_matrix, rows, row_indexes.data(), cols, col_indexes.data(),
               eigen_dense.data());

  return eigen_dense;
}

MatrixX<double> MakeDenseMatrix(const Mat& petsc_matrix) {
  return MatrixX<double>(MakeEigenRowMajorMatrix(petsc_matrix));
}

PetscSparseMatrix::PetscSparseMatrix(const Mat& petsc_matrix) {
  // Ensure that the matrix has been assembled.
  MatAssemblyBegin(petsc_matrix, MAT_FINAL_ASSEMBLY);
  MatAssemblyEnd(petsc_matrix, MAT_FINAL_ASSEMBLY);

  MatConvert(petsc_matrix, MATSEQAIJ, MAT_INITIAL_MATRIX, &A_);
  MatGetSize(petsc_matrix, &rows_, &cols_);
}

PetscSparseMatrix::~PetscSparseMatrix() { MatDestroy(&A_); }

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
