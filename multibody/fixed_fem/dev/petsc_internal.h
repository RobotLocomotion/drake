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

MatrixX<double> MakeDenseMatrix(const Mat& petsc_matrix);

/* Wrapper around a PETSc sparse matrix (SEQAIJ). */
class PetscSparseMatrix {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PetscSparseMatrix);

  /* Constructs a PetscSparseMatrix.
   @param petsc_matrix A PETSc matrix, not necessarily of type SEQAIJ. */
  explicit PetscSparseMatrix(const Mat& petsc_matrix);

  ~PetscSparseMatrix();

  /* Returns the underlying PETsc matrix of type SEQAIJ. */
  const Mat& get_A() const { return A_; }

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
