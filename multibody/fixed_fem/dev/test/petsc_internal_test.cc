#include "drake/multibody/fixed_fem/dev/petsc_internal.h"

#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::MatrixXd;
using std::iota;
using std::unique_ptr;
using std::vector;

constexpr double kEps = 4.0 * std::numeric_limits<double>::epsilon();
// clang-format off
constexpr int kRows = 3;
constexpr int kCols = 4;
const Eigen::Matrix<double, kRows, kCols> A =
    (Eigen::Matrix<double, kRows, kCols>() << 1, 2, 3, 1.23,
                          4, 5, 6, 4.56,
                          7, 8, 9, 7.89).finished();
// clang-format on

/* Makes a PETSc matrix of type MATSEQAIJ with values supplied by matrix `A`
 defined above. */
unique_ptr<Mat> MakePetscMatrix() {
  auto matrix = std::make_unique<Mat>();
  vector<int> row_indexes(kRows);
  vector<int> col_indexes(kCols);
  iota(row_indexes.begin(), row_indexes.end(), 0);
  iota(col_indexes.begin(), col_indexes.end(), 0);
  MatCreateSeqAIJ(PETSC_COMM_SELF, kRows, kCols, kRows * kCols, PETSC_NULL,
                  matrix.get());
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_row =
      A;
  MatSetValues(*matrix, kRows, row_indexes.data(), kCols, col_indexes.data(),
               A_row.data(), INSERT_VALUES);
  MatAssemblyBegin(*matrix, MAT_FINAL_ASSEMBLY);
  MatAssemblyEnd(*matrix, MAT_FINAL_ASSEMBLY);
  return matrix;
}

GTEST_TEST(PetscInternalTest, MakeEigenRowMajorMatrix) {
  PetscInitialize(PETSC_NULL, PETSC_NULL, PETSC_NULL, PETSC_NULL);
  auto petsc_matrix = MakePetscMatrix();
  const MatrixXd col_major_matrix = MakeDenseMatrix(*petsc_matrix);
  EXPECT_TRUE(CompareMatrices(col_major_matrix, A, kEps));
  MatDestroy(petsc_matrix.get());
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
