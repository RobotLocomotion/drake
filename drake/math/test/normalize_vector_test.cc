#include "normalize_vector.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace math {
namespace {

void NormalizeVectorTestFun(const Eigen::VecorXd x) {

}
GTEST_TEST(NormalizeVectorTest, NormalizeVector) {
  const int ntests = 1;
  const int x_rows = 4;

  for (int testnr = 0; testnr < ntests; testnr++) {
    auto x = Matrix<double, x_rows, 1>::Random().eval();
    Matrix<double, x_rows, 1> x_norm;
    Matrix<double, x_rows, x_rows> dx_norm;
    Matrix<double, x_rows * x_rows, x_rows> ddx_norm;
    normalizeVec(x, x_norm, &dx_norm, &ddx_norm);
  }
}
} // namespace
} // namespace math
} // namespace drake