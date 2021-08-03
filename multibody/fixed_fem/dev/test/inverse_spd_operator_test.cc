#include "drake/multibody/fixed_fem/dev/inverse_spd_operator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using Eigen::VectorXd;

/* Dimension of the operator under test. */
constexpr int kD = 4;

/* Generate an arbitrary symmetric positive definite (SPD) matrix. */
Eigen::Matrix<double, kD, kD> MakeSpdMatrix() {
  Eigen::Matrix<double, kD, kD> B;
  // clang-format off
  B << 3.7, 6.9, 9.3, 6.3,
       1.2, 3.7, 1.8, 2.1,
       5.6, 1.1, 3.3, 8.2,
       1.6, 4.9, 1.9, 8.3;
  // clang-format on
  /* A = Bᵀ * B + I₄ is guaranteed to be symmetric positive definite for any
   given B. */
  return B.transpose() * B + Eigen::Matrix<double, kD, kD>::Identity();
}

GTEST_TEST(InverseSpdOperator, Construction) {
  const InverseSpdOperator<double> inverse_operator("A inverse",
                                                    MakeSpdMatrix());
  EXPECT_EQ(inverse_operator.name(), "A inverse");
  EXPECT_EQ(inverse_operator.rows(), kD);
  EXPECT_EQ(inverse_operator.cols(), kD);
}

/* Verifies that calling Multiply() method of the inverse operator of A on a
 vector b results in A⁻¹b. */
GTEST_TEST(InverseSpdOperator, Multiply) {
  const Vector<double, kD> b(0.1, 0.2, 0.3, 0.4);
  const Eigen::Matrix<double, kD, kD> A = MakeSpdMatrix();
  const InverseSpdOperator<double> A_inverse("A inverse", A);
  VectorXd x(kD);
  /* x = A⁻¹b. */
  A_inverse.Multiply(b, &x);
  EXPECT_TRUE(CompareMatrices(A * x, b, 1e-14));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
