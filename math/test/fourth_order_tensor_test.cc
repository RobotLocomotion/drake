#include "drake/math/fourth_order_tensor.h"

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace drake {
namespace math {
namespace internal {
namespace {

Eigen::Matrix<double, 9, 9> MakeArbitraryMatrix() {
  Eigen::Matrix<double, 9, 9> data;
  for (int i = 0; i < 9; ++i) {
    for (int j = 0; j < 9; ++j) {
      data(i, j) = i + j;
    }
  }
  return data;
}

GTEST_TEST(FourthOrderTensorTest, DefaultConstructor) {
  FourthOrderTensor<double> tensor;
  EXPECT_TRUE(tensor.data().isZero());
  const Vector3d u(1.0, 2.0, 3.0);
  const Vector3d v(4.0, 5.0, 6.0);
  Matrix3d B;

  tensor.ContractWithVectors(u, v, &B);
  EXPECT_TRUE(B.isZero());
}

GTEST_TEST(FourthOrderTensorTest, ConstructWithData) {
  const Eigen::Matrix<double, 9, 9> data = MakeArbitraryMatrix();
  FourthOrderTensor<double> tensor(data);
  /* Getter and mutable getter. */
  EXPECT_EQ(tensor.data(), data);
  EXPECT_EQ(tensor.data(), tensor.mutable_data());
  tensor.mutable_data() = data.transpose();
  EXPECT_EQ(tensor.data(), data.transpose());
  /* Settor */
  tensor.set_data(data);
  EXPECT_EQ(tensor.data(), data);
  /* Operator with four indices. */
  EXPECT_EQ(tensor(0, 0, 0, 0), data(0, 0));
  EXPECT_EQ(tensor(1, 1, 1, 1), data(4, 4));
  /* Operator with two indices. */
  EXPECT_EQ(tensor(0, 0), data(0, 0));
  EXPECT_EQ(tensor(3, 3), data(3, 3));
}

GTEST_TEST(FourthOrderTensorTest, ContractWithVectors) {
  FourthOrderTensor<double> tensor(MakeArbitraryMatrix());

  /* If any vector input is zero, the contraction is zero. */
  Vector3d u = Vector3d::Zero();
  Vector3d v(4.0, 5.0, 6.0);
  Matrix3d B;
  tensor.ContractWithVectors(u, v, &B);
  EXPECT_TRUE(B.isZero());
  tensor.ContractWithVectors(v, u, &B);
  EXPECT_TRUE(B.isZero());

  /* If the 9x9 representation of the tensor has a repeating pattern in the
   blocks, the contraction is a multiple of that block. */
  Matrix3d block;
  block << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  Eigen::Matrix<double, 9, 9> data;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      data.block<3, 3>(3 * i, 3 * j) = block;
    }
  }
  tensor = FourthOrderTensor<double>(data);
  u << 1.0, 2.0, 3.0;
  v << 4.0, 5.0, 6.0;
  tensor.ContractWithVectors(u, v, &B);
  EXPECT_TRUE(CompareMatrices(B, block * (u * v.transpose()).sum()));
}

}  // namespace
}  // namespace internal
}  // namespace math
}  // namespace drake
