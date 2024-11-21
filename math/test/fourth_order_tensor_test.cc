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
      data(i, j) = i + 2 * j;
    }
  }
  return data;
}

GTEST_TEST(FourthOrderTensorTest, DefaultConstructor) {
  FourthOrderTensor<double> tensor;
  EXPECT_TRUE(tensor.data().isZero());
}

GTEST_TEST(FourthOrderTensorTest, ConstructWithData) {
  const Eigen::Matrix<double, 9, 9> data = MakeArbitraryMatrix();
  FourthOrderTensor<double> tensor(data);
  /* Getter and mutable getter. */
  EXPECT_EQ(tensor.data(), data);
  EXPECT_EQ(tensor.data(), tensor.mutable_data());
  tensor.mutable_data() = data.transpose();
  EXPECT_EQ(tensor.data(), data.transpose());
  /* Setter */
  tensor.set_data(data);
  EXPECT_EQ(tensor.data(), data);
  /* Operator with four indices. */
  EXPECT_EQ(tensor(0, 0, 0, 0), data(0, 0));
  EXPECT_EQ(tensor(1, 1, 1, 1), data(4, 4));
  EXPECT_EQ(tensor(0, 1, 2, 2), data(3, 8));
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

GTEST_TEST(FourthOrderTensorTest, SetAsOuterProduct) {
  FourthOrderTensor<double> t1, t2;
  Matrix3d M, N;
  M << 1, 0, 3, 0, 5, 0, 7, 0, 9;
  N << 0, 2, 0, 4, 0, 6, 0, 8, 0;
  t1.SetAsOuterProduct(M, N);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        for (int l = 0; l < 3; ++l) {
          EXPECT_EQ(t1(i, j, k, l), M(i, j) * N(k, l));
        }
      }
    }
  }
  t2.SetAsOuterProduct(N, M);
  EXPECT_TRUE(CompareMatrices(t1.data(), t2.data().transpose()));
}

GTEST_TEST(FourthOrderTensorTest, MakeSymmetricIdentity) {
  const double scale = 1.23;
  const FourthOrderTensor<double> tensor =
      FourthOrderTensor<double>::MakeSymmetricIdentity(scale);
  /* The expected result is  scale * 1/2 * (δᵢₖδⱼₗ + δᵢₗδⱼₖ).  */
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        for (int l = 0; l < 3; ++l) {
          double expected_entry = 0.0;
          if (i == k && j == l) expected_entry += 0.5 * scale;
          if (i == l && j == k) expected_entry += 0.5 * scale;
          EXPECT_EQ(tensor(i, j, k, l), expected_entry);
        }
      }
    }
  }
}

GTEST_TEST(FourthOrderTensorTest, MakeMajorIdentity) {
  const double scale = 1.23;
  const FourthOrderTensor<double> tensor =
      FourthOrderTensor<double>::MakeMajorIdentity(scale);
  /* The expected result is scale * δᵢₖδⱼₗ.  */
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        for (int l = 0; l < 3; ++l) {
          double expected_entry = 0.0;
          if (i == k && j == l) expected_entry = scale;
          EXPECT_EQ(tensor(i, j, k, l), expected_entry);
        }
      }
    }
  }
}

GTEST_TEST(FourthOrderTensorTest, Addition) {
  const Eigen::Matrix<double, 9, 9> data1 = MakeArbitraryMatrix();
  const Eigen::Matrix<double, 9, 9> data2 = MakeArbitraryMatrix().transpose();
  FourthOrderTensor<double> tensor1(data1);
  const FourthOrderTensor<double> tensor2(data2);
  tensor1 += tensor2;
  EXPECT_EQ(tensor1.data(), data1 + data2);

  const FourthOrderTensor<double> tensor3 = tensor1 + tensor2;
  EXPECT_EQ(tensor3.data(), data1 + 2.0 * data2);
}

}  // namespace
}  // namespace internal
}  // namespace math
}  // namespace drake
