#include "drake/multibody/contact_solvers/pooled_sap/eigen_pool.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::pair;
using std::vector;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

const Eigen::Matrix3d S33 =
    (Eigen::Matrix3d() << 4, 1, 2, 1, 5, 3, 2, 3, 6).finished();

#if 0
GTEST_TEST(EigenPoolTest, ColumnVectorConstructor) {
  vector<int> sizes = {3, 2, 4};
  EigenPool<Eigen::VectorXd> pool(sizes);

  ASSERT_EQ(pool.size(), 3);
  EXPECT_EQ(pool[0].rows(), 3);
  EXPECT_EQ(pool[1].rows(), 2);
  EXPECT_EQ(pool[2].rows(), 4);
}


GTEST_TEST(EigenPoolTest, MatrixConstructor) {
  vector<pair<int, int>> shape_list = {{2, 2}, {3, 1}, {1, 4}};
  EigenPool<MatrixXd> pool(shape_list);

  ASSERT_EQ(pool.size(), 3);
  EXPECT_EQ(pool[0].rows(), 2);
  EXPECT_EQ(pool[0].cols(), 2);
  EXPECT_EQ(pool[1].rows(), 3);
  EXPECT_EQ(pool[1].cols(), 1);
  EXPECT_EQ(pool[2].rows(), 1);
  EXPECT_EQ(pool[2].cols(), 4);
}

GTEST_TEST(EigenPoolTest, Matrix3Constructor) {
  const int kDim = 3;

  vector<pair<int, int>> inconsistent_shapes = {
      {kDim, 2}, {kDim, kDim}, {kDim, kDim}};
  DRAKE_EXPECT_THROWS_MESSAGE(EigenPool<Matrix3d>(inconsistent_shapes),
                              "Shape does not match compile time sizes.");

  vector<pair<int, int>> shapes = {{kDim, kDim}, {kDim, kDim}};
  EigenPool<Matrix3d> pool(shapes);

  ASSERT_EQ(pool.size(), 2);
  EXPECT_EQ(pool[0].rows(), kDim);
  EXPECT_EQ(pool[0].cols(), kDim);
  EXPECT_EQ(pool[1].rows(), kDim);
  EXPECT_EQ(pool[1].cols(), kDim);
}
#endif

GTEST_TEST(EigenPoolTest, ResizeForFixedSizedElements) {
  EigenPool<Matrix3d> pool;
  EXPECT_EQ(pool.size(), 0);

  pool.Reserve(10);
  EXPECT_EQ(pool.size(), 0);

  // We already reserved.
  {
    drake::test::LimitMalloc guard;
    pool.Resize(3);
  }
  EXPECT_EQ(pool.size(), 3);

  for (int i = 0; i < pool.size(); ++i) {
    pool[i] = i * S33;
  }
  pool.PushBack(S33);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(pool[i], Matrix3d(i * S33));
  }
  EXPECT_EQ(pool[3], S33);

  pool.Clear();
  EXPECT_EQ(pool.size(), 0);
}

GTEST_TEST(EigenPoolTest, ResizeForPoolsOfVectorX) {
  EigenPool<VectorXd> pool;
  // pool.Resize(3); Doesn't compile, is_fixed_size_v<MatrixXd> is "false".

  std::vector<int> sizes = {3, 5, 7};
  {
    // We expect two allocations. One for scalars and one for meta-data (sizes).
    drake::test::LimitMalloc guard({.max_num_allocations = 2});
    pool.Resize(sizes);
    // N.B. pool.Resize(sizes, sizes) is valid also, though "cols" is ignored.
  }
  EXPECT_EQ(pool.size(), 3);
  EXPECT_EQ(pool[0].size(), 3);
  EXPECT_EQ(pool[1].size(), 5);
  EXPECT_EQ(pool[2].size(), 7);

  for (int i = 0; i < ssize(sizes); ++i) {
    VectorXd x = VectorXd::LinSpaced(sizes[i], 1.0, 10.0);
    pool[i] = x;
    const auto& const_pool = pool;
    EXPECT_EQ(const_pool[i], x);
  }
}

GTEST_TEST(EigenPoolTest, ResizeForPoolsOfMatrixX) {
  EigenPool<MatrixXd> pool;
  std::vector<int> rows = {3, 5, 7};
  std::vector<int> cols = {1, 2, 3};
  // pool.Resize(rows) Doesn't even compile since it's only for vectors.

  {
    // We expect two allocations. One for scalars and one for meta-data (sizes).
    drake::test::LimitMalloc guard({.max_num_allocations = 2});
    pool.Resize(rows, cols);
  }
  EXPECT_EQ(pool.size(), 3);
  EXPECT_EQ(pool[0].size(), 3);
  EXPECT_EQ(pool[1].size(), 10);
  EXPECT_EQ(pool[2].size(), 21);

  for (int i = 0; i < ssize(rows); ++i) {
    MatrixXd A = VectorXd::LinSpaced(rows[i] * cols[i], 1.0, 10.0)
                     .reshaped(rows[i], cols[i]);
    pool[i] = A;
    const auto& const_pool = pool;
    EXPECT_EQ(const_pool[i], A);
  }
}

GTEST_TEST(EigenPoolTest, PushBack) {
  EigenPool<Eigen::Matrix3d> pool;
  EXPECT_EQ(pool.size(), 0);

  std::vector<Matrix3d> data = {S33, 2.0 * S33, 3.0 * S33};
  pool.PushBack(data);
  EXPECT_EQ(pool.size(), 3);

  // Internally, pool should pretty much resolve to a
  // std::vector<Eigen::Matrix3d>.
  EXPECT_EQ(sizeof(pool), sizeof(std::vector<Eigen::Matrix3d>));

  for (int i = 0; i < pool.size(); ++i) {
    EXPECT_EQ(Matrix3d(pool[i]), data[i]);
  }

  // Clear and push again.
  // There should be no memory allocation.
  {
    drake::test::LimitMalloc guard;
    pool.Clear();
    EXPECT_EQ(pool.size(), 0);
    pool.PushBack(data);
    EXPECT_EQ(pool.size(), 3);
    for (int i = 0; i < pool.size(); ++i) {
      EXPECT_EQ(Matrix3d(pool[i]), data[i]);
    }
  }

#if 0
  {
    fmt::print(" sz: {}\n", pool.size());
    const double* ptr = pool.data().data();
    for (int i = 0; i < pool.size(); ++i) {
      fmt::print(" data:\n{}\n", fmt_eigen(Eigen::Map<const Matrix3d>(ptr)));
      ptr += 9;
    }
  }
#endif

  // We allow allocation here.
  pool.PushBack(data);

#if 0
  {
    fmt::print(" sz: {}\n", pool.size());
    const double* ptr = pool.data().data();
    for (int i = 0; i < pool.size(); ++i) {
      fmt::print(" data:\n{}\n", fmt_eigen(Eigen::Map<const Matrix3d>(ptr)));
      ptr += 9;
    }
  }
#endif

  EXPECT_EQ(pool.size(), 6);
  for (int i = 0; i < ssize(data); ++i) {
    EXPECT_EQ(Matrix3d(pool[i]), data[i]);
  }
  for (int i = 0; i < ssize(data); ++i) {
    EXPECT_EQ(Matrix3d(pool[i + 3]), data[i]);
  }
}

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
