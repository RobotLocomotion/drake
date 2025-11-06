#include "drake/multibody/contact_solvers/icf/eigen_pool.h"

#include <utility>
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
namespace icf {

const Eigen::Matrix3d S33 =
    (Eigen::Matrix3d() << 4, 1, 2, 1, 5, 3, 2, 3, 6).finished();

GTEST_TEST(EigenPoolTest, ResizeForFixedSizedElements) {
  EigenPool<Matrix3d> pool;
  EXPECT_EQ(pool.size(), 0);

  // Reserve some memory by resizing, then clearing
  pool.Resize(10);
  pool.Clear();
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
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(pool[i], Matrix3d(i * S33));
  }

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

GTEST_TEST(EigenPoolTest, ResizeForPoolsOfMatrixWithFixedRows) {
  EigenPool<Matrix3X<double>> pool;
  std::vector<int> cols = {1, 2, 3, 4};

  {
    // We expect two allocations. One for scalars and one for meta-data (sizes).
    drake::test::LimitMalloc guard({.max_num_allocations = 2});
    pool.Resize(cols);
  }
  EXPECT_EQ(pool.size(), 4);
  EXPECT_EQ(pool[0].size(), 3);
  EXPECT_EQ(pool[1].size(), 6);
  EXPECT_EQ(pool[2].size(), 9);
  EXPECT_EQ(pool[3].size(), 12);

  for (int i = 0; i < ssize(cols); ++i) {
    Matrix3X<double> A =
        VectorXd::LinSpaced(3 * cols[i], 1.0, 10.0).reshaped(3, cols[i]);
    pool[i] = A;
    const auto& const_pool = pool;
    EXPECT_EQ(const_pool[i], A);
  }
}

}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
