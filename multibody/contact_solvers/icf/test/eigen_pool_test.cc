#include "drake/multibody/contact_solvers/icf/eigen_pool.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

GTEST_TEST(EigenPoolTest, EigenFixedSize) {
  EigenPool<Matrix3d> pool;
  EXPECT_EQ(pool.size(), 0);

  // Zeroing should be a no-op.
  pool.SetZero();
  EXPECT_EQ(pool.size(), 0);

  // Reserve some memory by resizing, then clearing.
  pool.Resize(10, 3, 3);
  pool.Clear();
  EXPECT_EQ(pool.size(), 0);

  // We already reserved.
  {
    drake::test::LimitMalloc guard;
    pool.Resize(4, 3, 3);
  }
  ASSERT_EQ(pool.size(), 4);

  // Check setting & re-getting.
  const Matrix3d S33 = (Matrix3d() << 4, 1, 2, 1, 5, 3, 2, 3, 6).finished();
  for (int i = 0; i < 4; ++i) {
    pool[i] = i * S33;
    const auto& const_pool = pool;
    EXPECT_EQ(pool[i], Matrix3d(i * S33));
    EXPECT_EQ(const_pool[i], Matrix3d(i * S33));
  }

  // Check zeroing.
  pool.SetZero();
  for (int i = 0; i < 4; ++i) {
    const auto& const_pool = pool;
    EXPECT_EQ(pool[i], Matrix3d::Zero());
    EXPECT_EQ(const_pool[i], Matrix3d::Zero());
  }

  // Check clearing.
  pool.Clear();
  EXPECT_EQ(pool.size(), 0);
}

GTEST_TEST(EigenPoolTest, EigenVectorX) {
  EigenPool<VectorXd> pool;
  EXPECT_EQ(pool.size(), 0);

  // This Resize uses the heterogeneous-size overload.
  std::vector<int> sizes = {3, 5, 7};
  {
    // We expect two allocations. One for scalars and one for meta-data (sizes).
    drake::test::LimitMalloc guard({.max_num_allocations = 2});
    pool.Resize(3, sizes);
  }
  ASSERT_EQ(pool.size(), 3);
  EXPECT_EQ(pool[0].size(), 3);
  EXPECT_EQ(pool[1].size(), 5);
  EXPECT_EQ(pool[2].size(), 7);

  // Check setting & re-getting.
  for (int i = 0; i < ssize(sizes); ++i) {
    VectorXd x = VectorXd::LinSpaced(sizes[i], 1.0, 10.0);
    pool[i] = x;
    const auto& const_pool = pool;
    EXPECT_EQ(pool[i], x);
    EXPECT_EQ(const_pool[i], x);
  }

  // This Resize uses the homogeneous-size overload.
  {
    // No new allocations. Everything stayed the same or got smaller.
    drake::test::LimitMalloc guard({.max_num_allocations = 0});
    pool.Resize(3, 3, 1);
  }
  EXPECT_EQ(pool.size(), 3);
  EXPECT_EQ(pool[0].size(), 3);
  EXPECT_EQ(pool[1].size(), 3);
  EXPECT_EQ(pool[2].size(), 3);

  // Check setting & re-getting.
  for (int i = 0; i < ssize(sizes); ++i) {
    VectorXd x = VectorXd::LinSpaced(3, 1.0, 10.0);
    pool[i] = x;
    const auto& const_pool = pool;
    EXPECT_EQ(const_pool[i], x);
  }

  // Check zeroing.
  pool.SetZero();
  for (int i = 0; i < 3; ++i) {
    const auto& const_pool = pool;
    EXPECT_EQ(pool[i], Vector3d::Zero());
    EXPECT_EQ(const_pool[i], Vector3d::Zero());
  }

  // Check clearing.
  pool.Clear();
  EXPECT_EQ(pool.size(), 0);
}

GTEST_TEST(EigenPoolTest, EigenMatrixX) {
  EigenPool<MatrixXd> pool;
  EXPECT_EQ(pool.size(), 0);

  std::vector<int> rows = {3, 5, 7};
  std::vector<int> cols = {1, 2, 3};
  {
    // We expect two allocations. One for scalars and one for meta-data (sizes).
    drake::test::LimitMalloc guard({.max_num_allocations = 2});
    pool.Resize(3, rows, cols);
  }
  EXPECT_EQ(pool.size(), 3);
  EXPECT_EQ(pool[0].size(), 3);
  EXPECT_EQ(pool[1].size(), 10);
  EXPECT_EQ(pool[2].size(), 21);

  // Check setting & re-getting.
  for (int i = 0; i < ssize(rows); ++i) {
    MatrixXd A = VectorXd::LinSpaced(rows[i] * cols[i], 1.0, 10.0)
                     .reshaped(rows[i], cols[i]);
    pool[i] = A;
    const auto& const_pool = pool;
    EXPECT_EQ(pool[i], A);
    EXPECT_EQ(const_pool[i], A);
  }

  // Check zeroing.
  pool.SetZero();
  for (int i = 0; i < 3; ++i) {
    const auto& const_pool = pool;
    EXPECT_EQ(pool[i], MatrixXd::Zero(rows[i], cols[i]));
    EXPECT_EQ(const_pool[i], MatrixXd::Zero(rows[i], cols[i]));
  }

  // Check clearing.
  pool.Clear();
  EXPECT_EQ(pool.size(), 0);
}

GTEST_TEST(EigenPoolTest, EigenMatrix3X) {
  EigenPool<Matrix3X<double>> pool;
  EXPECT_EQ(pool.size(), 0);

  std::vector<int> cols = {1, 2, 3, 4};
  {
    // We expect two allocations. One for scalars and one for meta-data (sizes).
    drake::test::LimitMalloc guard({.max_num_allocations = 2});
    pool.Resize(4, 3, cols);
  }
  EXPECT_EQ(pool.size(), 4);
  EXPECT_EQ(pool[0].size(), 3);
  EXPECT_EQ(pool[1].size(), 6);
  EXPECT_EQ(pool[2].size(), 9);
  EXPECT_EQ(pool[3].size(), 12);

  // Check setting & re-getting.
  for (int i = 0; i < ssize(cols); ++i) {
    Matrix3X<double> A =
        VectorXd::LinSpaced(3 * cols[i], 1.0, 10.0).reshaped(3, cols[i]);
    pool[i] = A;
    const auto& const_pool = pool;
    EXPECT_EQ(pool[i], A);
    EXPECT_EQ(const_pool[i], A);
  }

  // Check zeroing.
  pool.SetZero();
  for (int i = 0; i < 4; ++i) {
    const auto& const_pool = pool;
    EXPECT_EQ(pool[i], MatrixXd::Zero(3, cols[i]));
    EXPECT_EQ(const_pool[i], MatrixXd::Zero(3, cols[i]));
  }

  // Check clearing.
  pool.Clear();
  EXPECT_EQ(pool.size(), 0);
}

GTEST_TEST(EigenPoolTest, AllInstantiationsExist) {
  EXPECT_EQ(EigenPool<Matrix3<double>>().size(), 0);
  EXPECT_EQ(EigenPool<Matrix3<AutoDiffXd>>().size(), 0);
  EXPECT_EQ(EigenPool<Matrix3X<double>>().size(), 0);
  EXPECT_EQ(EigenPool<Matrix3X<AutoDiffXd>>().size(), 0);
  EXPECT_EQ(EigenPool<Matrix6<double>>().size(), 0);
  EXPECT_EQ(EigenPool<Matrix6<AutoDiffXd>>().size(), 0);
  EXPECT_EQ(EigenPool<Matrix6X<double>>().size(), 0);
  EXPECT_EQ(EigenPool<Matrix6X<AutoDiffXd>>().size(), 0);
  EXPECT_EQ(EigenPool<MatrixX<double>>().size(), 0);
  EXPECT_EQ(EigenPool<MatrixX<AutoDiffXd>>().size(), 0);
  EXPECT_EQ(EigenPool<Vector3<double>>().size(), 0);
  EXPECT_EQ(EigenPool<Vector3<AutoDiffXd>>().size(), 0);
  EXPECT_EQ(EigenPool<Vector6<double>>().size(), 0);
  EXPECT_EQ(EigenPool<Vector6<AutoDiffXd>>().size(), 0);
  EXPECT_EQ(EigenPool<VectorX<double>>().size(), 0);
  EXPECT_EQ(EigenPool<VectorX<AutoDiffXd>>().size(), 0);
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
