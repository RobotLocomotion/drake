#include <memory>

#include <gtest/gtest.h>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace ad {
namespace {

using Eigen::Matrix;
using Eigen::Vector3d;
using test::LimitMalloc;
using test::LimitMallocParams;

class EigenSpecializationsHeapTest : public ::testing::Test {
 protected:
  [[nodiscard]] std::unique_ptr<LimitMalloc> ExpectNumAllocs(int num) {
#if 1
    return std::make_unique<LimitMalloc>(LimitMallocParams{
        .max_num_allocations = num});
#else
    return std::make_unique<LimitMalloc>(LimitMallocParams{
        .max_num_allocations = num,
        .min_num_allocations = num});
#endif
  }

  // The inputs x,y,z have unit-vector partials.
  const AutoDiff x_{0.5, 3, 0};
  const AutoDiff y_{0.25, 3, 1};
  const AutoDiff z_{0.125, 3, 2};

  // The inputs a..f have fully-dense partials.
  const AutoDiff a_{3, 1e-2 * Vector3d::LinSpaced(1, 3)};
  const AutoDiff b_{5, 1e-4 * Vector3d::LinSpaced(1, 3)};
  const AutoDiff c_{7, 1e-6 * Vector3d::LinSpaced(1, 3)};
  const AutoDiff d_{11, 1e-8 * Vector3d::LinSpaced(1, 3)};
  const AutoDiff e_{13, 1e-10 * Vector3d::LinSpaced(1, 3)};
  const AutoDiff f_{17, 1e-12 * Vector3d::LinSpaced(1, 3)};
};

#if 0
// With A = [a, b, c] compute AAᵀ = aa + bb + cc.
// Each product costs 1 allocation, but the summation re-uses the storage.
// Ideally we could get down to 1 total allocations here, or even to zero in
// case the result storage was already pre-allocated.
TEST_F(EigenSpecializationsHeapTest, DotProductSelf) {
  Eigen::Matrix<AutoDiff, 1, 3> A;
  A << a_, b_, c_;
  auto guard = ExpectNumAllocs(0);
  Eigen::Matrix<AutoDiff, 1, 1> dot = A * A.transpose();
}
#endif

#if 0
// With A = [a, b, c] compute AAᵀ = aa + bb + cc.
// Each product costs 1 allocation, but the summation re-uses the storage.
// Ideally we could get down to 1 total allocations here, or even to zero in
// case the result storage was already pre-allocated.
TEST_F(EigenSpecializationsHeapTest, DotProductSelf) {
  Eigen::Matrix<AutoDiff, 1, 3> A;
  A << a_, b_, c_;
  auto guard = ExpectNumAllocs(0);
  AutoDiff dot = A.dot(A);
}
#endif

#if 0
// Compute C =
//   ax + by + cz
//   dx + ey + fz
// Each product costs 1 allocation, but the summation re-uses the storage.
// Ideally we could get down to 2 total allocations here, or even to zero in
// case C was already pre-allocated.
TEST_F(EigenSpecializationsHeapTest, MatrixProduct) {
  Eigen::Matrix<AutoDiff, 2, 3> A;
  A << a_, b_, c_,
       d_, e_, f_;
  Eigen::Matrix<AutoDiff, 3, 1> B;
  B << x_,
       y_,
       z_;
  auto guard = ExpectNumAllocs(5);
  Matrix<AutoDiff, 2, 1> C;
  C = A * B;
}
#endif

}  // namespace
}  // namespace ad
}  // namespace drake
