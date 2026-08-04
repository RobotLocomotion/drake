#include <gtest/gtest.h>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace ad {
namespace internal {
namespace {

using Eigen::Vector3d;
using test::LimitMalloc;

class MatrixHeapTest : public ::testing::Test {
 protected:
  const AutoDiff a00_{0.1, Vector3d::LinSpaced(1.0, 2.0)};
  const AutoDiff a01_{0.2, Vector3d::LinSpaced(2.0, 3.0)};
  const AutoDiff a02_{0.3, Vector3d::LinSpaced(3.0, 4.0)};
  const AutoDiff a10_{0.4, Vector3d::LinSpaced(4.0, 5.0)};
  const AutoDiff a11_{0.5, Vector3d::LinSpaced(5.0, 6.0)};
  const AutoDiff a12_{0.6, Vector3d::LinSpaced(6.0, 7.0)};

  const AutoDiff b00_{0.7, Vector3d::LinSpaced(7.0, 8.0)};
  const AutoDiff b10_{0.8, Vector3d::LinSpaced(8.0, 9.0)};
  const AutoDiff b20_{0.9, Vector3d::LinSpaced(9.0, 10.0)};

  // clang-format off
  const MatrixX<AutoDiff> A_ad_ =
      (MatrixX<AutoDiff>(2, 3)
       << a00_, a01_, a02_,
          a10_, a11_, a12_).finished();
  const MatrixX<AutoDiff> B_ad_ =
      (MatrixX<AutoDiff>(3, 1)
       << b00_,
          b10_,
          b20_).finished();
  // clang-format on
};

TEST_F(MatrixHeapTest, MatrixProduct) {
  // The expected allocations are:
  // - X's matrix storage (2 x 1)
  // - X(0, 0)'s partials
  // - X(1, 0)'s partials
  // Note in particular that none of the intermediate results need their own
  // allocations -- their storage is moved/reused all the way into the final
  // result.
  int size = -1;
  {
    LimitMalloc guard({3});
    MatrixX<AutoDiff> X = A_ad_ * B_ad_;
    size = X.size();
  }
  EXPECT_EQ(size, 2);
}

}  // namespace
}  // namespace internal
}  // namespace ad
}  // namespace drake
