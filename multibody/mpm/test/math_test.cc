#include "drake/multibody/mpm/math.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using Eigen::Vector3d;

GTEST_TEST(BsplineTest, PartitionOfUnity) {
  const Vector3d x = Vector3d(0.01, 0.02, 0.03);
  const double dx = 0.01;
  BsplineWeights<double> bspline(x, dx);
  double total_weight = 0.0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        total_weight += bspline.weight(i, j, k);
      }
    }
  }
  EXPECT_DOUBLE_EQ(total_weight, 1.0);
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
