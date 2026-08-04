#include "drake/geometry/poisson_disk.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

GTEST_TEST(PoissonDiskTest, TestOnePoint) {
  const double r = 2;
  const Box box = Box::MakeCube(0.2);
  const std::vector<Vector3<double>> result = PoissonDiskSampling(r, box);
  /* Radius is larger than diagonal of bounding box, should return only one
   point. */
  EXPECT_EQ(ssize(result), 1);
  /* The sampled point is in the box. */
  EXPECT_LT(result[0].cwiseAbs().maxCoeff(), 0.1);
}

GTEST_TEST(PoissonDiskTest, TestDistance) {
  const double r = 0.05;
  const Box box = Box::MakeCube(0.2);
  const std::vector<Vector3<double>> result = PoissonDiskSampling(r, box);

  for (int i = 0; i < ssize(result); ++i) {
    /* Every sampled point is inside the bounding box. */
    EXPECT_LT(result[i].cwiseAbs().maxCoeff(), 0.1);
  }

  /* There's at least two points 0.05 away in a cube with edge length 0.2. */
  EXPECT_GT(ssize(result), 1);

  for (int i = 0; i < ssize(result); ++i) {
    for (int j = i + 1; j < ssize(result); ++j) {
      const double distance_sq = (result[i] - result[j]).squaredNorm();
      EXPECT_GE(distance_sq, r * r);
    }
  }
}

/* Halfspace is not supported for sampling points. */
GTEST_TEST(PoissonDiskTest, UnsupportedShape) {
  const HalfSpace hs;
  const double r = 2;
  DRAKE_EXPECT_THROWS_MESSAGE(PoissonDiskSampling(r, hs), ".*not supported.*");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
