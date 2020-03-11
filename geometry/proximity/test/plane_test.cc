#include "drake/geometry/proximity/plane.h"

#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

// This simply tests arbitrary normals to make sure they satisfy the
// normalization test. They hypothesis is that *any* Vector3 normalized must
// have a magnitude that is less than 1 epsilon away from 1 (except for the
// zero vector, obviously).
GTEST_TEST(PlnaeTest, Construction) {
  const double kEps = std::numeric_limits<double>::epsilon();
  std::vector<Vector3d> dirs{
      {1., kEps, kEps},
      {1., std::sqrt(kEps), std::sqrt(kEps)},
      {1., 2 * std::sqrt(kEps), 2 * std::sqrt(kEps)},
      {1.123412345, 10.1231231235, -200.23298298374}
  };
  // NOTE: The frame on these are irrelevant.
  const Vector3d point{2.5, -1.25, 0.25};
  for (const Vector3d& dir : dirs) {
    // This would throw for normal vectors that aren't unit length.
    EXPECT_NO_THROW(Plane<double>(dir.normalized(), point));
  }
}

// Confirms that a Plane normal that is *insufficiently* unit length
// throws.
GTEST_TEST(PlnaeTest, Unnormalized) {
  const double kDelta = 4 * std::sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_THROW(
      Plane<double>(Vector3d{1, kDelta, kDelta}, Vector3d{0, 0, 0}),
      std::exception);
}

// Confirms that the signed distance behaves as expected.
GTEST_TEST(PlnaeTest, CalcSignedDistance) {
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  // Normal in arbitrary direction.
  const Vector3d nhat_F = Vector3d{-1, 0.25, 1.3}.normalized();
  // Point on boundary of half space.
  const Vector3d p_FB{0.25, 0.5, -1};

  // Expected signed distances for two query points.
  const double d_Qs[] = {0.5, -0.75};
  const Vector3d p_FQs[] = {p_FB + d_Qs[0] * nhat_F, p_FB + d_Qs[1] * nhat_F};
  {
    Plane<double> hs_F{nhat_F, p_FB};
    for (int i = 0; i < 2; ++i) {
      EXPECT_NEAR(hs_F.CalcHeight(p_FQs[i]), d_Qs[i], kEps);
    }
  }

  // Reversing the normal direction should simply reverse the sign of the
  // distance.
  {
    Plane<double> hs_F{-nhat_F, p_FB};
    for (int i = 0; i < 2; ++i) {
      EXPECT_NEAR(hs_F.CalcHeight(p_FQs[i]), -d_Qs[i], kEps);
    }
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
