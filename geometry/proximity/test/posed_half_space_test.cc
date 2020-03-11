#include "drake/geometry/proximity/posed_half_space.h"

#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

// Because the PosedHalfSpace _completely_ relies on the Plane, we don't have to
// rigorously test the full mathematical behavior. Some coarse sampling that
// indicates correlation will be enough.

GTEST_TEST(PosedHalfSpaceTest, Construction) {
  const Vector3d nhat_F = Vector3d{1, 2, 3}.normalized();
  const Vector3d p_FP{2.5, -1.25, 0.25};

  EXPECT_NO_THROW(PosedHalfSpace<double>(nhat_F, p_FP));
  EXPECT_THROW(PosedHalfSpace<double>(5e-11 * nhat_F, p_FP),
               std::exception);
  EXPECT_NO_THROW(PosedHalfSpace<double>(0.5 * nhat_F, p_FP, true));
}

// Confirms that the signed distance behaves as expected.
GTEST_TEST(PosedHalfSpaceTest, CalcSignedDistance) {
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  // Normal in arbitrary direction.
  const Vector3d nhat_F = Vector3d{-1, 0.25, 1.3}.normalized();
  // Point P on the boundary of the half space.
  const Vector3d p_FP{0.25, 0.5, -1};

  const PosedHalfSpace<double> hs_F(nhat_F, p_FP);

  EXPECT_NEAR(hs_F.CalcSignedDistance(p_FP + 0.5 * nhat_F), 0.5, kEps);
  EXPECT_NEAR(hs_F.CalcSignedDistance(p_FP - 0.5 * nhat_F), -0.5, kEps);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
