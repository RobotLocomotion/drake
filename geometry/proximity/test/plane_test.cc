#include "drake/geometry/proximity/plane.h"

#include <limits>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using Planed = Plane<double>;

// Tests the constructor -- with particular emphasis on normalization behavior.
GTEST_TEST(PlaneTest, Construction) {
  constexpr double kEps = std::numeric_limits<double>::epsilon();

  // The reference normal value.
  const Vector3d nhat_F = Vector3d{1, 2, 3}.normalized();
  // A point on the plane.
  const Vector3d p_FP = Vector3d{0.25, -0.5, 0.75};
  {
    // Case: unnormalized vector produces normalized results.
    const Planed plane_F{0.5 * nhat_F, p_FP};
    EXPECT_TRUE(CompareMatrices(plane_F.normal(), nhat_F, kEps));
  }

  {
    // Case: normalized vector comes through untouched.
    const Planed plane_F{nhat_F, p_FP};
    EXPECT_TRUE(CompareMatrices(plane_F.normal(), nhat_F));
  }

  {
    // Case: Unnormalized vector forced to be used is preserved.
    const Planed plane_F{0.5 * nhat_F, p_FP, true /* already_normalized */};
    EXPECT_TRUE(CompareMatrices(plane_F.normal(), 0.5 * nhat_F));
  }

  {
    // Case: Normalizing a vector whose magnitude is too small throws.
    DRAKE_EXPECT_THROWS_MESSAGE(
        Planed(nhat_F * 5e-11, p_FP), std::runtime_error,
        "Cannot instantiate plane from normal n_F = .*; its magnitude is too "
        "small: .*");
  }
}

// Confirms that height behaves as expected.
GTEST_TEST(PlnaeTest, CalcHeight) {
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  // Normal in arbitrary direction.
  const Vector3d nhat_F = Vector3d{-1, 0.25, 1.3}.normalized();
  // Point on the plane.
  const Vector3d p_FB{0.25, 0.5, -1};

  // Expected heights for two query points (one above and one below the plane).
  const double h_Qs[] = {0.5, -0.75};
  const Vector3d p_FQs[] = {p_FB + h_Qs[0] * nhat_F, p_FB + h_Qs[1] * nhat_F};
  {
    Planed plane_F{nhat_F, p_FB};
    for (int i = 0; i < 2; ++i) {
      EXPECT_NEAR(plane_F.CalcHeight(p_FQs[i]), h_Qs[i], kEps);
    }
  }

  // Reversing the normal direction should simply reverse the sign of the
  // distance.
  {
    Planed plane_F{-nhat_F, p_FB};
    for (int i = 0; i < 2; ++i) {
      EXPECT_NEAR(plane_F.CalcHeight(p_FQs[i]), -h_Qs[i], kEps);
    }
  }

  {
    // If the normal vector is not unit length and the constructor is told to
    // think it is normalized, it is documented that the height values will not
    // be actual height; confirm that.
    const double scale = 0.5;
    Planed plane_F{scale * nhat_F, p_FB, true /* already_normalized */};
    for (int i = 0; i < 2; ++i) {
      // In this case, although not documented, we know the reported height is
      // the scaled height. We use that knowledge to show that the reported
      // height is _not_ the actual height.
      EXPECT_NEAR(plane_F.CalcHeight(p_FQs[i]), scale * h_Qs[i], kEps);
    }
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
