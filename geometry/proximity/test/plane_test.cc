#include "drake/geometry/proximity/plane.h"

#include <limits>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff.h"

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
    // Case: Declaring the vector already normalized.
    if (kDrakeAssertIsArmed) {
      // With assertions armed, it must be sufficiently unit length.
      DRAKE_EXPECT_THROWS_MESSAGE(
          Planed(nhat_F * 0.99999, p_FP, true), std::runtime_error,
          "Plane constructed with a normal vector that was declared normalized;"
          " the vector is not unit length.*");
      // Within a tolerance it is accepted.
      EXPECT_NO_THROW(Planed(nhat_F * (1 - 10 * kEps), p_FP, true));
    } else {
      // In release, it is simply used.
      const Planed plane_F{0.5 * nhat_F, p_FP, true /* already_normalized */};
      EXPECT_TRUE(CompareMatrices(plane_F.normal(), 0.5 * nhat_F));
    }
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
GTEST_TEST(PlaneTest, CalcHeight) {
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
}

// Confirm that mixed-scalar computation "works". In the case of this test,
// "works" means that the query point scalar type and plane scalar types are
// independent, the return type is the promoted type, and derivatives propagate
// through.
GTEST_TEST(PlaneTest, MixedScalar) {
  // Arbitrary Normal and origins.
  const Vector3d nhat_F_d = Vector3d{-1, 0.25, 1.3}.normalized();
  const Vector3d p_FB_d{0.25, 0.5, -1};
  Plane<double> plane_F_d(nhat_F_d, p_FB_d);

  const Vector3<AutoDiffXd> p_FB_ad = math::InitializeAutoDiff(p_FB_d);
  Plane<AutoDiffXd> plane_F_ad(nhat_F_d.cast<AutoDiffXd>(), p_FB_ad);

  const double expected_distance = 0.125;
  const Vector3d p_FQ_d = p_FB_d + expected_distance * nhat_F_d;
  const Vector3<AutoDiffXd> p_FQ_ad_derivs = math::InitializeAutoDiff(p_FQ_d);
  const Vector3<AutoDiffXd> p_FQ_ad_no_derivs = p_FQ_d.cast<AutoDiffXd>();

  {
    // Double-valued plane and double-valued point -> double valued result.
    const double distance = plane_F_d.CalcHeight(p_FQ_d);
    EXPECT_EQ(distance, expected_distance);
  }

  {
    // Double-valued plane and AutoDiff-valued point -> AutoDiff valued result
    // with the same number of derivatives as the point.
    const AutoDiffXd distance1 = plane_F_d.CalcHeight(p_FQ_ad_derivs);
    EXPECT_EQ(distance1.value(), expected_distance);
    EXPECT_EQ(distance1.derivatives().size(), 3);

    const AutoDiffXd distance2 = plane_F_d.CalcHeight(p_FQ_ad_no_derivs);
    EXPECT_EQ(distance2.value(), expected_distance);
    EXPECT_EQ(distance2.derivatives().size(), 0);
  }

  {
    // AutoDiff-valued plane and double-valued point -> AutoDiff valued result
    // with the same number of derivatives as the plane.
    const AutoDiffXd distance = plane_F_ad.CalcHeight(p_FQ_d);
    EXPECT_EQ(distance.value(), expected_distance);
    EXPECT_EQ(distance.derivatives().size(), 3);
  }

  {
    // AutoDiff-valued plane and AutoDiff-valued point -> AutoDiff valued result
    // with the same number of derivatives as the plane (and point).
    const AutoDiffXd distance = plane_F_ad.CalcHeight(p_FQ_ad_derivs);
    EXPECT_EQ(distance.value(), expected_distance);
    EXPECT_EQ(distance.derivatives().size(), 3);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
