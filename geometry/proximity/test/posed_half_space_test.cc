#include "drake/geometry/proximity/posed_half_space.h"

#include <limits>
#include <vector>

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

// Because the PosedHalfSpace _completely_ relies on the Plane, we don't have to
// rigorously test the full mathematical behavior. Some coarse sampling that
// indicates correlation will be enough.

GTEST_TEST(PosedHalfSpaceTest, Construction) {
  const Vector3d nhat_F = Vector3d{1, 2, 3}.normalized();
  const Vector3d p_FP{2.5, -1.25, 0.25};

  EXPECT_NO_THROW(PosedHalfSpace<double>(nhat_F, p_FP));
  EXPECT_THROW(PosedHalfSpace<double>(5e-11 * nhat_F, p_FP),
               std::exception);

  // Case: Declaring the vector already normalized.
  if (kDrakeAssertIsArmed) {
    // With assertions armed, it must be sufficiently unit length.
    DRAKE_EXPECT_THROWS_MESSAGE(
        PosedHalfSpace<double>(nhat_F * 0.99999, p_FP, true),
        std::runtime_error,
        "Plane constructed with a normal vector that was declared normalized;"
        " the vector is not unit length.*");
  } else {
    // In release, it is simply used.
    PosedHalfSpace<double> hs_F{0.5 * nhat_F, p_FP,
                                true /* already_normalized */};
    EXPECT_TRUE(CompareMatrices(hs_F.normal(), 0.5 * nhat_F));
  }
}

// Confirms that the signed distance behaves as expected.
GTEST_TEST(PosedHalfSpaceTest, CalcSignedDistance) {
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  // Normal in arbitrary direction.
  const Vector3d nhat_F = Vector3d{-1, 0.25, 1.3}.normalized();
  // Point P on the boundary of the half space.
  const Vector3d p_FP{0.25, 0.5, -1};

  const PosedHalfSpace<double> hs_F(nhat_F, p_FP);

  EXPECT_NEAR(hs_F.CalcSignedDistance(Vector3d{p_FP + 0.5 * nhat_F}), 0.5,
              kEps);
  EXPECT_NEAR(hs_F.CalcSignedDistance(Vector3d{p_FP - 0.5 * nhat_F}), -0.5,
              kEps);
}

// Confirm that mixed-scalar computation "works". In the case of this test,
// "works" means that the query point scalar type and half space scalar types
// are independent, the return type is the promoted type, and derivatives
// propagate through.
GTEST_TEST(PosedHalfSpaceTest, MixedScalar) {
  // Arbitrary Normal and origins.
  const Vector3d nhat_F_d = Vector3d{-1, 0.25, 1.3}.normalized();
  const Vector3d p_FB_d{0.25, 0.5, -1};
  PosedHalfSpace<double> hs_F_d(nhat_F_d, p_FB_d);

  const Vector3<AutoDiffXd> p_FB_ad = math::InitializeAutoDiff(p_FB_d);
  PosedHalfSpace<AutoDiffXd> hs_F_ad(nhat_F_d.cast<AutoDiffXd>(), p_FB_ad);

  const double expected_distance = 0.125;
  const Vector3d p_FQ_d = p_FB_d + expected_distance * nhat_F_d;
  const Vector3<AutoDiffXd> p_FQ_ad_derivs = math::InitializeAutoDiff(p_FQ_d);
  const Vector3<AutoDiffXd> p_FQ_ad_no_derivs = p_FQ_d.cast<AutoDiffXd>();

  {
    // Double-valued plane and double-valued point -> double valued result.
    const double distance = hs_F_d.CalcSignedDistance(p_FQ_d);
    EXPECT_EQ(distance, expected_distance);
  }

  {
    // Double-valued plane and AutoDiff-valued point -> AutoDiff valued result
    // with the same number of derivatives as the point.
    const AutoDiffXd distance1 = hs_F_d.CalcSignedDistance(p_FQ_ad_derivs);
    EXPECT_EQ(distance1.value(), expected_distance);
    EXPECT_EQ(distance1.derivatives().size(), 3);

    const AutoDiffXd distance2 = hs_F_d.CalcSignedDistance(p_FQ_ad_no_derivs);
    EXPECT_EQ(distance2.value(), expected_distance);
    EXPECT_EQ(distance2.derivatives().size(), 0);
  }

  {
    // AutoDiff-valued plane and double-valued point -> AutoDiff valued result
    // with the same number of derivatives as the plane.
    const AutoDiffXd distance = hs_F_ad.CalcSignedDistance(p_FQ_d);
    EXPECT_EQ(distance.value(), expected_distance);
    EXPECT_EQ(distance.derivatives().size(), 3);
  }

  {
    // AutoDiff-valued plane and AutoDiff-valued point -> AutoDiff valued result
    // with the same number of derivatives as the plane (and point).
    const AutoDiffXd distance = hs_F_ad.CalcSignedDistance(p_FQ_ad_derivs);
    EXPECT_EQ(distance.value(), expected_distance);
    EXPECT_EQ(distance.derivatives().size(), 3);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
