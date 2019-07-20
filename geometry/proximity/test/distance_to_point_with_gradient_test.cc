#include "drake/geometry/proximity/distance_to_point_with_gradient.h"

#include <limits.h>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace geometry {
namespace internal {

const double kEps = std::numeric_limits<double>::epsilon();

void CheckDistanceToSphere(const fcl::Sphered& sphere,
                           const math::RigidTransformd& X_WG,
                           const Eigen::Vector3d& p_GQ) {
  const Eigen::Vector3d p_WQ = X_WG * p_GQ;
  const DistanceToPointWithGradient distance_to_point(GeometryId(), X_WG, p_WQ);
  const auto& signed_distance = distance_to_point(sphere);

  const double p_GQ_norm = p_GQ.norm();
  const Vector3<AutoDiffd<3>> p_GQ_autodiff = math::initializeAutoDiff<3>(p_GQ);
  const double tol = DistanceToPointRelativeTolerance(sphere.radius);
  Vector3<AutoDiffd<3>> grad_W_expected;
  if (p_GQ_norm > DistanceToPointRelativeTolerance(sphere.radius)) {
    // Q is away from the sphere center beyond a distance tolerance.
    const AutoDiffd<3> dist_autodiff =
        p_GQ_autodiff.norm() - AutoDiffd<3>(sphere.radius);
    EXPECT_NEAR(dist_autodiff.value(), signed_distance.distance, tol);
    EXPECT_TRUE(CompareMatrices(dist_autodiff.derivatives().transpose(),
                                signed_distance.ddistance_dp_GQ, tol));
    const Vector3<AutoDiffd<3>> grad_G = p_GQ_autodiff.normalized();
    const Vector3<AutoDiffd<3>> p_GN_expected = grad_G * sphere.radius;
    EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(p_GN_expected),
                                signed_distance.p_GN, tol));
    EXPECT_TRUE(CompareMatrices(math::autoDiffToGradientMatrix(p_GN_expected),
                                signed_distance.dp_GN_dp_GQ, tol));
    grad_W_expected = X_WG.rotation().cast<AutoDiffd<3>>() * grad_G;
  } else {
    // Q is close to the sphere center.
    EXPECT_NEAR(signed_distance.distance, -sphere.radius, tol);
    grad_W_expected =
        X_WG.rotation().cast<AutoDiffd<3>>() * Vector3<AutoDiffd<3>>::UnitX();
    // Since p_GN is fixed when Q coincides with the sphere center, dp_GN_dp_GQ
    // is a zero matrix.
    EXPECT_TRUE(CompareMatrices(signed_distance.dp_GN_dp_GQ,
                                Eigen::Matrix3d::Zero(), tol));
  }
  // Check grad_W and its gradient.
  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(grad_W_expected),
                              signed_distance.grad_W, tol));
  EXPECT_TRUE(CompareMatrices(math::autoDiffToGradientMatrix(grad_W_expected),
                              signed_distance.dgrad_W_dp_GQ, tol));
  // The invariance is ∂ distance / ∂ p_GQ = R_GW * grad_W.
  const Eigen::Vector3d grad_G =
      X_WG.rotation().inverse() * signed_distance.grad_W;
  EXPECT_TRUE(CompareMatrices(signed_distance.ddistance_dp_GQ.transpose(),
                              grad_G, tol));
  // The invariance is p_GN = grad_G * radius
  EXPECT_TRUE(
      CompareMatrices(signed_distance.p_GN, grad_G * sphere.radius, tol));
}

GTEST_TEST(DistanceToPointTest, TestSphere) {
  const math::RigidTransformd X_WG(
    Eigen::AngleAxisd(0.2 * M_PI, Eigen::Vector3d(0.1, 0.5, -0.3).normalized()),
    Eigen::Vector3d(0.5, -0.5, 0.3));
  Eigen::Vector3d p_GQ(0.4, 0.8, 1);

  fcl::Sphered sphere(p_GQ.norm() * 0.5);
  // query point is outside of the sphere.
  CheckDistanceToSphere(sphere, X_WG, p_GQ);
  // query point is inside the sphere.
  sphere.radius = p_GQ.norm() * 2;
  CheckDistanceToSphere(sphere, X_WG, p_GQ);
  // query point is on the surface of the sphere.
  sphere.radius = p_GQ.norm();
  CheckDistanceToSphere(sphere, X_WG, p_GQ);
  // query point is at the sphere center.
  p_GQ.setZero();
  CheckDistanceToSphere(sphere, X_WG, p_GQ);
}

void CheckDistanceToHalfspace(const fcl::Halfspaced& halfspace,
                              const math::RigidTransformd& X_WG,
                              const Eigen::Vector3d& p_GQ) {
  const Eigen::Vector3d p_WQ = X_WG * p_GQ;
  const DistanceToPointWithGradient distance_to_point(GeometryId(), X_WG, p_WQ);
  const auto& signed_distance = distance_to_point(halfspace);

  const double tol = 100 * kEps;
  // First check that p_GN is on the boundary of the halfspace.
  EXPECT_NEAR(signed_distance.p_GN.dot(halfspace.n), halfspace.d, tol);
  // ddistance_dp_GQ is the same as n
  EXPECT_TRUE(CompareMatrices(signed_distance.ddistance_dp_GQ.transpose(),
                              halfspace.n, tol));
  // Now check that p_GQ - p_GN is parallel to n
  const Eigen::Vector3d p_NQ_G = p_GQ - signed_distance.p_GN;
  EXPECT_NEAR(std::abs(p_NQ_G.dot(halfspace.n)), p_NQ_G.norm(), tol);
  // Check |NQ| = distance
  const int sign = p_GQ.dot(halfspace.n) >= halfspace.d ? 1 : -1;
  EXPECT_NEAR(sign * p_NQ_G.norm(), signed_distance.distance, tol);
  EXPECT_NEAR(signed_distance.distance, halfspace.signedDistance(p_GQ), tol);
  // Check grad_W = R_WG * n
  EXPECT_TRUE(CompareMatrices(signed_distance.grad_W,
                              X_WG.rotation() * halfspace.n, tol));
  // Check dgrad_W_dp_GQ = 0
  EXPECT_TRUE(CompareMatrices(signed_distance.dgrad_W_dp_GQ,
                              Eigen::Matrix3d::Zero(), tol));
  // Since nᵀ * p_GN = d, we know nᵀ * dp_GN_dp_GQ = 0
  EXPECT_TRUE(
      CompareMatrices(halfspace.n.transpose() * signed_distance.dp_GN_dp_GQ,
                      Eigen::RowVector3d::Zero(), tol));
  // Since n.cross(p_GQ - p_GN) = 0, we know n.cross(I - dp_GN_dp_GQ) = 0.
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(CompareMatrices(
        halfspace.n.cross(
            (Eigen::Matrix3d::Identity() - signed_distance.dp_GN_dp_GQ).col(i)),
        Eigen::Vector3d::Zero(), tol));
  }
}

GTEST_TEST(DistanceToPointTest, TestHalfspace) {
  // Arbitrary X_WG and p_GQ
  const math::RigidTransformd X_WG(
    Eigen::AngleAxisd(0.2 * M_PI, Eigen::Vector3d(0.1, 0.5, -0.3).normalized()),
    Eigen::Vector3d(0.5, -0.5, 0.3));

  // Drake initializes *all* halfspaces to n = (0, 0, 1) and d = 0.
  fcl::Halfspaced halfspace(Eigen::Vector3d::UnitZ(), 0.0);

  // Check Q outside of the halfspace
  Eigen::Vector3d p_GQ = 1.5 * halfspace.n;
  CheckDistanceToHalfspace(halfspace, X_WG, p_GQ);
  p_GQ += X_WG.rotation().col(1);
  CheckDistanceToHalfspace(halfspace, X_WG, p_GQ);
  // Check Q on the boundary of the halfspace.
  p_GQ = Eigen::Vector3d::Zero();
  CheckDistanceToHalfspace(halfspace, X_WG, p_GQ);
  p_GQ += X_WG.rotation().col(1);
  CheckDistanceToHalfspace(halfspace, X_WG, p_GQ);
  // Check Q inside the halfspace.
  p_GQ = -0.1 * halfspace.n;
  CheckDistanceToHalfspace(halfspace, X_WG, p_GQ);
}
}  // namespace internal
}  // namespace geometry
}  // namespace drake
