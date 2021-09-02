#include "drake/geometry/proximity/distance_to_point_callback.h"

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/utilities.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace point_distance {
namespace {

// TODO(SeanCurtis-TRI): Run through proximity_engine_test and pull the tests
// from there that better belong here.

using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RotationMatrix;
using std::make_shared;

// Performs a point-to-shape signed-distance query and tests the result. This
// particularly focuses on the AutoDiff-valued version of these methods. It
// does a limited smoke-test on the results.
//
// It computes ∂distance / ∂p_WQ and compares it with the reported grad_W value
// with the assumption that they should be the same.
//
// The caller provides the nearest point p_GN_G (in the geometry frame G) and
// the offset from the nearest point to the query point Q (p_NQ_G) also in the
// geometry frame G. The query point is inferred from these two values. It
// performs the query and examines the results. It generally assumes
// non-negative signed distance. If the point is inside (i.e., negative signed
// distance), then the additional boolean `is_inside` should be set to true.
// To test robustness, the frame G can have an arbitrary pose in the world
// frame (defined by X_WG). Finally, an absolute tolerance is provided to
// define the scope of correctness.
//
// Note: this is *not* the complete test; we should also confirm that the
// derivatives of grad_W and the witness points are likewise correct.
// TODO(hongkai.dai): Extend these tests to test the AutoDiff derivatives of
// the reported gradient and witness points w.r.t. arbitrary basis as well.
template <typename Shape>
class PointShapeAutoDiffSignedDistanceTester {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PointShapeAutoDiffSignedDistanceTester)

  // Constructs a tester for a given shape G, pose in world X_WG, and tolerance.
  // The shape must be non-null and must persist beyond the life of the tester
  // as a reference to the shape will be stored.
  PointShapeAutoDiffSignedDistanceTester(const Shape* shape,
                                         const RigidTransformd& X_WG,
                                         double tolerance)
      : shape_(*shape), X_WG_(X_WG), tolerance_(tolerance) {}

  // Perform the test with the particular N and Q.
  ::testing::AssertionResult Test(const Vector3d& p_GN_G,
                                  const Vector3d& p_NQ_G,
                                  bool is_inside = false,
                                  bool is_grad_W_unique = true) {
    const double sign = is_inside ? -1 : 1;
    const double expected_distance = sign * p_NQ_G.norm();
    const Vector3d p_GQ = p_GN_G + p_NQ_G;
    const Vector3d p_WQ = X_WG_ * p_GQ;

    ::testing::AssertionResult failure = ::testing::AssertionFailure();
    bool error = false;

    // We take the gradient of the signed distance query w.r.t p_WQ.
    Vector3<AutoDiffXd> p_WQ_ad = math::InitializeAutoDiff(p_WQ);
    // The size of the variables we take gradient with (p_WQ) is 3.
    const int grad_size = 3;
    DistanceToPoint<AutoDiffXd> distance_to_point(
        GeometryId::get_new_id(), X_WG_.cast<AutoDiffXd>(), p_WQ_ad);

    SignedDistanceToPoint<AutoDiffXd> result = distance_to_point(shape_);
    if (std::abs(result.distance.value() - expected_distance) > tolerance_) {
      error = true;
      failure << "The difference between expected distance and tested distance "
                 "is greater than the given tolerance_:\n"
              << "  Expected distance: " << expected_distance << "\n"
              << "  Tested distance: " << result.distance.value() << "\n"
              << "  tolerance: " << tolerance_ << "\n"
              << "  difference: "
              << (std::abs(result.distance.value() - expected_distance));
    }
    // The hand-computed `grad_W` value should match the autodiff-computed
    // gradient.
    if (result.distance.derivatives().size() != 3) {
      if (error) failure << "\n";
      error = true;
      failure << "Test distance has no derivatives";
    }
    const Vector3d ddistance_dp_WQ = result.distance.derivatives();
    const Vector3d grad_W_val = math::ExtractValue(result.grad_W);
    if (grad_W_val.array().isNaN().any()) {
      if (error) failure << "\n";
      error = true;
      failure << "Analytical gradient contains NaN: " << grad_W_val.transpose();
    }
    auto gradient_compare =
        CompareMatrices(ddistance_dp_WQ, grad_W_val, tolerance_);
    if (!gradient_compare) {
      if (error) failure << "\n";
      error = true;
      failure << "grad_W and distance.derivatives() don't match:\n"
              << gradient_compare.message();
    }

    // Since grad_W is a unit vector, we know that grad_Wᵀ * grad_W = 1.
    const AutoDiffXd grad_W_squared_norm = result.grad_W.dot(result.grad_W);
    EXPECT_NEAR(grad_W_squared_norm.value(), 1, tolerance_);
    // The gradient of grad_W_squared_norm should be 0.
    if (grad_W_squared_norm.derivatives().size() > 0) {
      auto grad_W_unit_length_derivative_compare =
          CompareMatrices(grad_W_squared_norm.derivatives(),
                          Eigen::VectorXd::Zero(grad_size), tolerance_);
      if (!grad_W_unit_length_derivative_compare) {
        if (error) failure << "\n";
        error = true;
        failure << "grad_W_squared_norm.derivatives() isn't right:\n"
                << grad_W_unit_length_derivative_compare.message();
      }
    }

    // We have the invariance p_WQ = p_WN + distance * grad_W.
    const Vector3<AutoDiffXd> p_WN_ad = X_WG_.cast<AutoDiffXd>() * result.p_GN;
    const Vector3<AutoDiffXd> p_WQ_ad_expected =
        p_WN_ad + result.distance * result.grad_W;
    auto p_WQ_val_compare = CompareMatrices(
        math::ExtractValue(p_WQ_ad),
        math::ExtractValue(p_WQ_ad_expected), tolerance_);
    if (!p_WQ_val_compare) {
      if (error) failure << "\n";
      error = true;
      failure << "p_WQ does not equal to p_WN + distance * grad_W:\n"
              << p_WQ_val_compare.message();
    }

    // We'll only test the derivatives of the gradient if we expect it to be
    // unique.
    if (is_grad_W_unique) {
      auto p_WQ_derivative_compare = CompareMatrices(
          math::ExtractGradient(p_WQ_ad),
          math::ExtractGradient(p_WQ_ad_expected),
          tolerance_);
      if (!p_WQ_derivative_compare) {
        if (error) failure << "\n";
        error = true;
        failure << "Gradient of p_WQ does not equal to gradient of (p_WN + "
                   "distance * grad_W):\n"
                << p_WQ_derivative_compare.message();
      }
    }

    if (!error) return ::testing::AssertionSuccess();
    return failure;
}

 private:
  const Shape& shape_;
  const RigidTransformd X_WG_;
  const double tolerance_{std::numeric_limits<double>::epsilon()};
};

// Simple smoke test for signed distance to Box. It does the following:
//   Perform test of three different points w.r.t. a box: outside, on
//     surface, and inside.
//   Do it with AutoDiff relative to the query point's position.
//   Confirm the *values* of the reported quantity.
//   Confirm that the derivative of distance (extracted from AutoDiff) matches
//     the derivative computed by hand (grad_W).
GTEST_TEST(DistanceToPoint, Box) {
  const double kEps = 4 * std::numeric_limits<double>::epsilon();

  // Provide some arbitrary pose of the box in the world.
  const RotationMatrix<double> R_WG(
      AngleAxis<double>(M_PI / 5, Vector3d{1, 2, 3}.normalized()));
  const Vector3d p_WG{0.5, 1.25, -2};
  const RigidTransformd X_WG(R_WG, p_WG);
  const fcl::Boxd box(1.0, 2.0, 3.0);
  PointShapeAutoDiffSignedDistanceTester<fcl::Boxd> tester(&box, X_WG, kEps);

  // Case: Nearest point is a vertex.
  {
    for (double x : {-1, 1}) {
      for (double y : {-1, 1}) {
        for (double z : {-1, 1}) {
          const Vector3d p_NQ_G{x, y, z};
          const Vector3d p_GN_G = p_NQ_G.cwiseProduct(box.side / 2);
          // The query point lies outside the box.
          EXPECT_TRUE(tester.Test(p_GN_G, p_NQ_G));
          // The query point lies on the vertex of the box.
          EXPECT_TRUE(tester.Test(p_GN_G, Vector3d::Zero(),
                                  false /* not inside the box */,
                                  false /* gradient ill defined */));
        }
      }
    }
    // A query point *inside* the box would not be nearest the vertex.
  }

  // Case: Nearest point lies on an edge.
  for (int coord1 = 0; coord1 < 3; ++coord1) {
    // p_NQ_G(coord1) = 0
    int coord2 = (coord1 + 1) % 3;
    int coord3 = (coord1 + 2) % 3;
    for (double coord2_val : {-1, 1}) {
      for (double coord3_val : {-1, 1}) {
        Vector3d p_NQ_G;
        p_NQ_G(coord1) = 0;
        p_NQ_G(coord2) = coord2_val;
        p_NQ_G(coord3) = coord3_val;
        const Vector3d p_GN_G = p_NQ_G.cwiseProduct(box.side / 2);
        // The query point lies outside the box.
        EXPECT_TRUE(tester.Test(p_GN_G, p_NQ_G));
        // The query point lies on the edge of the box.
        EXPECT_TRUE(tester.Test(p_GN_G, Vector3d::Zero(),
                                false /* not inside the box */,
                                false /* gradient ill defined */));

        // A query point *inside* the box would not be nearest the edge.
      }
    }
  }

  // Case point lies *outside* the box, nearest a face.
  {
    for (double sign : {-1, 1}) {
      for (int axis : {0, 1, 2}) {
        Vector3d vhat_NG = Vector3d::Zero();
        vhat_NG(axis) = sign;
        Vector3d p_NQ_G = 1.5 * vhat_NG;
        const Vector3d p_GN_G = vhat_NG.cwiseProduct(box.side / 2);

        // The query point lies outside the box.
        EXPECT_TRUE(tester.Test(p_GN_G, p_NQ_G));

        // The query point lies on the face of the box.
        EXPECT_TRUE(tester.Test(p_GN_G, Vector3d::Zero()));

        // The query point lies inside the box.
        EXPECT_TRUE(tester.Test(p_GN_G, -0.1 * vhat_NG, true /* is inside */));
      }
    }
  }
}

// Simple smoke test for signed distance to Capsule. It does the following:
//   Perform test of three different points w.r.t. a capsule: outside, on
//     surface, and inside.
//   Do it with AutoDiff relative to the query point's position.
//   Confirm the *values* of the reported quantity.
//   Confirm that the derivative of distance (extracted from AutoDiff) matches
//     the derivative computed by hand (grad_W).
GTEST_TEST(DistanceToPoint, Capsule) {
  const double kEps = 6 * std::numeric_limits<double>::epsilon();

  // Provide some arbitrary pose of the capsule G in the world.
  const RotationMatrix<double> R_WG(
      AngleAxis<double>(M_PI / 5, Vector3d{1, 2, 3}.normalized()));
  const Vector3d p_WG{0.5, 1.25, -2};
  const RigidTransform<double> X_WG(R_WG, p_WG);
  const fcl::Capsuled capsule(0.7, 1.3);
  PointShapeAutoDiffSignedDistanceTester<fcl::Capsuled> tester(&capsule, X_WG,
                                                               kEps);
  // We want to test the 3 sections for when the point Q is nearest to:
  //   1. The top end cap of the capsule.
  //   2. The spine of the capsule.
  //   3. The bottom end cap of the capsule.
  // In each section, we pick a direction away from the capsule that *isn't*
  // aligned with the frame basis and prepare the witness point N accordingly.
  const Vector3d vhat_NQ_Gs[3] = {
      Vector3d{2, -3, 6}.normalized(),  // Upwards and away.
      Vector3d{2, -3, 0}.normalized(),  // Perpendicularly outwards.
      Vector3d{2, -3, -6}.normalized()  // Downwards and away.
  };
  const Vector3d p_GN_Gs[3] = {
      capsule.radius * vhat_NQ_Gs[0] + Vector3d{0, 0, capsule.lz / 2},
      capsule.radius * vhat_NQ_Gs[1] + Vector3d{0, 0, capsule.lz / 4},
      capsule.radius * vhat_NQ_Gs[2] + Vector3d{0, 0, -capsule.lz / 2}};

  for (int i = 0; i < 3; ++i) {
    const Vector3d& vhat_NQ_G = vhat_NQ_Gs[i];
    const Vector3d& p_GN_G = p_GN_Gs[i];

    // Case: point lies *outside* the capsule.
    {
      const Vector3d p_NQ_G = 1.5 * vhat_NQ_G;
      EXPECT_TRUE(tester.Test(p_GN_G, p_NQ_G));
    }

    // Case: point lies *on* the capsule.
    {
      const Vector3d p_NQ_G = Vector3d::Zero();
      EXPECT_TRUE(tester.Test(p_GN_G, p_NQ_G));
    }

    // Case: point lies *in* the capsule.
    {
      const Vector3d p_NQ_G = -(0.5 * capsule.radius) * vhat_NQ_G;
      EXPECT_TRUE(tester.Test(p_GN_G, p_NQ_G, true /* is inside */));
    }

    // Case: point lies on the capsule's spine.
    {
      const Vector3d p_NQ_G = -capsule.radius * vhat_NQ_G;
      EXPECT_TRUE(tester.Test(p_GN_G, p_NQ_G, true /* is inside */,
                              false /* gradient ill defined */));
    }
  }
}

// Test the evaluation of signed distance to Ellipsoid. This explicitly spells
// out a double-valued test because AutoDiffXd is not yet supported. We hand
// construct a few points (and corresponding normals) on the surface of the
// ellipsoid and construct query points relative to those surface points.
// The surface points should report as the witness point, the normal is the
// gradient, etc.
//
// For each sample, we try a different distance and confirm all the fields of
// the returned type.
GTEST_TEST(DistanceToPoint, Ellipsoid) {
  // Provide some arbitrary pose of the ellipsoid G in the world.
  const RotationMatrix<double> R_WG(
      AngleAxis<double>(M_PI / 5, Vector3d{1, 2, 3}.normalized()));
  const Vector3d p_WG{0.5, 1.25, -2};
  const RigidTransform<double> X_WG(R_WG, p_WG);

  // Note: the more eccentric the ellipsoid becomes (with higher curvature), the
  // more error there will be near the regions with high curvature. This is
  // because we're using GJK/EPA to compute distance. So, to avoid highly
  // variable error across the point samples, we'll keep the ellipsoid "close"
  // to a sphere. If we stretch it out, we need either a) a larger tolerance
  // value for the vectors or b) a per-sample tolerance values.
  const fcl::Ellipsoidd ellipsoid(1.5, 0.75, 1.25);

  // TODO(SeanCurtis-TRI): When point-to-ellipsoid supports AutoDiffXd, modify
  //  this to exercise PointShapeAutoDiffSignedDistanceTester.

  auto get_sample = [&ellipsoid](double theta, double phi) {
    // Compute a point on the surface of the ellipsoid and the normal at
    // that point. For the point, we use the parametric equation of the
    // ellipsoid (on two periodic parameters):
    //    E = [a⋅cos(θ)⋅sin(ϕ), b⋅sin(θ)⋅sin(ϕ), c⋅cos(ϕ)].
    // For the normal, we normalize the gradient of
    //    f = x² / a² + y² / b² + z² / c²
    //    ∇f = <2x / a², 2y / b², 2z / c²>
    //    n = ∇f / |∇f|
    const double a = ellipsoid.radii.x();
    const double b = ellipsoid.radii.y();
    const double c = ellipsoid.radii.z();
    const double x = a * std::cos(theta) * std::sin(phi);
    const double y = b * std::sin(theta) * std::sin(phi);
    const double z = c * std::cos(phi);
    // Because we're normalizing the gradient, we simply omit the redundant
    // scale factor of 2.
    return std::make_pair(
        Vector3d{x, y, z},
        Vector3d{x / a / a, y / b / b, z / c / c}.normalized());
  };

  struct EllipseCoord {
    double theta{};
    double phi{};
  };

  std::vector<EllipseCoord> coords{
      EllipseCoord{0, 0},                       // Bottom pole.
      EllipseCoord{7 * M_PI / 5, M_PI / 6},     // Lower half.
      EllipseCoord{3 * M_PI / 7, 4 * M_PI / 5}  // Upper half.
  };

  constexpr double kDistTolerance = 5e-5;
  constexpr double kVectorTolerance = 5e-4;
  const GeometryId id = GeometryId::get_new_id();

  for (const auto& coord : coords) {
    const auto& [p_GN, n_G] = get_sample(coord.theta, coord.phi);
    for (const double distance : {-0.125, 0.0, 0.2}) {
      const Vector3d p_GQ = p_GN + n_G * distance;
      const Vector3d p_WQ = X_WG * p_GQ;
      DistanceToPoint<double> distance_to_point(id, X_WG, p_WQ);
      const SignedDistanceToPoint<double> result = distance_to_point(ellipsoid);

      SCOPED_TRACE(fmt::format(
          "theta = {}, phi = {}, distance = {}\n  p_GQ: {} {} {}", coord.theta,
          coord.phi, distance, p_GQ.x(), p_GQ.y(), p_GQ.z()));
      EXPECT_EQ(result.id_G, id);
      EXPECT_NEAR(result.distance, distance, kDistTolerance);
      EXPECT_TRUE(CompareMatrices(result.p_GN, p_GN, kVectorTolerance));
      EXPECT_TRUE(CompareMatrices(result.grad_W, X_WG.rotation() * n_G,
                                  kVectorTolerance));
    }
  }

  // Special case where the query point lies on the medial axis of
  // the ellipsoid. We don't know what FCL will produce as the nearest point.
  // However, we can confirm:
  //   - distance from query to nearest point.
  //   - gradient is perpendicular to a known circle.
  //   - id is correct.
  // To facilitate this, we'll pick the query point (0, 0, 0). Then the nearest
  // point must be a distance equal to the shortest radii (0.75), it must lie
  // either on the +y or -y axis (the axis with the shortest radii) and have
  // a normal that points in one of those two directions.
  const double min_radius = ellipsoid.radii.y();
  const Vector3d min_axis_G(0, 1, 0);
  const Vector3d p_WQ = X_WG * Vector3d::Zero();
  DistanceToPoint<double> distance_to_point(id, X_WG, p_WQ);
  const SignedDistanceToPoint<double> result = distance_to_point(ellipsoid);
  EXPECT_EQ(result.id_G, id);
  EXPECT_NEAR(result.distance, -min_radius, kDistTolerance);
  EXPECT_TRUE(CompareMatrices(result.p_GN.cwiseAbs(), min_axis_G * min_radius,
                              kVectorTolerance));
  EXPECT_TRUE(
      CompareMatrices((X_WG.rotation().inverse() * result.grad_W).cwiseAbs(),
                      min_axis_G, kVectorTolerance));
}

// Simple smoke test for signed distance to Halfspace. It does the following:
//   Perform test of three different points w.r.t. a halfspace: outside, on
//     surface, and inside.
//   Do it with AutoDiff relative to the query point's position.
//   Confirm the *values* of the reported quantity.
//   Confirm that the derivative of distance (extracted from AutoDiff) matches
//     the derivative computed by hand (grad_W).
GTEST_TEST(DistanceToPoint, Halfspace) {
  const double kEps = 4 * std::numeric_limits<double>::epsilon();

  // Provide some arbitrary pose of the halfspace in the world.
  const RotationMatrix<double> R_WG(
      AngleAxis<double>(M_PI / 5, Vector3d{1, 2, 3}.normalized()));
  const Vector3d p_WG{0.5, 1.25, -2};
  const RigidTransformd X_WG(R_WG, p_WG);
  const fcl::Halfspaced hs(Vector3d::UnitZ(), 0);
  PointShapeAutoDiffSignedDistanceTester<fcl::Halfspaced> tester(&hs, X_WG,
                                                                 kEps);

  // An arbitrary direction away from the origin that *isn't* aligned with the
  // frame basis.
  const Vector3d phat_NQ_G = Vector3d::UnitZ();
  const Vector3d p_NQ_G = 1.5 * phat_NQ_G;
  const Vector3d p_GN_G = Vector3d(0.25, 0.5, 0);

  // Case: point lies *outside* the halfspace.
  EXPECT_TRUE(tester.Test(p_GN_G, p_NQ_G));

  // Case: point lies *on* the halfspace.
  EXPECT_TRUE(tester.Test(p_GN_G, Vector3d::Zero()));

  // Case: point lies *in* the halfspace.
  EXPECT_TRUE(tester.Test(p_GN_G, -0.1 * phat_NQ_G, true /* is inside */));
}

// Simple smoke test for signed distance to Sphere. It does the following:
//   Perform test of three different points w.r.t. a sphere: outside, on
//     surface, and inside.
//   Do it with AutoDiff relative to the query point's position.
//   Confirm the *values* of the reported quantity.
//   Confirm that the derivative of distance (extracted from AutoDiff) matches
//     the derivative computed by hand (grad_W).
GTEST_TEST(DistanceToPoint, Sphere) {
  const double kEps = 4 * std::numeric_limits<double>::epsilon();

  // Provide some arbitrary pose of the sphere in the world.
  const RotationMatrix<double> R_WG(
      AngleAxis<double>(M_PI / 5, Vector3d{1, 2, 3}.normalized()));
  const Vector3d p_WG{0.5, 1.25, -2};
  const RigidTransform<double> X_WG(R_WG, p_WG);
  const fcl::Sphered sphere(0.7);
  PointShapeAutoDiffSignedDistanceTester<fcl::Sphered> tester(&sphere, X_WG,
                                                              kEps);

  // An arbitrary direction away from the origin that *isn't* aligned with the
  // frame basis.
  const Vector3d vhat_NQ = Vector3d{2, -3, 6}.normalized();
  const Vector3d p_NQ_G = 1.5 * vhat_NQ;
  const Vector3d p_GN_G = sphere.radius * vhat_NQ;

  // Case: point lies *outside* the sphere.
  EXPECT_TRUE(tester.Test(p_GN_G, p_NQ_G));

  // Case: point lies *on* the sphere.
  EXPECT_TRUE(tester.Test(p_GN_G, Vector3d::Zero()));

  // Case: point lies *in* the sphere.
  EXPECT_TRUE(tester.Test(p_GN_G, -0.1 * p_NQ_G, true /* is inside */));

  // Case: point lies at origin of the sphere.
  EXPECT_TRUE(tester.Test(p_GN_G, -sphere.radius * vhat_NQ,
                          true /* is inside */,
                          false /* gradient ill defined */));
}

// TODO(SeanCurtis-TRI): Point-to-cylinder with AutoDiff has been "disabled".
//  However, this has been done at the callback level and these tests are
//  structured to exercise DistanceToPoint directly. This is a short-term
//  issue and will be addressed with a reformulation of the point-cylinder
//  calculation. When fixed, these tests will become meaningful, so we're
//  leaving it in to prevent losing the effort.
#if 0
// Simple smoke test for signed distance to Cylinder. It does the following:
//   Perform test of three different points w.r.t. a cylinder: outside, on
//     surface, and inside.
//   Do it with AutoDiff relative to the query point's position.
//   Confirm the *values* of the reported quantity.
//   Confirm that the derivative of distance (extracted from AutoDiff) matches
//     the derivative computed by hand (grad_W).
GTEST_TEST(DistanceToPoint, Cylinder) {
  const double kEps = 4 * std::numeric_limits<double>::epsilon();

  // Provide some arbitrary pose of the cylinder in the world.
  const RotationMatrix<double> R_WG(
      AngleAxis<double>(M_PI / 5, Vector3d{1, 2, 3}.normalized()));
  const Vector3d p_WG{0.5, 1.25, -2};
  const RigidTransform<double> X_WG(R_WG, p_WG);
  const fcl::Cylinderd cylinder(0.75, 2.5);
  PointShapeAutoDiffSignedDistanceTester<fcl::Cylinderd> tester(&cylinder, X_WG,
                                                                kEps);

  // Case: Nearest point is on the cap.
  {
    // Test top and bottom caps.
    for (double sign : {-1, 1}) {
      // The nearest point N is the cap of the cylinder -- slightly perturbed to
      // be away from the cap center.
      const Vector3d p_GN_G{cylinder.radius * 0.25, cylinder.radius * 0.25,
                            sign * cylinder.lz / 2};

      // The query point lies outside the cylinder
      EXPECT_TRUE(tester.Test(p_GN_G, Vector3d{0, 0, sign * 0.75}));

      // The query point lies on the cap of the cylinder.
      EXPECT_TRUE(tester.Test(p_GN_G, Vector3d{0, 0, 0}));

      // The query point lies just inside the cap of the cylinder.
      EXPECT_TRUE(tester.Test(p_GN_G, Vector3d{0, 0, -0.1 * sign},
                              true /* is inside */));

      // The query point lies outside, but *on* the central axis.
      EXPECT_TRUE(
          tester.Test(Vector3d{0, 0, cylinder.lz / 2}, Vector3d{0, 0, 1.5}));
      EXPECT_TRUE(
          tester.Test(Vector3d{0, 0, cylinder.lz / 2}, Vector3d{0, 0, 0}));
      EXPECT_TRUE(tester.Test(Vector3d{0, 0, cylinder.lz / 2},
                              Vector3d{0, 0, -0.1}, true /* is inside */));
    }
  }

  // Case: Nearest point is on circular rim.
  {
    // The dimensions of the boundary of the cylinder. Used to find the
    // "support" point on the cylinder rim in an arbitrary direction.
    const Vector3d dim{cylinder.radius, cylinder.radius, cylinder.lz / 2};

    // Test top and bottom rims.
    for (double sign : {-1, 1}) {
      // Put the nearest point on the rim at some arbitrary, non-trivial angle.
      const double theta = M_PI / 7;
      const Vector3d p_NQ_G{std::cos(theta), std::sin(theta), sign};
      const Vector3d p_GN_G = p_NQ_G.cwiseProduct(dim);

      // The query point lies outside the cylinder.
      EXPECT_TRUE(tester.Test(p_GN_G, p_NQ_G));

      // The query point lies on the cap of the cylinder.
      EXPECT_TRUE(tester.Test(p_GN_G, Vector3d::Zero()));

      // A query point *inside* the cylinder would not be nearest the rim.
    }
  }

  // Case: Nearest point is on the barrel.
  {
    // The dimensions of the cylinder -- note it's shorter to make sure that the
    // query points are well within the region of the barrel.
    const Vector3d dim{cylinder.radius, cylinder.radius, cylinder.lz / 4};

    // Test upper- and lower-halves of the barrel.
    for (double sign : {-1, 1}) {
      // Put the nearest point on the rim at some arbitrary, non-trivial angle.
      const double theta = M_PI / 7;
      const double cos_theta = std::cos(theta);
      const double sin_theta = std::sin(theta);
      const Vector3d p_NQ_G{cos_theta, sin_theta, 0};
      const Vector3d p_GN_G =
          dim.cwiseProduct(Vector3d{cos_theta, sin_theta, sign});

      // The query point lies outside the cylinder.
      EXPECT_TRUE(tester.Test(p_GN_G, p_NQ_G));

      // The query point lies on the barrel of the cylinder.
      EXPECT_TRUE(tester.Test(p_GN_G, Vector3d::Zero()));

      // The query point lies inside the cylinder.
      EXPECT_TRUE(tester.Test(p_GN_G, -0.1 * p_NQ_G, true /* is inside */));
    }
  }
}
#endif

// Helper functions to indicate expectation on whether I get a distance result
// with a cylinder based on scalar type.
template <typename T>
int ExpectedCylinderResult() {
  return 1;
}

template <>
int ExpectedCylinderResult<AutoDiffXd>() {
  return 0;
}

// Helper functions to indicate expectation on whether I get a distance result
// with a ellipsoid based on scalar type.
template <typename T>
int ExpectedEllipsoidResult() {
  return 1;
}

template <>
int ExpectedEllipsoidResult<AutoDiffXd>() {
  return 0;
}

template <typename T>
void TestScalarShapeSupport() {
  // Configure the basic query.
  Vector3<T> p_WQ{10, 10, 10};
  RigidTransform<T> X_WQ{Translation3<T>{p_WQ}};
  auto point_geometry = make_shared<fcl::Sphered>(0);
  const GeometryId point_id = GeometryId::get_new_id();
  fcl::CollisionObjectd query_point(point_geometry);
  query_point.setTranslation(convert_to_double(p_WQ));
  EncodedData encoding(point_id, true);
  encoding.write_to(&query_point);
  std::vector<SignedDistanceToPoint<T>> distances;
  double threshold = std::numeric_limits<double>::max();
  const GeometryId other_id = GeometryId::get_new_id();
  std::unordered_map<GeometryId, RigidTransform<T>> X_WGs{
      {point_id, X_WQ}, {other_id, RigidTransform<T>::Identity()}};
  CallbackData<T> data{&query_point, threshold, p_WQ, &X_WGs, &distances};

  // The Drake-supported geometries (minus Mesh which isn't supported by
  // ProximityEngine yet).

  auto run_callback = [&query_point, &threshold, &distances, &data, other_id](
      auto geometry_shared_ptr) {
    // Note: the `threshold` value gets reset by invoking Callback(). So, we
    // need to reset it each time.
    threshold = std::numeric_limits<double>::max();
    distances.clear();
    fcl::CollisionObjectd object(geometry_shared_ptr);
    EncodedData object_encoding(other_id, true);
    object_encoding.write_to(&object);
    Callback<T>(&query_point, &object, &data, threshold);
  };

  // Box
  {
    run_callback(make_shared<fcl::Boxd>(1.0, 2.0, 3.5));
    EXPECT_EQ(distances.size(), 1);
  }

  // Capsule
  {
    run_callback(make_shared<fcl::Capsuled>(1.0, 2.0));
    EXPECT_EQ(distances.size(), 1);
  }

  // Cylinder
  {
    run_callback(make_shared<fcl::Cylinderd>(1.0, 2.0));
    EXPECT_EQ(distances.size(), ExpectedCylinderResult<T>());
  }

  // Ellipsoid
  { run_callback(make_shared<fcl::Ellipsoidd>(1.5, 0.7, 3));
    EXPECT_EQ(distances.size(), ExpectedEllipsoidResult<T>());
  }

  // HalfSpace
  {
    run_callback(make_shared<fcl::Halfspaced>(Vector3d::UnitZ(), 0));
    EXPECT_EQ(distances.size(), 1);
  }

  // Sphere
  {
    run_callback(make_shared<fcl::Sphered>(1.0));
    EXPECT_EQ(distances.size(), 1);
  }

  // Convex
  // TODO(SeanCurtis-TRI): Add convex that is *not* supported; create a small
  // utility test to generate a tetrahedron.
}

// This test simply confirms which scalar-shape combinations produce answers
// and which don't. That culling takes place at the Callback level so that is
// what is exercised here. Lack of support is signaled by no distance data
// being returned.
GTEST_TEST(DistanceToPoint, ScalarShapeSupportDouble) {
  TestScalarShapeSupport<double>();
}

// The autodiff version of DistanceToPoint.ScalarShapeSupportDouble.
GTEST_TEST(DistanceToPoint, ScalarShapeSupportAutoDiff) {
  TestScalarShapeSupport<AutoDiffXd>();
}

}  // namespace
}  // namespace point_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
