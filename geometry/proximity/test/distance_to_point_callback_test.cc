#include "drake/geometry/proximity/distance_to_point_callback.h"

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity/test/fcl_utilities.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/utilities.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace point_distance {
namespace {

using Eigen::Translation3d;
using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrix;
using math::RotationMatrixd;
using std::make_shared;
using std::shared_ptr;
using std::unordered_map;
using std::vector;
using symbolic::Expression;

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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PointShapeAutoDiffSignedDistanceTester);

  // Constructs a tester for a given shape G, pose in world X_WG, and tolerance.
  // The shape must be non-null and must persist beyond the life of the tester
  // as a reference to the shape will be stored.
  PointShapeAutoDiffSignedDistanceTester(const Shape* shape,
                                         const RigidTransformd& X_WG,
                                         double tolerance)
      : shape_(DRAKE_DEREF(shape)), X_WG_(X_WG), tolerance_(tolerance) {}

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
      failure << fmt::format("Analytical gradient contains NaN: {}",
                             fmt_eigen(grad_W_val));
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
                          Eigen::VectorXd::Zero(grad_size), 1.4 * tolerance_);
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
    auto p_WQ_val_compare =
        CompareMatrices(math::ExtractValue(p_WQ_ad),
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
      auto p_WQ_derivative_compare =
          CompareMatrices(math::ExtractGradient(p_WQ_ad),
                          math::ExtractGradient(p_WQ_ad_expected), tolerance_);
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
  const double tolerance_{1.4 * std::numeric_limits<double>::epsilon()};
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

// Test the DistanceToPoint functor interacting with MeshDistanceBoundary.
// 1. If the VolumeMeshBoundary doesn't have feature normals, it should throw
//    with whatever message is there.
// 2. It calls CalcSignedDistanceToSurfaceMesh() and packages the results
//    correctly (copies most values, but re-expresses gradient).
GTEST_TEST(DistanceToPoint, MeshDistanceBoundary) {
  const GeometryId mesh_geometry_id = GeometryId::get_new_id();
  // A generic pose of frame G of the mesh in World frame.
  const RigidTransformd X_WG(RollPitchYawd(M_PI_4, M_PI / 3, M_PI / 2),
                             Vector3d(0.5, 1.25, -2));
  // The query point Q in frame G of the mesh is at 10 meters above the origin.
  const Vector3d p_GQ{0, 0, 10};
  const Vector3d p_WQ = X_WG * p_GQ;
  DistanceToPoint<double> distance_to_point(mesh_geometry_id, X_WG, p_WQ);

  // Throw if there is no feature normals.
  {
    // This mesh has zero-degree knife edges prohibiting the feature normals
    // at the edges because all vertices are on the X-Y plane.
    const VolumeMesh<double> one_flat_tetrahedron_G(
        {VolumeElement(0, 1, 2, 3)}, {Vector3d::Zero(), Vector3d::UnitX(),
                                      Vector3d::UnitY(), Vector3d(1, 1, 0)});
    MeshDistanceBoundary no_feature_normals(one_flat_tetrahedron_G);
    ASSERT_FALSE(std::holds_alternative<FeatureNormalSet>(
        no_feature_normals.feature_normal()));
    DRAKE_EXPECT_THROWS_MESSAGE(distance_to_point(no_feature_normals),
                                "DistanceToPoint from meshes:.*");
  }

  // Package the results with re-expressed gradients.
  {
    // This mesh consists of a standard tetrahedron. The query point (0,0,10)
    // is 9 meters above the top vertex (0,0,1), which is the nearest point.
    const VolumeMesh<double> standard_tetrahedron_G(
        {VolumeElement{0, 1, 2, 3}}, {Vector3d::Zero(), Vector3d::UnitX(),
                                      Vector3d::UnitY(), Vector3d::UnitZ()});
    MeshDistanceBoundary mesh_boundary_G(standard_tetrahedron_G);

    const SignedDistanceToPoint<double> result =
        distance_to_point(mesh_boundary_G);
    const double kTolerance = 1e-14;
    EXPECT_EQ(result.id_G, mesh_geometry_id);
    EXPECT_NEAR(result.distance, 9, kTolerance);
    EXPECT_EQ(result.p_GN, Vector3d::UnitZ());
    EXPECT_TRUE(CompareMatrices(
        result.grad_W, X_WG.rotation() * Vector3d::UnitZ(), kTolerance));
  }
}

// Simple smoke test for signed distance to Sphere. It does the following:
//   Perform test of three different points w.r.t. a sphere: outside, on
//     surface, and inside.
//   Do it with AutoDiff relative to the query point's position.
//   Confirm the *values* of the reported quantity.
//   Confirm that the derivative of distance (extracted from AutoDiff) matches
//     the derivative computed by hand (grad_W).
GTEST_TEST(DistanceToPoint, Sphere) {
  const double kEps = 5 * std::numeric_limits<double>::epsilon();

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

// Helper function to indicate expectation on whether I get a distance result
// based on scalar type T and fcl shape S.
template <typename T, typename S>
int ExpectedResult() {
  if constexpr (std::is_same_v<T, double>) {
    return 1;
  }
  if constexpr (std::is_same_v<T, AutoDiffXd>) {
    if (std::is_same_v<S, fcl::Convexd> || std::is_same_v<S, fcl::Cylinderd> ||
        std::is_same_v<S, fcl::Ellipsoidd>) {
      return 0;
    }
    return 1;
  }
  if constexpr (std::is_same_v<T, symbolic::Expression>) {
    return 0;
  }
  DRAKE_UNREACHABLE();
}

// Makes the callback's representation of the query point Q.
template <typename T>
fcl::CollisionObjectd MakeQueryPoint(const Vector3<T>& p_WQ) {
  fcl::CollisionObjectd query_point(make_shared<fcl::Sphered>(0.0));
  query_point.setTranslation(convert_to_double(p_WQ));
  return query_point;
}

template <typename T>
void TestScalarShapeSupport() {
  // Configure the basic query.
  Vector3<T> p_WQ{10, 10, 10};
  RigidTransform<T> X_WQ{Eigen::Translation<T, 3>{p_WQ}};

  const GeometryId point_id = GeometryId::get_new_id();
  fcl::CollisionObjectd query_point = MakeQueryPoint(p_WQ);

  std::vector<SignedDistanceToPoint<T>> distances;
  double threshold = std::numeric_limits<double>::max();
  const GeometryId other_id = GeometryId::get_new_id();
  std::unordered_map<GeometryId, RigidTransform<T>> X_WGs{
      {point_id, X_WQ}, {other_id, RigidTransform<T>::Identity()}};
  std::unordered_map<GeometryId, MeshDistanceBoundary> mesh_data{
      {other_id, MeshDistanceBoundary(VolumeMesh<double>(
                     std::vector<VolumeElement>{{0, 1, 2, 3}},
                     std::vector<Vector3d>{
                         {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}))}};
  CallbackData<T> data{&query_point, threshold,  p_WQ,
                       &X_WGs,       &mesh_data, &distances};

  auto run_callback = [&query_point, &threshold, &distances, &data,
                       other_id](auto geometry_shared_ptr) {
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
    EXPECT_EQ(distances.size(), (ExpectedResult<T, fcl::Boxd>()));
  }

  // Capsule
  {
    run_callback(make_shared<fcl::Capsuled>(1.0, 2.0));
    EXPECT_EQ(distances.size(), (ExpectedResult<T, fcl::Capsuled>()));
  }

  // Convex and Mesh. Both drake::geometry::Mesh and Convex use fcl::Convexd.
  {
    // This test is independent of the content of the fcl::Convexd because
    // the mesh data is in the point_distance::CallbackData<T>, not the
    // fcl::CollisionObjectd's. For simplicity, we use a minimally valid
    // convex shape: a single vertex.
    run_callback(make_shared<fcl::Convexd>(
        make_shared<const std::vector<Vector3d>>(1, Vector3d{0, 0, 0}), 0,
        make_shared<const std::vector<int>>()));
    EXPECT_EQ(distances.size(), (ExpectedResult<T, fcl::Convexd>()));
  }

  // Cylinder
  {
    run_callback(make_shared<fcl::Cylinderd>(1.0, 2.0));
    EXPECT_EQ(distances.size(), (ExpectedResult<T, fcl::Cylinderd>()));
  }

  // Ellipsoid
  {
    run_callback(make_shared<fcl::Ellipsoidd>(1.5, 0.7, 3));
    EXPECT_EQ(distances.size(), (ExpectedResult<T, fcl::Ellipsoidd>()));
  }

  // HalfSpace
  {
    run_callback(make_shared<fcl::Halfspaced>(Vector3d::UnitZ(), 0));
    EXPECT_EQ(distances.size(), (ExpectedResult<T, fcl::Halfspaced>()));
  }

  // Sphere
  {
    run_callback(make_shared<fcl::Sphered>(1.0));
    EXPECT_EQ(distances.size(), (ExpectedResult<T, fcl::Sphered>()));
  }
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

GTEST_TEST(DistanceToPoint, ScalarShapeSupportExpression) {
  TestScalarShapeSupport<symbolic::Expression>();
}

// Test point_distance::Callback() for meshes (Mesh and Convex).
// If the fcl representation is fcl::GEOM_CONVEX, check to see if it has
// the corresponding MeshDistanceBoundary in the CallbackData.
// 1. If so, dispatch it to the DistanceToPoint functor.
// 2. Otherwise, it's a no-op.
GTEST_TEST(Callback, MeshAndConvex) {
  const Vector3d p_WQ{0, 1, 2};
  fcl::CollisionObjectd query_point = MakeQueryPoint(p_WQ);

  // Both drake::geometry::Mesh and Convex use fcl::Convexd. Its content
  // is irrelevant for this test because the mesh data is in CallbackData.
  // For simplicity, we use a minimally valid convex shape: a single vertex.
  auto mesh_fcl_geometry = make_shared<fcl::Convexd>(
      make_shared<const std::vector<Vector3d>>(1, Vector3d{0, 0, 0}), 0,
      make_shared<const std::vector<int>>());
  const GeometryId mesh_id = GeometryId::get_new_id();
  // The pose of the mesh's frame M in World frame.
  const RigidTransformd X_WM{Vector3d{1, 2, 3}};
  // For completeness, we set the pose of the mesh in the CollisionObject
  // even though we don't need it for this test. For calculation in drake, we
  // use the pose in CallbackData. For calculation in FCL, it uses the pose
  // in CollisionObject.
  fcl::CollisionObjectd mesh_collision_object(
      mesh_fcl_geometry, X_WM.rotation().matrix(), X_WM.translation());
  EncodedData(mesh_id, true).write_to(&mesh_collision_object);

  // Remaining components of CallbackData other than the mesh_boundaries.
  const double kThreshold100Meters = 100;
  const std::unordered_map<GeometryId, RigidTransformd> X_WGs{{mesh_id, X_WM}};

  // There is MeshDistanceBoundary.
  {
    const std::unordered_map<GeometryId, MeshDistanceBoundary> mesh_boundaries{
        {mesh_id, MeshDistanceBoundary(VolumeMesh<double>(
                      {VolumeElement{0, 1, 2, 3}},
                      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
                       Vector3d::UnitZ()}))}};
    std::vector<SignedDistanceToPoint<double>> distances;
    CallbackData<double> callback_data{
        &query_point, kThreshold100Meters, p_WQ,
        &X_WGs,       &mesh_boundaries,    &distances};

    double threshold_out = 0;
    // Expect Callback() to return false, so the broad-phase fcl will continue
    // to other objects.
    EXPECT_FALSE(Callback<double>(&query_point, &mesh_collision_object,
                                  &callback_data, threshold_out));
    EXPECT_EQ(distances.size(), 1);
    EXPECT_EQ(threshold_out, kThreshold100Meters);
  }

  // No MeshDistanceBoundary.
  {
    const std::unordered_map<GeometryId, MeshDistanceBoundary>
        no_mesh_boundaries;
    std::vector<SignedDistanceToPoint<double>> distances;
    CallbackData<double> callback_data{
        &query_point, kThreshold100Meters, p_WQ,
        &X_WGs,       &no_mesh_boundaries, &distances};

    double threshold_out = 0;
    // Expect Callback() to return false, so the broad-phase fcl will continue
    // to other objects.
    EXPECT_FALSE(Callback<double>(&query_point, &mesh_collision_object,
                                  &callback_data, threshold_out));
    EXPECT_EQ(distances.size(), 0);
    EXPECT_EQ(threshold_out, kThreshold100Meters);
  }
}

// Test the narrow-phase part of ComputeSignedDistanceToPoint.

// Parameter for the value-parameterized test fixture SignedDistanceToPointTest.
struct SignedDistanceToPointTestData {
  SignedDistanceToPointTestData(shared_ptr<Shape> geometry_in,
                                const RigidTransformd& X_WG_in,
                                const Vector3d& p_WQ_in,
                                const SignedDistanceToPoint<double>& expect_in)
      : geometry(geometry_in),
        X_WG(X_WG_in),
        p_WQ(p_WQ_in),
        expected_result(expect_in) {}

  static SignedDistanceToPointTestData Make(shared_ptr<Shape> shape,
                                            const RigidTransformd& X_WG,
                                            const Vector3d& p_WQ,
                                            const Vector3d& p_GN,
                                            double distance,
                                            const Vector3d& grad_W) {
    return SignedDistanceToPointTestData{
        shape, X_WG, p_WQ,
        SignedDistanceToPoint{GeometryId::get_new_id(), p_GN, distance,
                              grad_W}};
  }

  // Google Test uses this operator to report the test data in the log file
  // when a test fails.
  friend std::ostream& operator<<(std::ostream& os,
                                  const SignedDistanceToPointTestData& obj) {
    os << fmt::format(
        "{{\n"
        "  geometry: (not printed)\n"
        "  X_WG: (not printed)\n"
        "  p_WQ: {}\n"
        "  expected_result.p_GN: {}\n"
        "  expected_result.distance: {}\n"
        "  expected_result.grad_W: {}\n"
        "}}",
        fmt_eigen(obj.p_WQ), fmt_eigen(obj.expected_result.p_GN),
        obj.expected_result.distance, fmt_eigen(obj.expected_result.grad_W));
    return os;
  }

  shared_ptr<Shape> geometry;
  const RigidTransformd X_WG;
  const Vector3d p_WQ;
  const SignedDistanceToPoint<double> expected_result;
};

std::vector<SignedDistanceToPointTestData> GenDistanceTestDataSphere(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  // We use these identities
  //           2^2   + 3^2    + 6^2   = 7^2         (1)
  //           0.2^2 + 0.3^2  + 0.6^2 = 0.7^2       (2)
  //           0.1^2 + 0.15^2 + 0.3^2 = 0.35^2      (3)
  // to set up the radius of the sphere geometry G and the position of the
  // query point Q in such a way that both the signed distance and the
  // position of the nearest point N can be expressed with fixed-point numbers.
  //
  // We will set a query point Q both outside G and inside G in such a way
  // that Q is collinear to the center Go of the sphere and the nearest point N.
  //
  //            Q
  //           /
  //     ooo  /
  //  o      N
  // o      Q  o
  // o    Go   o
  // o         o
  //  o       o
  //     ooo
  //
  // From (2), we will set the radius of G to 0.7, so later we will have
  // p_GN at (0.2, 0.3, 0.6). From (1), we will set p_GQ to (2,3,6), so it
  // will be outside G at the positive distance 7 - 0.7 = 6.3. From (3), we will
  // set p_GQ to (0.1, 0.15, 0.3), so it will be inside G at the negative
  // distance 0.35 - 0.7 = -0.35. Both positions of Q will have the same
  // nearest point N and the gradient vector.
  auto sphere = make_shared<Sphere>(0.7);
  std::vector<SignedDistanceToPointTestData> test_data{
      // p_GQ = (2,3,6) is outside G at the positive distance 6.3 = 7 - 0.7.
      SignedDistanceToPointTestData::Make(
          sphere, X_WG, X_WG * Vector3d{2, 3, 6}, Vector3d(0.2, 0.3, 0.6), 6.3,
          X_WG.rotation() * Vector3d(2, 3, 6) / 7),
      // p_GQ = (0.1,0.15,0.3) is inside G at the negative distance -0.35.
      SignedDistanceToPointTestData::Make(
          sphere, X_WG, X_WG * Vector3d{0.1, 0.15, 0.3},
          Vector3d(0.2, 0.3, 0.6), -0.35,
          X_WG.rotation() * Vector3d(2, 3, 6) / 7),
      // Reports an arbitrary gradient vector (as defined in the
      // QueryObject::ComputeSignedDistanceToPoint() documentation) at the
      // center of the sphere.
      SignedDistanceToPointTestData::Make(
          sphere, X_WG, X_WG * Vector3d{0, 0, 0}, Vector3d(0.7, 0, 0), -0.7,
          X_WG.rotation() * Vector3d(1, 0, 0))};
  return test_data;
}

std::vector<SignedDistanceToPointTestData> GenDistTestTransformSphere() {
  const RigidTransformd X_WG(RollPitchYawd(M_PI / 6, M_PI / 3, M_PI_2),
                             Vector3d{10, 11, 12});
  return GenDistanceTestDataSphere(X_WG);
}

// We declare this function here, so we can call it from
// GenDistanceTestDataOutsideBox() below.
std::vector<SignedDistanceToPointTestData> GenDistanceTestDataBoxBoundary(
    const RigidTransformd& X_WG = RigidTransformd::Identity());

// Generates test data for a query point Q outside a box geometry G nearest
// to one of the 6 faces, the 12 edges, and the 8 vertices of the box.
// First we call GenDistanceTestDataBoxBoundary() to generate test data for
// query points on the box boundary. Then, we move the query point along the
// gradient vector by a unit distance.  In each case, the nearest point to Q
// on ∂G stays the same, the signed distance becomes +1, and the gradient
// vector stays the same.
std::vector<SignedDistanceToPointTestData> GenDistanceTestDataOutsideBox(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const std::vector<SignedDistanceToPointTestData> test_data_box_boundary =
      GenDistanceTestDataBoxBoundary(X_WG);
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const auto& data : test_data_box_boundary) {
    const shared_ptr<Shape> shape = data.geometry;
    // We expect the shape to be a box.
    DRAKE_DEMAND(dynamic_cast<Box*>(shape.get()) != nullptr);
    // The gradient grad_W has unit length by construction.
    const Vector3d p_WQ = data.p_WQ + data.expected_result.grad_W;
    const GeometryId& id = data.expected_result.id_G;
    const Vector3d& p_GN = data.expected_result.p_GN;
    const double distance = 1;
    const Vector3d& grad_W = data.expected_result.grad_W;
    test_data.emplace_back(
        shape, X_WG, p_WQ,
        SignedDistanceToPoint<double>(id, p_GN, distance, grad_W));
  }
  return test_data;
}

std::vector<SignedDistanceToPointTestData> GenDistTestTransformOutsideBox() {
  RigidTransformd X_WG(RollPitchYawd(M_PI / 3, M_PI / 6, M_PI_2),
                       Vector3d{10, 11, 12});
  return GenDistanceTestDataOutsideBox(X_WG);
}

// Generates test data for a query point Q on the boundary ∂G of a box
// geometry G. Q can be at the 8 corners, at the midpoints of the 12 edges, or
// at the centers of the 6 faces of G. The set of all 26 positions can be
// expressed in G's frame as the following Cartesian product excluding the
// origin (3x3x3-1 = 26) :
//
//     p_GQ ∈ {-h(x),0,+h(x)} x {-h(y),0,+h(y)} x {-h(z),0,+h(z)} - {(0,0,0)},
//
// where h(x), h(y), and h(z) are the half width, half depth, and half height
// of G, respectively. We do not allow p_GQ=(0,0,0) because it is in the
// interior of G. The number of zeroes in p_GQ corresponds to the location at
// a corner (no zero, 3 non-zeroes), at the midpoint of an edge (1 zero,
// 2 non-zeroes), or at the center of a face (2 zeroes, 1 non-zero).
//     The positions above is parameterized by the sign vector s expressed in
// G's frame as:
//
//     s_G = (sx,sy,sz) ∈ {-1,0,+1} x {-1,0,+1} x {-1,0,+1} - {(0,0,0)},
//     p_GQ(s) = s_G ∘ h_G,
//
// where h_G = (h(x), h(y), h(z)), which is the vector from the origin Go to a
// vertex of G expressed in G's frame. The operator ∘ is the entrywise product
// (also known as Hadamard product): (a,b,c)∘(u,v,w) = (a*u, b*v, c*w).
//     In each case, Q is also its own nearest point on ∂G, the signed distance
// is always zero, and the gradient vector equals the normalized unit vector of
// the vector s.
std::vector<SignedDistanceToPointTestData> GenDistanceTestDataBoxBoundary(
    const RigidTransformd& X_WG) {
  auto box = make_shared<Box>(20.0, 30.0, 10.0);
  const Vector3d h_G = box->size() / 2;
  const double distance = 0;
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const double sx : {-1, 0, 1}) {
    for (const double sy : {-1, 0, 1}) {
      for (const double sz : {-1, 0, 1}) {
        // Skip the origin.
        if (sx == 0 && sy == 0 && sz == 0) continue;
        const Vector3d s_G(sx, sy, sz);
        const Vector3d p_GQ = s_G.cwiseProduct(h_G);
        const Vector3d p_WQ = X_WG * p_GQ;
        // We create new id for each test case to help distinguish them.
        const GeometryId id = GeometryId::get_new_id();
        // Q is its own nearest point on ∂G.
        const Vector3d& p_GN = p_GQ;
        // Rotation matrix for transforming vector expression from G to world.
        const RotationMatrixd& R_WG = X_WG.rotation();
        const Vector3d grad_G = s_G.normalized();
        const Vector3d grad_W = R_WG * grad_G;
        test_data.emplace_back(
            box, X_WG, p_WQ,
            SignedDistanceToPoint<double>(id, p_GN, distance, grad_W));
      }
    }
  }
  return test_data;
}

std::vector<SignedDistanceToPointTestData> GenDistTestTransformBoxBoundary() {
  RigidTransformd X_WG(RollPitchYawd(M_PI / 3, M_PI / 6, M_PI_2),
                       Vector3d{10, 11, 12});
  return GenDistanceTestDataBoxBoundary(X_WG);
}

// Generates test data for a query point Q inside a box geometry G with a
// unique nearest point on the boundary ∂G. Unlike Q on ∂G or outside G that
// has 26 cases each, we have only 6 cases of Q inside G. In each case, Q is
// unambiguously closest to a single face.
//     The position of Q is parameterized by a chosen distance d and the
// outward unit normal vector s of the six faces of G. We calculate the
// center C of the face and offset C by the distance d inwards into G:
//
//         d    = min {h(x), h(y), h(z)} / 2,
//         s_G  ∈ {-x, +x, -y, +y, -z, +z}
//         p_GC = s_G ∘ h_G
//         p_GQ = p_GC - d * s_G,
//
// where h_G = (h(x), h(y), h(z)) is the vector of the half width, half depth,
// and half height of G. It is the vector from the origin Go to a corner
// of G expressed in G's frame. The operator ∘ is the entry-wise product:
// (a,b,c)∘(u,v,w) = (a*u, b*v, c*w).
//     The chosen d is small enough that Q at the inward normal offset from a
// face center is still unambiguously closest to that face.
//     In each case, the nearest point N on ∂G is C, the negative signed
// distance is -d, and the gradient vector is the face normal vector s.
std::vector<SignedDistanceToPointTestData> GenDistTestDataInsideBoxUnique(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  // Create a box [-10,10]x[-15,15]x[-5,5],
  auto box = make_shared<Box>(20.0, 30.0, 10.0);
  const Vector3d h_G = box->size() / 2;
  const double d = h_G.minCoeff() / 2;
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const Vector3d unit_vector :
       {Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()}) {
    for (const double sign : {-1, 1}) {
      // Unit face normal vector.
      const Vector3d s_G = sign * unit_vector;
      // Center of a face.
      const Vector3d p_GC = s_G.cwiseProduct(h_G);
      const Vector3d p_GQ = p_GC - d * s_G;
      const Vector3d p_WQ = X_WG * p_GQ;
      // We create new id for each test case to help distinguish them.
      const GeometryId id = GeometryId::get_new_id();
      // The nearest point is at the face center.
      const Vector3d& p_GN = p_GC;
      // Rotation matrix for transforming vector expression from G to world.
      const RotationMatrixd R_WG = X_WG.rotation();
      const Vector3d& grad_G = s_G;
      const Vector3d grad_W = R_WG * grad_G;
      test_data.emplace_back(
          box, X_WG, p_WQ, SignedDistanceToPoint<double>(id, p_GN, -d, grad_W));
    }
  }
  return test_data;
}

// We test a rigid transform with the signed distance to query point Q inside
// a box B only when Q has a unique nearest point on the boundary ∂B.
std::vector<SignedDistanceToPointTestData>
GenDistTestTransformInsideBoxUnique() {
  RigidTransformd X_WG(RollPitchYawd(M_PI / 3, M_PI / 6, M_PI_2),
                       Vector3d{10, 11, 12});
  return GenDistTestDataInsideBoxUnique(X_WG);
}

// A query point Q inside a box G with multiple nearest points on the
// boundary ∂B needs a tie-breaking rule. However, a rigid transform can
// contaminate the tie breaking due to rounding errors.  We test the tie
// breaking with the identity transform only.
std::vector<SignedDistanceToPointTestData> GenDistTestDataInsideBoxNonUnique() {
  // We set up a 20x10x10 box [-10,10]x[-5,5]x[-5,5].  Having the same depth
  // and height allows Q to have five nearest faces in the last case below.
  auto box = make_shared<Box>(20.0, 10.0, 10.0);
  const RigidTransformd& X_WG = RigidTransformd::Identity();
  std::vector<SignedDistanceToPointTestData> test_data{
      // Q is nearest to the face [-10,10]x[-5,5]x{5}.
      SignedDistanceToPointTestData::Make(box, X_WG, X_WG * Vector3d{6, 1, 2},
                                          Vector3d(6, 1, 5), -3.0,
                                          X_WG.rotation() * Vector3d(0, 0, 1)),
      // Q is nearest to two faces {10}x[-5,5]x[-5,5] and [-10,10]x[-5,5]x{5}.
      SignedDistanceToPointTestData::Make(box, X_WG, X_WG * Vector3d{6, 0, 1},
                                          Vector3d(10, 0, 1), -4.0,
                                          X_WG.rotation() * Vector3d(1, 0, 0)),
      // Q is nearest to three faces {10}x[-5,5]x[-5,5], [-10,10]x{5}x[-5,5],
      // and [-10,10]x[-5,5]x{5}.
      SignedDistanceToPointTestData::Make(box, X_WG, X_WG * Vector3d{6, 1, 1},
                                          Vector3d(10, 1, 1), -4.0,
                                          X_WG.rotation() * Vector3d(1, 0, 0)),
      // Q at the center of the box is nearest to four faces
      // [-10,10]x{5}x[-5,5], [-10,10]x{-5}x[-5,5], [-10,10]x[5,-5]x{5},
      // and [-10,10]x[5,-5]x{-5}.
      SignedDistanceToPointTestData::Make(box, X_WG, X_WG * Vector3d{0, 0, 0},
                                          Vector3d(0, 5, 0), -5.0,
                                          X_WG.rotation() * Vector3d(0, 1, 0)),
      // Q is nearest to five faces {10}x[-5,5]x[-5,5], [-10,10]x{5}x[-5,5],
      // [-10,10]x{-5}x[-5,5], [-10,10]x[5,-5]x{5}, and [-10,10]x[5,-5]x{-5}.
      SignedDistanceToPointTestData::Make(box, X_WG, X_WG * Vector3d{5, 0, 0},
                                          Vector3d(10, 0, 0), -5.0,
                                          X_WG.rotation() * Vector3d(1, 0, 0))};
  return test_data;
}

std::vector<SignedDistanceToPointTestData> GenDistanceTestDataTranslateBox() {
  // We set up a 20x10x30 box G centered at the origin [-10,10]x[-5,5]x[-15,15].
  auto box = make_shared<Box>(20.0, 10.0, 30.0);
  std::vector<SignedDistanceToPointTestData> test_data{
      // We translate G by (10,20,30) to [0,20]x[15,25]x[15,45] in World frame.
      // The position of the query point p_WQ(23,20,30) is closest to G at
      // (20,20,30) in World frame, which is p_GN=(10,0,0) in G's frame.
      // The gradient vector is expressed in both frames as (1,0,0).
      SignedDistanceToPointTestData::Make(
          box, Translation3d(10, 20, 30), Vector3d{23, 20, 30},
          Vector3d(10, 0, 0), 3.0, Vector3d(1, 0, 0))};
  return test_data;
}

// Generate test data for a query point Q from a cylinder G.

// We separate the test data for Q on the boundary ∂G into two parts: Q on
// the top/bottom circles (GenDistTestDataCylinderBoundaryCircle) and Q on
// the cap/barrel surfaces (GenDistTestDataCylinderBoundarySurface).
// Here, a circle is a 1-dimensional closed curve, and a cap is a
// 2-dimensional flat surface bounded by a circle.
//     We separate the two cases because we will generate Q outside/inside G
// by moving Q on ∂G outwards/inwards along its gradient vector. In most
// cases, we can move Q a small distance, and its nearest point N on ∂G stays
// the same, namely the original Q on ∂G. However, moving Q on the top/bottom
// circles inwards would change its nearest point N, and we don't want to do
// that.
//     Later we will combine them into GenDistTestDataCylinderBoundary which
// will be used by GenDistTestDataOutsideCylinder.
//     In summary, the call graph looks like this:
//
// GenDistTestDataInsideCylinder
// |
// |  GenDistTestDataOutsideCylinder
// |  |
// |  +--> GenDistTestDataCylinderBoundary
// |       |
// |       +--> GenDistTestDataCylinderBoundaryCircle
// |       |
// +-----> +--> GenDistTestDataCylinderBoundarySurface

// Generates test data for a query point Q on the top/bottom circles of a
// cylinder geometry G.
std::vector<SignedDistanceToPointTestData>
GenDistTestDataCylinderBoundaryCircle(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const RotationMatrixd& R_WG = X_WG.rotation();
  auto cylinder = make_shared<Cylinder>(3.0, 5.0);
  const double radius = cylinder->radius();
  const double half_length = cylinder->length() / 2;
  // We want the test to cover all the combinations of positive, negative,
  // and zero values of both x and y coordinates on the two boundary circles
  // of the cylinder. Furthermore, each (x,y) has |x| ≠ |y| to avoid
  // symmetry that might hide problems. We achieve this by having 12 points
  // equally spread around a unit circle and map them to the two boundary
  // circles later.
  const int kNumVectors = 12;
  // unit vectors in x-y plane of G's frame.
  std::vector<Vector3d> all_xy_vectors;
  const Vector3d kXVector(1, 0, 0);
  for (int c = 0; c < kNumVectors; ++c) {
    all_xy_vectors.push_back(
        RotationMatrixd(RollPitchYawd(0, 0, 2 * M_PI * c / kNumVectors)) *
        kXVector);
  }
  const double distance = 0;  // Q on ∂G has distance zero.
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const auto& z_vector : {Vector3d(0, 0, 1), Vector3d(0, 0, -1)}) {
    for (const auto& xy_vector : all_xy_vectors) {
      const GeometryId id = GeometryId::get_new_id();
      // xy_vector and z_vector are unit vectors in x-y plane and z-axis of G.
      const Vector3d p_GQ = radius * xy_vector + half_length * z_vector;
      const Vector3d p_WQ = X_WG * p_GQ;
      // Q is its own nearest point on ∂G.
      const Vector3d p_GN = p_GQ;
      // We set the gradient vector according to the convention described in
      // QueryObject::ComputeSignedDistanceToPoint().  Mathematically it is
      // undefined.
      const Vector3d grad_G = (xy_vector + z_vector).normalized();
      const Vector3d grad_W = R_WG * grad_G;
      test_data.emplace_back(
          cylinder, X_WG, p_WQ,
          SignedDistanceToPoint<double>(id, p_GN, distance, grad_W));
    }
  }
  return test_data;
}

// Generates test data for a query point Q on the boundary surface ∂G of a
// cylinder geometry G. Q can be on the barrel or on the top or bottom caps.
std::vector<SignedDistanceToPointTestData>
GenDistTestDataCylinderBoundarySurface(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const RotationMatrixd& R_WG = X_WG.rotation();
  auto cylinder = make_shared<Cylinder>(3.0, 5.0);
  const double radius = cylinder->radius();
  const double half_length = cylinder->length() / 2;
  // We want the test to cover all the combinations of positive, negative,
  // and zero values of both x and y coordinates on some circles on the
  // barrel or on the caps. Furthermore, each (x,y) has |x| ≠ |y| to avoid
  // symmetry that might hide problems. We achieve this goal by having
  // 12 points equally spread around a unit circle and map them to the barrel
  // or the caps later.
  const int kNumVectors = 12;
  // unit vectors in x-y plane of G's frame.
  std::vector<Vector3d> all_xy_vectors;
  const Vector3d kXVector(1, 0, 0);
  for (int c = 0; c < kNumVectors; ++c) {
    all_xy_vectors.push_back(
        RotationMatrixd(RollPitchYawd(0, 0, 2 * M_PI * c / kNumVectors)) *
        kXVector);
  }
  struct LocalTestData {
    LocalTestData(const Vector3d& p_GQ_in, const Vector3d& grad_G_in)
        : p_GQ(p_GQ_in), grad_G(grad_G_in) {}
    Vector3d p_GQ;
    Vector3d grad_G;
  };
  // Generate LocalTestData that will convert to
  // SignedDistanceToPointTestData later.
  std::vector<LocalTestData> test_data_barrel;
  std::vector<LocalTestData> test_data_caps;
  std::vector<LocalTestData> test_data_cap_centers;
  for (const auto& z_vector : {Vector3d(0, 0, 1), Vector3d(0, 0, -1)}) {
    // Q at the centers of the top and bottom caps.
    {
      // z_vector is a unit vector.
      const Vector3d p_GQ = half_length * z_vector;
      // The gradient vector on the circular disks is along the z-axis of G.
      const Vector3d grad_G = z_vector;
      test_data_cap_centers.emplace_back(p_GQ, grad_G);
    }
    for (const auto& xy_vector : all_xy_vectors) {
      // Q on the barrel.
      {
        // Control how far vertically Q is from x-y plane of G.
        const double kZFactor = 0.5;
        // xy_vector and z_vector are unit vectors in x-y plane and z-axis of G.
        const Vector3d p_GQ =
            radius * xy_vector + half_length * kZFactor * z_vector;
        // The gradient vector on the barrel is parallel to the x-y plane of G.
        const Vector3d grad_G = xy_vector;
        test_data_barrel.emplace_back(p_GQ, grad_G);
      }
      // Q on the top and bottom caps.
      {
        // Control how far radially Q is from z-axis of G.
        const double kRFactor = 0.6;
        // xy_vector and z_vector are unit vectors in x-y plane and z-axis of G.
        const Vector3d p_GQ =
            radius * kRFactor * xy_vector + half_length * z_vector;
        // The gradient vector on the caps is along the z-axis of G.
        const Vector3d grad_G = z_vector;
        test_data_caps.emplace_back(p_GQ, grad_G);
      }
    }
  }
  std::vector<SignedDistanceToPointTestData> test_data;
  // Convert LocalTestData to SignedDistanceToPointTestData and add to
  // test_data.
  auto convert = [&cylinder, &X_WG, &R_WG,
                  &test_data](const LocalTestData& local) {
    const Vector3d p_WQ = X_WG * local.p_GQ;
    // Q on ∂G has distance zero.
    const double distance = 0;
    // Q is its own nearest point on ∂G.
    const Vector3d p_GN = local.p_GQ;
    const Vector3d grad_W = R_WG * local.grad_G;
    // Implicitly generate a new geometry id for every test record.
    test_data.push_back(SignedDistanceToPointTestData::Make(
        cylinder, X_WG, p_WQ, p_GN, distance, grad_W));
  };
  std::for_each(test_data_barrel.begin(), test_data_barrel.end(), convert);
  std::for_each(test_data_caps.begin(), test_data_caps.end(), convert);
  std::for_each(test_data_cap_centers.begin(), test_data_cap_centers.end(),
                convert);
  return test_data;
}

// Generates test data for a query point Q on the boundary of a cylinder
// geometry G. Combine GenDistTestDataCylinderBoundaryCircle with
// GenDistTestDataCylinderBoundarySurface.
std::vector<SignedDistanceToPointTestData> GenDistTestDataCylinderBoundary(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  auto test_data = GenDistTestDataCylinderBoundaryCircle(X_WG);
  const auto test_data_boundary_surface =
      GenDistTestDataCylinderBoundarySurface(X_WG);
  for (const auto& data : test_data_boundary_surface) {
    test_data.emplace_back(data.geometry, data.X_WG, data.p_WQ,
                           data.expected_result);
  }
  return test_data;
}

// Generates test data for a query point Q outside a cylinder geometry G.
// First we call GenDistTestDataCylinderBoundary() to generate test data for
// query points on the boundary.  Then, we move the query point along the
// gradient vector by a unit distance outward. The nearest point to Q on ∂G
// stays the same, the signed distance becomes +1, and the gradient vector
// stays the same.
std::vector<SignedDistanceToPointTestData> GenDistTestDataOutsideCylinder(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const auto test_data_cylinder_boundary =
      GenDistTestDataCylinderBoundary(X_WG);
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const auto& data : test_data_cylinder_boundary) {
    const shared_ptr<Shape>& shape = data.geometry;
    // We expect the shape to be a cylinder.
    DRAKE_DEMAND(dynamic_cast<Cylinder*>(shape.get()) != nullptr);
    // The gradient grad_W has unit length by construction.
    const Vector3d p_WQ = data.p_WQ + data.expected_result.grad_W;
    const GeometryId& id = data.expected_result.id_G;
    const Vector3d& p_GN = data.expected_result.p_GN;
    const double distance = 1;
    const Vector3d& grad_W = data.expected_result.grad_W;
    test_data.emplace_back(
        shape, X_WG, p_WQ,
        SignedDistanceToPoint<double>(id, p_GN, distance, grad_W));
  }
  return test_data;
}

// Generates test data for a query point Q inside a cylinder geometry G with
// a unique nearest point on the boundary ∂G. Unlike Q on ∂G or outside G, Q
// inside G cannot have its nearest point N on the top and bottom boundary
// circles; however, it can have N on the top and bottom cap surfaces.
// First we call GenDistTestDataCylinderBoundarySurface() to generate data on
// the boundary surface.  Then, we move the query point along the gradient
// vector a small negative distance into the interior.  The nearest point N
// stays the same, the signed distance becomes the negative distance, and the
// gradient vector stays the same.
std::vector<SignedDistanceToPointTestData> GenDistTestDataInsideCylinder(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const auto test_data_cylinder_boundary_surface =
      GenDistTestDataCylinderBoundarySurface(X_WG);
  std::vector<SignedDistanceToPointTestData> test_data;
  const double kNegativeDistance = -0.1;
  for (const auto& data : test_data_cylinder_boundary_surface) {
    const shared_ptr<Shape>& shape = data.geometry;
    // We expect the shape to be a cylinder.
    DRAKE_DEMAND(dynamic_cast<Cylinder*>(shape.get()) != nullptr);
    // The gradient grad_W has unit length by construction.
    const Vector3d p_WQ =
        data.p_WQ + kNegativeDistance * data.expected_result.grad_W;
    const GeometryId& id = data.expected_result.id_G;
    const Vector3d& p_GN = data.expected_result.p_GN;
    const Vector3d& grad_W = data.expected_result.grad_W;
    test_data.emplace_back(
        shape, X_WG, p_WQ,
        SignedDistanceToPoint<double>(id, p_GN, kNegativeDistance, grad_W));
  }
  return test_data;
}

// Generates test data for a query point Q at the center of a long cylinder
// geometry G. Q's nearest point on ∂G is not unique. Our code picks the one
// on the x's axis of G's frame.
std::vector<SignedDistanceToPointTestData> GenDistTestDataCylinderCenter(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const RotationMatrixd& R_WG = X_WG.rotation();
  auto long_cylinder = make_shared<Cylinder>(1.0, 20.0);
  const double radius = long_cylinder->radius();
  const GeometryId id = GeometryId::get_new_id();
  // The query point Q is at the center of the cylinder.
  const Vector3d p_GQ(0, 0, 0);
  const Vector3d p_WQ = X_WG * p_GQ;
  const double distance = -radius;
  // The nearest point N is on the x's axis of G's frame by convention.
  const Vector3d p_GN = radius * Vector3d(1, 0, 0);
  const Vector3d grad_G = Vector3d(1, 0, 0);
  const Vector3d grad_W = R_WG * grad_G;
  std::vector<SignedDistanceToPointTestData> test_data;
  test_data.emplace_back(
      long_cylinder, X_WG, p_WQ,
      SignedDistanceToPoint<double>(id, p_GN, distance, grad_W));
  return test_data;
}

// Generates test data for a query point Q on the boundary, inside, and
// outside a cylinder with a rigid transform.
std::vector<SignedDistanceToPointTestData> GenDistTestDataCylinderTransform() {
  RigidTransformd X_WG(RollPitchYawd(M_PI / 3, M_PI / 6, M_PI_2),
                       Vector3d{10, 11, 12});
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const auto& record : GenDistTestDataCylinderBoundary(X_WG))
    test_data.emplace_back(record);
  for (const auto& record : GenDistTestDataOutsideCylinder(X_WG))
    test_data.emplace_back(record);
  for (const auto& record : GenDistTestDataInsideCylinder(X_WG))
    test_data.emplace_back(record);
  for (const auto& record : GenDistTestDataCylinderCenter(X_WG))
    test_data.emplace_back(record);
  return test_data;
}

// Generate test data for a query point Q relative to a half space.
std::vector<SignedDistanceToPointTestData> GenDistTestDataHalfSpace() {
  std::vector<SignedDistanceToPointTestData> test_data;

  const RigidTransformd X_WG1(RollPitchYawd(M_PI / 3, M_PI / 6, M_PI_2),
                              Vector3d{10, 11, 12});
  auto hs = make_shared<HalfSpace>();
  const GeometryId id = GeometryId::get_new_id();

  for (const auto& X_WG : {RigidTransformd(), X_WG1}) {
    const auto& R_WG = X_WG.rotation();
    const Vector3d grad_W = R_WG.col(2);
    const Vector3d p_GN(1.25, 1.5, 0);
    const Vector3d p_WN = X_WG * p_GN;

    // Outside the half space.
    const double distance = 1.5;
    const Vector3d p_WQ1 = p_WN + distance * grad_W;
    test_data.emplace_back(
        hs, X_WG, p_WQ1,
        SignedDistanceToPoint<double>(id, p_GN, distance, grad_W));

    // On the half space boundary.
    const Vector3d p_WQ2 = p_WN;
    test_data.emplace_back(
        hs, X_WG, p_WQ2, SignedDistanceToPoint<double>(id, p_GN, 0.0, grad_W));

    // Inside the half space.
    const Vector3d p_WQ3 = p_WN - distance * grad_W;
    test_data.emplace_back(
        hs, X_WG, p_WQ3,
        SignedDistanceToPoint<double>(id, p_GN, -distance, grad_W));
  }

  return test_data;
}

// This test fixture takes data generated by GenDistanceTestData*(),
// GenDistTestData*(), and GenDistTestTransform*() above.
struct SignedDistanceToPointTest
    : public testing::TestWithParam<SignedDistanceToPointTestData> {
  unordered_map<GeometryId, RigidTransformd> X_WGs;

  // The tolerance value for determining equivalency between expected and
  // tested results. The underlying algorithms have an empirically-determined,
  // hard-coded tolerance of 1e-14 to account for loss of precision due to
  // rigid transformations and this tolerance reflects that.
  static constexpr double kTolerance = 1e-14;

  SignedDistanceToPointTest() {
    const auto& data = GetParam();
    X_WGs[data.expected_result.id_G] = data.X_WG;
  }

  // Creates the FCL objects, invokes Callback<double>, and returns results.
  vector<SignedDistanceToPoint<double>> RunCallback(double threshold) {
    const auto& data = GetParam();
    const GeometryId id = data.expected_result.id_G;

    fcl::CollisionObjectd query_point = MakeQueryPoint(data.p_WQ);

    std::unique_ptr<fcl::CollisionObjectd> geometry_object =
        MakeFclObject(*data.geometry, id, /* is_dynamic= */ true);

    vector<SignedDistanceToPoint<double>> distances;
    unordered_map<GeometryId, MeshDistanceBoundary> no_meshes;
    CallbackData<double> callback_data{&query_point, threshold,  data.p_WQ,
                                       &X_WGs,       &no_meshes, &distances};
    Callback<double>(&query_point, geometry_object.get(), &callback_data,
                     threshold);
    return distances;
  }
};

TEST_P(SignedDistanceToPointTest, SingleQueryPoint) {
  const auto& data = GetParam();

  auto results = RunCallback(std::numeric_limits<double>::max());
  EXPECT_EQ(results.size(), 1);
  EXPECT_EQ(results[0].id_G, data.expected_result.id_G);
  EXPECT_TRUE(
      CompareMatrices(results[0].p_GN, data.expected_result.p_GN, kTolerance))
      << "Incorrect nearest point.";
  EXPECT_NEAR(results[0].distance, data.expected_result.distance, kTolerance)
      << "Incorrect signed distance.";
  EXPECT_TRUE(CompareMatrices(results[0].grad_W, data.expected_result.grad_W,
                              kTolerance))
      << "Incorrect gradient vector.";
}

TEST_P(SignedDistanceToPointTest, SingleQueryPointWithThreshold) {
  const auto& data = GetParam();

  const double large_threshold = data.expected_result.distance + 0.01;
  auto results = RunCallback(large_threshold);
  // The large threshold allows one object in the results.
  EXPECT_EQ(results.size(), 1);

  const double small_threshold = data.expected_result.distance - 0.01;
  results = RunCallback(small_threshold);
  // The small threshold skips all objects.
  EXPECT_EQ(results.size(), 0);
}

TEST_P(SignedDistanceToPointTest, SingleQueryPointSymbolic) {
  const auto& data = GetParam();
  const GeometryId id = data.expected_result.id_G;
  double threshold = data.expected_result.distance + 0.01;

  fcl::CollisionObjectd query_point = MakeQueryPoint(data.p_WQ);

  std::unique_ptr<fcl::CollisionObjectd> geometry_object =
      MakeFclObject(*data.geometry, id, /* is_dynamic= */ true);

  vector<SignedDistanceToPoint<Expression>> sym_distances;
  unordered_map<GeometryId, RigidTransform<Expression>> X_WGs_sym{
      {id, data.X_WG.cast<Expression>()}};
  unordered_map<GeometryId, MeshDistanceBoundary> no_meshes;
  CallbackData<Expression> callback_data{
      &query_point, threshold,  data.p_WQ.cast<Expression>(),
      &X_WGs_sym,   &no_meshes, &sym_distances};
  Callback<Expression>(&query_point, geometry_object.get(), &callback_data,
                       threshold);
  // No geometries are supported yet for Expression. Currently, this call
  // succeeds, but will always return empty results.
  EXPECT_EQ(sym_distances.size(), 0);
}

// To debug a specific test, you can use Bazel flag --test_filter and
// --test_output.  For example, you can use the command:
// ```
//   bazel test //geometry/proximity:distance_to_point_callback_test
//       --test_filter=Sphere/SignedDistanceToPointTest.SingleQueryPoint/0
//       --test_output=all
// ```
// to run the first case from the test data generated by
// GenDistanceTestDataSphere() with the function
// TEST_P(SignedDistanceToPointTest, SingleQueryPoint).
// Sphere
INSTANTIATE_TEST_SUITE_P(Sphere, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistanceTestDataSphere()));

INSTANTIATE_TEST_SUITE_P(TransformSphere, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestTransformSphere()));

// Box
INSTANTIATE_TEST_SUITE_P(OutsideBox, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistanceTestDataOutsideBox()));
INSTANTIATE_TEST_SUITE_P(BoxBoundary, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistanceTestDataBoxBoundary()));
INSTANTIATE_TEST_SUITE_P(InsideBoxUnique, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestDataInsideBoxUnique()));
INSTANTIATE_TEST_SUITE_P(
    InsideBoxNonUnique, SignedDistanceToPointTest,
    testing::ValuesIn(GenDistTestDataInsideBoxNonUnique()));
INSTANTIATE_TEST_SUITE_P(TranslateBox, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistanceTestDataTranslateBox()));
INSTANTIATE_TEST_SUITE_P(TransformOutsideBox, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestTransformOutsideBox()));
INSTANTIATE_TEST_SUITE_P(TransformBoxBoundary, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestTransformBoxBoundary()));
INSTANTIATE_TEST_SUITE_P(
    TransformInsideBoxUnique, SignedDistanceToPointTest,
    testing::ValuesIn(GenDistTestTransformInsideBoxUnique()));

// Cylinder
INSTANTIATE_TEST_SUITE_P(CylinderBoundary, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestDataCylinderBoundary()));
INSTANTIATE_TEST_SUITE_P(OutsideCylinder, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestDataOutsideCylinder()));
INSTANTIATE_TEST_SUITE_P(InsideCylinder, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestDataInsideCylinder()));
INSTANTIATE_TEST_SUITE_P(CenterCylinder, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestDataCylinderCenter()));
INSTANTIATE_TEST_SUITE_P(CylinderTransform, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestDataCylinderTransform()));

// Half space
INSTANTIATE_TEST_SUITE_P(Halfspace, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestDataHalfSpace()));

}  // namespace
}  // namespace point_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
