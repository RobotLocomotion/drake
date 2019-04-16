#include "drake/geometry/proximity/distance_to_point.h"

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

using Eigen::Isometry3d;
using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RotationMatrix;
using Eigen::Translation3d;
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
                                  bool is_grad_W_well_defined = true) {
    const double sign = is_inside ? -1 : 1;
    const double expected_distance = sign * p_NQ_G.norm();
    const Vector3d p_GQ = p_GN_G + p_NQ_G;
    const Vector3d p_WQ = X_WG_ * p_GQ;

    ::testing::AssertionResult failure = ::testing::AssertionFailure();
    bool error = false;

    // We take the gradient of the signed distance query w.r.t p_WQ.
    Vector3<AutoDiffXd> p_WQ_ad = math::initializeAutoDiff(p_WQ);
    // The size of the variables we take gradient with (p_WQ) is 3.
    const int grad_size = 3;
    DistanceToPoint<AutoDiffXd> distance_to_point(
        GeometryId::get_new_id(), X_WG_.cast<AutoDiffXd>(), p_WQ_ad);

    SignedDistanceToPoint<AutoDiffXd> result = distance_to_point(shape_);
    EXPECT_EQ(result.is_grad_W_well_defined, is_grad_W_well_defined);
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
    const Vector3d grad_W_val = math::autoDiffToValueMatrix(result.grad_W);
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
        math::autoDiffToValueMatrix(p_WQ_ad),
        math::autoDiffToValueMatrix(p_WQ_ad_expected), tolerance_);
    if (!p_WQ_val_compare) {
      if (error) failure << "\n";
      error = true;
      failure << "p_WQ does not equal to p_WN + distance * grad_W:\n"
              << p_WQ_val_compare.message();
    }

    if (result.is_grad_W_well_defined) {
      auto p_WQ_derivative_compare = CompareMatrices(
          math::autoDiffToGradientMatrix(p_WQ_ad),
          math::autoDiffToGradientMatrix(p_WQ_ad_expected), tolerance_);
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

// Simple smoke test for signed distance to Sphere. It does the following:
//   Perform test of three different points w.r.t. a sphere: outside, on
//     surface, and inside.
//   Do it with AutoDiff relative to the query point's position.
//   Confirm the *values* of the reported quantity.
//   Confirm that the derivative of distance (extracted from AutoDiff) matches
//     the derivative computed by hand (grad_W).
GTEST_TEST(DistanceToPoint, Sphere) {
  const double kEps = 4 * std::numeric_limits<double>::epsilon();

  // Provide some arbitary pose of the sphere in the world.
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
                          true /* is inside */, false /* ill defined */));
}

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
                                  false /* not well defined */));
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
                                false /* not well defined */));

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

  // Provide some arbitary pose of the cylinder in the world.
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

// TODO(SeanCurtis-TRI): Refactor these tests to reduce duplication.

// This test simply confirms which scalar-shape combinations produce answers
// and which don't. That culling takes place at the Callback level so that is
// what is exercised here. Lack of support is signaled by no distance data
// being returned.
GTEST_TEST(DistanceToPoint, ScalarShapeSupportDouble) {
  // Configure the basic query.
  Vector3d p_WQ{10, 10, 10};
  Isometry3d X_WQ{Translation3d{p_WQ}};
  auto point_geometry = make_shared<fcl::Sphered>(0);
  fcl::CollisionObjectd query_point(point_geometry);
  query_point.setTranslation(p_WQ);
  EncodedData encoding(GeometryIndex(0), true);
  encoding.write_to(&query_point);
  std::vector<SignedDistanceToPoint<double>> distances;
  std::vector<GeometryId> geometry_map{GeometryId::get_new_id(),
                                       GeometryId::get_new_id()};
  double threshold = std::numeric_limits<double>::max();
  std::vector<Isometry3d> X_WGs{X_WQ, Isometry3d::Identity()};
  CallbackData<double> data{&query_point, &geometry_map, threshold,
                            p_WQ,         &X_WGs,        &distances};


  // The Drake-supported geometries (minus Mesh which isn't supported by
  // ProximityEngine yet).

  auto run_callback = [&query_point, &threshold, &distances, &data](
                          auto geometry_shared_ptr) {
    threshold = std::numeric_limits<double>::max();
    distances.clear();
    fcl::CollisionObjectd object(geometry_shared_ptr);
    EncodedData object_encoding(GeometryIndex(1), true);
    object_encoding.write_to(&object);
    Callback<double>(&query_point, &object, &data, threshold);
  };

  // Sphere
  {
    run_callback(make_shared<fcl::Sphered>(1.0));
    EXPECT_EQ(distances.size(), 1);
  }

  // Cylinder
  {
    run_callback(make_shared<fcl::Cylinderd>(1.0, 2.0));
    EXPECT_EQ(distances.size(), 1);
  }

  // HalfSpace
  {
    run_callback(make_shared<fcl::Halfspaced>(0, 0, 1, 0));
    EXPECT_EQ(distances.size(), 0);  // Query not supported.
  }

  // Box
  {
    run_callback(make_shared<fcl::Boxd>(1.0, 2.0, 3.5));
    EXPECT_EQ(distances.size(), 1);
  }

  // Convex
  // TODO(SeanCurtis-TRI): Add convex that is *not* supported; create a small
  // utility test to generate a tetrahedron.
}

// The autodiff version of DistanceToPoint.ScalarShapeSupportDouble.
GTEST_TEST(DistanceToPoint, ScalarShapeSupportAutoDiff) {
  // Configure the basic query.
  Vector3<AutoDiffXd> p_WQ{10, 10, 10};
  Isometry3<AutoDiffXd> X_WQ{Translation3<AutoDiffXd>{p_WQ}};
  auto point_geometry = make_shared<fcl::Sphered>(0);
  fcl::CollisionObjectd query_point(point_geometry);
  query_point.setTranslation(convert_to_double(p_WQ));
  EncodedData encoding(GeometryIndex(0), true);
  encoding.write_to(&query_point);
  std::vector<SignedDistanceToPoint<AutoDiffXd>> distances;
std::vector<GeometryId> geometry_map{GeometryId::get_new_id(),
                                     GeometryId::get_new_id()};
  double threshold = std::numeric_limits<double>::max();
  std::vector<Isometry3<AutoDiffXd>> X_WGs{X_WQ,
                                           Isometry3<AutoDiffXd>::Identity()};
  CallbackData<AutoDiffXd> data{&query_point, &geometry_map, threshold,
                                p_WQ,         &X_WGs,        &distances};

  // The Drake-supported geometries (minus Mesh which isn't supported by
  // ProximityEngine yet).

  auto run_callback = [&query_point, &threshold, &distances,
                       &data](auto geometry_shared_ptr) {
    threshold = std::numeric_limits<double>::max();
    distances.clear();
    fcl::CollisionObjectd object(geometry_shared_ptr);
    EncodedData object_encoding(GeometryIndex(1), true);
    object_encoding.write_to(&object);
    Callback<AutoDiffXd>(&query_point, &object, &data, threshold);
  };

  // Sphere
  {
    run_callback(make_shared<fcl::Sphered>(1.0));
    EXPECT_EQ(distances.size(), 1);
  }

  // Cylinder
  {
    run_callback(make_shared<fcl::Cylinderd>(1.0, 2.0));
    EXPECT_EQ(distances.size(), 0);  // Query not supported.
  }

  // HalfSpace
  {
    run_callback(make_shared<fcl::Halfspaced>(0, 0, 1, 0));
    EXPECT_EQ(distances.size(), 0);  // Query not supported.
  }

  // Box
  {
    run_callback(make_shared<fcl::Boxd>(1.0, 2.0, 3.5));
    EXPECT_EQ(distances.size(), 1);
  }

  // Convex
  // TODO(SeanCurtis-TRI): Add convex that is *not* supported; create a small
  // utility test to generate a tetrahedron.
}

}  // namespace
}  // namespace point_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
