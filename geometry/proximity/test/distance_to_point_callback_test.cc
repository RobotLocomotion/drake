#include "drake/geometry/proximity/distance_to_point_callback.h"

#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_eigen.h"
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

using Eigen::Matrix3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrix;
using math::RotationMatrixd;
using std::make_shared;
using std::shared_ptr;

// Test the evaluation of signed distance to Ellipsoid: medial-axis special
// case. A query point at the geometric center of the ellipsoid is equidistant
// from multiple surface points, so the nearest point is only determined up to
// sign and cannot be expressed as an exact expected value. This case is
// therefore kept as a standalone test rather than folded into
// SignedDistanceToPointTest. The 9 surface-sample cases (outside, on boundary,
// inside at several orientations) are covered by
// Ellipsoid/SignedDistanceToPointTest.
GTEST_TEST(DistanceToPoint, EllipsoidMedialAxis) {
  const RotationMatrix<double> R_WG(
      AngleAxis<double>(M_PI / 5, Vector3d{1, 2, 3}.normalized()));
  const Vector3d p_WG{0.5, 1.25, -2};
  const RigidTransform<double> X_WG(R_WG, p_WG);
  const fcl::Ellipsoidd ellipsoid(1.5, 0.75, 1.25);

  // The query point lies on the medial axis. We don't know which of the two
  // antipodal surface points FCL will choose, but we can confirm:
  //   - |distance| equals the shortest semi-axis length (0.75).
  //   - The nearest point lies on the ±y axis.
  //   - The gradient points along ±y in G.
  constexpr double kDistTolerance = 5e-5;
  constexpr double kVectorTolerance = 5e-4;
  const double min_radius = ellipsoid.radii.y();
  const Vector3d min_axis_G(0, 1, 0);
  const GeometryId id = GeometryId::get_new_id();
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

// For general meshes, distance to point is computed using
// CalcSignedDistanceToSurfaceMesh(). That code is already tested in its own
// unit tests. This test focuses on the callback's particular responsibilities;
// 1. If the VolumeMeshBoundary doesn't have feature normals, it should throw
//    with whatever message is there.
// 2. Correctly invokes CalcSignedDistanceToSurfaceMesh() and packages the
//    results correctly (copies most values, but re-expresses gradient).
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

// Helper function to indicate expectation on whether I get a distance result
// based on scalar type T and fcl shape S.
template <typename T, typename S>
int ExpectedResultCount() {
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

template <typename T>
void TestScalarShapeSupport() {
  // Configure the basic query.
  Vector3<T> p_WQ{10, 10, 10};
  RigidTransform<T> X_WQ{Eigen::Translation<T, 3>{p_WQ}};
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
    EXPECT_EQ(distances.size(), (ExpectedResultCount<T, fcl::Boxd>()));
  }

  // Capsule
  {
    run_callback(make_shared<fcl::Capsuled>(1.0, 2.0));
    EXPECT_EQ(distances.size(), (ExpectedResultCount<T, fcl::Capsuled>()));
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
    EXPECT_EQ(distances.size(), (ExpectedResultCount<T, fcl::Convexd>()));
  }

  // Cylinder
  {
    run_callback(make_shared<fcl::Cylinderd>(1.0, 2.0));
    EXPECT_EQ(distances.size(), (ExpectedResultCount<T, fcl::Cylinderd>()));
  }

  // Ellipsoid
  {
    run_callback(make_shared<fcl::Ellipsoidd>(1.5, 0.7, 3));
    EXPECT_EQ(distances.size(), (ExpectedResultCount<T, fcl::Ellipsoidd>()));
  }

  // HalfSpace
  {
    run_callback(make_shared<fcl::Halfspaced>(Vector3d::UnitZ(), 0));
    EXPECT_EQ(distances.size(), (ExpectedResultCount<T, fcl::Halfspaced>()));
  }

  // Sphere
  {
    run_callback(make_shared<fcl::Sphered>(1.0));
    EXPECT_EQ(distances.size(), (ExpectedResultCount<T, fcl::Sphered>()));
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
  fcl::CollisionObjectd query_point(make_shared<fcl::Sphered>(0),
                                    RotationMatrixd().matrix(), p_WQ);

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

// ============================================================================
// Unified signed-distance-to-point tests.
//
// A test case comprises a shape, a range of query points (sampling relevant
// geometric regions -- inside, outside, and on boundary), and hand-constructed
// expected results. They get evaluated in both both axis-aligned and arbitrary
// poses.
//
// Each test case is evaluated twice:
//
//   Double pass (always):
//     - Calls Callback<double>.
//     - Checks the exact values of returned values against hand-constructed
//       ground truth.
//
//   AutoDiff pass (when `supports_autodiff` is true):
//     - Calls Callback<AutoDiffXd> with InitializeAutoDiff(p_WQ).
//     - Verifies grad_W == ∂distance/∂p_WQ; the analytic formula agrees with
//       the autodiff-computed derivative.
//     - Verifies grad_W is a unit vector with zero derivative w.r.t. p_WQ.
//     - Verifies the invariant p_WQ = p_WN + distance⋅grad_W in both value
//       and (when is_grad_W_unique) derivatives.
//
// `supports_autodiff`: Not all shapes (e.g., Cylinder) support AutoDiff.
//  `supports_autodiff` will be false for incompatible shapes.
//
// `is_grad_W_unique`: the query point is at a degenerate location where
//   the gradient direction is arbitrary (sphere center, box edge/vertex); the
//   AutoDiff pass still runs but skips the full Jacobian-of-invariant check.
// ============================================================================

// Holds one parameterized test case for SignedDistanceToPointTest.
//
// Each instance carries:
//   - A collision object against which the query is performed.
//   - Query inputs X_WG and p_WQ.
//   - Expected output (GeometryId, p_GN, distance, grad_W).
//   - supports_autodiff: false for shapes where DistanceToPoint<AutoDiffXd> is
//     not implemented (e.g. Cylinder).
//   - is_grad_W_unique: false at degenerate locations (sphere center, box
//     edge/vertex) where the gradient direction is algorithm-dependent, so the
//     AutoDiff derivative-of-invariant check is skipped.
struct SignedDistanceToPointTestData {
  // Builds a test record. A fresh GeometryId is minted and embedded in
  // expected_result; the same id is passed to query_fn and autodiff_query_fn
  // at test time.
  static SignedDistanceToPointTestData Make(
      shared_ptr<fcl::CollisionObjectd> object, const RigidTransformd& X_WG,
      const Vector3d& p_WQ, const Vector3d& p_GN, double distance,
      const Vector3d& grad_W, bool supports_autodiff = true,
      bool is_grad_W_unique = true, double tolerance = 1e-14) {
    EncodedData encoded(*object);
    return {.collision_object = std::move(object),
            .X_WG = X_WG,
            .p_WQ = p_WQ,
            .expected_result = {encoded.id(), p_GN, distance, grad_W},
            .supports_autodiff = supports_autodiff,
            .is_grad_W_unique = is_grad_W_unique,
            .tolerance = tolerance};
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  const SignedDistanceToPointTestData& obj) {
    os << fmt::format("{{ p_WQ: {}, expected distance: {} }}",
                      fmt_eigen(obj.p_WQ.transpose()),
                      obj.expected_result.distance);
    return os;
  }

  shared_ptr<fcl::CollisionObjectd> collision_object;
  RigidTransformd X_WG;
  Vector3d p_WQ;
  SignedDistanceToPoint<double> expected_result;
  bool supports_autodiff{true};
  bool is_grad_W_unique{true};
  // Tolerance for value checks in SingleQueryPoint. Defaults to 1e-14 (exact
  // analytical shapes). Set higher for GJK/EPA-backed shapes (e.g. Ellipsoid).
  double tolerance{1e-14};
};

// Helper to convert a Drake Shape to an equivalent FCL collision object.
// Supports the shape types used in the test data generators.
shared_ptr<fcl::CollisionObjectd> MakeFclCollision(const Shape& shape) {
  shared_ptr<fcl::ShapeBase<double>> shape_base;
  if (const auto* box = dynamic_cast<const Box*>(&shape)) {
    shape_base = make_shared<fcl::Boxd>(box->size());
  } else if (const auto* capsule = dynamic_cast<const Capsule*>(&shape)) {
    shape_base =
        make_shared<fcl::Capsuled>(capsule->radius(), capsule->length());
  } else if (const auto* cylinder = dynamic_cast<const Cylinder*>(&shape)) {
    shape_base =
        make_shared<fcl::Cylinderd>(cylinder->radius(), cylinder->length());
  } else if (const auto* ellipsoid = dynamic_cast<const Ellipsoid*>(&shape)) {
    const Vector3d radii(ellipsoid->a(), ellipsoid->b(), ellipsoid->c());
    shape_base = make_shared<fcl::Ellipsoidd>(radii);
  } else if (dynamic_cast<const HalfSpace*>(&shape) != nullptr) {
    shape_base = make_shared<fcl::Halfspaced>(0, 0, 1, 0);
  } else if (const auto* sphere = dynamic_cast<const Sphere*>(&shape)) {
    shape_base = make_shared<fcl::Sphered>(sphere->radius());
  } else {
    throw std::logic_error(fmt::format(
        "Unsupported shape type ({}) in MakeFclGeometry", shape.type_name()));
  }

  shared_ptr<fcl::CollisionGeometryd> geometry(std::move(shape_base));
  auto collision_object = make_shared<fcl::CollisionObjectd>(geometry);
  const GeometryId geometry_id = GeometryId::get_new_id();
  EncodedData(geometry_id, true).write_to(collision_object.get());
  return collision_object;
}

// ---------------------------------------------------------------------------
// Sphere generators

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
  shared_ptr<fcl::CollisionObjectd> sphere = MakeFclCollision(Sphere(0.7));
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
      // center of the sphere. The AutoDiffXd is skipped in this case.
      SignedDistanceToPointTestData::Make(
          sphere, X_WG, X_WG * Vector3d{0, 0, 0}, Vector3d(0.7, 0, 0), -0.7,
          X_WG.rotation() * Vector3d(1, 0, 0), true /* supports_autodiff */,
          false /* is_grad_W_unique */)};
  return test_data;
}

std::vector<SignedDistanceToPointTestData> GenDistTestTransformSphere() {
  const RigidTransformd X_WG(RollPitchYawd(M_PI / 6, M_PI / 3, M_PI_2),
                             Vector3d{10, 11, 12});
  return GenDistanceTestDataSphere(X_WG);
}

// ---------------------------------------------------------------------------
// Box generators

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

  // Must match the box dimensions in GenDistanceTestDataBoxBoundary.
  shared_ptr<fcl::CollisionObjectd> box = MakeFclCollision(Box(20, 30, 10));
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const auto& data : test_data_box_boundary) {
    const Vector3d p_WQ = data.p_WQ + data.expected_result.grad_W;
    // Propagate is_grad_W_unique from the source boundary point: the derivative
    // of the nearest-point function remains undefined when Q is directly
    // outside a box edge or vertex direction.
    test_data.emplace_back(SignedDistanceToPointTestData::Make(
        box, X_WG, p_WQ, data.expected_result.p_GN, 1.0,
        data.expected_result.grad_W, true /* supports_autodiff */,
        data.is_grad_W_unique));
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
  const Vector3d size(20, 30, 10);
  shared_ptr<fcl::CollisionObjectd> box = MakeFclCollision(Box(size));
  const Vector3d h_G = size / 2;
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
        // Q is its own nearest point on ∂G.
        const Vector3d& p_GN = p_GQ;
        const Vector3d grad_W = X_WG.rotation() * s_G.normalized();
        // Q is on a box edge (2 non-zero coords) or vertex (3 non-zero): the
        // gradient direction is algorithm-dependent, so skip the derivative
        // check in the AutoDiff pass.
        const int num_nonzero =
            (sx != 0 ? 1 : 0) + (sy != 0 ? 1 : 0) + (sz != 0 ? 1 : 0);
        const bool is_grad_W_unique = (num_nonzero == 1);
        test_data.emplace_back(SignedDistanceToPointTestData::Make(
            box, X_WG, p_WQ, p_GN, distance, grad_W,
            true /* supports_autodiff */, is_grad_W_unique));
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
  const Vector3d size(20, 30, 10);
  shared_ptr<fcl::CollisionObjectd> box = MakeFclCollision(Box(size));
  const Vector3d h_G = size / 2;
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
      // The nearest point is at the face center.
      const Vector3d& p_GN = p_GC;
      const Vector3d grad_W = X_WG.rotation() * s_G;
      test_data.emplace_back(SignedDistanceToPointTestData::Make(
          box, X_WG, p_WQ, p_GN, -d, grad_W));
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
  shared_ptr<fcl::CollisionObjectd> box = MakeFclCollision(Box(20, 10, 10));
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

// Translation test: box translated away from origin, Q outside.
std::vector<SignedDistanceToPointTestData> GenDistanceTestDataTranslateBox() {
  // We set up a 20x10x30 box G centered at the origin [-10,10]x[-5,5]x[-15,15].
  shared_ptr<fcl::CollisionObjectd> box = MakeFclCollision(Box(20, 10, 30));
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

// ---------------------------------------------------------------------------
// Cylinder generators
//
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
  const double radius = 3.0;
  const double length = 5.0;
  shared_ptr<fcl::CollisionObjectd> cylinder =
      MakeFclCollision(Cylinder(radius, length));
  const double half_length = length / 2;
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
      // xy_vector and z_vector are unit vectors in x-y plane and z-axis of G.
      const Vector3d p_GQ = radius * xy_vector + half_length * z_vector;
      const Vector3d p_WQ = X_WG * p_GQ;
      // Q is its own nearest point on ∂G.
      const Vector3d& p_GN = p_GQ;
      // We set the gradient vector according to the convention described in
      // QueryObject::ComputeSignedDistanceToPoint().  Mathematically it is
      // undefined.
      const Vector3d grad_W = R_WG * (xy_vector + z_vector).normalized();
      // Cylinder does not support AutoDiffXd.
      test_data.emplace_back(SignedDistanceToPointTestData::Make(
          cylinder, X_WG, p_WQ, p_GN, distance, grad_W,
          false /* supports_autodiff */));
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
  const double radius = 3.0;
  const double length = 5.0;
  shared_ptr<fcl::CollisionObjectd> cylinder =
      MakeFclCollision(Cylinder(radius, length));
  const double half_length = length / 2;
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
  // Generate LocalData that will convert to SignedDistanceToPointTestData
  // later.
  struct LocalData {
    Vector3d p_GQ;
    Vector3d grad_G;
  };
  std::vector<LocalData> barrel_data, cap_data, cap_center_data;
  for (const auto& z_vector : {Vector3d(0, 0, 1), Vector3d(0, 0, -1)}) {
    // Q at the centers of the top and bottom caps.
    cap_center_data.push_back(
        {half_length * z_vector, z_vector});  // cap center
    for (const auto& xy_vector : all_xy_vectors) {
      // Barrel: at 0.5 * half_length along z.
      barrel_data.push_back(
          {radius * xy_vector + half_length * 0.5 * z_vector, xy_vector});
      // Cap: at 0.6 * radius radially.
      cap_data.push_back(
          {radius * 0.6 * xy_vector + half_length * z_vector, z_vector});
    }
  }
  std::vector<SignedDistanceToPointTestData> test_data;
  // Convert LocalData to SignedDistanceToPointTestData and add to test_data.
  // Note: Cylinder does not support AutoDiffXd.
  auto convert = [&](const LocalData& local) {
    const Vector3d p_WQ = X_WG * local.p_GQ;
    const Vector3d grad_W = R_WG * local.grad_G;
    test_data.push_back(SignedDistanceToPointTestData::Make(
        cylinder, X_WG, p_WQ, local.p_GQ, 0.0, grad_W,
        false /* supports_autodiff */));
  };
  std::for_each(barrel_data.begin(), barrel_data.end(), convert);
  std::for_each(cap_data.begin(), cap_data.end(), convert);
  std::for_each(cap_center_data.begin(), cap_center_data.end(), convert);
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
    test_data.push_back(data);
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
  shared_ptr<fcl::CollisionObjectd> cylinder = MakeFclCollision(Cylinder(3, 5));
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const auto& data : GenDistTestDataCylinderBoundary(X_WG)) {
    const Vector3d p_WQ = data.p_WQ + data.expected_result.grad_W;
    // Cylinder does not support AutoDiffXd.
    test_data.emplace_back(SignedDistanceToPointTestData::Make(
        cylinder, X_WG, p_WQ, data.expected_result.p_GN, 1.0,
        data.expected_result.grad_W, false /* supports_autodiff */));
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
  shared_ptr<fcl::CollisionObjectd> cylinder = MakeFclCollision(Cylinder(3, 5));
  const double kNegDist = -0.1;
  std::vector<SignedDistanceToPointTestData> test_data;
  for (const auto& data : GenDistTestDataCylinderBoundarySurface(X_WG)) {
    const Vector3d p_WQ = data.p_WQ + kNegDist * data.expected_result.grad_W;
    // Cylinder does not support AutoDiffXd.
    test_data.emplace_back(SignedDistanceToPointTestData::Make(
        cylinder, X_WG, p_WQ, data.expected_result.p_GN, kNegDist,
        data.expected_result.grad_W, false /* supports_autodiff */));
  }
  return test_data;
}

// Generates test data for a query point Q at the center of a long cylinder
// geometry G. Q's nearest point on ∂G is not unique. Our code picks the one
// on the x's axis of G's frame.
std::vector<SignedDistanceToPointTestData> GenDistTestDataCylinderCenter(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const double radius = 1.0;
  shared_ptr<fcl::CollisionObjectd> cylinder =
      MakeFclCollision(Cylinder(radius, 20));
  const Vector3d p_WQ = X_WG * Vector3d(0, 0, 0);
  // The nearest point N is on the x's axis of G's frame by convention.
  const Vector3d p_GN = radius * Vector3d(1, 0, 0);
  const Vector3d grad_W = X_WG.rotation() * Vector3d(1, 0, 0);
  std::vector<SignedDistanceToPointTestData> test_data;
  // Cylinder does not support AutoDiffXd.
  test_data.emplace_back(SignedDistanceToPointTestData::Make(
      cylinder, X_WG, p_WQ, p_GN, -radius, grad_W,
      false /* supports_autodiff */));
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

// ---------------------------------------------------------------------------
// HalfSpace generators

// Generate test data for a query point Q relative to a half space.
std::vector<SignedDistanceToPointTestData> GenDistTestDataHalfSpace() {
  std::vector<SignedDistanceToPointTestData> test_data;
  const RigidTransformd X_WG1(RollPitchYawd(M_PI / 3, M_PI / 6, M_PI_2),
                              Vector3d{10, 11, 12});
  shared_ptr<fcl::CollisionObjectd> half_space = MakeFclCollision(HalfSpace());

  for (const auto& X_WG : {RigidTransformd(), X_WG1}) {
    const Vector3d grad_W = X_WG.rotation().col(2);
    const Vector3d p_GN(1.25, 1.5, 0);
    const Vector3d p_WN = X_WG * p_GN;

    // Outside the half space.
    const double distance = 1.5;
    const Vector3d p_WQ1 = p_WN + distance * grad_W;
    test_data.emplace_back(SignedDistanceToPointTestData::Make(
        half_space, X_WG, p_WQ1, p_GN, distance, grad_W));

    // On the half space boundary.
    const Vector3d p_WQ2 = p_WN;
    test_data.emplace_back(SignedDistanceToPointTestData::Make(
        half_space, X_WG, p_WQ2, p_GN, 0.0, grad_W));

    // Inside the half space.
    const Vector3d p_WQ3 = p_WN - distance * grad_W;
    test_data.emplace_back(SignedDistanceToPointTestData::Make(
        half_space, X_WG, p_WQ3, p_GN, -distance, grad_W));
  }
  return test_data;
}

// ---------------------------------------------------------------------------
// Ellipsoid generators
//
// Point-to-ellipsoid uses GJK/EPA internally, so tolerances are significantly
// looser than for analytically-computed shapes. AutoDiffXd is not supported.
// The medial-axis degenerate case is covered by GTEST_TEST(DistanceToPoint,
// EllipsoidMedialAxis) because the nearest point is only determined up to sign.
std::vector<SignedDistanceToPointTestData> GenDistTestDataEllipsoid() {
  const Vector3d radii(1.5, 0.75, 1.25);
  shared_ptr<fcl::CollisionObjectd> ellipsoid =
      MakeFclCollision(Ellipsoid(radii));

  // Matches the pose used in GTEST_TEST(DistanceToPoint, EllipsoidMedialAxis).
  const RotationMatrixd R_WG(
      AngleAxis<double>(M_PI / 5, Vector3d{1, 2, 3}.normalized()));
  const RigidTransformd X_WG(R_WG, Vector3d{0.5, 1.25, -2});

  // Compute a surface point and outward unit normal from parametric angles.
  // Parametric surface:  E = [a cos(θ)sin(ϕ),  b sin(θ)sin(ϕ),  c cos(ϕ)].
  // Outward normal: normalize ∇(x²/a² + y²/b² + z²/c²) = (2x/a², 2y/b², 2z/c²).
  const double a = radii.x();
  const double b = radii.y();
  const double c = radii.z();
  auto get_sample = [&](double theta, double phi) {
    const double x = a * std::cos(theta) * std::sin(phi);
    const double y = b * std::sin(theta) * std::sin(phi);
    const double z = c * std::cos(phi);
    return std::make_pair(
        Vector3d{x, y, z},
        Vector3d{x / a / a, y / b / b, z / c / c}.normalized());
  };

  // GJK/EPA accuracy for this near-spherical ellipsoid; covers both distance
  // and vector fields. See the note in GTEST_TEST(DistanceToPoint, Ellipsoid)
  // about why the ellipsoid is kept close to a sphere.
  constexpr double kEllipsoidTol = 5e-4;

  struct EllipseCoord {
    double theta{};
    double phi{};
  };
  const std::vector<EllipseCoord> coords{
      {0, 0},                        // Bottom pole.
      {7 * M_PI / 5, M_PI / 6},      // Lower half.
      {3 * M_PI / 7, 4 * M_PI / 5},  // Upper half.
  };

  std::vector<SignedDistanceToPointTestData> test_data;
  for (const auto& coord : coords) {
    const auto& [p_GN, n_G] = get_sample(coord.theta, coord.phi);
    const Vector3d grad_W = X_WG.rotation() * n_G;
    for (const double distance : {-0.125, 0.0, 0.2}) {
      const Vector3d p_WQ = X_WG * (p_GN + n_G * distance);
      test_data.emplace_back(SignedDistanceToPointTestData::Make(
          ellipsoid, X_WG, p_WQ, p_GN, distance, grad_W,
          false /* supports_autodiff */, true /* is_grad_W_unique */,
          kEllipsoidTol));
    }
  }
  return test_data;
}

// ---------------------------------------------------------------------------
// Capsule generators

// Q outside, on the surface, and inside the capsule, in three representative
// regions: toward the top end cap, toward the barrel, and toward the bottom
// end cap.
std::vector<SignedDistanceToPointTestData> GenDistTestDataCapsule(
    const RigidTransformd& X_WG = RigidTransformd::Identity()) {
  const double radius = 0.7;
  const double length = 1.3;
  shared_ptr<fcl::CollisionObjectd> capsule =
      MakeFclCollision(Capsule(radius, length));
  const double half_lz = length / 2;
  // Three unit outward directions: upward (top cap), lateral (barrel),
  // downward (bottom cap).
  const Vector3d vhats[3] = {Vector3d{2, -3, 6}.normalized(),
                             Vector3d{2, -3, 0}.normalized(),
                             Vector3d{2, -3, -6}.normalized()};
  // Nearest surface points for each direction/region.
  const Vector3d p_GNs[3] = {radius * vhats[0] + Vector3d{0, 0, half_lz},
                             radius * vhats[1] + Vector3d{0, 0, half_lz / 2},
                             radius * vhats[2] + Vector3d{0, 0, -half_lz}};
  std::vector<SignedDistanceToPointTestData> test_data;
  for (int i = 0; i < 3; ++i) {
    const Vector3d& vhat = vhats[i];
    const Vector3d& p_GN = p_GNs[i];
    const Vector3d grad_W = X_WG.rotation() * vhat;
    // Outside.
    test_data.emplace_back(SignedDistanceToPointTestData::Make(
        capsule, X_WG, X_WG * (p_GN + 1.5 * vhat), p_GN, 1.5, grad_W));
    // On surface.
    test_data.emplace_back(SignedDistanceToPointTestData::Make(
        capsule, X_WG, X_WG * p_GN, p_GN, 0.0, grad_W));
    // Inside (halfway to surface from p_GN inward).
    test_data.emplace_back(SignedDistanceToPointTestData::Make(
        capsule, X_WG, X_WG * (p_GN - 0.5 * radius * vhat), p_GN, -0.5 * radius,
        grad_W));
  }
  return test_data;
}

std::vector<SignedDistanceToPointTestData> GenDistTestTransformCapsule() {
  return GenDistTestDataCapsule(RigidTransformd(
      RollPitchYawd(M_PI / 6, M_PI / 3, M_PI_2), Vector3d{10, 11, 12}));
}

// ---------------------------------------------------------------------------
// Test fixture and test body

// Calls point_distance::Callback() with the given shape and test data.
template <typename T>
std::vector<SignedDistanceToPoint<T>> CallCallback(
    const SignedDistanceToPointTestData& data, const Vector3<T>& p_WQ) {
  // Build the query point FCL object (a zero-radius sphere).
  fcl::CollisionObjectd query_point(make_shared<fcl::Sphered>(0.0));
  query_point.setTranslation(ExtractDoubleOrThrow(p_WQ));

  const GeometryId geometry_id = data.expected_result.id_G;
  const std::unordered_map<GeometryId, RigidTransform<T>> X_WGs{
      {geometry_id, data.X_WG.cast<T>()}};
  std::vector<SignedDistanceToPoint<T>> results;
  const std::unordered_map<GeometryId, MeshDistanceBoundary> mesh_boundaries;
  CallbackData<T> callback_data{&query_point, 1e8 /*data.tolerance*/, p_WQ,
                                &X_WGs,       &mesh_boundaries,       &results};
  double threshold_out = 1e8;  // data.tolerance;
  Callback<T>(&query_point, data.collision_object.get(), &callback_data,
              threshold_out);
  return results;
}

// Fixture for SignedDistanceToPointTest.
//
// SingleQueryPoint runs a double pass (always) and an AutoDiff pass (when
// supports_autodiff is true). See the section comment above for the full
// description of what each pass verifies.
struct SignedDistanceToPointTest
    : public testing::TestWithParam<SignedDistanceToPointTestData> {
  // Tolerance for AutoDiff derivative checks: should be near machine precision.
  static constexpr double kAutoDiffTol =
      8 * std::numeric_limits<double>::epsilon();
};

TEST_P(SignedDistanceToPointTest, SingleQueryPoint) {
  const auto& data = GetParam();

  // --- Double pass: verify output values ---
  const SignedDistanceToPoint<double> result_d =
      CallCallback<double>(data, data.p_WQ).front();

  EXPECT_EQ(result_d.id_G, data.expected_result.id_G);
  EXPECT_TRUE(CompareMatrices(result_d.p_GN, data.expected_result.p_GN,
                              data.tolerance));
  EXPECT_NEAR(result_d.distance, data.expected_result.distance, data.tolerance);
  EXPECT_TRUE(CompareMatrices(result_d.grad_W, data.expected_result.grad_W,
                              data.tolerance));

  if (!data.supports_autodiff) return;

  // --- AutoDiff pass: gradient consistency ---
  // Initialize p_WQ as the three independent variables.
  const Vector3<AutoDiffXd> p_WQ_ad = math::InitializeAutoDiff(data.p_WQ);
  const SignedDistanceToPoint<AutoDiffXd> result_ad =
      CallCallback<AutoDiffXd>(data, p_WQ_ad).front();

  // Distance value matches.
  EXPECT_NEAR(result_ad.distance.value(), data.expected_result.distance,
              data.tolerance)
      << "AutoDiff: incorrect signed distance value.";

  // The analytic grad_W equals ∂distance/∂p_WQ.  The tolerance here matches
  // the double pass: the distance algorithm involves rotations and
  // normalizations that introduce O(eps * condition_number) round-off, so
  // near-machine-precision tolerance is too tight under an arbitrary pose.
  DRAKE_DEMAND(result_ad.distance.derivatives().size() == 3);
  const Vector3d ddistance_dp_WQ = result_ad.distance.derivatives();
  const Vector3d grad_W_val = math::ExtractValue(result_ad.grad_W);
  EXPECT_FALSE(grad_W_val.array().isNaN().any())
      << "AutoDiff: grad_W contains NaN.";
  EXPECT_TRUE(CompareMatrices(ddistance_dp_WQ, grad_W_val, data.tolerance))
      << "AutoDiff: grad_W != ∂distance/∂p_WQ.";

  // grad_W is a unit vector with zero derivative w.r.t. p_WQ.
  const AutoDiffXd grad_W_sq_norm = result_ad.grad_W.dot(result_ad.grad_W);
  EXPECT_NEAR(grad_W_sq_norm.value(), 1.0, kAutoDiffTol);
  if (grad_W_sq_norm.derivatives().size() > 0) {
    EXPECT_TRUE(CompareMatrices(grad_W_sq_norm.derivatives(),
                                Eigen::VectorXd::Zero(3), 1.4 * kAutoDiffTol));
  }

  // The invariant p_WQ = p_WN + distance * grad_W holds in values.
  const Vector3<AutoDiffXd> p_WN_ad =
      data.X_WG.cast<AutoDiffXd>() * result_ad.p_GN;
  const Vector3<AutoDiffXd> p_WQ_ad_expected =
      p_WN_ad + result_ad.distance * result_ad.grad_W;
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(p_WQ_ad),
                              math::ExtractValue(p_WQ_ad_expected),
                              data.tolerance))
      << "AutoDiff: p_WQ != p_WN + distance * grad_W (values).";

  // The invariant also holds in derivatives when the gradient is unique, i.e.,
  // not at degenerate locations such as a sphere center or box edge/vertex.
  if (data.is_grad_W_unique) {
    EXPECT_TRUE(CompareMatrices(math::ExtractGradient(p_WQ_ad),
                                math::ExtractGradient(p_WQ_ad_expected),
                                kAutoDiffTol))
        << "AutoDiff: p_WQ != p_WN + distance * grad_W (derivatives).";
  }
}

// Sphere
INSTANTIATE_TEST_SUITE_P(Sphere, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistanceTestDataSphere()));
INSTANTIATE_TEST_SUITE_P(TransformSphere, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestTransformSphere()));

// Ellipsoid (GJK/EPA-backed; AutoDiffXd not supported; looser tolerance)
INSTANTIATE_TEST_SUITE_P(Ellipsoid, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestDataEllipsoid()));

// Capsule
INSTANTIATE_TEST_SUITE_P(Capsule, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestDataCapsule()));
INSTANTIATE_TEST_SUITE_P(TransformCapsule, SignedDistanceToPointTest,
                         testing::ValuesIn(GenDistTestTransformCapsule()));

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
