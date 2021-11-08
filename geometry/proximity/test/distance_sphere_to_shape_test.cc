#include <cmath>
#include <limits>
#include <regex>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/geometry/proximity/distance_to_shape_callback.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/query_results/signed_distance_to_point.h"
#include "drake/geometry/utilities.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"

// TODO(SeanCurtis-TRI): Figure out the right decomposition of code for
// primitive tests.

/* @file
 This tests only the code in distance_to_shape_callback.h that supports
 (sphere-shape) signed distance queries. Ultimately, we'll have unit tests for
 all shapeA-shapeB primitive functions.  */

namespace drake {
namespace geometry {
namespace internal {
namespace shape_distance {

namespace {

using Eigen::Vector3d;
using fcl::Boxd;
using fcl::Capsuled;
using fcl::CollisionObjectd;
using fcl::Convexd;
using fcl::Cylinderd;
using fcl::Ellipsoidd;
using fcl::Halfspaced;
using fcl::Sphered;
using math::RigidTransform;
using math::RigidTransformd;
using math::RotationMatrix;
using math::RotationMatrixd;
using std::make_shared;

constexpr double kInf = std::numeric_limits<double>::infinity();

// TODO(SeanCurtis-TRI): Troll through proximity_engine_test and pull/remove all
// tests there that more rightly belong here.

// This tests the expected fallback support based on scalar type (i.e., we have
// a fallback for double, but not for AutoDiff). We merely confirm the throw/no
// throw nature of the test. Other tests evaluate the correctness of the
// numerical values.
GTEST_TEST(SphereShapeDistance, FallbackSupport) {
  // NOTE: The shape type or pose are unimportant to this test.
  auto sphere = make_shared<Sphered>(0);
  CollisionObjectd obj_a(sphere);
  CollisionObjectd obj_b(sphere);
  fcl::DistanceRequestd request{};
  const GeometryId id_a = GeometryId::get_new_id();
  const GeometryId id_b = GeometryId::get_new_id();
  EncodedData(id_a, true).write_to(&obj_a);
  EncodedData(id_b, true).write_to(&obj_b);

  SignedDistancePair<double> distance_pair_d{};
  DRAKE_EXPECT_NO_THROW(
      CalcDistanceFallback<double>(obj_a, obj_b, request, &distance_pair_d));
  EXPECT_TRUE(distance_pair_d.id_A.is_valid());
  EXPECT_TRUE(distance_pair_d.id_B.is_valid());
  ASSERT_LT(id_a, id_b);  // Confirm assumption that the next two tests require.
  EXPECT_EQ(distance_pair_d.id_A, id_a);
  EXPECT_EQ(distance_pair_d.id_B, id_b);

  SignedDistancePair<AutoDiffXd> distance_pair_ad{};
  DRAKE_EXPECT_THROWS_MESSAGE(
      CalcDistanceFallback<AutoDiffXd>(obj_a, obj_b, request,
                                       &distance_pair_ad),
      std::logic_error,
      "Signed distance queries between shapes .+ and .+ are not supported for "
      "scalar type .*AutoDiffXd");
}
// TODO(SeanCurtis-TRI): Create a more general test framework when we have
//  shape-shape primitives that *aren't* covered by the point-shape tests.
//  For arbitrary derivatives, use `ComputeNumericalGradient()` to test.

// TODO(SeanCurtis-TRI): Assuming the sphere is A, confirm that the derivatives
//  w.r.t. R_WA are as expected (e.g., distance and nhat are untouched, but
//  p_ACa are.

// This relies on the knowledge that sphere-shape distance is a slight
// modification of point-shape distance. Essentially, I should get the same
// answers with the caveat that the distance is reduced by the sphere radius.
// So, we perform *both* tests and compare the answers. However, we rely on the
// tests for distance-to-point for general correctness of the underlying code.
//
// In the AutoDiff tests, it repeats the pattern in distance_to_point_test.
// Looks at the derivatives w.r.t. the query sphere's position. We assume that
// correct derivatives in this one case is representative of not messing up the
// derivatives created by the point-shape distance query.
class DistancePairGeometryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    R_WSphere_ = RotationMatrixd::Identity();
    p_WSphere_ = Vector3d{2, 3, 5};
    X_WShape_.set(RotationMatrixd(AngleAxis<double>(
                      M_PI / 5, Vector3d{2, 4, 7}.normalized())),
                  Vector3d::Zero());
  }

  template <typename T, typename Shape>
  ::testing::AssertionResult ResultsMatch(const Shape& test_shape) {
    SignedDistancePair<T> shape_result;
    const GeometryId sphere_id = GeometryId::get_new_id();
    const GeometryId shape_id = GeometryId::get_new_id();
    RigidTransform<T> X_WSphere(R_WSphere_.cast<T>(), p_WSphere<T>());
    DistancePairGeometry<T> distance_functor(
        sphere_id, shape_id, X_WSphere, X_WShape_.cast<T>(), &shape_result);

    // Computes sphere-shape results.
    distance_functor(query_sphere_, test_shape);
    point_distance::DistanceToPoint<T> distance_to_point(
        shape_id, X_WShape_.cast<T>(), p_WSphere<T>());
    // Computes point-shape results.
    SignedDistanceToPoint<T> point_result = distance_to_point(test_shape);

    // Compares the results.
    const double kEps = std::numeric_limits<double>::epsilon();
    using std::abs;
    if (abs(shape_result.distance - (point_result.distance - kRadius)) > kEps) {
      return ::testing::AssertionFailure() << fmt::format(
                 "Distances didn't match. Distance to shapes was {}, expected "
                 "{}",
                 shape_result.distance, point_result.distance - kRadius);
    }

    ::testing::AssertionResult result = ::testing::AssertionSuccess();

    result = CompareMatrices(shape_result.nhat_BA_W, point_result.grad_W);
    if (!result) return result;

    // Note this comparison can be performed directly because B = G and N = C.
    // So, it is ostensibly the same point expressed in the same frame.
    result = CompareMatrices(shape_result.p_BCb, point_result.p_GN);
    if (!result) return result;

    // Distance to point provides no explicit value to compare with p_ACa.
    // However, it is simply the point that is radius units away from the
    // sphere's origin in the direction of the gradient. In this case, we put
    // them both in A's frame.
    result = CompareMatrices(
        shape_result.p_ACa,
        R_WSphere_.inverse().cast<T>() * (-kRadius * point_result.grad_W));

    if (!shape_result.id_A.is_valid()) {
      return ::testing::AssertionFailure() << "id_A is not valid";
    }

    if (shape_result.id_A != sphere_id) {
      return ::testing::AssertionFailure() << fmt::format(
                 "Sphere-shape distance misreported sphere id. Expected {} got "
                 "{}",
                 sphere_id, shape_result.id_A);
    }

    if (!shape_result.id_B.is_valid()) {
      return ::testing::AssertionFailure() << "id_B is not valid";
    }

    if (shape_result.id_B != point_result.id_G) {
      return ::testing::AssertionFailure() << fmt::format(
                 "Sphere-shape distance misreported shape id. Expected {} got "
                 "{}",
                 point_result.id_G, shape_result.id_B);
    }
    if (!result) return result;

    result = TestDerivatives(shape_result, point_result);
    if (!result) return result;

    return result;
  }

  template <typename T>
  Vector3<T> p_WSphere() {
    return p_WSphere_;
  }

  ::testing::AssertionResult TestDerivatives(
      const SignedDistancePair<double>&, const SignedDistanceToPoint<double>&) {
    return ::testing::AssertionSuccess();
  }

  ::testing::AssertionResult TestDerivatives(
      const SignedDistancePair<AutoDiffXd>& shape_result,
      const SignedDistanceToPoint<AutoDiffXd>& point_result) {
    // TODO(SeanCurtis-TRI): Add a test that *doesn't* assume the basis of the
    // differentiation.

    // These tests assume a very specific basis of differentiation: p_WAₒ. For
    // notational convenience we'll use Aₒ to mean p_WAₒ. So, for every quantity
    // Q, we're going to determine that ∂(Q)/∂Aₒ is as expected. In most cases,
    // we can match the derivatives in one result directly to the other. The
    // exception is p_ACa (see below).

    ::testing::AssertionResult result = ::testing::AssertionSuccess();

    // Derivatives of distance.
    const Vector3d dd_dAo_shape = shape_result.distance.derivatives();
    const Vector3d dd_dAo_point = point_result.distance.derivatives();
    result = CompareMatrices(dd_dAo_shape, dd_dAo_point);
    if (!result) {
      return ::testing::AssertionFailure()
             << "Distance derivatives don't match; "
             << "expected: <" << dd_dAo_point << ">, got: <" << dd_dAo_shape
             << ">";
    }

    // Derivatives of nhat_BA_W; point distance grad_W member is the same
    // quantity with a different name.
    const auto dgrad_dAo_shape = math::ExtractGradient(shape_result.nhat_BA_W);
    const auto dgrad_dAo_point = math::ExtractGradient(point_result.grad_W);
    result = CompareMatrices(dgrad_dAo_shape, dgrad_dAo_point);
    if (!result) {
      return ::testing::AssertionFailure() << "Normal derivatives don't match; "
                                           << "expected:\n"
                                           << dgrad_dAo_point << "\ngot:\n"
                                           << dgrad_dAo_shape;
    }

    // Derivatives of p_BCb.
    auto dCb_dAo = math::ExtractGradient(shape_result.p_BCb);
    const auto dN_dAo = math::ExtractGradient(point_result.p_GN);
    result = CompareMatrices(dCb_dAo, dN_dAo);
    if (!result) {
      return ::testing::AssertionFailure() << "p_BCb derivatives don't match; "
                                           << "expected:\n"
                                           << dN_dAo << "\ngot:\n"
                                           << dCb_dAo;
    }

    // Derivatives of p_ACa.
    // We have no pre-computed value to compare against. Given the assumptions
    // of this problem, we _can_ express ∂(p_ACa)/∂Aₒ in terms of givens.
    //
    // p_ACa = X_AW⋅p_WCa
    //       = R_AW⋅p_WCa + p_WAₒ
    // ∂(p_ACa)/∂Aₒ = ∂(R_AW⋅p_WCa - p_WAₒ)/∂Aₒ
    //              = ∂(R_AW)/∂Aₒ⋅p_WCa + R_AW⋅∂(p_WCa)/∂Aₒ + (p_WAₒ)/∂Aₒ
    //              =         0         + R_AW⋅∂(p_WCa)/∂Aₒ -     I
    //
    // ∂(p_ACa)/∂Aₒ = R_AW⋅∂(p_WCa)/∂Aₒ - I   (Eq 1)
    //
    // We have R_AW, and need ∂(p_WCa)/∂Aₒ.
    // p_WCa = p_WCb + d⋅∇φ_B_W
    // p_WCa = X_WB⋅p_BCb + d⋅∇φ_B_W
    // ∂(p_WCa)/∂Aₒ = ∂(X_WB⋅p_BCb + d⋅∇φ_B_W)/∂Aₒ
    // ∂(p_WCa)/∂Aₒ = ∂(R_WB⋅p_BCb + p_WB + d⋅∇φ_B_W)/∂Aₒ
    // ∂(p_WCa)/∂Aₒ = ∂(R_WB)/∂Aₒ⋅p_BCb + R_WB⋅∂(p_BCb)/∂Aₒ + ∂(p_WB)/∂Aₒ
    //                ∇φ_B_W⋅∂(d)/∂Aₒᵀ + d⋅∂(∇φ_B_W)/∂Aₒ
    //              =           0       + R_WB⋅∂(p_BCb)/∂Aₒ +     0
    //                ∇φ_B_W⋅∂(d)/∂Aₒᵀ + d⋅∂(∇φ_B_W)/∂Aₒ
    //
    // ∂(p_WCa)/∂Aₒ = R_WB⋅∂(p_BCb)/∂Aₒ + ∇φ_B_W⋅∂(d)/∂Aₒᵀ + d⋅∂(∇φ_B_W)/∂Aₒ
    //                                                                    (Eq 2)
    //
    // We have all of the quantities on the right-hand side. So, we can rewrite
    // Eq 1 by substituting Eq 2 into it:
    //
    // ∂(p_ACa)/∂Aₒ =
    //   R_AW⋅[R_WB⋅∂(p_BCb)/∂Aₒ + ∂(d)/∂Aₒ⋅∇φ_B_W + d⋅∂(∇φ_B_W)/∂Aₒ] - I (Eq 3)
    const double kEps = std::numeric_limits<double>::epsilon();
    const Matrix3<double> R_AW = R_WSphere_.matrix().transpose();
    const Matrix3<double> R_WB = X_WShape_.rotation().matrix();
    const Vector3<double> grad_W = convert_to_double(point_result.grad_W);

    // Note: for some geometries, the derivatives of some of the quantities are
    // zero (e.g., the derivative of the gradient when the point is nearest a
    // face -- any infinitesimal movement of the point will *not* change the
    // normal). This is reported as having an *empty* derivatives vector. Here
    // we explicitly set it to the zero vector so Eigen sizes match up.
    if (dCb_dAo.size() == 0) dCb_dAo = Matrix3<double>::Zero();

    const Matrix3<double> dp_WCa_dAo =
        R_WB * dCb_dAo + grad_W * dd_dAo_shape.transpose() +
        shape_result.distance.value() * dgrad_dAo_shape;
    Matrix3<double> dCa_dAo_expected =
        R_AW * dp_WCa_dAo - Matrix3<double>::Identity();
    const auto dCa_dAo = math::ExtractGradient(shape_result.p_ACa);
    result = CompareMatrices(dCa_dAo, dCa_dAo_expected, 4 * kEps);
    if (!result) {
      return ::testing::AssertionFailure() << "p_ACa derivatives don't match; "
                                           << "expected:\n"
                                           << dCa_dAo_expected << "\ngot:\n"
                                           << dCa_dAo;
    }

    return result;
  }

  RotationMatrixd R_WSphere_;
  Vector3d p_WSphere_;
  RigidTransformd X_WShape_;
  static const double kRadius;
  Sphered query_sphere_{kRadius};
};

const double DistancePairGeometryTest::kRadius = 0.75;

template <>
Vector3<AutoDiffXd> DistancePairGeometryTest::p_WSphere<AutoDiffXd>() {
  return math::InitializeAutoDiff(p_WSphere_);
}

TEST_F(DistancePairGeometryTest, SphereSphereDouble) {
  EXPECT_TRUE((ResultsMatch<double, Sphered>(Sphered(1.3))));
}

TEST_F(DistancePairGeometryTest, SphereBoxDouble) {
  EXPECT_TRUE((ResultsMatch<double, Boxd>(Boxd{1.3, 2.3, 0.7})));
}

TEST_F(DistancePairGeometryTest, SphereCapsuleDouble) {
  EXPECT_TRUE((ResultsMatch<double, Capsuled>(Capsuled{1.3, 2.3})));
}

TEST_F(DistancePairGeometryTest, SphereCylinderDouble) {
  EXPECT_TRUE((ResultsMatch<double, Cylinderd>(Cylinderd{1.3, 2.3})));
}

TEST_F(DistancePairGeometryTest, SphereSphereAutoDiff) {
  EXPECT_TRUE((ResultsMatch<AutoDiffXd, Sphered>(Sphered(1.3))));
}

TEST_F(DistancePairGeometryTest, SphereBoxAutoDiffXd) {
  EXPECT_TRUE((ResultsMatch<AutoDiffXd, Boxd>(Boxd{1.3, 2.3, 0.7})));
}

TEST_F(DistancePairGeometryTest, SphereCapsuleAutoDiffXd) {
  EXPECT_TRUE((ResultsMatch<AutoDiffXd, Capsuled>(Capsuled{1.3, 2.3})));
}

TEST_F(DistancePairGeometryTest, SphereCylinderAutoDiffXd) {
  EXPECT_TRUE((ResultsMatch<AutoDiffXd, Cylinderd>(Cylinderd{1.3, 2.3})));
}

// Test the fallback logic. It is not  invoked for (sphere-X) pairs
// where X is in {Box, Capsule, Cylinder, HalfSpace, Sphere}.
GTEST_TEST(ComputeNarrowPhaseDistance, NoFallbackInvocation) {
  const CollisionObjectd box(make_shared<Boxd>(1, 2, 3));
  const CollisionObjectd capsule(make_shared<Capsuled>(1, 2));
  const CollisionObjectd cylinder(make_shared<Cylinderd>(1, 2));
  const CollisionObjectd half_space(
      make_shared<Halfspaced>(Vector3d{0, 0, 1}, 0));
  const CollisionObjectd sphere(make_shared<Sphered>(1));

  // All pairs which have Drake implementations.
  for (const CollisionObjectd* other :
       {&box, &capsule, &cylinder, &half_space, &sphere}) {
    EXPECT_FALSE(RequiresFallback(sphere, *other));
  }
}

// Confirms that the fallback *is* invoked for all other geometry pairs.
GTEST_TEST(ComputeNarrowPhaseDistance, FallbackInvocation) {
  const CollisionObjectd box(make_shared<Boxd>(1, 2, 3));
  const CollisionObjectd capsule(make_shared<Capsuled>(1, 2));
  // A minimally valid convex shape: a single vertex.
  const CollisionObjectd convex(make_shared<Convexd>(
      make_shared<const std::vector<Vector3d>>(1, Vector3d{0, 0, 0}), 0,
      make_shared<const std::vector<int>>()));
  const CollisionObjectd cylinder(make_shared<Cylinderd>(1, 2));
  const CollisionObjectd ellipsoid(make_shared<Ellipsoidd>(1, 2, 3));
  const CollisionObjectd half_space(
      make_shared<Halfspaced>(Vector3d{0, 0, 1}, 0));
  const CollisionObjectd sphere(make_shared<Sphered>(1));

  // The two exceptions involving a sphere.
  for (const CollisionObjectd* other : {&convex, &ellipsoid}) {
    EXPECT_TRUE(RequiresFallback(sphere, *other));
  }

  // All other cases.
  const std::vector<const CollisionObjectd*> others{&box, &capsule, &convex,
                                                    &cylinder, &half_space};
  for (const auto* s1 : others) {
    for (const auto* s2 : others) {
      EXPECT_TRUE(RequiresFallback(*s1, *s2));
    }
  }
}

// This confirms that I get the same answer, regardless of the order of the
// two geometries using Sphere-Box as a representative sample.
GTEST_TEST(ComputeNarrowPhaseDistance, OrderInvariance) {
  // Sphere
  CollisionObjectd sphere(make_shared<Sphered>(1));
  const GeometryId sphere_id = GeometryId::get_new_id();
  EncodedData(sphere_id, true).write_to(&sphere);

  // Box
  CollisionObjectd box(make_shared<Boxd>(1, 1, 1));
  const GeometryId box_id = GeometryId::get_new_id();
  EncodedData(box_id, true).write_to(&box);

  fcl::DistanceRequestd request{};
  const RigidTransformd X_WS(Vector3d{2, 2, 2});
  const RigidTransformd X_WB(
      AngleAxis<double>(M_PI / 5, Vector3d{2, 4, 7}.normalized()),
      Vector3d::Zero());
  SignedDistancePair<double> result_BS;
  SignedDistancePair<double> result_SB;

  ComputeNarrowPhaseDistance<double>(sphere, X_WS, box, X_WB, request,
                                     &result_SB);

  ComputeNarrowPhaseDistance<double>(box, X_WB, sphere, X_WS, request,
                                     &result_BS);

  EXPECT_EQ(result_SB.id_A, result_BS.id_B);
  EXPECT_EQ(result_SB.id_B, result_BS.id_A);
  // These values are bit identical because, regardless of parameter ordering,
  // because the sphere is always transformed into the box's frame.
  EXPECT_EQ(result_SB.distance, result_BS.distance);
  EXPECT_TRUE(CompareMatrices(result_SB.p_BCb, result_BS.p_ACa));
  EXPECT_TRUE(CompareMatrices(result_SB.p_ACa, result_BS.p_BCb));
  EXPECT_TRUE(CompareMatrices(result_SB.nhat_BA_W, -result_BS.nhat_BA_W));
}

// When a sphere is just touching a shape, confirms that the two witness
// points are at the same locations and nhat_BA_W is not NaN.  Other unit
// tests already checked the same code path as this test. Here we add this
// test to emphasize this special case.  We use Sphere-Box as a
// representative sample.
GTEST_TEST(ComputeNarrowPhaseDistance, sphere_touches_shape) {
  // Sphere
  const double radius = 1;
  CollisionObjectd sphere(make_shared<Sphered>(radius));
  const GeometryId sphere_id = GeometryId::get_new_id();
  EncodedData(sphere_id, true).write_to(&sphere);

  // Box [-1,1]x[-1,1]x[-1,1].
  const double side = 2;
  CollisionObjectd box(make_shared<Boxd>(side, side, side));
  const GeometryId box_id = GeometryId::get_new_id();
  EncodedData(box_id, true).write_to(&box);
  const RigidTransformd X_WB(RigidTransformd::Identity());
  const fcl::DistanceRequestd request{};

  // The sphere touches the box in the middle of a face of the box.
  const RigidTransformd X_WS(Vector3d{radius + side / 2., 0, 0});
  SignedDistancePair<double> result;
  ComputeNarrowPhaseDistance<double>(sphere, X_WS, box, X_WB, request, &result);
  const auto p_WCs = X_WS * result.p_ACa;
  const auto p_WCb = X_WB * result.p_BCb;
  EXPECT_EQ(p_WCs, p_WCb);
  EXPECT_FALSE((isnan(result.nhat_BA_W.array())).any());
  // The sphere A touches the box B on the right face (+x) of the box.
  EXPECT_EQ(Vector3d(1, 0, 0), result.nhat_BA_W);
}

template <typename T>
class CallbackScalarSupport : public ::testing::Test {
 public:
  CallbackScalarSupport()
      : collision_filter_(),
        X_WGs_(),
        results_(),
        data_(&collision_filter_, &X_WGs_, kInf, &results_),
        spheres_() {}

 protected:
  void SetUp() override {
    // Populate the callback data structures.
    const GeometryId id_A = GeometryId::get_new_id();
    const GeometryId id_B = GeometryId::get_new_id();
    EncodedData data_A(id_A, true);
    EncodedData data_B(id_B, true);
    collision_filter_.AddGeometry(data_A.id());
    collision_filter_.AddGeometry(data_B.id());
    X_WGs_[id_A] = RigidTransform<T>{Translation3<T>{10, 11, 12}};
    X_WGs_[id_B] = RigidTransform<T>::Identity();

    auto apply_data = [&data_A, &data_B](auto& shapes) {
      data_A.write_to(&shapes[0]);
      data_B.write_to(&shapes[1]);
    };

    spheres_.emplace_back(make_shared<Sphered>(0.25));
    spheres_.emplace_back(make_shared<Sphered>(0.75));
    apply_data(spheres_);

    boxes_.emplace_back(make_shared<Boxd>(0.25, 0.3, 0.4));
    boxes_.emplace_back(make_shared<Boxd>(0.4, 0.3, 0.2));
    apply_data(boxes_);

    capsules_.emplace_back(make_shared<Capsuled>(0.25, 0.75));
    capsules_.emplace_back(make_shared<Capsuled>(0.75, 0.25));
    apply_data(capsules_);

    cylinders_.emplace_back(make_shared<Cylinderd>(0.3, 0.4));
    cylinders_.emplace_back(make_shared<Cylinderd>(0.3, 0.2));
    apply_data(cylinders_);

    // NOTE: The mapping from drake::geometry::HalfSpace to fcl::Halfspaced
    // always encodes the normal as UnitZ() and the offset as zero.
    halfspaces_.emplace_back(make_shared<Halfspaced>(Vector3d{0, 0, 1}, 0));
    halfspaces_.emplace_back(make_shared<Halfspaced>(Vector3d{0, 0, 1}, 0));
    apply_data(halfspaces_);
  }

  std::vector<std::pair<CollisionObjectd&, CollisionObjectd&>>
  supported_pairs() {
    throw std::logic_error(
        "No supported pairs implemented -- check scalar type");
  }

  std::vector<std::pair<CollisionObjectd&, CollisionObjectd&>>
  unsupported_pairs() {
    throw std::logic_error(
        "No unsupported pairs implemented -- check scalar type");
  }

 protected:
  CollisionFilter collision_filter_;
  std::unordered_map<GeometryId, RigidTransform<T>> X_WGs_;
  std::vector<SignedDistancePair<T>> results_;
  CallbackData<T> data_;
  std::vector<CollisionObjectd> spheres_;
  std::vector<CollisionObjectd> boxes_;
  std::vector<CollisionObjectd> capsules_;
  std::vector<CollisionObjectd> cylinders_;
  std::vector<CollisionObjectd> halfspaces_;
};

template <>
std::vector<std::pair<CollisionObjectd&, CollisionObjectd&>>
CallbackScalarSupport<double>::supported_pairs() {
  // Given the shared geometry ids, it is important that the indices in each
  // pair include 0 and 1 (as shapes with a common index share a geometry id).
  return {{spheres_[0], spheres_[1]},    {spheres_[0], boxes_[1]},
          {spheres_[0], capsules_[1]},   {spheres_[0], cylinders_[1]},
          {spheres_[0], halfspaces_[1]}, {boxes_[0], boxes_[1]},
          {boxes_[0], capsules_[1]},     {boxes_[0], cylinders_[1]},
          {capsules_[0], capsules_[1]},  {cylinders_[0], capsules_[1]},
          {cylinders_[0], cylinders_[1]}};
}

template <>
std::vector<std::pair<CollisionObjectd&, CollisionObjectd&>>
CallbackScalarSupport<double>::unsupported_pairs() {
  return {
      {boxes_[0], halfspaces_[1]},
      {capsules_[0], halfspaces_[1]},
      {cylinders_[0], halfspaces_[1]},
      {halfspaces_[0], halfspaces_[1]},
  };
}

template <>
std::vector<std::pair<CollisionObjectd&, CollisionObjectd&>>
CallbackScalarSupport<AutoDiffXd>::supported_pairs() {
  return {{spheres_[0], spheres_[1]},
          {spheres_[0], boxes_[1]},
          {spheres_[0], halfspaces_[1]}};
}

template <>
std::vector<std::pair<CollisionObjectd&, CollisionObjectd&>>
CallbackScalarSupport<AutoDiffXd>::unsupported_pairs() {
  return {
      {spheres_[0], cylinders_[1]},     {boxes_[0], boxes_[1]},
      {spheres_[0], capsules_[1]},      {boxes_[0], capsules_[1]},
      {boxes_[0], cylinders_[1]},       {boxes_[0], halfspaces_[1]},
      {capsules_[0], capsules_[1]},     {cylinders_[0], capsules_[1]},
      {cylinders_[0], cylinders_[1]},   {cylinders_[0], halfspaces_[1]},
      {halfspaces_[0], halfspaces_[1]},
  };
}

using ScalarTypes = ::testing::Types<double, AutoDiffXd>;
TYPED_TEST_SUITE(CallbackScalarSupport, ScalarTypes);

// Tests that the pairs that are reported supported, run normally. The pairs
// that are marked *unsupported* throw.
TYPED_TEST(CallbackScalarSupport, SupportedPairClassification) {
  auto run_callback = [this](auto& object_A, auto& object_B) {
    double threshold = std::numeric_limits<double>::max();
    this->results_.clear();
    Callback<TypeParam>(&object_A, &object_B, &this->data_, threshold);
  };

  // Supported pairs will produce a result.
  for (auto& pair : this->supported_pairs()) {
    run_callback(pair.first, pair.second);
    EXPECT_EQ(this->results_.size(), 1);
    run_callback(pair.second, pair.first);
    EXPECT_EQ(this->results_.size(), 1);
  }
  // Unsupported pairs will *not* produce a result.
  for (auto& pair : this->unsupported_pairs()) {
    EXPECT_THROW(run_callback(pair.first, pair.second), std::logic_error);
    EXPECT_THROW(run_callback(pair.second, pair.first), std::logic_error);
  }
}

// Tests that an unsupported pair that is nevertheless *filtered* is not
// affected by scalar support.
GTEST_TEST(Callback, ScalarSupportWithFilters) {
  using T = AutoDiffXd;
  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  CollisionFilter collision_filter;

  EncodedData data_A(id_A, true);
  EncodedData data_B(id_B, true);
  collision_filter.AddGeometry(data_A.id());
  collision_filter.AddGeometry(data_B.id());

  // Filter the pair (A, B); we'll put the ids in a set and simply return that
  // set for the extract ids function.
  std::unordered_set<GeometryId> ids{data_A.id(), data_B.id()};
  CollisionFilter::ExtractIds extract = [&ids](const GeometrySet&) {
    return ids;
  };
  collision_filter.Apply(CollisionFilterDeclaration().ExcludeWithin(
                             GeometrySet{data_A.id(), data_B.id()}),
                         extract, false /* is_invariant */);

  const std::unordered_map<GeometryId, RigidTransform<T>> X_WGs{
      {id_A, RigidTransform<T>::Identity()},
      {id_B, RigidTransform<T>::Identity()}};

  CollisionObjectd box_A(make_shared<fcl::Boxd>(0.25, 0.3, 0.4));
  data_A.write_to(&box_A);
  CollisionObjectd box_B(make_shared<fcl::Boxd>(0.4, 0.3, 0.2));
  data_B.write_to(&box_B);

  std::vector<SignedDistancePair<T>> results;
  CallbackData<T> data(&collision_filter, &X_WGs, kInf, &results);
  double threshold = kInf;
  DRAKE_EXPECT_NO_THROW(Callback<T>(&box_A, &box_B, &data, threshold));
  EXPECT_EQ(results.size(), 0u);
}

GTEST_TEST(Callback, RespectCollisionFiltering) {
  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  CollisionObjectd sphere_A(make_shared<Sphered>(0.25));
  CollisionObjectd sphere_B(make_shared<Sphered>(0.25));
  EncodedData data_A(id_A, true);
  EncodedData data_B(id_B, true);
  data_A.write_to(&sphere_A);
  data_B.write_to(&sphere_B);
  const std::unordered_map<GeometryId, RigidTransformd> X_WGs{
      {id_A, RigidTransformd{Vector3d{10, 11, 12}}},
      {id_B, RigidTransformd::Identity()}};

  std::vector<SignedDistancePair<double>> results;
  CollisionFilter collision_filter;
  collision_filter.AddGeometry(data_A.id());
  collision_filter.AddGeometry(data_B.id());

  CallbackData<double> data{&collision_filter, &X_WGs, kInf, &results};

  // Case: No collision filters added should produce a single result.
  double threshold = std::numeric_limits<double>::max();
  Callback<double>(&sphere_A, &sphere_B, &data, threshold);
  EXPECT_EQ(results.size(), 1u);

  // Case: filtered collisions.

  // Filter the pair (A, B); we'll put the ids in a set and simply return that
  // set for the extract ids function.
  std::unordered_set<GeometryId> ids{data_A.id(), data_B.id()};
  CollisionFilter::ExtractIds extract = [&ids](const GeometrySet&) {
    return ids;
  };
  collision_filter.Apply(CollisionFilterDeclaration().ExcludeWithin(
                             GeometrySet{data_A.id(), data_B.id()}),
                         extract, false /* is_invariant */);
  results.clear();
  threshold = std::numeric_limits<double>::max();
  Callback<double>(&sphere_A, &sphere_B, &data, threshold);
  EXPECT_EQ(results.size(), 0u);
}

// Confirms that regardless of the order of the two objects, the result always
// has the same (A, B) ordering such that id_A < id_B.
GTEST_TEST(Callback, ABOrdering) {
  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  CollisionObjectd sphere_A(make_shared<Sphered>(0.25));
  CollisionObjectd sphere_B(make_shared<Sphered>(0.25));
  EncodedData data_A(id_A, true);
  EncodedData data_B(id_B, true);
  data_A.write_to(&sphere_A);
  data_B.write_to(&sphere_B);
  const std::unordered_map<GeometryId, RigidTransformd> X_WGs{
      {id_A, RigidTransformd{Vector3d{10, 11, 12}}},
      {id_B, RigidTransformd::Identity()}};
  CollisionFilter collision_filter;
  collision_filter.AddGeometry(data_A.id());
  collision_filter.AddGeometry(data_B.id());
  double threshold = std::numeric_limits<double>::max();

  // Pass in the two geometries in order (A, B).
  std::vector<SignedDistancePair<double>> results1;
  CallbackData<double> data1{&collision_filter, &X_WGs, kInf, &results1};
  Callback<double>(&sphere_A, &sphere_B, &data1, threshold);
  ASSERT_EQ(results1.size(), 1u);
  EXPECT_TRUE(results1[0].id_A < results1[0].id_B);

  // Pass in the two geometries in order (B, A).
  std::vector<SignedDistancePair<double>> results2;
  CallbackData<double> data2{&collision_filter, &X_WGs, kInf, &results2};
  Callback<double>(&sphere_B, &sphere_A, &data2, threshold);
  ASSERT_EQ(results2.size(), 1u);
  EXPECT_TRUE(results2[0].id_A < results2[0].id_B);

  // The numerical values match.
  EXPECT_EQ(results1[0].distance, results2[0].distance);
  EXPECT_TRUE(CompareMatrices(results1[0].p_BCb, results2[0].p_BCb));
  EXPECT_TRUE(CompareMatrices(results1[0].p_ACa, results2[0].p_ACa));
  EXPECT_TRUE(CompareMatrices(results1[0].nhat_BA_W, results2[0].nhat_BA_W));
}

// Tests that the max distance parameter affects the results returned. Because
// the max_distance parameter doesn't depend on shape type (it's applied
// strictly to the result of shape-shape computations -- it depends *only* on
// the distance value), it is sufficient to test with a representative shape
// pair (i.e., sphere-sphere). (Also confirms that the callback max_distance is
// set.)
template <typename T>
class CallbackMaxDistanceTest : public ::testing::Test {};
TYPED_TEST_SUITE(CallbackMaxDistanceTest, ScalarTypes);

TYPED_TEST(CallbackMaxDistanceTest, MaxDistanceThreshold) {
  using T = TypeParam;

  CollisionFilter collision_filter;

  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  EncodedData data_A(id_A, true);
  EncodedData data_B(id_B, true);
  collision_filter.AddGeometry(data_A.id());
  collision_filter.AddGeometry(data_B.id());

  // Two spheres with arbitrary radii. One is at the origin and the other is
  // placed at two distances: one just inside the max distance and one just
  // outside.
  const double kMaxDistance = 1.0;
  const double radius_A = 0.5;
  const double radius_B = 0.4;
  const double kEps = 2 * std::numeric_limits<double>::epsilon();
  CollisionObjectd sphere_A(make_shared<fcl::Sphered>(radius_A));
  data_A.write_to(&sphere_A);
  CollisionObjectd sphere_B(make_shared<fcl::Sphered>(radius_B));
  data_B.write_to(&sphere_B);
  const Vector3<T> p_WB = Vector3<T>(2, 3, 4).normalized() *
      (kMaxDistance + radius_A + radius_B - kEps);
  std::unordered_map<GeometryId, RigidTransform<T>> X_WGs{
      {id_A, RigidTransform<T>::Identity()},
      {id_B, RigidTransform<T>{Translation3<T>{p_WB}}}};

  // Case: just inside the max distance.
  {
    std::vector<SignedDistancePair<T>> results;
    CallbackData<T> data(&collision_filter, &X_WGs, kMaxDistance, &results);
    // NOTE: When done, this should match kMaxDistance.
    double threshold = kInf;
    DRAKE_EXPECT_NO_THROW(Callback<T>(&sphere_A, &sphere_B, &data, threshold));
    EXPECT_EQ(results.size(), 1u);
    EXPECT_EQ(threshold, kMaxDistance);
  }

  // Case: just outside the max distance.
  {
    X_WGs.at(id_B) = RigidTransform<T>(
        Translation3<T>{Vector3<T>(2, 3, 4).normalized() *
                        (kMaxDistance + radius_A + radius_B + kEps)});
    std::vector<SignedDistancePair<T>> results;
    CallbackData<T> data(&collision_filter, &X_WGs, kMaxDistance, &results);
    // NOTE: When done, this should match kMaxDistance.
    double threshold = kInf;
    DRAKE_EXPECT_NO_THROW(Callback<T>(&sphere_A, &sphere_B, &data, threshold));
    EXPECT_EQ(results.size(), 0u);
    EXPECT_EQ(threshold, kMaxDistance);
  }
}

// TODO(SeanCurtis-TRI): Ostensibly, setting the threshold in the callback to
// kMaxDistance should reduce the number of pairs that are passed into the
// callback from the broadphase pass; produce a test that confirms this
// optimization.

}  // namespace
}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
