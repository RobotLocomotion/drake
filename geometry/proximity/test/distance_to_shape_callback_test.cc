#include "drake/geometry/proximity/distance_to_shape_callback.h"

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include <fcl/fcl.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "drake/common/fmt_eigen.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity/test/fcl_utilities.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector2d;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using std::make_shared;
using std::shared_ptr;
using std::unordered_map;
using symbolic::Expression;

namespace {

// Specify a DistanceRequest that matches the request that ProximityEngine uses
// prior to calling this Callback.
fcl::DistanceRequestd MakeProximityEngineRequest() {
  fcl::DistanceRequestd request;
  request.enable_nearest_points = true;
  request.enable_signed_distance = true;
  request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
  // Note: Proximity engine passes ProximityEngine::Impl::distance_tolerance_.
  // However, that is not yet externally configurable and currently defaults to
  // 1e-6.
  request.distance_tolerance = 1e-6;
  return request;
}

constexpr double kInf = std::numeric_limits<double>::infinity();

// Tests that an unsupported geometry causes the callback to throw.
GTEST_TEST(SignedDistanceCallbackTests, ExpressionUnsupported) {
  // Add two geometries that can't be queried.
  const GeometryId id1 = GeometryId::get_new_id();
  const GeometryId id2 = GeometryId::get_new_id();
  std::unique_ptr<fcl::CollisionObjectd> obj1 =
      MakeFclObject(Box(1, 2, 3), id1, /* is_dynamic= */ true);
  std::unique_ptr<fcl::CollisionObjectd> obj2 =
      MakeFclObject(Box(2, 4, 6), id2, /* is_dynamic= */ true);

  const unordered_map<GeometryId, math::RigidTransform<Expression>> X_WGs{
      {id1, math::RigidTransform<Expression>::Identity()},
      {id2, math::RigidTransform<Expression>::Identity()}};
  std::vector<SignedDistancePair<Expression>> pairs;
  shape_distance::CallbackData<Expression> data{nullptr, &X_WGs, kInf, &pairs};
  data.request = MakeProximityEngineRequest();
  double max_dist = kInf;
  DRAKE_EXPECT_THROWS_MESSAGE(
      shape_distance::Callback<Expression>(obj1.get(), obj2.get(), &data,
                                           max_dist),
      "Signed distance queries between shapes 'Box' and 'Box' are not "
      "supported for scalar type drake::symbolic::Expression.*");
}

// Confirms that non-positive thresholds produce the right value. Creates three
// spheres: A, B, & C. A is separated from B & C and B & C are penetrating.
// We confirm that query tolerance of 0, returns B & C and that a tolerance
// of penetration depth + epsilon returns B & C, and depth - epsilon omits
// everything.
GTEST_TEST(SignedDistanceCallbackTests,
           PairwiseSignedDistanceNonPositiveThreshold) {
  const GeometryId id1 = GeometryId::get_new_id();
  const GeometryId id2 = GeometryId::get_new_id();
  const GeometryId id3 = GeometryId::get_new_id();
  const double kRadius = 0.5;
  const unordered_map<GeometryId, RigidTransformd> X_WGs{
      {id1, RigidTransformd{Vector3d{0, 2 * kRadius, 0}}},
      {id2, RigidTransformd{Vector3d{-kRadius * 0.9, 0, 0}}},
      {id3, RigidTransformd{Vector3d{kRadius * 0.9, 0, 0}}}};

  Sphere sphere{kRadius};
  std::unique_ptr<fcl::CollisionObjectd> obj1 =
      MakeFclObject(sphere, id1, /* is_dynamic= */ true);
  std::unique_ptr<fcl::CollisionObjectd> obj2 =
      MakeFclObject(sphere, id2, /* is_dynamic= */ true);
  std::unique_ptr<fcl::CollisionObjectd> obj3 =
      MakeFclObject(sphere, id3, /* is_dynamic= */ true);

  // Calls shape_distance::Callback for all three pairs with the given threshold
  // and returns the collected results.
  auto invoke_all_pairs =
      [&](double threshold) -> std::vector<SignedDistancePair<double>> {
    std::vector<SignedDistancePair<double>> results;
    shape_distance::CallbackData<double> data{nullptr, &X_WGs, threshold,
                                              &results};
    data.request = MakeProximityEngineRequest();
    double max_dist = threshold;
    shape_distance::Callback<double>(obj1.get(), obj2.get(), &data, max_dist);
    max_dist = threshold;
    shape_distance::Callback<double>(obj1.get(), obj3.get(), &data, max_dist);
    max_dist = threshold;
    shape_distance::Callback<double>(obj2.get(), obj3.get(), &data, max_dist);
    return results;
  };

  EXPECT_EQ(invoke_all_pairs(kInf).size(), 3u);

  EXPECT_EQ(invoke_all_pairs(0).size(), 1u);

  const double penetration = -kRadius * 0.2;
  const double kEps = std::numeric_limits<double>::epsilon();

  EXPECT_EQ(invoke_all_pairs(penetration + kEps).size(), 1u);

  EXPECT_EQ(invoke_all_pairs(penetration - kEps).size(), 0u);
}

// Test shape_distance::Callback with sphere-sphere, sphere-box,
// sphere-capsule, sphere-cylinder pairs, and sphere-half space.
// The definition of this test suite consists of four sections.
// 1. Generate test data as a vector of SignedDistancePairTestData. Each
//    record consists of both input and expected result.  See the function
//    GenDistancePairTestSphereSphere(), for example.
// 2. Define a test fixture parameterized by the test data.  It initializes
//    each test according to the test data. See the class
//    SignedDistancePairTest below.
// 3. Define one or more test procedures for the same test fixture. We call
//    shape_distance::Callback directly from here.
//    See TEST_P(SignedDistancePairTest, SinglePair), for example.
// 4. Initiate all the tests by specifying the test fixture and the test data.
//    See INSTANTIATE_TEST_SUITE_P() below.
class SignedDistancePairTestData {
 public:
  SignedDistancePairTestData(shared_ptr<const Shape> a,
                             shared_ptr<const Shape> b,
                             const RigidTransformd& X_WA,
                             const RigidTransformd& X_WB,
                             const SignedDistancePair<double>& expect,
                             const double tolerance = 0)
      : a_(a),
        b_(b),
        X_WA_(X_WA),
        X_WB_(X_WB),
        expected_result_(expect),
        tolerance_(tolerance) {}

  // Construct test data. The normal is optional; when not provided it is zero.
  // If a meaningful normal is expected, it needs to be provided.
  static SignedDistancePairTestData Make(
      shared_ptr<const Shape> a, shared_ptr<const Shape> b,
      const RigidTransformd& X_WA, const RigidTransformd& X_WB,
      const Vector3d& p_ACa_A, const Vector3d& p_BCb_B, double distance,
      const Vector3d& nhat_BA_W = Vector3d::Zero(), double tolerance = 0) {
    return {a,
            b,
            X_WA,
            X_WB,
            SignedDistancePair<double>(GeometryId::get_new_id(),
                                       GeometryId::get_new_id(), p_ACa_A,
                                       p_BCb_B, distance, nhat_BA_W),
            tolerance};
  }

  // Generates new test data by swapping geometry A and geometry B. It will
  // help us test the symmetric interface. For example, we can generate the
  // test data for test(box_B, sphere_A) from the test data for
  // test(sphere_A, box_B).
  SignedDistancePairTestData GenSwapAB() const {
    auto& id_A = expected_result_.id_A;
    auto& id_B = expected_result_.id_B;
    auto& p_ACa = expected_result_.p_ACa;
    auto& p_BCb = expected_result_.p_BCb;
    auto& distance = expected_result_.distance;
    return SignedDistancePairTestData(
        b_, a_, X_WB_, X_WA_,
        SignedDistancePair<double>(id_B, id_A, p_BCb, p_ACa, distance,
                                   -expected_result_.nhat_BA_W),
        tolerance_);
  }

  // Google Test uses this operator to report the test data in the log file
  // when a test fails.
  friend std::ostream& operator<<(std::ostream& os,
                                  const SignedDistancePairTestData& obj) {
    fmt::print(os,
               "{{\n"
               " geometry A: (not printed)\n"
               " geometry B: (not printed)\n"
               " X_WA: (not printed)\n"
               " X_WB: (not printed)\n"
               " expected_result.id_A: {}\n"
               " expected_result.id_B: {}\n"
               " expected_result.distance: {}\n"
               " expected_result.p_ACa: {}\n"
               " expected_result.p_BCb: {}\n"
               "}}",
               obj.expected_result_.id_A, obj.expected_result_.id_B,
               obj.expected_result_.distance,
               fmt_eigen(obj.expected_result_.p_ACa.transpose()),
               fmt_eigen(obj.expected_result_.p_BCb.transpose()));
    return os;
  }

  shared_ptr<const Shape> a_;
  shared_ptr<const Shape> b_;
  const RigidTransformd X_WA_;
  const RigidTransformd X_WB_;
  const SignedDistancePair<double> expected_result_;
  const double tolerance_;
};

// Two spheres with varying degrees of overlapping.  The first sphere A is
// smaller than the second sphere B. We model the configurations in the frame
// of the first sphere A.  First we place B's center far enough to the right
// on the positive x-axis of A's frame that A and B do not overlap. Then, we
// move B's center towards A's center along the x-axis of A's frame until B
// covers A. The expressions of the witness points Ca ∈ ∂A and Cb ∈ ∂B in
// A's frame and B's frame respectively do not change during this motion.
//
// @param X_WA specifies the pose of A in world.
// @param R_WB specifies the orientation of B in world.
std::vector<SignedDistancePairTestData> GenDistancePairTestSphereSphere(
    const RigidTransformd& X_WA = RigidTransformd::Identity(),
    const RotationMatrixd& R_WB = RotationMatrixd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(1.0);
  auto sphere_B = make_shared<const Sphere>(2.0);
  double radius_A = sphere_A->radius();
  double radius_B = sphere_B->radius();
  // Set up R_AB and R_BA from X_WA and R_WB.
  const RotationMatrixd& R_WA = X_WA.rotation();
  const RotationMatrixd R_AW = R_WA.transpose();
  const RotationMatrixd R_AB = R_AW * R_WB;
  const RotationMatrixd R_BA = R_AB.transpose();
  struct Configuration {
    // Center of B in A's frame
    Vector3d p_ABo;
    double pair_distance;
  };
  const std::vector<Configuration> configurations{// Non-overlapping
                                                  {Vector3d(4, 0, 0), 1},
                                                  // B kisses A.
                                                  {Vector3d(3, 0, 0), 0},
                                                  // B overlaps A.
                                                  {Vector3d(2.5, 0, 0), -0.5},
                                                  // B covers A.
                                                  {Vector3d(1, 0, 0), -2}};
  const Vector3d p_ACa_A(radius_A, 0, 0);
  // Position from Bo to Cb expressed in A's frame.
  const Vector3d p_BCb_A(-radius_B, 0, 0);
  const Vector3d p_BCb_B = R_BA * p_BCb_A;
  std::vector<SignedDistancePairTestData> test_data;
  for (const auto& config : configurations) {
    const RigidTransformd X_AB(R_AB, config.p_ABo);
    const RigidTransformd X_WB = X_WA * X_AB;
    test_data.emplace_back(SignedDistancePairTestData::Make(
        sphere_A, sphere_B, X_WA, X_WB, p_ACa_A, p_BCb_B,
        config.pair_distance));
  }
  return test_data;
}

std::vector<SignedDistancePairTestData> GenDistPairTestSphereSphereTransform() {
  return GenDistancePairTestSphereSphere(
      RigidTransformd(RollPitchYawd(5 * M_PI / 8, M_PI / 6, M_PI / 12),
                      Vector3d(1, 2, 3)),
      RotationMatrixd(RollPitchYawd(M_PI / 4, 5 * M_PI / 6, M_PI / 3)));
}

// Gimbal lock of the orientation of the sphere at pitch = pi/2.
std::vector<SignedDistancePairTestData>
GenDistPairTestSphereSphereGimbalLock() {
  return GenDistancePairTestSphereSphere(
      RigidTransformd::Identity(),
      RotationMatrixd(RollPitchYawd(M_PI / 6, M_PI_2, M_PI / 6)));
}

// Two spheres with more general position. Here, we describe the configuration
// in the frame of the first sphere A.  The second sphere B is centered at
// (1,4,8) in A's frame.  We use these identities to design the configuration.
//     0.25^2 + 1^2 + 2^2 = 2.25^2   (1)
//        1^2 + 4^2 + 8^2 = 9^2      (2)
//
//              x x x x x
//           x             x
//         x                 x
//       x                     x
//     x                         x
//    x                           x
//   x                             x
//  x                               x
//  x                               x
//  x                               x
//  x              Bo               x
//  x             /                 x
//  x            /                  x
//  x           /                   x
//   x         /                   x
//    x  ooo  /                   x
//    ox     Ca                  x
//   o   x  /  o               x
//   o  Ao=Cb  o             x
//   o       x o           x
//    o       o x x x x x
//       ooo
//
// From (1), we set A to have radius r_A 2.25 and will have the witness point
// Ca ∈ ∂A at (0.25,1,2) in A's frame.  From (2), we set B to have radius
// r_B 9.0 with the center Bo (1,4,8) in A's frame. The boundary ∂B passes
// through A's center Ao, which is at the same position as the witness point
// Cb ∈ ∂B.  The signed distance between the two spheres equals -2.25, is the
// negative of the radius of A.
//
// @param X_WA the pose of A in world.
// @param R_WB the orientation of B in world.
std::vector<SignedDistancePairTestData> GenDistPairTestSphereSphereNonAligned(
    const RigidTransformd& X_WA = RigidTransformd::Identity(),
    const RotationMatrixd& R_WB = RotationMatrixd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(2.25);
  auto sphere_B = make_shared<const Sphere>(9.0);
  const double radius_A = sphere_A->radius();
  // Set up Ca and Bo in A's frame.
  const Vector3d p_ACa(0.25, 1, 2);
  const Vector3d p_ABo(1, 4, 8);
  // Set up X_AB and X_BA from the position of B's center p_ABo in A's frame,
  // the given R_WB, and the given X_WA.
  const RotationMatrixd& R_WA = X_WA.rotation();
  const RotationMatrixd R_AW = R_WA.transpose();
  const RotationMatrixd R_AB = R_AW * R_WB;
  const RigidTransformd X_AB(R_AB, p_ABo);
  const RigidTransformd X_BA = X_AB.inverse();
  // Set up X_WB
  const RigidTransformd X_WB = X_WA * X_AB;
  // Set up Cb = Ao.
  const Vector3d p_AAo(0, 0, 0);
  const Vector3d p_ACb = p_AAo;
  const Vector3d p_BCb = X_BA * p_ACb;

  std::vector<SignedDistancePairTestData> test_data{
      SignedDistancePairTestData::Make(sphere_A, sphere_B, X_WA, X_WB, p_ACa,
                                       p_BCb, -radius_A)};
  return test_data;
}

std::vector<SignedDistancePairTestData>
GenDistPairTestSphereSphereNonAlignedTransform() {
  return GenDistPairTestSphereSphereNonAligned(
      RigidTransformd(RollPitchYawd(3 * M_PI / 8, 5 * M_PI / 6, M_PI / 12),
                      Vector3d(1, 2, 3)),
      RotationMatrixd(RollPitchYawd(M_PI, 5 * M_PI / 6, 7 * M_PI / 12)));
}

// Sphere-box data for testing ComputeSignedDistancePairwiseClosestPoints.
// We move a small sphere through different configurations:-
// 1. outside the box,
// 2. touching the box,
// 3. slightly overlap the box,
// 4. half inside half outside the box,
// 5. more than half inside the box,
// 6. completely inside and osculating the box, and
// 7. deeply inside the box.
// The sphere's center always stays on the box's positive x-axis, and the
// witness points as expressed in the frames of the sphere and the box stay
// the same in all cases.
//
// @param R_WA specifies the orientation of the sphere A in world.
// @param X_WB specifies the pose of the box B in world.
// @return the test data for testing sphere-box pairwise signed distances.
std::vector<SignedDistancePairTestData> GenDistPairTestSphereBox(
    const RotationMatrixd& R_WA = RotationMatrixd::Identity(),
    const RigidTransformd& X_WB = RigidTransformd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(2.0);
  auto box_B = make_shared<const Box>(16.0, 12.0, 8.0);
  const double radius = sphere_A->radius();
  const double half_x = box_B->size()(0) / 2;
  struct Configuration {
    Vector3d p_BAo;
    double pair_distance;
  };
  const std::vector<Configuration> configurations{
      // The sphere is outside the box.
      {Vector3d(14, 0, 0), 4.0},
      // The sphere touches the box.
      {Vector3d(10, 0, 0), 0.0},
      // The sphere slightly overlaps the box.
      {Vector3d(9, 0, 0), -1.0},
      // The sphere is half inside and half outside the box.
      {Vector3d(8, 0, 0), -2.0},
      // More than half of the sphere is inside the box.
      {Vector3d(7, 0, 0), -3.0},
      // The sphere is completely inside and osculating the box.
      {Vector3d(6, 0, 0), -4.0},
      // The sphere is deeply inside the box.
      {Vector3d(5, 0, 0), -5.0}};
  const Vector3d p_BCb(half_x, 0, 0);
  std::vector<SignedDistancePairTestData> test_data;
  for (auto config : configurations) {
    // Set up the pose of A from the translation vector in the configuration
    // and the given rotation parameter R_WA.
    const Vector3d p_WAo = X_WB * config.p_BAo;
    const RigidTransformd X_WA(R_WA, p_WAo);
    // Set up the transformation from B's frame to A's frame.
    const RigidTransformd X_AW = X_WA.inverse();
    const RigidTransformd X_AB = X_AW * X_WB;
    // Calculate Ca in B's frame then change to A's frame.
    const Vector3d p_BCa = config.p_BAo + Vector3d(-radius, 0, 0);
    const Vector3d p_ACa = X_AB * p_BCa;

    test_data.emplace_back(SignedDistancePairTestData::Make(
        sphere_A, box_B, X_WA, X_WB, p_ACa, p_BCb, config.pair_distance));
  }
  return test_data;
}

std::vector<SignedDistancePairTestData> GenDistPairTestSphereBoxTransform() {
  return GenDistPairTestSphereBox(
      RotationMatrixd(RollPitchYawd(M_PI / 8, M_PI / 6, 2 * M_PI / 3)),
      RigidTransformd(RollPitchYawd(3 * M_PI / 8, 5 * M_PI / 6, M_PI / 12),
                      Vector3d(1, 2, 3)));
}

// Gimbal lock of the orientation of the sphere at pitch = pi/2.
std::vector<SignedDistancePairTestData> GenDistPairTestSphereBoxGimbalLock() {
  return GenDistPairTestSphereBox(
      RotationMatrixd(RollPitchYawd(M_PI / 8, M_PI_2, M_PI / 8)),
      RigidTransformd::Identity());
}

std::vector<SignedDistancePairTestData> GenDistPairTestBoxSphere() {
  std::vector<SignedDistancePairTestData> test_data;
  for (const auto& sphere_box : GenDistPairTestSphereBox()) {
    test_data.emplace_back(sphere_box.GenSwapAB());
  }
  return test_data;
}

std::vector<SignedDistancePairTestData> GenDistPairTestBoxSphereTransform() {
  std::vector<SignedDistancePairTestData> test_data;
  for (const auto& sphere_box : GenDistPairTestSphereBoxTransform()) {
    test_data.emplace_back(sphere_box.GenSwapAB());
  }
  return test_data;
}

// Generates test data for a sphere A with center Ao on the boundary ∂B of a
// box B. The 26 positions of Ao on ∂B can be expressed in B's frame as:-
//   p_BAo ∈ {-h(x),0,+h(x)} x {-h(y),0,+h(y)} x {-h(z),0,+h(z)} - {(0,0,0)},
// where h(x), h(y), and h(z) are the half width, half depth, and half height
// of B respectively. These positions are at the 8 corners, in the middle
// of the 12 edges, and in the middle of the 6 faces of B.
//
// The positions of Ao above is parameterized by the sign vector s expressed
// in B's frame as:
//     s_B = (sx,sy,sz) ∈ {-1,0,+1} x {-1,0,+1} x {-1,0,+1} - {(0,0,0)},
//     p_BAo(s) = (sx * h(x), sy * h(y), sz * h(z)).
//
// For these test cases, the signed distance between A and B is always -r,
// where r is the radius of A. The witness point Cb on ∂B is at Ao.
// The witness point Ca on ∂A is
//     Ca = Ao - r * s/|s|,
// whose position p_ACa in A's frame is expressed as
//     p_ACa = - r * s_A/|s_A|,
// where s_A is the sign vector s expressed in A's frame.
//
// @param R_WA specifies the orientation of the sphere in world.
// @param X_WB specifies the pose of the box in world.
// @return  the test data for testing sphere-box pairwise signed distances.
//
std::vector<SignedDistancePairTestData> GenDistPairTestSphereBoxBoundary(
    const RotationMatrixd& R_WA = RotationMatrixd::Identity(),
    const RigidTransformd& X_WB = RigidTransformd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(2.0);
  auto box_B = make_shared<const Box>(16.0, 12.0, 8.0);
  const double radius_A = sphere_A->radius();
  const Vector3d half_B = box_B->size() / 2;
  std::vector<SignedDistancePairTestData> test_data;
  // We use sign_x, sign_y, and sign_z to parameterize the positions on the
  // boundary of the box.
  for (const double sign_x : {-1, 0, 1}) {
    for (const double sign_y : {-1, 0, 1}) {
      for (const double sign_z : {-1, 0, 1}) {
        if (sign_x == 0 && sign_y == 0 && sign_z == 0) continue;
        const Vector3d sign_B(sign_x, sign_y, sign_z);
        // A's center is on ∂B.
        const Vector3d p_BAo = sign_B.array() * half_B.array();
        const Vector3d p_WAo = X_WB * p_BAo;
        const RigidTransformd X_WA(R_WA, p_WAo);
        // The expected witness point Cb on ∂B at A's center.
        const Vector3d p_BCb = p_BAo;
        // Set up the rotation matrix for vectors from B's frame to A's frame.
        const RotationMatrixd& R_WB = X_WB.rotation();
        const RotationMatrixd R_AW = R_WA.transpose();
        const RotationMatrixd R_AB = R_AW * R_WB;
        // Change the expression of the sign vector from sign_B in B's frame
        // to sign_A in A's frame.
        const Vector3d sign_A = R_AB * sign_B;
        // The expected witness point Ca on ∂A expressed in A's frame.
        const Vector3d p_ACa = -radius_A * sign_A.normalized();
        test_data.emplace_back(SignedDistancePairTestData::Make(
            sphere_A, box_B, X_WA, X_WB, p_ACa, p_BCb, -radius_A));
      }
    }
  }
  return test_data;
}

std::vector<SignedDistancePairTestData>
GenDistPairTestSphereBoxBoundaryTransform() {
  return GenDistPairTestSphereBoxBoundary(
      RotationMatrixd(RollPitchYawd(M_PI, M_PI / 6, M_PI / 3)),
      RigidTransformd(RollPitchYawd(3 * M_PI / 8, 5 * M_PI / 6, M_PI / 12),
                      Vector3d(1, 2, 3)));
}

// Sphere-capsule data for testing ComputeSignedDistancePairwiseClosestPoints.
// We move a small sphere through different configurations:-
// 1. outside the capsule,
// 2. touching the capsule,
// 3. slightly overlap the capsule,
// 4. half inside half outside the capsule,
// 5. more than half inside the capsule,
// 6. completely inside and osculating the capsule, and
// 7. deeply inside the capsule.
// The sphere's center always stays on the capsule's positive z-axis.
// The witness point on the capsule stays the same in all cases.
// The witness point on the sphere is always the lowest point as expressed
// in the frame of the capsule.
//
// @param R_WA specifies the orientation of the sphere A in world.
// @param X_WB specifies the pose of the capsule B in world.
// @return the test data for testing sphere-capsule pairwise signed distances.
std::vector<SignedDistancePairTestData> GenDistPairTestSphereCapsule(
    const RotationMatrixd& R_WA = RotationMatrixd::Identity(),
    const RigidTransformd& X_WB = RigidTransformd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(2.0);
  auto capsule_B = make_shared<const Capsule>(6.0, 16.0);
  const double radius_A = sphere_A->radius();
  const double half_length = capsule_B->length() / 2 + capsule_B->radius();
  const std::vector<double> pair_distances{
      // The sphere is outside the capsule.
      radius_A * 2,
      // The sphere touches the capsule.
      0.0,
      // The sphere slightly overlaps the capsule.
      -radius_A * 0.5,
      // The sphere is half inside and half outside the capsule.
      -radius_A,
      // The sphere's center is inside, but the sphere is not completely
      // encapsulated.
      -radius_A * 1.5,
      // The sphere is completely inside and osculating the capsule.
      -radius_A * 2,
      // The sphere is deeply inside the capsule (whose size is much larger
      // than radius).
      -radius_A * 2.5};
  // Witness point Cb on B.
  const Vector3d p_BCb(0, 0, half_length);
  std::vector<SignedDistancePairTestData> test_data;
  for (const auto& pair_distance : pair_distances) {
    const Vector3d p_BAo =
        Vector3d(0, 0, pair_distance + half_length + radius_A);
    // Set up the pose of A from the translation vector in the configuration
    // and the given rotation parameter R_WA.
    const Vector3d p_WAo = X_WB * p_BAo;
    const RigidTransformd X_WA(R_WA, p_WAo);
    // Set up the transformation form B's frame to A's frame.
    const RigidTransformd X_AW = X_WA.inverse();
    const RigidTransformd X_AB = X_AW * X_WB;
    // Witness point Ca on A. Calculate its position in B's frame then change
    // to A's frame.
    const Vector3d p_BCa = p_BAo + Vector3d(0, 0, -radius_A);
    const Vector3d p_ACa = X_AB * p_BCa;
    test_data.emplace_back(SignedDistancePairTestData::Make(
        sphere_A, capsule_B, X_WA, X_WB, p_ACa, p_BCb, pair_distance));
  }
  return test_data;
}

std::vector<SignedDistancePairTestData>
GenDistPairTestSphereCapsuleTransform() {
  return GenDistPairTestSphereCapsule(
      RotationMatrixd(RollPitchYawd(M_PI, M_PI / 6, M_PI / 3)),
      RigidTransformd(RollPitchYawd(3 * M_PI / 8, 5 * M_PI / 6, M_PI / 12),
                      Vector3d(1, 2, 3)));
}

// Sphere-cylinder data for testing ComputeSignedDistancePairwiseClosestPoints.
// We move a small sphere through different configurations:-
// 1. outside the cylinder,
// 2. touching the cylinder,
// 3. slightly overlap the cylinder,
// 4. half inside half outside the cylinder,
// 5. more than half inside the cylinder,
// 6. completely inside and osculating the cylinder, and
// 7. deeply inside the cylinder.
// The sphere's center always stays on the cylinder's positive z-axis.
// The witness point on the cylinder stays the same in all cases.
// The witness point on the sphere is always the lowest point as expressed
// in the frame of the cylinder.
//
// @param R_WA specifies the orientation of the sphere A in world.
// @param X_WB specifies the pose of the cylinder B in world.
// @return the test data for testing sphere-cylinder pairwise signed distances.
std::vector<SignedDistancePairTestData> GenDistPairTestSphereCylinder(
    const RotationMatrixd& R_WA = RotationMatrixd::Identity(),
    const RigidTransformd& X_WB = RigidTransformd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(2.0);
  auto cylinder_B = make_shared<const Cylinder>(12.0, 16.0);
  const double radius_A = sphere_A->radius();
  const double half_length = cylinder_B->length() / 2;
  struct Configuration {
    Vector3d p_BAo;
    double pair_distance;
  };
  const std::vector<Configuration> configurations{
      // The sphere is outside the cylinder.
      {Vector3d(0, 0, 14), 4.0},
      // The sphere touches the cylinder.
      {Vector3d(0, 0, 10), 0.0},
      // The sphere slightly overlaps the cylinder.
      {Vector3d(0, 0, 9), -1.0},
      // The sphere is half inside and half outside the cylinder.
      {Vector3d(0, 0, 8), -2.0},
      // More than half of the sphere is inside the cylinder.
      {Vector3d(0, 0, 7), -3.0},
      // The sphere is completely inside and osculating the cylinder.
      {Vector3d(0, 0, 6), -4.0},
      // The sphere is deeply inside the cylinder.
      {Vector3d(0, 0, 5), -5.0}};
  // Witness point Cb on B.
  const Vector3d p_BCb(0, 0, half_length);
  std::vector<SignedDistancePairTestData> test_data;
  for (const auto& config : configurations) {
    // Set up the pose of A from the translation vector in the configuration
    // and the given rotation parameter R_WA.
    const Vector3d p_WAo = X_WB * config.p_BAo;
    const RigidTransformd X_WA(R_WA, p_WAo);
    // Set up the transformation form B's frame to A's frame.
    const RigidTransformd X_AW = X_WA.inverse();
    const RigidTransformd X_AB = X_AW * X_WB;
    // Witness point Ca on A. Calculate its position in B's frame then change
    // to A's frame.
    const Vector3d p_BCa = config.p_BAo + Vector3d(0, 0, -radius_A);
    const Vector3d p_ACa = X_AB * p_BCa;
    test_data.emplace_back(SignedDistancePairTestData::Make(
        sphere_A, cylinder_B, X_WA, X_WB, p_ACa, p_BCb, config.pair_distance));
  }
  return test_data;
}

std::vector<SignedDistancePairTestData>
GenDistPairTestSphereCylinderTransform() {
  return GenDistPairTestSphereCylinder(
      RotationMatrixd(RollPitchYawd(M_PI, M_PI / 6, M_PI / 3)),
      RigidTransformd(RollPitchYawd(3 * M_PI / 8, 5 * M_PI / 6, M_PI / 12),
                      Vector3d(1, 2, 3)));
}

// Generate test data for a sphere A and cylinder B with center Ao on its
// boundary ∂B.  We use the following mapping to generate the test data from a
// uniform sampling (u,v,w) of the standard cube [-1,1]^3 to p_BAo = (x,y,z)
// on ∂B.
// 0. Generate a uniform sampling (u,v,w) ∈ [-1,1]^3
//       u,v,w ∈ (1/2)*{-2,-1,0,1,2}
//    Skip (u,v,w) with none of |u|,|v|,|w| equals 1.0.  Otherwise, they will
//    map to interior points.
// 1. Map s = (u,v) in the standard square [-1,1]^2 to d = (f,g) in the unit
//    disk centered at (0,0) with radius 1.
//      d = (f,g) = (0,0) if u = v = 0
//                = |u| * s.normalized() otherwise, if |u| >= |v|
//                = |v| * s.normalized() otherwise
// 2. Map d = (f,g) in the unit disk to the circular cross section (x,y) in the
//    cylinder B.
//      (x,y) = r_B * d,  where r_B is the radius of the cylinder B.
// 3. Map w in the standard interval [-1,1] to the vertical coordinate z in
//    the cylinder B.
//      z = w * h, where h is the half length of the cylinder B.
//
// @note: This mapping from (u,v,w) on the cube to (x,y,z) in the cylinder maps
// the top and bottom squares of the cube to the top and bottom caps of the
// cylinder.
//
// In all cases, the signed distance is -r_A, where r_A is the radius of the
// sphere A.

// Since p_BAo = (x,y,z) above is on ∂B, the witness point Cb of the cylinder
// B is always at Ao.
//
// We classify the following cases to compute the witness point Ca on the
// sphere A.  The expression is in B's frame.  The classification is in this
// order:
// 1. (|u|=1 or |v|=1) and |w|=1.
//    Ao is on the circular edges, i.e., the top and bottom circles.
//        p_AoCa_B = -r_A * (f,g,w).normalized()
// 2. |u|=1 or |v|=1.
//    Ao is on the barrel.
//        p_AoCa_B = -r_A * (f,g,0)
// 3. |w|=1.
//    Ao is on the top or bottom caps.
//        p_AoCa_B = -r_A * (0,0,w)
std::vector<SignedDistancePairTestData> GenDistPairTestSphereCylinderBoundary(
    const RotationMatrixd& R_WA = RotationMatrixd::Identity(),
    const RigidTransformd& X_WB = RigidTransformd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(4.0);
  auto cylinder_B = make_shared<const Cylinder>(8.0, 16.0);
  const double r_A = sphere_A->radius();
  const double r_B = cylinder_B->radius();
  const double h = cylinder_B->length() / 2;
  std::vector<SignedDistancePairTestData> test_data;
  for (const double i : {-2, -1, 0, 1, 2}) {
    for (const double j : {-2, -1, 0, 1, 2}) {
      for (const double k : {-2, -1, 0, 1, 2}) {
        // (u,v,w) in the standard cube [-1,1]^3
        const double u = i / 2;
        const double v = j / 2;
        const double w = k / 2;
        const double abs_u = std::abs(u);
        const double abs_v = std::abs(v);
        const double abs_w = std::abs(w);
        // Skip interior points.
        if (abs_u != 1 && abs_v != 1 && abs_w != 1) {
          continue;
        }
        // Map from s(u,v) in square to d(f,g) in the unit disk.
        const Vector2d s(u, v);
        const Vector2d d = (u == 0 && v == 0) ? Vector2d(0, 0)
                           : (abs_u >= abs_v) ? abs_u * s.normalized()
                                              : abs_v * s.normalized();
        // Map from d(f,g) in the unit disk together with w in [-1,1] to
        // (x,y,z) in the cylinder.
        const Vector2d xy = r_B * d;
        const double z = w * h;
        // The position of Ao in B's frame and world frame.
        const Vector3d p_BAo(xy(0), xy(1), z);
        const Vector3d p_WAo = X_WB * p_BAo;
        // The expected witness point Cb on ∂B is at A's center.
        const Vector3d& p_BCb = p_BAo;
        // The pose of the sphere A.
        const RigidTransformd X_WA(R_WA, p_WAo);
        // Calculate rotation matrix R_AB for transforming vector expression
        // from B's frame to A's frame.
        const RotationMatrixd& R_WB = X_WB.rotation();
        const RotationMatrixd& R_AW = R_WA.transpose();
        const RotationMatrixd& R_AB = R_AW * R_WB;
        // Compute the witness point Ca on the sphere A in three cases.  The
        // expression is in B's frame.
        const Vector3d p_AoCa_B =
            ((abs_u == 1 || abs_v == 1) && abs_w == 1)
                // Ao is on the circular edges.
                ? -r_A * Vector3d(d(0), d(1), w).normalized()
                : (abs_u == 1 || abs_v == 1)
                      // Ao is on the barrel.
                      ? -r_A * Vector3d(d(0), d(1), 0)
                      // Ao is on the top or bottom caps.
                      : -r_A * Vector3d(0, 0, w);
        const Vector3d p_ACa = R_AB * p_AoCa_B;
        // We implicltly create new id's for each test case in the invocation of
        // Make to help distinguish them.
        test_data.emplace_back(SignedDistancePairTestData::Make(
            sphere_A, cylinder_B, X_WA, X_WB, p_ACa, p_BCb, -r_A));
      }
    }
  }
  return test_data;
}

std::vector<SignedDistancePairTestData>
GenDistPairTestSphereCylinderBoundaryTransform() {
  return GenDistPairTestSphereCylinderBoundary(
      RotationMatrixd(RollPitchYawd(M_PI, M_PI / 6, M_PI / 3)),
      RigidTransformd(RollPitchYawd(3 * M_PI / 8, 5 * M_PI / 6, M_PI / 12),
                      Vector3d(1, 2, 3)));
}

std::vector<SignedDistancePairTestData> GenDistPairTestHalfspaceSphere() {
  const double distance = 0.25;
  const double radius = 1.75;
  std::vector<SignedDistancePairTestData> test_data;
  auto sphere = make_shared<const Sphere>(radius);
  const GeometryId sphere_id = GeometryId::get_new_id();
  auto half_space = make_shared<const HalfSpace>();
  const GeometryId half_space_id = GeometryId::get_new_id();
  const RigidTransformd X_WG1(RollPitchYawd(M_PI / 3, M_PI / 6, M_PI_2),
                              Vector3d{10, 11, 12});
  const RotationMatrixd R_WS;

  for (const auto& X_WH : {RigidTransformd(), X_WG1}) {
    const auto& R_WH = X_WH.rotation();
    // Half space's outward normal is in the direction of the z-axis of the
    // half space's canonical frame H.
    const Vector3d nhat_HS_W = R_WH.col(2);
    const Vector3d p_HCh(1.25, 1.5, 0);
    const Vector3d p_WCh = X_WH * p_HCh;

    // Outside the half space (with permuted ordering).
    {
      const Vector3d p_WS = p_WCh + (distance + radius) * nhat_HS_W;
      const RigidTransformd X_WS(p_WS);
      const Vector3d p_SCs = -radius * (R_WS.inverse() * nhat_HS_W);
      SignedDistancePair<double> expected(sphere_id, half_space_id, p_SCs,
                                          p_HCh, distance, nhat_HS_W);
      test_data.emplace_back(sphere, half_space, X_WS, X_WH, expected);
      expected.SwapAAndB();
      test_data.emplace_back(half_space, sphere, X_WH, X_WS, expected);
    }

    // On the half space boundary.
    {
      const Vector3d p_WS = p_WCh + radius * nhat_HS_W;
      const RigidTransformd X_WS(p_WS);
      const Vector3d p_SCs = -radius * (R_WS.inverse() * nhat_HS_W);
      SignedDistancePair<double> expected(sphere_id, half_space_id, p_SCs,
                                          p_HCh, 0, nhat_HS_W);
      test_data.emplace_back(sphere, half_space, X_WS, X_WH, expected);
      expected.SwapAAndB();
      test_data.emplace_back(half_space, sphere, X_WH, X_WS, expected);
    }

    // Inside the half space.
    {
      const Vector3d p_WS = p_WCh + (radius - distance) * nhat_HS_W;
      const RigidTransformd X_WS(p_WS);
      const Vector3d p_SCs = -radius * (R_WS.inverse() * nhat_HS_W);
      SignedDistancePair<double> expected(sphere_id, half_space_id, p_SCs,
                                          p_HCh, -distance, nhat_HS_W);
      test_data.emplace_back(sphere, half_space, X_WS, X_WH, expected);
      expected.SwapAAndB();
      test_data.emplace_back(half_space, sphere, X_WH, X_WS, expected);
    }
  }

  return test_data;
}

class SignedDistancePairTest
    : public testing::TestWithParam<SignedDistancePairTestData> {
 public:
  SignedDistancePairTest() {
    const auto& data = GetParam();
    fcl_object_A_ = MakeFclObject(*data.a_, data.expected_result_.id_A, false);
    fcl_object_B_ = MakeFclObject(*data.b_, data.expected_result_.id_B, true);
    X_WGs_[data.expected_result_.id_A] = data.X_WA_;
    X_WGs_[data.expected_result_.id_B] = data.X_WB_;
  }

 protected:
  std::unique_ptr<fcl::CollisionObjectd> fcl_object_A_;
  std::unique_ptr<fcl::CollisionObjectd> fcl_object_B_;
  unordered_map<GeometryId, RigidTransformd> X_WGs_;

 public:
  // The tolerance value for determining equivalency between expected and
  // tested results. The underlying algorithms have an empirically-determined,
  // hard-coded tolerance of 1e-14 to account for loss of precision due to
  // rigid transformations and this tolerance reflects that.
  static constexpr double kTolerance = 1e-14;
};

TEST_P(SignedDistancePairTest, SinglePair) {
  const auto& data = GetParam();
  std::vector<SignedDistancePair<double>> witness_pairs;
  shape_distance::CallbackData<double> callback_data{nullptr, &X_WGs_, kInf,
                                                     &witness_pairs};
  callback_data.request = MakeProximityEngineRequest();
  double max_dist = kInf;
  shape_distance::Callback<double>(fcl_object_A_.get(), fcl_object_B_.get(),
                                   &callback_data, max_dist);
  ASSERT_EQ(witness_pairs.size(), 1);
  const auto& result = witness_pairs[0];
  const double tolerance = data.tolerance_ ? data.tolerance_ : kTolerance;

  EXPECT_NEAR(result.distance, data.expected_result_.distance, tolerance)
      << "Incorrect signed distance";

  const bool a_then_b = (result.id_A == data.expected_result_.id_A) &&
                        (result.id_B == data.expected_result_.id_B);
  const bool b_then_a = (result.id_B == data.expected_result_.id_A) &&
                        (result.id_A == data.expected_result_.id_B);
  ASSERT_NE(a_then_b, b_then_a);
  const Vector3d& p_ACa = a_then_b ? result.p_ACa : result.p_BCb;
  const Vector3d& p_BCb = a_then_b ? result.p_BCb : result.p_ACa;

  EXPECT_TRUE(CompareMatrices(p_ACa, data.expected_result_.p_ACa, tolerance))
      << "Incorrect witness point.";
  EXPECT_TRUE(CompareMatrices(p_BCb, data.expected_result_.p_BCb, tolerance))
      << "Incorrect witness point.";

  // Check the invariance that the distance between the two witness points
  // equal the signed distance.
  const Vector3d p_WCb = data.X_WB_ * p_BCb;
  const RigidTransformd X_AW = data.X_WA_.inverse();
  const Vector3d p_ACb = X_AW * p_WCb;
  const double distance_between_witnesses = (p_ACa - p_ACb).norm();
  EXPECT_NEAR(distance_between_witnesses, std::abs(result.distance), tolerance)
      << "Distance between witness points do not equal the signed distance.";
}

INSTANTIATE_TEST_SUITE_P(SphereSphere, SignedDistancePairTest,
                         testing::ValuesIn(GenDistancePairTestSphereSphere()));
INSTANTIATE_TEST_SUITE_P(
    SphereSphereTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereSphereTransform()));
INSTANTIATE_TEST_SUITE_P(
    SphereSphereGimbalLock, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereSphereGimbalLock()));

INSTANTIATE_TEST_SUITE_P(
    SphereSphreNonAligned, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereSphereNonAligned()));
INSTANTIATE_TEST_SUITE_P(
    SphereSphreNonAlignedTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereSphereNonAlignedTransform()));

INSTANTIATE_TEST_SUITE_P(SphereBox, SignedDistancePairTest,
                         testing::ValuesIn(GenDistPairTestSphereBox()));
INSTANTIATE_TEST_SUITE_P(
    SphereBoxTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereBoxTransform()));
INSTANTIATE_TEST_SUITE_P(
    SphereBoxGimbalLock, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereBoxGimbalLock()));

INSTANTIATE_TEST_SUITE_P(BoxSphere, SignedDistancePairTest,
                         testing::ValuesIn(GenDistPairTestBoxSphere()));
INSTANTIATE_TEST_SUITE_P(
    BoxSphereTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestBoxSphereTransform()));

INSTANTIATE_TEST_SUITE_P(SphereBoxBoundary, SignedDistancePairTest,
                         testing::ValuesIn(GenDistPairTestSphereBoxBoundary()));
INSTANTIATE_TEST_SUITE_P(
    SphereBoxBoundaryTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereBoxBoundaryTransform()));

INSTANTIATE_TEST_SUITE_P(SphereCapsule, SignedDistancePairTest,
                         testing::ValuesIn(GenDistPairTestSphereCapsule()));
INSTANTIATE_TEST_SUITE_P(
    SphereCapsuleTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereCapsuleTransform()));

INSTANTIATE_TEST_SUITE_P(SphereCylinder, SignedDistancePairTest,
                         testing::ValuesIn(GenDistPairTestSphereCylinder()));
INSTANTIATE_TEST_SUITE_P(
    SphereCylinderTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereCylinderTransform()));
INSTANTIATE_TEST_SUITE_P(
    SphereCylinderBoundary, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereCylinderBoundary()));
INSTANTIATE_TEST_SUITE_P(
    SphereCylinderBoundaryTransform, SignedDistancePairTest,
    testing::ValuesIn(GenDistPairTestSphereCylinderBoundaryTransform()));

INSTANTIATE_TEST_SUITE_P(HalfspaceSphere, SignedDistancePairTest,
                         testing::ValuesIn(GenDistPairTestHalfspaceSphere()));

// This tests that signed distance queries against a halfspace throw an
// intelligible exception rather than a segfault.
GTEST_TEST(SignedDistancePairError, HalfspaceException) {
  // Note: this doesn't fully test the condition for emitting the error; we
  // have no *real* control over which geometry is the first geometry and
  // second geometry in the pair being tested. However, this test isn't
  // intended to be long-lasting; we want to correct FCL's short-coming soon
  // so that the behavior (and this test) can be removed.

  // We can't use sphere, because sphere-halfspace is supported.
  const GeometryId id_box = GeometryId::get_new_id();
  const GeometryId id_hs = GeometryId::get_new_id();
  std::unique_ptr<fcl::CollisionObjectd> box_obj =
      MakeFclObject(Box{0.5, 0.25, 0.75}, id_box, true);
  std::unique_ptr<fcl::CollisionObjectd> hs_obj =
      MakeFclObject(HalfSpace{}, id_hs, false);

  // NOTE: It's not necessary to put any poses into X_WGs; they never get
  // evaluated due to the error condition.
  unordered_map<GeometryId, RigidTransformd> X_WGs;
  std::vector<SignedDistancePair<double>> pairs;
  shape_distance::CallbackData<double> data{nullptr, &X_WGs, kInf, &pairs};
  data.request = MakeProximityEngineRequest();
  double max_dist = kInf;

  DRAKE_EXPECT_THROWS_MESSAGE(
      shape_distance::Callback<double>(box_obj.get(), hs_obj.get(), &data,
                                       max_dist),
      "Signed distance queries between shapes .* and .* are not supported.*");
}

// Concentric geometries A, B do not have a unique pair of witness points
// Na, Nb. We inherit another test fixture from SignedDistancePairTest, so we
// can define another TEST_P that checks |Na-Nb| = -signed_distance but does
// not check the locations of Na and Nb individually.
class SignedDistancePairConcentricTest : public SignedDistancePairTest {};

TEST_P(SignedDistancePairConcentricTest, DistanceInvariance) {
  const auto& data = GetParam();
  std::vector<SignedDistancePair<double>> witness_pairs;
  shape_distance::CallbackData<double> callback_data{nullptr, &X_WGs_, kInf,
                                                     &witness_pairs};
  callback_data.request = MakeProximityEngineRequest();
  double max_dist = kInf;
  shape_distance::Callback<double>(fcl_object_A_.get(), fcl_object_B_.get(),
                                   &callback_data, max_dist);
  ASSERT_EQ(witness_pairs.size(), 1);
  const auto& result = witness_pairs[0];

  EXPECT_NEAR(result.distance, data.expected_result_.distance, kTolerance)
      << "Incorrect signed distance";

  const bool a_then_b = (result.id_A == data.expected_result_.id_A) &&
                        (result.id_B == data.expected_result_.id_B);
  const bool b_then_a = (result.id_B == data.expected_result_.id_A) &&
                        (result.id_A == data.expected_result_.id_B);
  ASSERT_TRUE(a_then_b ^ b_then_a);
  const Vector3d& p_ACa = a_then_b ? result.p_ACa : result.p_BCb;
  const Vector3d& p_BCb = a_then_b ? result.p_BCb : result.p_ACa;

  // Check the invariance that the distance between the two witness points
  // equal the signed distance.
  const Vector3d p_WCb = data.X_WB_ * p_BCb;
  const RigidTransformd X_AW = data.X_WA_.inverse();
  const Vector3d p_ACb = X_AW * p_WCb;
  const double distance_between_witnesses = (p_ACa - p_ACb).norm();
  EXPECT_NEAR(distance_between_witnesses, -result.distance, kTolerance)
      << "Incorrect distance between witness points.";
}

// Generates one record of test data for two spheres whose centers are at the
// same point.
// @param radius_A specifies the radius of the first sphere A.
// @param radius_B specifies the radius of the second sphere B.
// @param X_WA specifies the pose of A in world.
// @param R_WB specifies the orientation of B in world.
SignedDistancePairTestData GenDistPairTestTwoSpheresConcentricBasic(
    const double radius_A = 1, const double radius_B = 1,
    const RigidTransformd& X_WA = RigidTransformd::Identity(),
    const RotationMatrixd& R_WB = RotationMatrixd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(radius_A);
  auto sphere_B = make_shared<const Sphere>(radius_B);
  const RigidTransformd X_WB(R_WB, X_WA.translation());
  const RigidTransformd X_AW = X_WA.inverse();
  const RigidTransformd X_AB = X_AW * X_WB;

  // Since the two spheres are concentric, we arbitrarily pick the witness
  // point Cb at (radius_B,0,0) in B's frame and calculate the corresponding
  // witness point Ca in A's frame, but the TEST_P will ignore them.
  const Vector3d p_BCb(radius_B, 0, 0);
  const Vector3d p_ACb = X_AB * p_BCb;
  const Vector3d p_ACa = -(radius_A / radius_B) * p_ACb;

  return SignedDistancePairTestData(SignedDistancePairTestData::Make(
      sphere_A, sphere_B, X_WA, X_WB, p_ACa, p_BCb, -radius_A - radius_B));
}

// Generates test data for two spheres A and B whose centers are at the same
// point with varying radii.
// @param X_WA specifies the pose of A in world.
// @param R_WB specifies the orientation of B in world.
std::vector<SignedDistancePairTestData> GenDistPairTestSphereSphereConcentric(
    const RigidTransformd& X_WA = RigidTransformd::Identity(),
    const RotationMatrixd& R_WB = RotationMatrixd::Identity()) {
  std::vector<SignedDistancePairTestData> test_data{
      // Both unit spheres.
      GenDistPairTestTwoSpheresConcentricBasic(1.0, 1.0, X_WA, R_WB),
      // One is smaller than a unit sphere, another is larger than a unit
      // sphere.
      GenDistPairTestTwoSpheresConcentricBasic(0.5, 2.0, X_WA, R_WB),
      // Different sizes. Both spheres are smaller than a unit sphere.
      GenDistPairTestTwoSpheresConcentricBasic(0.3, 0.7, X_WA, R_WB),
      // Same sizes. Both spheres are smaller than a unit sphere.
      GenDistPairTestTwoSpheresConcentricBasic(0.7, 0.7, X_WA, R_WB),
      // Different sizes. Both spheres are larger than a unit sphere.
      GenDistPairTestTwoSpheresConcentricBasic(3.0, 7.0, X_WA, R_WB),
      // Same sizes. Both spheres are larger than a unit sphere.
      GenDistPairTestTwoSpheresConcentricBasic(7.0, 7.0, X_WA, R_WB),
  };
  return test_data;
}

std::vector<SignedDistancePairTestData>
GenDistPairTestSphereSphereConcentricTransform() {
  return GenDistPairTestSphereSphereConcentric(
      RigidTransformd(RollPitchYawd(M_PI_4, M_PI_2, M_PI), Vector3d(1, 2, 3)),
      RotationMatrixd(RollPitchYawd(M_PI, M_PI / 6, M_PI)));
}

// Gimbal lock of the orientation of the sphere at pitch = pi/2.
std::vector<SignedDistancePairTestData>
GenDistPairTestSphereSphereConcentricGimbalLock() {
  return GenDistPairTestSphereSphereConcentric(
      RigidTransformd::Identity(),
      RotationMatrixd(RollPitchYawd(M_PI, M_PI_2, M_PI)));
}

// Generates test data for a sphere and a box with the same centers.
// @param X_WA specifies the pose of the sphere A in world.
// @param R_WB specifies the orientation of the box B in world.
std::vector<SignedDistancePairTestData> GenDistPairTestSphereBoxConcentric(
    const RigidTransformd& X_WA = RigidTransformd::Identity(),
    const RotationMatrixd& R_WB = RotationMatrixd::Identity()) {
  auto sphere_A = make_shared<const Sphere>(2.0);
  auto box_B = make_shared<const Box>(4.0, 8.0, 16.0);
  const double radius_A = sphere_A->radius();
  const Vector3d half_B = box_B->size() / 2;
  const RigidTransformd X_WB(R_WB, X_WA.translation());
  const RigidTransformd X_AW = X_WA.inverse();
  const RigidTransformd X_AB = X_AW * X_WB;

  // Since A and B are concentric, we arbitrarily pick the witness point Cb
  // at (half_B.x, 0, 0) in B's frame and calculate the corresponding witness
  // point Ca in A's frame, but the TEST_P will ignore them.
  const Vector3d p_BCb(half_B(0), 0, 0);
  const Vector3d p_ACb = X_AB * p_BCb;
  const Vector3d p_ACa = -(radius_A / half_B(0)) * p_ACb;

  std::vector<SignedDistancePairTestData> test_data{
      SignedDistancePairTestData::Make(sphere_A, box_B, X_WA, X_WB, p_ACa,
                                       p_BCb, -radius_A - half_B(0))};
  return test_data;
}

std::vector<SignedDistancePairTestData>
GenDistPairTestSphereBoxConcentricTransform() {
  return GenDistPairTestSphereBoxConcentric(
      RigidTransformd(RollPitchYawd(M_PI_4, M_PI_2, M_PI), Vector3d(1, 2, 3)),
      RotationMatrixd(RollPitchYawd(M_PI, M_PI / 6, M_PI)));
}

INSTANTIATE_TEST_SUITE_P(
    SphereSphereConcentric, SignedDistancePairConcentricTest,
    testing::ValuesIn(GenDistPairTestSphereSphereConcentric()));
INSTANTIATE_TEST_SUITE_P(
    SphereSphereConcentricTransform, SignedDistancePairConcentricTest,
    testing::ValuesIn(GenDistPairTestSphereSphereConcentricTransform()));
INSTANTIATE_TEST_SUITE_P(
    SphereSphereConcentricGimbalLock, SignedDistancePairConcentricTest,
    testing::ValuesIn(GenDistPairTestSphereSphereConcentricGimbalLock()));

INSTANTIATE_TEST_SUITE_P(
    SphereBoxConcentric, SignedDistancePairConcentricTest,
    testing::ValuesIn(GenDistPairTestSphereBoxConcentric()));
INSTANTIATE_TEST_SUITE_P(
    SphereBoxConcentricTransform, SignedDistancePairConcentricTest,
    testing::ValuesIn(GenDistPairTestSphereBoxConcentricTransform()));

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
