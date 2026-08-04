#include "drake/geometry/proximity/penetration_as_point_pair_callback.h"

#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity/test/fcl_utilities.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace penetration_as_point_pair {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using fcl::CollisionObjectd;
using fcl::Sphered;
using math::RigidTransform;
using math::RigidTransformd;
using math::RotationMatrix;
using math::RotationMatrixd;
using std::make_shared;
using std::unordered_map;
using std::vector;
using symbolic::Expression;

const double kEps = std::numeric_limits<double>::epsilon();

// These tests represent the main tests of the actual callback. The callback
// has limited responsibility:
//   1. Determine if the pair is filtered.
//   2. If not filtered, exercise some black-box geometric code to measure
//      possible intersection.
//   3. Package the result (if one exists) into a PenetrationAsPointPair with
//      consistent ids and values.
//   4. Always return false to make sure that the broadphase continues
//      traversal.
// The callback is agnostic of the geometry type and relies on the black box's
// correctness for the *values* of the collision data to be correct. Thus, unit
// tests of the callback should not concern themselves with the values to any
// undue extent.
//
// The tests make use of two spheres of the same size, both positioned such that
// their centers are coincident. The individual tests are responsible for
// changing the relative poses.
class PenetrationAsPointPairCallbackTest : public ::testing::Test {
 public:
  PenetrationAsPointPairCallbackTest()
      : ::testing::Test(),
        sphere_A_(make_shared<Sphered>(kRadius)),
        sphere_B_(make_shared<Sphered>(kRadius)),
        box_(make_shared<fcl::Boxd>(box_size_[0], box_size_[1], box_size_[2])),
        cylinder_(
            make_shared<fcl::Cylinderd>(cylinder_size_[0], cylinder_size_[1])),
        halfspace_(
            make_shared<fcl::Halfspaced>(halfspace_normal_, halfspace_offset_)),
        capsule_(
            make_shared<fcl::Capsuled>(capsule_size_[0], capsule_size_[1])),
        id_A_(GeometryId::get_new_id()),
        id_B_(GeometryId::get_new_id()),
        id_box_(GeometryId::get_new_id()),
        id_cylinder_(GeometryId::get_new_id()),
        id_halfspace_(GeometryId::get_new_id()),
        id_capsule_(GeometryId::get_new_id()) {}
  // TODO(DamrongGuoy): add tests for ellipsoid.

 protected:
  void SetUp() override {
    auto encode_data = [this](GeometryId id, CollisionObjectd* shape) {
      const EncodedData data(id, true);
      data.write_to(shape);
      this->collision_filter_.AddGeometry(data.id());
    };
    encode_data(id_A_, &sphere_A_);
    encode_data(id_B_, &sphere_B_);
    encode_data(id_box_, &box_);
    encode_data(id_cylinder_, &cylinder_);
    encode_data(id_halfspace_, &halfspace_);
    encode_data(id_capsule_, &capsule_);
  }

  template <typename T>
  void TestNoCollision() {
    // Move sphere B away from A.
    const RigidTransform<T> X_WA = RigidTransform<T>::Identity();
    const Vector3<T> p_WB = Vector3d(kRadius * 3, 0, 0);
    const RigidTransform<T> X_WB = RigidTransform<T>(p_WB);

    const std::unordered_map<GeometryId, RigidTransform<T>> X_WGs{
        {{id_A_, X_WA}, {id_B_, X_WB}}};
    vector<PenetrationAsPointPair<T>> point_pairs;
    CallbackData<T> callback_data(&collision_filter_, &X_WGs, &point_pairs);
    EXPECT_FALSE(Callback<T>(&sphere_A_, &sphere_B_, &callback_data));
    EXPECT_EQ(point_pairs.size(), 0u);

    EXPECT_FALSE(Callback<T>(&sphere_B_, &sphere_A_, &callback_data));
    EXPECT_EQ(point_pairs.size(), 0u);
  }

  // Confirms that a pair of geometries _in_ collision but not filtered produce
  // expected results. And that the result is expected, regardless of the order
  // of the objects as parameters.
  // And confirms that if the pair is filtered, no collision is reported.
  template <typename T>
  void TestCollisionFilterRespected() {
    // Move sphere B away from origin in an arbitrary direction with an
    // arbitrary rotation, such that it penetrates A to a depth of 0.1 units. We
    // want to make sure the two spheres have a non-trivial transform between
    // their frames and show that regardless of their ordering in the callback,
    // the result is bit identical.
    const double target_depth = 0.1;
    const double center_distance = kRadius * 2 - target_depth;
    Vector3<T> p_WBo;
    if constexpr (std::is_same_v<T, AutoDiffXd>) {
      p_WBo = math::InitializeAutoDiff(
          (Vector3d{1, -2, 3}.normalized() * center_distance));
    } else {
      p_WBo = (Vector3d{1, -2, 3}.normalized() * center_distance);
    }
    const RigidTransform<T> X_WB =
        RigidTransform<T>{RotationMatrix<T>::MakeYRotation(M_PI / 3) *
                              RotationMatrix<T>::MakeZRotation(-M_PI / 7),
                          p_WBo};
    const RigidTransform<T> X_WA = RigidTransform<T>::Identity();
    const std::unordered_map<GeometryId, RigidTransform<T>> X_WGs{
        {{id_A_, X_WA}, {id_B_, X_WB}}};

    // Two executions with the order of the objects reversed -- should produce
    // identical results.
    vector<PenetrationAsPointPair<T>> point_pairs;
    CallbackData<T> callback_data(&collision_filter_, &X_WGs, &point_pairs);
    EXPECT_FALSE(Callback<T>(&sphere_A_, &sphere_B_, &callback_data));
    ASSERT_EQ(point_pairs.size(), 1u);
    const PenetrationAsPointPair<T> first_result = point_pairs[0];
    point_pairs.clear();
    const Eigen::Vector3d p_WCa = ExtractDoubleOrThrow(first_result.p_WCa);
    const Eigen::Vector3d p_WCb = ExtractDoubleOrThrow(first_result.p_WCb);
    const Eigen::Vector3d nhat_BA_W =
        ExtractDoubleOrThrow(first_result.nhat_BA_W);
    const double depth = ExtractDoubleOrThrow(first_result.depth);
    EXPECT_NEAR((p_WCa - p_WCb).norm(), depth, kEps);
    EXPECT_TRUE(CompareMatrices(p_WCb - p_WCa, depth * nhat_BA_W, kEps));

    EXPECT_FALSE(Callback<T>(&sphere_B_, &sphere_A_, &callback_data));
    ASSERT_EQ(point_pairs.size(), 1u);
    const PenetrationAsPointPair<T> second_result = point_pairs[0];
    point_pairs.clear();

    ASSERT_EQ(first_result.id_A, second_result.id_A);
    ASSERT_EQ(first_result.id_B, second_result.id_B);
    ASSERT_NEAR(ExtractDoubleOrThrow(first_result.depth), target_depth, kEps);
    EXPECT_EQ(ExtractDoubleOrThrow(second_result.depth),
              ExtractDoubleOrThrow(first_result.depth));
    ASSERT_TRUE(CompareMatrices(ExtractDoubleOrThrow(first_result.nhat_BA_W),
                                ExtractDoubleOrThrow(second_result.nhat_BA_W)));
    ASSERT_TRUE(CompareMatrices(ExtractDoubleOrThrow(first_result.p_WCa),
                                ExtractDoubleOrThrow(second_result.p_WCa)));
    ASSERT_TRUE(CompareMatrices(ExtractDoubleOrThrow(first_result.p_WCb),
                                ExtractDoubleOrThrow(second_result.p_WCb)));

    // Filter the pair (A, B); we'll put the ids in a set and simply return that
    // set for the extract ids function.
    std::unordered_set<GeometryId> ids{id_A_, id_B_};
    CollisionFilter::ExtractIds extract = [&ids](const GeometrySet&,
                                                 CollisionFilterScope) {
      return ids;
    };
    collision_filter_.Apply(
        CollisionFilterDeclaration().ExcludeWithin(GeometrySet{id_A_, id_B_}),
        extract, false /* is_invariant */);

    EXPECT_FALSE(Callback<T>(&sphere_A_, &sphere_B_, &callback_data));
    EXPECT_EQ(point_pairs.size(), 0u);

    EXPECT_FALSE(Callback<T>(&sphere_B_, &sphere_A_, &callback_data));
    EXPECT_EQ(point_pairs.size(), 0u);
  }

  template <typename T>
  void TestSphereShape(double target_depth, const RigidTransform<T>& X_WB,
                       fcl::CollisionObjectd shape, GeometryId shape_id) {
    // We compute the collision between the shape and the sphere located at the
    // world origin. This shape should in contact with the sphere. If this test
    // is instantiated with T=AutoDiffXd, then this test expect the
    // sphere-to-shape collision is supported for AutoDiffXd. We test that the
    // order of the geometries doesn't matter, namely sphere-to-shape and
    // shape-to-sphere should give the same result.
    const RigidTransform<T> X_WA = RigidTransform<T>::Identity();
    const std::unordered_map<GeometryId, RigidTransform<T>> X_WGs{
        {{id_A_, X_WA}, {shape_id, X_WB}}};

    vector<PenetrationAsPointPair<T>> point_pairs;
    CallbackData<T> callback_data(&collision_filter_, &X_WGs, &point_pairs);
    EXPECT_FALSE(Callback<T>(&sphere_A_, &shape, &callback_data));
    ASSERT_EQ(point_pairs.size(), 1u);
    const PenetrationAsPointPair<T> first_result = point_pairs[0];
    EXPECT_NEAR(ExtractDoubleOrThrow(first_result.depth), target_depth, kEps);
    EXPECT_NEAR(ExtractDoubleOrThrow((first_result.p_WCb - first_result.p_WCa)
                                         .dot(first_result.nhat_BA_W)),
                target_depth, kEps);
    point_pairs.clear();

    // Now reverse the order of the geometries.
    EXPECT_FALSE(Callback<T>(&shape, &sphere_A_, &callback_data));
    ASSERT_EQ(point_pairs.size(), 1u);
    const PenetrationAsPointPair<T> second_result = point_pairs[0];
    EXPECT_EQ(ExtractDoubleOrThrow(second_result.depth),
              ExtractDoubleOrThrow(first_result.depth));
    EXPECT_TRUE(CompareMatrices(ExtractDoubleOrThrow(first_result.p_WCa),
                                ExtractDoubleOrThrow(second_result.p_WCa)));
    EXPECT_TRUE(CompareMatrices(ExtractDoubleOrThrow(first_result.p_WCb),
                                ExtractDoubleOrThrow(second_result.p_WCb)));
    EXPECT_TRUE(CompareMatrices(ExtractDoubleOrThrow(first_result.nhat_BA_W),
                                ExtractDoubleOrThrow(second_result.nhat_BA_W)));

    if constexpr (std::is_same_v<T, AutoDiffXd>) {
      // Make sure that callback with T=AutoDiffXd and T=double produces the
      // same result within numerical noise.
      const RigidTransform<double> X_WA_double =
          RigidTransform<double>::Identity();
      const RigidTransform<double> X_WB_double(
          RotationMatrix<double>(math::ExtractValue(X_WB.rotation().matrix())),
          math::ExtractValue(X_WB.translation()));
      const std::unordered_map<GeometryId, RigidTransform<double>> X_WGs_double{
          {{id_A_, X_WA_double}, {shape_id, X_WB_double}}};

      vector<PenetrationAsPointPair<double>> point_pairs_double;
      CallbackData<double> callback_data_double(
          &collision_filter_, &X_WGs_double, &point_pairs_double);
      EXPECT_FALSE(Callback<double>(&sphere_A_, &shape, &callback_data_double));
      ASSERT_EQ(point_pairs_double.size(), 1u);
      EXPECT_NEAR(ExtractDoubleOrThrow(first_result.depth),
                  point_pairs_double[0].depth, 2 * kEps);
      EXPECT_TRUE(CompareMatrices(math::ExtractValue(first_result.p_WCa),
                                  point_pairs_double[0].p_WCa));
      EXPECT_TRUE(CompareMatrices(math::ExtractValue(first_result.p_WCb),
                                  point_pairs_double[0].p_WCb, kEps));
      EXPECT_TRUE(CompareMatrices(math::ExtractValue(first_result.nhat_BA_W),
                                  point_pairs_double[0].nhat_BA_W));
    }
  }

  template <typename T>
  RigidTransform<T> CalcBoxPoseInSphereBox(double target_depth) {
    DRAKE_DEMAND(target_depth > 0 && target_depth < kRadius);
    const double center_distance = kRadius + box_size_[0] / 2 - target_depth;
    Vector3<T> p_WBo;
    const RotationMatrix<T> R_WB = RotationMatrix<T>::MakeZRotation(0.2);
    if constexpr (std::is_same_v<T, AutoDiffXd>) {
      p_WBo =
          R_WB * math::InitializeAutoDiff(Vector3d::UnitX() * center_distance);
    } else {
      p_WBo = R_WB * Vector3d::UnitX() * center_distance;
    }
    const RigidTransform<T> X_WB = RigidTransform<T>{R_WB, p_WBo};
    return X_WB;
  }

  template <typename T>
  RigidTransform<T> CalcCylinderPoseInSphereCylinder(double target_depth) {
    DRAKE_DEMAND(target_depth > 0 && target_depth < cylinder_size_[0] &&
                 target_depth < kRadius);
    const double center_distance = kRadius + cylinder_size_[0] - target_depth;
    Vector3<T> p_WBo;
    const RotationMatrix<T> R_WB = RotationMatrix<T>::MakeZRotation(0.2);
    if constexpr (std::is_same_v<T, AutoDiffXd>) {
      p_WBo =
          R_WB * math::InitializeAutoDiff(Vector3d::UnitX() * center_distance);
    } else {
      p_WBo = R_WB * Vector3d::UnitX() * center_distance;
    }
    const RigidTransform<T> X_WB = RigidTransform<T>{R_WB, p_WBo};
    return X_WB;
  }

  template <typename T>
  std::pair<RigidTransform<T>, double> CalcHalfspacePoseInSphereHalfspace() {
    const RotationMatrix<T> R_WB = RotationMatrix<T>::MakeZRotation(0.1) *
                                   RotationMatrix<T>::MakeXRotation(0.5);
    Vector3<T> p_WBo;
    if constexpr (std::is_same_v<T, AutoDiffXd>) {
      p_WBo = math::InitializeAutoDiff(Eigen::Vector3d(0.5, -0.2, 0.9));
    } else {
      p_WBo = Eigen::Vector3d(0.5, -0.2, 0.9);
    }
    const double target_depth =
        halfspace_offset_ +
        halfspace_normal_.dot(ExtractDoubleOrThrow(R_WB.inverse() * p_WBo)) +
        kRadius;
    const RigidTransform<T> X_WB = RigidTransform<T>{R_WB, p_WBo};
    return std::make_pair(X_WB, target_depth);
  }

  template <typename T>
  RigidTransform<T> CalcCapsulePoseInSphereCapsule(double target_depth) {
    DRAKE_DEMAND(target_depth > 0 && target_depth < kRadius &&
                 target_depth < capsule_size_[0]);
    const double center_distance = kRadius + capsule_size_[0] - target_depth;
    Vector3<T> p_WBo;
    const RotationMatrix<T> R_WB = RotationMatrix<T>::MakeZRotation(0.2);
    if constexpr (std::is_same_v<T, AutoDiffXd>) {
      p_WBo =
          R_WB * math::InitializeAutoDiff(Vector3d::UnitX() * center_distance);
    } else {
      p_WBo = R_WB * Vector3d::UnitX() * center_distance;
    }
    const RigidTransform<T> X_WB = RigidTransform<T>{R_WB, p_WBo};
    return X_WB;
  }

  template <typename T>
  void UnsupportedGeometry(fcl::CollisionObjectd shape1,
                           fcl::CollisionObjectd shape2, GeometryId id1,
                           GeometryId id2) {
    const std::unordered_map<GeometryId, RigidTransform<T>> X_WGs{
        {{id1, RigidTransform<T>::Identity()},
         {id2, RigidTransform<T>::Identity()}}};
    vector<PenetrationAsPointPair<T>> point_pairs;
    CallbackData<T> callback_data(&collision_filter_, &X_WGs, &point_pairs);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Callback<T>(&shape1, &shape2, &callback_data),
        "Penetration queries between shapes .* and .* are not supported for "
        "scalar type .*");
  }

  static const double kRadius;
  const std::array<double, 3> box_size_{0.1, 0.2, 0.3};
  const std::array<double, 2> cylinder_size_{1.2, 2.1};
  const Eigen::Vector3d halfspace_normal_{0, 0, 1};
  const double halfspace_offset_{0.};
  const std::array<double, 2> capsule_size_{1.3, 2.3};
  CollisionObjectd sphere_A_;
  CollisionObjectd sphere_B_;
  CollisionObjectd box_;
  CollisionObjectd cylinder_;
  CollisionObjectd halfspace_;
  CollisionObjectd capsule_;
  GeometryId id_A_;
  GeometryId id_B_;
  GeometryId id_box_;
  GeometryId id_cylinder_;
  GeometryId id_halfspace_;
  GeometryId id_capsule_;
  CollisionFilter collision_filter_;
};

// TODO(SeanCurtis-TRI): Make this static constexpr when our gcc version doesn't
//  cry in debug builds.
const double PenetrationAsPointPairCallbackTest::kRadius = 0.5;

// Confirms that a pair of geometries that are demonstrably not in collision and
// are not filtered produce no results.
TEST_F(PenetrationAsPointPairCallbackTest, NonCollisionDouble) {
  TestNoCollision<double>();
}

TEST_F(PenetrationAsPointPairCallbackTest, NonCollisionAutoDiffXd) {
  TestNoCollision<AutoDiffXd>();
}

TEST_F(PenetrationAsPointPairCallbackTest, CollisionFilterRespectedDouble) {
  TestCollisionFilterRespected<double>();
}

TEST_F(PenetrationAsPointPairCallbackTest, CollisionFilterRespectedAutoDiffXd) {
  TestCollisionFilterRespected<AutoDiffXd>();
}

TEST_F(PenetrationAsPointPairCallbackTest, TestGradient) {
  // We can compute the gradient of the penetration result w.r.t the position of
  // the sphere by hand, and then compare that gradient against autodiff result.
  const double target_depth = 0.1;
  const double center_distance = kRadius * 2 - target_depth;
  const Eigen::Vector3d p_WBo_val =
      Vector3d{1, -2, 3}.normalized() * center_distance;
  const Vector3<AutoDiffXd> p_WBo = math::InitializeAutoDiff(p_WBo_val);
  std::unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs;
  const RigidTransform<AutoDiffXd> X_WB = RigidTransform<AutoDiffXd>{
      RotationMatrix<AutoDiffXd>::MakeYRotation(M_PI / 3) *
          RotationMatrix<AutoDiffXd>::MakeZRotation(-M_PI / 7),
      p_WBo};
  const RigidTransform<AutoDiffXd> X_WA =
      RigidTransform<AutoDiffXd>::Identity();
  X_WGs.emplace(id_A_, X_WA);
  X_WGs.emplace(id_B_, X_WB);
  vector<PenetrationAsPointPair<AutoDiffXd>> point_pairs;
  CallbackData<AutoDiffXd> callback_data(&collision_filter_, &X_WGs,
                                         &point_pairs);
  EXPECT_FALSE(Callback<AutoDiffXd>(&sphere_A_, &sphere_B_, &callback_data));
  ASSERT_EQ(point_pairs.size(), 1u);
  const Eigen::Vector3d ddepth_dp_WBo = -p_WBo_val / p_WBo_val.norm();
  const PenetrationAsPointPair<AutoDiffXd> result = point_pairs[0];
  EXPECT_TRUE(CompareMatrices(result.depth.derivatives(), ddepth_dp_WBo, kEps));
  // ∂ (x/|x|) /∂ x where x = p_WBo
  const Eigen::Matrix3d dp_WBo_normalized_dp_WBo =
      (p_WBo_val.squaredNorm() * Eigen::Matrix3d::Identity() -
       p_WBo_val * p_WBo_val.transpose()) /
      std::pow(p_WBo_val.norm(), 3);
  // nhat_BA_W = -p_WBo / |p_WBo|, hence ∂nhat_BA_W /∂p_WBo = -∂(p_WBo/|p_WBo|)
  // / ∂p_WBo
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(result.nhat_BA_W),
                              -dp_WBo_normalized_dp_WBo, 2 * kEps));
  // p_WCa = radius * p_WBo / |p_WBo|.
  const Eigen::Matrix3d dp_WCa_dp_WBo = dp_WBo_normalized_dp_WBo * kRadius;
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(result.p_WCa),
                              dp_WCa_dp_WBo, kEps));
  // p_WCb = p_WBo - radius * p_WBo / |p_WBo|.
  const Eigen::Matrix3d dp_WCb_dp_WBo =
      Eigen::Matrix3d::Identity() - dp_WBo_normalized_dp_WBo * kRadius;
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(result.p_WCb),
                              dp_WCb_dp_WBo, kEps));
}

TEST_F(PenetrationAsPointPairCallbackTest, SphereBoxDouble) {
  const double target_depth = 0.1;
  const RigidTransform<double> X_WB =
      CalcBoxPoseInSphereBox<double>(target_depth);
  TestSphereShape(target_depth, X_WB, box_, id_box_);
}

TEST_F(PenetrationAsPointPairCallbackTest, SphereBoxAutoDiffXd) {
  const double target_depth = 0.1;
  const RigidTransform<AutoDiffXd> X_WB =
      CalcBoxPoseInSphereBox<AutoDiffXd>(target_depth);
  TestSphereShape(target_depth, X_WB, box_, id_box_);
}

TEST_F(PenetrationAsPointPairCallbackTest, SphereCylinderDouble) {
  const double target_depth = 0.1;
  const RigidTransform<double> X_WB =
      CalcCylinderPoseInSphereCylinder<double>(target_depth);
  TestSphereShape(target_depth, X_WB, cylinder_, id_cylinder_);
}

TEST_F(PenetrationAsPointPairCallbackTest, SphereCylinderAutoDiffXd) {
  const double target_depth = 0.1;
  const RigidTransform<AutoDiffXd> X_WB =
      CalcCylinderPoseInSphereCylinder<AutoDiffXd>(target_depth);
  TestSphereShape(target_depth, X_WB, cylinder_, id_cylinder_);
}

TEST_F(PenetrationAsPointPairCallbackTest, SphereHalfspaceDouble) {
  auto [X_WB, target_depth] = CalcHalfspacePoseInSphereHalfspace<double>();
  TestSphereShape(target_depth, X_WB, halfspace_, id_halfspace_);
}

TEST_F(PenetrationAsPointPairCallbackTest, SphereHalfspaceAutoDiffXd) {
  auto [X_WB, target_depth] = CalcHalfspacePoseInSphereHalfspace<AutoDiffXd>();
  TestSphereShape(target_depth, X_WB, halfspace_, id_halfspace_);
}

TEST_F(PenetrationAsPointPairCallbackTest, SphereCapsuleDouble) {
  const double target_depth = 0.1;
  const RigidTransform<double> X_WB =
      CalcCapsulePoseInSphereCapsule<double>(target_depth);
  TestSphereShape(target_depth, X_WB, capsule_, id_capsule_);
}

TEST_F(PenetrationAsPointPairCallbackTest, SphereCapsuleAutoDiffXd) {
  const double target_depth = 0.1;
  const RigidTransform<AutoDiffXd> X_WB =
      CalcCapsulePoseInSphereCapsule<AutoDiffXd>(target_depth);
  TestSphereShape(target_depth, X_WB, capsule_, id_capsule_);
}

TEST_F(PenetrationAsPointPairCallbackTest, UnsupportedAutoDiffXd) {
  // We don't support penetration query between overlapping box-cylinder with
  // AutoDiffXd yet.
  std::vector<std::pair<fcl::CollisionObjectd, GeometryId>>
      unsupported_geometries;
  unsupported_geometries.emplace_back(box_, id_box_);
  unsupported_geometries.emplace_back(cylinder_, id_cylinder_);
  unsupported_geometries.emplace_back(halfspace_, id_halfspace_);
  unsupported_geometries.emplace_back(capsule_, id_capsule_);
  for (int i = 0; i < static_cast<int>(unsupported_geometries.size()); ++i) {
    for (int j = 0; j < static_cast<int>(unsupported_geometries.size()); ++j) {
      if (i != j) {
        UnsupportedGeometry<AutoDiffXd>(
            unsupported_geometries[i].first, unsupported_geometries[j].first,
            unsupported_geometries[i].second, unsupported_geometries[j].second);
      }
    }
  }
}

TEST_F(PenetrationAsPointPairCallbackTest, UnsupportedExpression) {
  // We don't support penetration queries between any shapes for Expression.
  std::vector<std::pair<fcl::CollisionObjectd, GeometryId>>
      unsupported_geometries;
  unsupported_geometries.emplace_back(sphere_A_, id_A_);
  unsupported_geometries.emplace_back(box_, id_box_);
  unsupported_geometries.emplace_back(cylinder_, id_cylinder_);
  unsupported_geometries.emplace_back(halfspace_, id_halfspace_);
  unsupported_geometries.emplace_back(capsule_, id_capsule_);
  for (int i = 0; i < static_cast<int>(unsupported_geometries.size()); ++i) {
    for (int j = 0; j < static_cast<int>(unsupported_geometries.size()); ++j) {
      if (i != j) {
        UnsupportedGeometry<Expression>(
            unsupported_geometries[i].first, unsupported_geometries[j].first,
            unsupported_geometries[i].second, unsupported_geometries[j].second);
      }
    }
  }
}

// HalfSpace-HalfSpace should be unsupported for *all* scalars. Half spaces
// are either in a non-colliding configuration or their penetration has infinite
// depth. It is not a useful query to answer.
TEST_F(PenetrationAsPointPairCallbackTest, UnsupportedHalfSpaceHalfSpace) {
  // Create a second half space.
  CollisionObjectd halfspace2(
      make_shared<fcl::Halfspaced>(Vector3d{1, 0, 0}, 0));
  const GeometryId hs2_id = GeometryId::get_new_id();
  const EncodedData data(hs2_id, true);
  data.write_to(&halfspace2);
  this->collision_filter_.AddGeometry(data.id());
  UnsupportedGeometry<double>(this->halfspace_, halfspace2, this->id_halfspace_,
                              hs2_id);
  UnsupportedGeometry<AutoDiffXd>(this->halfspace_, halfspace2,
                                  this->id_halfspace_, hs2_id);
}

// This is a one-off test. Exposed in issue #10577. A point penetration pair
// was returned for a zero-depth contact. This reproduces the geometry that
// manifested the error. The reproduction isn't *exact*; it's been reduced to
// a simpler configuration. Specifically, the important characteristics are:
//   - both box and cylinder are ill aspected (one dimension is several orders
//     of magnitude smaller than the other two),
//   - the cylinder is placed away from the center of the box face,
//   - the box is rotated 90 degrees around it's z-axis -- note swapping box
//     dimensions with an identity rotation did *not* produce equivalent
//     results, and
//   - FCL uses GJK/EPA to solve box-cylinder collision (this is beyond control
//     of this test).
//
// Libccd upgraded how it handles degenerate simplices. The upshot of that is
// FCL would still return the same penetration depth, but instead of returning
// a gibberish normal, it returns a zero vector. We want to make sure we don't
// report zero-penetration as penetration, even in these numerically,
// ill-conditioned scenarios. So, we address it up to a tolerance.
TEST_F(PenetrationAsPointPairCallbackTest, Issue10577Regression_Osculation) {
  GeometryId id_A = GeometryId::get_new_id();
  GeometryId id_B = GeometryId::get_new_id();

  // Original translations were p_WA = (-0.145, -0.63, 0.2425) and
  // p_WB = (0, -0.6, 0.251), respectively.
  RigidTransformd X_WA(Eigen::AngleAxisd{M_PI_2, Vector3d::UnitZ()},
                       Vector3d{-0.25, 0, 0});
  RigidTransformd X_WB(Vector3d{0, 0, 0.0085});
  std::unique_ptr<CollisionObjectd> box =
      MakeFclObject(Box(0.49, 0.63, 0.015), id_A, /* is_dynamic= */ true, X_WA);
  std::unique_ptr<CollisionObjectd> cylinder =
      MakeFclObject(Cylinder(0.08, 0.002), id_B, /* is_dynamic= */ true, X_WB);
  const unordered_map<GeometryId, RigidTransformd> X_WG{{id_A, X_WA},
                                                        {id_B, X_WB}};

  vector<PenetrationAsPointPair<double>> pairs;
  CallbackData<double> callback_data(&collision_filter_, &X_WG, &pairs);
  Callback<double>(box.get(), cylinder.get(), &callback_data);
  EXPECT_EQ(pairs.size(), 0);
}

// Robust Box-Primitive tests. Tests collision of the box with other primitives
// in a uniform framework. These tests parallel tests located in fcl.
//
// All of the tests below here are using the callback to exercise the black box.
// They exist because of FCL; FCL's unit tests were sporadic at best and these
// tests revealed errors/properties of FCL that weren't otherwise apparent.
// Ultimately, these tests don't belong here. But they can be re-used when we
// replace FCL with Drake's own implementations.
//
// This performs a very specific test. It collides a rotated box with a
// surface that is tangent to the z = 0 plane. The box is a cube with unit size.
// The goal is to transform the box such that:
//   1. the corner of the box C, located at p_BoC_B = (-0.5, -0.5, -0.5),
//      transformed by the box's pose X_WB, ends up at the position
//      p_WC = (0, 0, -kDepth), i.e., p_WC = X_WB * p_BoC_B, and
//   2. all other corners are transformed to lie above the z = 0 plane.
//
// In this configuration, the corner C becomes the unique point of deepest
// penetration in the interior of the half space.
//
// It is approximately *this* picture
//        ┆  ╱╲
//        ┆ ╱  ╲
//        ┆╱    ╲
//       ╱┆╲    ╱
//      ╱ ┆ ╲  ╱
//     ╱  ┆  ╲╱
//     ╲  ┆  ╱
//      ╲ ┆ ╱
//  _____╲┆╱_____    ╱____ With small penetration depth of d
//  ░░░░░░┆░░░░░░    ╲
//  ░░░░░░┆░░░░░░
//  ░░░Tangent░░░
//  ░░░░shape░░░░
//  ░░interior░░░
//  ░░░░░░┆░░░░░░
//
// We can use this against various *convex* shapes to determine uniformity of
// behavior. As long as the convex tangent shape *touches* the z = 0 plane at
// (0, 0, 0), and the shape is *large* compared to the penetration depth, then
// we should get a fixed, known contact. Specifically, if we assume the tangent
// shape is A and the box is B, then
//   - the normal is (0, 0, -1) from box into the plane,
//   - the penetration depth is the specified depth used to configure the
//     position of the box, and
//   - the contact position is (0, 0, -depth / 2).
//
// Every convex shape type can be used as the tangent shape as follows:
//   - plane: simply define the z = 0 plane.
//   - box: define a box whose top face lies on the z = 0 and encloses the
//     origin.
//   - sphere: place the sphere at (0, 0 -radius).
//   - cylinder: There are two valid configurations (radius & length >> depth).
//     - Place a "standing" cylinder at (0, 0, -length/2).
//     - Rotate the cylinder so that its length axis is parallel with the z = 0
//       plane and then displace downward (0, 0, -radius). Described as "prone".
//   - capsule: There are two valid configurations (radius & length >> depth).
//     - Place a "standing" capsule at (0, 0, -length/2 - radius).
//     - Rotate the capsule so that its length axis is parallel with the z = 0
//       plane and then displace downward (0, 0, -radius). Described as "prone".
//
// Note: there are an infinite number of orientations that satisfy the
// configuration described above. They should *all* provide the same collision
// results. We provide two representative orientations to illustrate issues
// with the underlying functionality -- the *quality* of the answer depends on
// the orientation of the box. When the problem listed in Drake issue 7656 is
// resolved, all tests should resolve to the same answer with the same tight
// precision.
// TODO(SeanCurtis-TRI): Add other shapes as they become available.
class BoxPenetrationTest : public ::testing::Test {
 protected:
  void SetUp() {
    // Confirm all of the constants are consistent (in case someone tweaks them
    // later). In this case, tangent geometry must be much larger than the
    // depth.
    EXPECT_GT(kRadius, kDepth * 100);
    EXPECT_GT(kLength, kDepth * 100);

    // Confirm that the poses of the box satisfy the conditions described above.
    for (auto func : {X_WB_1, X_WB_2}) {
      const math::RigidTransformd X_WB = func(p_BoC_B_, p_WC_);
      // Confirm that p_BoC_B transforms to p_WC.
      const Vector3d p_WC_test = X_WB * p_BoC_B_;
      EXPECT_TRUE(CompareMatrices(p_WC_test, p_WC_, 1e-15,
                                  MatrixCompareType::absolute));

      // Confirm that *all* other points map to values where z > 0.
      for (double x : {-0.5, 0.5}) {
        for (double y : {-0.5, 0.5}) {
          for (double z : {-0.5, 0.5}) {
            const Vector3d p_BoC_B{x, y, z};
            if (p_BoC_B.isApprox(p_BoC_B_)) continue;
            const Vector3d p_WC = X_WB * p_BoC_B;
            EXPECT_GT(p_WC(2), 0);
          }
        }
      }
    }

    // Configure the expected penetration characterization.
    expected_penetration_.p_WCa << 0, 0, 0;       // Tangent plane
    expected_penetration_.p_WCb = p_WC_;          // Cube
    expected_penetration_.nhat_BA_W << 0, 0, -1;  // From cube into plane
    expected_penetration_.depth = kDepth;
    // NOTE: The ids are set by the individual calling tests.
  }

  enum TangentShape {
    TangentPlane,
    TangentSphere,
    TangentBox,
    TangentStandingCylinder,
    TangentProneCylinder,
    TangentConvex,
    TangentStandingCapsule,
    TangentProneCapsule
  };

  // The test that produces *bad* results based on the box orientation. Not
  // called "bad" because when FCL is fixed, they'll both be good.
  void TestCollision1(TangentShape shape_type, double tolerance) {
    TestCollision(shape_type, tolerance, X_WB_1(p_BoC_B_, p_WC_));
  }

  // The test that produces *good* results based on the box orientation. Not
  // called "good" because when FCL is fixed, they'll both be good.
  void TestCollision2(TangentShape shape_type, double tolerance) {
    TestCollision(shape_type, tolerance, X_WB_2(p_BoC_B_, p_WC_));
  }

 private:
  // Perform the collision test against the indicated shape and confirm the
  // results to the given tolerance.
  void TestCollision(TangentShape shape_type, double tolerance,
                     const math::RigidTransformd& X_WB) {
    const GeometryId tangent_id = GeometryId::get_new_id();
    const RigidTransformd X_WA = shape_pose(shape_type);
    std::unique_ptr<CollisionObjectd> tangent_object = MakeFclObject(
        shape(shape_type), tangent_id, /* is_dynamic= */ true, X_WA);

    const GeometryId box_id = GeometryId::get_new_id();
    std::unique_ptr<CollisionObjectd> box_object =
        MakeFclObject(box_, box_id, /* is_dynamic= */ true, X_WB);

    // Note: we need a CollisionFilter for the callback data, but we don't need
    // to populate it, because we're not actually filtering anything.
    CollisionFilter collision_filter;
    const unordered_map<GeometryId, RigidTransformd> X_WGs{{tangent_id, X_WA},
                                                           {box_id, X_WB}};
    vector<PenetrationAsPointPair<double>> results;
    CallbackData<double> callback_data(&collision_filter, &X_WGs, &results);
    Callback<double>(tangent_object.get(), box_object.get(), &callback_data);

    ASSERT_EQ(results.size(), 1u)
        << "Against tangent " << shape_name(shape_type);

    const PenetrationAsPointPair<double>& contact = results[0];
    Vector3d normal;
    Vector3d p_Ac;
    Vector3d p_Bc;
    if (contact.id_A == tangent_id && contact.id_B == box_id) {
      // The documented encoding of expected_penetration_.
      normal = expected_penetration_.nhat_BA_W;
      p_Ac = expected_penetration_.p_WCa;
      p_Bc = expected_penetration_.p_WCb;
    } else if (contact.id_A == box_id && contact.id_B == tangent_id) {
      // The reversed encoding of expected_penetration_.
      normal = -expected_penetration_.nhat_BA_W;
      p_Ac = expected_penetration_.p_WCb;
      p_Bc = expected_penetration_.p_WCa;
    } else {
      GTEST_FAIL() << fmt::format(
          "Wrong geometry ids reported in contact for tangent {}. Expected {} "
          "and {}. Got {} and {}",
          shape_name(shape_type), tangent_id, box_id, contact.id_A,
          contact.id_B);
    }
    EXPECT_TRUE(CompareMatrices(contact.nhat_BA_W, normal, tolerance))
        << "Against tangent " << shape_name(shape_type);
    EXPECT_TRUE(CompareMatrices(contact.p_WCa, p_Ac, tolerance))
        << "Against tangent " << shape_name(shape_type);
    EXPECT_TRUE(CompareMatrices(contact.p_WCb, p_Bc, tolerance))
        << "Against tangent " << shape_name(shape_type);
    EXPECT_NEAR(contact.depth, expected_penetration_.depth, tolerance)
        << "Against tangent " << shape_name(shape_type);
  }

  // The expected collision result -- assumes that A is the tangent object and
  // B is the colliding box.
  PenetrationAsPointPair<double> expected_penetration_;

  // Produces the X_WB that produces high-quality answers.
  static math::RigidTransformd X_WB_2(const Vector3d& p_BoC_B,
                                      const Vector3d& p_WC) {
    // Compute the pose of the colliding box.
    // a. Orient the box so that the corner p_BoC_B = (-0.5, -0.5, -0.5) lies in
    //    the most -z extent. With only rotation, p_BoC_B != p_WC.
    const math::RotationMatrixd R_WB(
        AngleAxisd(std::atan(M_SQRT2), Vector3d(M_SQRT1_2, -M_SQRT1_2, 0)));

    // b. Translate it so that the rotated corner p_BoC_W lies at
    //    (0, 0, -d).
    const Vector3d p_BoC_W = R_WB * p_BoC_B;
    const Vector3d p_WB = p_WC - p_BoC_W;
    return math::RigidTransformd(R_WB, p_WB);
  }

  // Produces the X_WB that produces low-quality answers.
  static math::RigidTransformd X_WB_1(const Vector3d& p_BoC_B,
                                      const Vector3d& p_WC) {
    // Compute the pose of the colliding box.
    // a. Orient the box so that the corner p_BoC_B = (-0.5, -0.5, -0.5) lies in
    //    the most -z extent. With only rotation, p_BoC_B != p_WC.
    const math::RotationMatrixd R_WB(AngleAxisd(-M_PI_4, Vector3d::UnitY()) *
                                     AngleAxisd(M_PI_4, Vector3d::UnitX()));

    // b. Translate it so that the rotated corner p_BoC_W lies at
    //    (0, 0, -d).
    const Vector3d p_BoC_W = R_WB * p_BoC_B;
    const Vector3d p_WB = p_WC - p_BoC_W;
    return math::RigidTransformd(R_WB, p_WB);
  }

  // Map enumeration to string for error messages.
  static const char* shape_name(TangentShape shape) {
    switch (shape) {
      case TangentPlane:
        return "plane";
      case TangentSphere:
        return "sphere";
      case TangentBox:
        return "box";
      case TangentStandingCylinder:
        return "standing cylinder";
      case TangentProneCylinder:
        return "prone cylinder";
      case TangentConvex:
        return "convex";
      case TangentStandingCapsule:
        return "standing capsule";
      case TangentProneCapsule:
        return "prone capsule";
    }
    return "undefined shape";
  }

  // Map enumeration to the configured shapes.
  const Shape& shape(TangentShape shape) {
    switch (shape) {
      case TangentPlane:
        return tangent_plane_;
      case TangentSphere:
        return tangent_sphere_;
      case TangentBox:
        return tangent_box_;
      case TangentStandingCylinder:
      case TangentProneCylinder:
        return tangent_cylinder_;
      case TangentConvex:
        return tangent_convex_;
      case TangentStandingCapsule:
      case TangentProneCapsule:
        return tangent_capsule_;
    }
    // GCC considers this function ill-formed - no apparent return value. This
    // exception alleviates its concern.
    throw std::logic_error(
        "Trying to acquire shape for unknown shape enumerated value: " +
        std::to_string(shape));
  }

  // Map enumeration to tangent pose.
  RigidTransformd shape_pose(TangentShape shape) {
    RigidTransformd pose = RigidTransformd::Identity();
    switch (shape) {
      case TangentPlane:
        break;  // leave it at the identity
      case TangentSphere:
        pose.set_translation({0, 0, -kRadius});
        break;
      case TangentBox:
      // The tangent convex is a cube of the same size as the tangent box.
      // That is why we give them the same pose.
      case TangentConvex:
      case TangentStandingCylinder:
        pose.set_translation({0, 0, -kLength / 2});
        break;
      case TangentStandingCapsule:
        pose.set_translation({0, 0, -kLength / 2 - kRadius});
        break;
      case TangentProneCylinder:
      case TangentProneCapsule:
        pose = RigidTransformd(AngleAxisd{M_PI_2, Vector3d::UnitX()},
                               Vector3d{0, 0, -kRadius});
        break;
    }
    return pose;
  }

  // Test constants. Geometric measures must be much larger than depth. The test
  // enforces a ratio of at least 100. Using these in a GTEST precludes the
  // possibility of being constexpr initialized; GTEST takes references.
  static const double kDepth;
  static const double kRadius;
  static const double kLength;

  // The various geometries used in the collision test.
  const Box box_{1, 1, 1};
  const Sphere tangent_sphere_{kRadius};
  const Box tangent_box_{kLength, kLength, kLength};
  const HalfSpace tangent_plane_;  // Default construct the z = 0 plane.
  const Cylinder tangent_cylinder_{kRadius, kLength};
  // We scale the convex shape by 5.0 to match the tangent_box_ of size 10.0.
  // The file "quad_cube.obj" contains the cube of size 2.0.
  const Convex tangent_convex_{
      drake::FindResourceOrThrow("drake/geometry/test/quad_cube.obj"), 5.0};
  const Capsule tangent_capsule_{kRadius, kLength};

  const Vector3d p_WC_{0, 0, -kDepth};
  const Vector3d p_BoC_B_{-0.5, -0.5, -0.5};
};

// See documentation. All geometry constants must be >= kDepth * 100.
const double BoxPenetrationTest::kDepth = 1e-3;
const double BoxPenetrationTest::kRadius = 1;
const double BoxPenetrationTest::kLength = 10;

TEST_F(BoxPenetrationTest, TangentPlane1) {
  TestCollision1(TangentPlane, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentPlane2) {
  TestCollision2(TangentPlane, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentBox1) {
  TestCollision1(TangentBox, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentBox2) {
  TestCollision2(TangentBox, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentSphere1) {
  // TODO(SeanCurtis-TRI): There are underlying fcl issues that prevent the
  // collision result from being more precise. Largely due to normal
  // calculation. See related Drake issue 7656.
  TestCollision1(TangentSphere, 1e-1);
}

TEST_F(BoxPenetrationTest, TangentSphere2) {
  TestCollision2(TangentSphere, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentStandingCylinder1) {
  // TODO(SeanCurtis-TRI): There are underlying fcl issues that prevent the
  // collision result from being more precise. See related Drake issue 7656.
  TestCollision1(TangentStandingCylinder, 1e-3);
}

TEST_F(BoxPenetrationTest, TangentStandingCylinder2) {
  TestCollision2(TangentStandingCylinder, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentProneCylinder1) {
  // TODO(SeanCurtis-TRI): There are underlying fcl issues that prevent the
  // collision result from being more precise. Largely due to normal
  // calculation. See related Drake issue 7656.
  TestCollision1(TangentProneCylinder, 1e-1);
}

TEST_F(BoxPenetrationTest, TangentProneCylinder2) {
  // TODO(SeanCurtis-TRI): There are underlying fcl issues that prevent the
  // collision result from being more precise. In this case, the largest error
  // is in the position of the contact points. See related Drake issue 7656.
  TestCollision2(TangentProneCylinder, 1e-4);
}

TEST_F(BoxPenetrationTest, TangentConvex1) {
  // TODO(DamrongGuoy): We should check why we cannot use a smaller tolerance.
  TestCollision1(TangentConvex, 1e-3);
}

TEST_F(BoxPenetrationTest, TangentConvex2) {
  // TODO(DamrongGuoy): We should check why we cannot use a smaller tolerance.
  TestCollision2(TangentConvex, 1e-3);
}

TEST_F(BoxPenetrationTest, TangentStandingCapsule1) {
  // TODO(tehbelinda): We should check why we cannot use a smaller tolerance.
  TestCollision1(TangentStandingCapsule, 1e-1);
}

TEST_F(BoxPenetrationTest, TangentStandingCapsule2) {
  TestCollision2(TangentStandingCapsule, 1e-12);
}

TEST_F(BoxPenetrationTest, TangentProneCapsule1) {
  // TODO(tehbelinda): We should check why we cannot use a smaller tolerance.
  TestCollision1(TangentProneCapsule, 1e-1);
}

TEST_F(BoxPenetrationTest, TangentProneCapsule2) {
  // TODO(tehbelinda): We should check why we cannot use a smaller tolerance.
  TestCollision2(TangentProneCapsule, 1e-4);
}

}  // namespace
}  // namespace penetration_as_point_pair
}  // namespace internal
}  // namespace geometry
}  // namespace drake
