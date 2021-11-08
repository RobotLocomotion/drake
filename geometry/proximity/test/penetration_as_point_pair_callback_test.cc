#include "drake/geometry/proximity/penetration_as_point_pair_callback.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace penetration_as_point_pair {
namespace {

using Eigen::Vector3d;
using fcl::CollisionObjectd;
using fcl::Sphered;
using math::RigidTransform;
using math::RigidTransformd;
using math::RotationMatrix;
using math::RotationMatrixd;
using std::make_shared;
using std::vector;

const double kEps = std::numeric_limits<double>::epsilon();

template <typename Derived>
Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
ExtractMatrixValue(const Derived& v) {
  if constexpr (std::is_same_v<typename Derived::Scalar, double>) {
    return v;
  } else if constexpr (std::is_same_v<typename Derived::Scalar, AutoDiffXd>) {
    return math::ExtractValue(v);
  } else {
    static_assert("Unsupported type T");
  }
}

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
    const Eigen::Vector3d p_WCa = ExtractMatrixValue(first_result.p_WCa);
    const Eigen::Vector3d p_WCb = ExtractMatrixValue(first_result.p_WCb);
    const Eigen::Vector3d nhat_BA_W =
        ExtractMatrixValue(first_result.nhat_BA_W);
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
    ASSERT_TRUE(CompareMatrices(ExtractMatrixValue(first_result.nhat_BA_W),
                                ExtractMatrixValue(second_result.nhat_BA_W)));
    ASSERT_TRUE(CompareMatrices(ExtractMatrixValue(first_result.p_WCa),
                                ExtractMatrixValue(second_result.p_WCa)));
    ASSERT_TRUE(CompareMatrices(ExtractMatrixValue(first_result.p_WCb),
                                ExtractMatrixValue(second_result.p_WCb)));

    // Filter the pair (A, B); we'll put the ids in a set and simply return that
    // set for the extract ids function.
    std::unordered_set<GeometryId> ids{id_A_, id_B_};
    CollisionFilter::ExtractIds extract = [&ids](const GeometrySet&) {
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
    EXPECT_TRUE(CompareMatrices(ExtractMatrixValue(first_result.p_WCa),
                                ExtractMatrixValue(second_result.p_WCa)));
    EXPECT_TRUE(CompareMatrices(ExtractMatrixValue(first_result.p_WCb),
                                ExtractMatrixValue(second_result.p_WCb)));
    EXPECT_TRUE(CompareMatrices(ExtractMatrixValue(first_result.nhat_BA_W),
                                ExtractMatrixValue(second_result.nhat_BA_W)));

    if constexpr (std::is_same_v<T, AutoDiffXd>) {
      // Make sure that callback with T=AutoDiffXd and T=double produces the
      // same result.
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
      EXPECT_EQ(ExtractDoubleOrThrow(first_result.depth),
                point_pairs_double[0].depth);
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
        halfspace_normal_.dot(ExtractMatrixValue(R_WB.inverse() * p_WBo)) +
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
        Callback<T>(&shape1, &shape2, &callback_data), std::logic_error,
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
  EXPECT_TRUE(
      CompareMatrices(math::ExtractGradient(result.nhat_BA_W),
                      -dp_WBo_normalized_dp_WBo, kEps));
  // p_WCa = radius * p_WBo / |p_WBo|.
  const Eigen::Matrix3d dp_WCa_dp_WBo = dp_WBo_normalized_dp_WBo * kRadius;
  EXPECT_TRUE(
      CompareMatrices(math::ExtractGradient(result.p_WCa),
                      dp_WCa_dp_WBo, kEps));
  // p_WCb = p_WBo - radius * p_WBo / |p_WBo|.
  const Eigen::Matrix3d dp_WCb_dp_WBo =
      Eigen::Matrix3d::Identity() - dp_WBo_normalized_dp_WBo * kRadius;
  EXPECT_TRUE(
      CompareMatrices(math::ExtractGradient(result.p_WCb),
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

}  // namespace
}  // namespace penetration_as_point_pair
}  // namespace internal
}  // namespace geometry
}  // namespace drake
