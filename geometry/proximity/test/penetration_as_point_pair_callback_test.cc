#include "drake/geometry/proximity/penetration_as_point_pair_callback.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
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
using math::RigidTransformd;
using math::RotationMatrixd;
using std::make_shared;
using std::vector;

template <typename T>
double ExtractValue(const T& val) {
  if constexpr (std::is_same<T, double>::value) {
    return val;
  } else if constexpr (std::is_same<T, AutoDiffXd>::value) {
    return val.value();
  } else {
    static_assert("Unsupported type T");
  }
}

template <typename Derived>
Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
ExtractMatrixValue(const Derived& v) {
  if constexpr (std::is_same<typename Derived::Scalar, double>::value) {
    return v;
  // NOLINTNEXTLINE
  } else if constexpr (std::is_same<typename Derived::Scalar,
                                    AutoDiffXd>::value) {
    return math::autoDiffToValueMatrix(v);
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
        id_A_(GeometryId::get_new_id()),
        id_B_(GeometryId::get_new_id()) {}

 protected:
  void SetUp() override {
    const EncodedData data_A(id_A_, true);
    data_A.write_to(&sphere_A_);
    collision_filter_.AddGeometry(data_A.encoding());

    const EncodedData data_B(id_B_, true);
    data_B.write_to(&sphere_B_);
    collision_filter_.AddGeometry(data_B.encoding());
  }

  template <typename T>
  void TestNoCollision() {
    // Move sphere B away from A.
    std::unordered_map<GeometryId, math::RigidTransform<T>> X_WGs;
    const math::RigidTransform<T> X_WA = math::RigidTransform<T>::Identity();
    const Vector3<T> p_WB = Vector3d(kRadius * 3, 0, 0).cast<T>();
    const math::RigidTransform<T> X_WB = math::RigidTransform<T>(p_WB);

    X_WGs.emplace(id_A_, X_WA);
    X_WGs.emplace(id_B_, X_WB);
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
    if constexpr (std::is_same<T, AutoDiffXd>::value) {
      p_WBo = math::initializeAutoDiff(
          (Vector3d{1, -2, 3}.normalized() * center_distance));
    } else {
      p_WBo = (Vector3d{1, -2, 3}.normalized() * center_distance).cast<T>();
    }
    std::unordered_map<GeometryId, math::RigidTransform<T>> X_WGs;
    const math::RigidTransform<T> X_WB = math::RigidTransform<T>{
        math::RotationMatrix<T>::MakeYRotation(M_PI / 3) *
            math::RotationMatrix<T>::MakeZRotation(-M_PI / 7),
        p_WBo};
    const math::RigidTransform<T> X_WA = math::RigidTransform<T>::Identity();
    X_WGs.emplace(id_A_, X_WA);
    X_WGs.emplace(id_B_, X_WB);

    // Two executions with the order of the objects reversed -- should produce
    // identical results.
    vector<PenetrationAsPointPair<T>> point_pairs;
    CallbackData<T> callback_data(&collision_filter_, &X_WGs, &point_pairs);
    EXPECT_FALSE(Callback<T>(&sphere_A_, &sphere_B_, &callback_data));
    ASSERT_EQ(point_pairs.size(), 1u);
    const PenetrationAsPointPair<T> first_result = point_pairs[0];
    point_pairs.clear();
    const double kEps = std::numeric_limits<double>::epsilon();
    const Eigen::Vector3d p_WCa = ExtractMatrixValue(first_result.p_WCa);
    const Eigen::Vector3d p_WCb = ExtractMatrixValue(first_result.p_WCb);
    const Eigen::Vector3d nhat_BA_W =
        ExtractMatrixValue(first_result.nhat_BA_W);
    const double depth = ExtractValue(first_result.depth);
    EXPECT_NEAR((p_WCa - p_WCb).norm(), depth, kEps);
    EXPECT_TRUE(CompareMatrices(p_WCb - p_WCa, depth * nhat_BA_W, kEps));

    EXPECT_FALSE(Callback<T>(&sphere_B_, &sphere_A_, &callback_data));
    ASSERT_EQ(point_pairs.size(), 1u);
    const PenetrationAsPointPair<T> second_result = point_pairs[0];
    point_pairs.clear();

    ASSERT_EQ(first_result.id_A, second_result.id_A);
    ASSERT_EQ(first_result.id_B, second_result.id_B);
    ASSERT_NEAR(ExtractValue(first_result.depth), target_depth, kEps);
    EXPECT_NEAR(ExtractValue(second_result.depth),
                ExtractValue(first_result.depth), kEps);
    ASSERT_TRUE(CompareMatrices(ExtractMatrixValue(first_result.nhat_BA_W),
                                ExtractMatrixValue(second_result.nhat_BA_W)));
    ASSERT_TRUE(CompareMatrices(ExtractMatrixValue(first_result.p_WCa),
                                ExtractMatrixValue(second_result.p_WCa)));
    ASSERT_TRUE(CompareMatrices(ExtractMatrixValue(first_result.p_WCb),
                                ExtractMatrixValue(second_result.p_WCb)));

    // Now filter the geometries.
    const int common_clique = 1;
    collision_filter_.AddToCollisionClique(EncodedData(id_A_, true).encoding(),
                                           common_clique);
    collision_filter_.AddToCollisionClique(EncodedData(id_B_, true).encoding(),
                                           common_clique);

    EXPECT_FALSE(Callback<T>(&sphere_A_, &sphere_B_, &callback_data));
    EXPECT_EQ(point_pairs.size(), 0u);

    EXPECT_FALSE(Callback<T>(&sphere_B_, &sphere_A_, &callback_data));
    EXPECT_EQ(point_pairs.size(), 0u);
  }

  static const double kRadius;
  CollisionObjectd sphere_A_;
  CollisionObjectd sphere_B_;
  GeometryId id_A_;
  GeometryId id_B_;
  CollisionFilterLegacy collision_filter_;
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
  const Vector3<AutoDiffXd> p_WBo = math::initializeAutoDiff(p_WBo_val);
  std::unordered_map<GeometryId, math::RigidTransform<AutoDiffXd>> X_WGs;
  const math::RigidTransform<AutoDiffXd> X_WB =
      math::RigidTransform<AutoDiffXd>{
          math::RotationMatrix<AutoDiffXd>::MakeYRotation(M_PI / 3) *
              math::RotationMatrix<AutoDiffXd>::MakeZRotation(-M_PI / 7),
          p_WBo};
  const math::RigidTransform<AutoDiffXd> X_WA =
      math::RigidTransform<AutoDiffXd>::Identity();
  X_WGs.emplace(id_A_, X_WA);
  X_WGs.emplace(id_B_, X_WB);
  vector<PenetrationAsPointPair<AutoDiffXd>> point_pairs;
  CallbackData<AutoDiffXd> callback_data(&collision_filter_, &X_WGs,
                                         &point_pairs);
  EXPECT_FALSE(Callback<AutoDiffXd>(&sphere_A_, &sphere_B_, &callback_data));
  ASSERT_EQ(point_pairs.size(), 1u);
  const Eigen::Vector3d ddepth_dp_WBo = -p_WBo_val / p_WBo_val.norm();
  const PenetrationAsPointPair<AutoDiffXd> result = point_pairs[0];
  const double kEps = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(result.depth.derivatives(), ddepth_dp_WBo, kEps));
  // ∂ (x/|x|) /∂ x where x = p_WBo
  const Eigen::Matrix3d dp_WBo_normalized_dp_WBo =
      (p_WBo_val.squaredNorm() * Eigen::Matrix3d::Identity() -
       p_WBo_val * p_WBo_val.transpose()) /
      std::pow(p_WBo_val.norm(), 3);
  // nhat_BA_W = -p_WBo / |p_WBo|, hence ∂nhat_BA_W /∂p_WBo = -∂(p_WBo/|p_WBo|)
  // / ∂p_WBo
  EXPECT_TRUE(CompareMatrices(math::autoDiffToGradientMatrix(result.nhat_BA_W),
                              -dp_WBo_normalized_dp_WBo, kEps));
  // p_WCa = radius * p_WBo / |p_WBo|.
  const Eigen::Matrix3d dp_WCa_dp_WBo = dp_WBo_normalized_dp_WBo * kRadius;
  EXPECT_TRUE(CompareMatrices(math::autoDiffToGradientMatrix(result.p_WCa),
                              dp_WCa_dp_WBo, kEps));
  // p_WCb = p_WBo - radius * p_WBo / |p_WBo|.
  const Eigen::Matrix3d dp_WCb_dp_WBo =
      Eigen::Matrix3d::Identity() - dp_WBo_normalized_dp_WBo * kRadius;
  EXPECT_TRUE(CompareMatrices(math::autoDiffToGradientMatrix(result.p_WCb),
                              dp_WCb_dp_WBo, kEps));
}

}  // namespace
}  // namespace penetration_as_point_pair
}  // namespace internal
}  // namespace geometry
}  // namespace drake
