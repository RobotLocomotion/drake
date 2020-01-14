#include "drake/geometry/proximity/penetration_as_point_pair_callback.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace penetration_as_point_pair {
namespace {

using Eigen::Vector3d;
using fcl::CollisionObjectd;
using fcl::Sphered;
using math::RigidTransformd;
using std::make_shared;
using std::vector;

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
        id_B_(GeometryId::get_new_id()),
        callback_data_(&collision_filter_, &point_pairs_) {}

 protected:
  void SetUp() override {
    const EncodedData data_A(id_A_, true);
    data_A.write_to(&sphere_A_);
    collision_filter_.AddGeometry(data_A.encoding());

    const EncodedData data_B(id_B_, true);
    data_B.write_to(&sphere_B_);
    collision_filter_.AddGeometry(data_B.encoding());
  }

  static const double kRadius;
  CollisionObjectd sphere_A_;
  CollisionObjectd sphere_B_;
  GeometryId id_A_;
  GeometryId id_B_;
  CollisionFilterLegacy collision_filter_;
  vector<PenetrationAsPointPair<double>> point_pairs_;
  CallbackData callback_data_;
};

// TODO(SeanCurtis-TRI): Make this static constexpr when our gcc version doesn't
//  cry in debug builds.
const double PenetrationAsPointPairCallbackTest::kRadius = 0.5;

// Confirms that a pair of geometries that are demonstrably not in collision and
// are not filtered produce no results.
TEST_F(PenetrationAsPointPairCallbackTest, NonCollision) {
  // Move sphere B away from A.
  sphere_B_.setTransform(
      RigidTransformd{Vector3d{kRadius * 3, 0, 0}}.GetAsIsometry3());

  EXPECT_FALSE(Callback(&sphere_A_, &sphere_B_, &callback_data_));
  EXPECT_EQ(point_pairs_.size(), 0u);

  EXPECT_FALSE(Callback(&sphere_B_, &sphere_A_, &callback_data_));
  EXPECT_EQ(point_pairs_.size(), 0u);
}

// Confirms that a pair of geometries _in_ collision but not filtered produce
// expected results. And that the result is expected, regardless of the order
// of the objects as parameters.
// And confirms that if the pair is filtered, no collision is reported.
TEST_F(PenetrationAsPointPairCallbackTest, CollisionFilterRespected) {
  // Move sphere B away from origin such that it penetrations A 0.1 units.
  const double target_depth = 0.1;
  sphere_B_.setTransform(
      RigidTransformd{Vector3d{kRadius * 2 - target_depth, 0, 0}}
          .GetAsIsometry3());

  // Two executions with the order of the objects reversed -- should produce
  // identical results.
  EXPECT_FALSE(Callback(&sphere_A_, &sphere_B_, &callback_data_));
  ASSERT_EQ(point_pairs_.size(), 1u);
  const PenetrationAsPointPair<double> first_result = point_pairs_[0];
  point_pairs_.clear();

  EXPECT_FALSE(Callback(&sphere_B_, &sphere_A_, &callback_data_));
  ASSERT_EQ(point_pairs_.size(), 1u);
  const PenetrationAsPointPair<double> second_result = point_pairs_[0];
  point_pairs_.clear();

  const double kEps = std::numeric_limits<double>::epsilon();
  ASSERT_EQ(first_result.id_A, second_result.id_A);
  ASSERT_EQ(first_result.id_B, second_result.id_B);
  ASSERT_NEAR(first_result.depth, target_depth, kEps);
  ASSERT_NEAR(second_result.depth, target_depth, kEps);
  ASSERT_TRUE(CompareMatrices(first_result.nhat_BA_W, second_result.nhat_BA_W));
  ASSERT_TRUE(CompareMatrices(first_result.p_WCa, second_result.p_WCa));
  ASSERT_TRUE(CompareMatrices(first_result.p_WCb, second_result.p_WCb));

  // Now filter the geometries.
  const int common_clique = 1;
  collision_filter_.AddToCollisionClique(EncodedData(id_A_, true).encoding(),
                                         common_clique);
  collision_filter_.AddToCollisionClique(EncodedData(id_B_, true).encoding(),
                                         common_clique);

  EXPECT_FALSE(Callback(&sphere_A_, &sphere_B_, &callback_data_));
  EXPECT_EQ(point_pairs_.size(), 0u);

  EXPECT_FALSE(Callback(&sphere_B_, &sphere_A_, &callback_data_));
  EXPECT_EQ(point_pairs_.size(), 0u);
}

}  // namespace
}  // namespace penetration_as_point_pair
}  // namespace internal
}  // namespace geometry
}  // namespace drake
