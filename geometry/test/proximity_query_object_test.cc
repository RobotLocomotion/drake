#include "drake/geometry/proximity_query_object.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {

using std::vector;

// Friend class to QueryObject -- left in `drake::geometry` to match the friend
// declaration. The name gives it access to the private/protected access of
// ProximityQueryObject's *parent* class, QueryObject.
class QueryObjectTest : public ::testing::Test {
 protected:
  template <typename T>
  static ::testing::AssertionResult is_default(const QueryObject<T>& object) {
    if (object.scene_graph_ != nullptr || object.context_ != nullptr ||
        object.state_ != nullptr) {
      return ::testing::AssertionFailure()
             << "A default query object should have all null fields. Has "
                "scene_graph: "
             << object.scene_graph_ << ", context: " << object.context_
             << ", state: " << object.state_.get();
    }
    return ::testing::AssertionSuccess();
  }

  template <typename T>
  static void ThrowIfNotCallable(const QueryObject<T>& object) {
    object.ThrowIfNotCallable();
  }
};

// NOTE: This doesn't test the specific queries; ProximityQueryObject simply
// wraps the class (GeometryState) that actually *performs* those queries. The
// correctness of those queries is handled in geometry_state_test.cc. The
// wrapper merely confirms that the state is correct and that wrapper
// functionality is tested in DefaultQueryThrows.
TEST_F(QueryObjectTest, DefaultQueryThrows) {
  ProximityQueryObject<double> default_object;

  EXPECT_TRUE(is_default(default_object));

#define EXPECT_DEFAULT_ERROR(expression) \
  DRAKE_EXPECT_THROWS_MESSAGE(expression, std::runtime_error, \
      "Attempting to perform query on invalid QueryObject.+");

  EXPECT_DEFAULT_ERROR(ThrowIfNotCallable(default_object));

  // Enumerate *all* queries to confirm they throw the proper exception.

  // Penetration queries.
  EXPECT_DEFAULT_ERROR(default_object.ComputePointPairPenetration());
  EXPECT_DEFAULT_ERROR(default_object.ComputeContactSurfaces());
  vector<ContactSurface<double>> surfaces;
  vector<PenetrationAsPointPair<double>> point_pairs;
  EXPECT_DEFAULT_ERROR(default_object.ComputeContactSurfacesWithFallback(
      &surfaces, &point_pairs));

  // Signed distance queries.
  EXPECT_DEFAULT_ERROR(
      default_object.ComputeSignedDistancePairwiseClosestPoints());
  EXPECT_DEFAULT_ERROR(default_object.ComputeSignedDistancePairClosestPoints(
      GeometryId::get_new_id(), GeometryId::get_new_id()));
  EXPECT_DEFAULT_ERROR(
      default_object.ComputeSignedDistanceToPoint(Vector3<double>::Zero()));

  EXPECT_DEFAULT_ERROR(default_object.FindCollisionCandidates());
  EXPECT_DEFAULT_ERROR(default_object.HasCollisions());

#undef EXPECT_DEFAULT_ERROR
}

}  // namespace geometry
}  // namespace drake
