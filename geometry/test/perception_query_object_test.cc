#include "drake/geometry/perception_query_object.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

using math::RigidTransformd;
using render::DepthCameraProperties;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

// Friend class to QueryObject -- left in `drake::geometry` to match the friend
// declaration. The name gives it access to the private/protected access of
// PerceptionQueryObject's *parent* class, QueryObject.
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

// NOTE: This doesn't test the specific queries; PerceptionQueryObject simply
// wraps the class (GeometryState) that actually *performs* those queries. The
// correctness of those queries is handled in geometry_state_test.cc. The
// wrapper merely confirms that the state is correct and that wrapper
// functionality is tested in DefaultQueryThrows.
TEST_F(QueryObjectTest, DefaultQueryThrows) {
  PerceptionQueryObject<double> default_object;

  EXPECT_TRUE(is_default(default_object));

#define EXPECT_DEFAULT_ERROR(expression) \
  DRAKE_EXPECT_THROWS_MESSAGE(expression, std::runtime_error, \
      "Attempting to perform query on invalid QueryObject.+");

  EXPECT_DEFAULT_ERROR(ThrowIfNotCallable(default_object));

  // Enumerate *all* queries to confirm they throw the proper exception.

  // Render queries.
  DepthCameraProperties properties(2, 2, M_PI, "dummy_renderer", 0.1, 5.0);
  RigidTransformd X_WC = RigidTransformd::Identity();
  ImageRgba8U color;
  EXPECT_DEFAULT_ERROR(default_object.RenderColorImage(
      properties, FrameId::get_new_id(), X_WC, false, &color));

  ImageDepth32F depth;
  EXPECT_DEFAULT_ERROR(default_object.RenderDepthImage(
      properties, FrameId::get_new_id(), X_WC, &depth));

  ImageLabel16I label;
  EXPECT_DEFAULT_ERROR(default_object.RenderLabelImage(
      properties, FrameId::get_new_id(), X_WC, false, &label));

#undef EXPECT_DEFAULT_ERROR
}

}  // namespace geometry
}  // namespace drake
