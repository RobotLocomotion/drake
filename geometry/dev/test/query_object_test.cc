#include "drake/geometry/dev/query_object.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/dev/geometry_context.h"
#include "drake/geometry/dev/render/camera_properties.h"
#include "drake/geometry/dev/scene_graph.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"

namespace drake {
namespace geometry {
namespace dev {

// Helper class for testing the query object.
class QueryObjectTester {
 public:
  QueryObjectTester() = delete;

  template <typename T>
  static std::unique_ptr<QueryObject<T>> MakeQueryObject() {
    return std::unique_ptr<QueryObject<T>>(new QueryObject<T>());
  }

  template <typename T>
  static std::unique_ptr<QueryObject<T>> MakeQueryObject(
      const GeometryContext<T>* context, const SceneGraph<T>* scene_graph) {
    auto q = std::unique_ptr<QueryObject<T>>(new QueryObject<T>());
    q->set(context, scene_graph);
    return q;
  }

  template <typename T>
  static void expect_default(const QueryObject<T>& object) {
    EXPECT_EQ(object.scene_graph_, nullptr);
    EXPECT_EQ(object.context_, nullptr);
  }

  template <typename T>
  static void expect_live(const QueryObject<T>& object) {
    EXPECT_NE(object.scene_graph_, nullptr);
    EXPECT_NE(object.context_, nullptr);
  }

  template <typename T>
  static void ThrowIfDefault(const QueryObject<T>& object) {
    object.ThrowIfDefault();
  }
};

namespace {

using std::make_unique;
using std::unique_ptr;
using systems::Context;

class QueryObjectTest : public ::testing::Test {
 protected:
  using QOT = QueryObjectTester;

  void SetUp() override {
    context_ = scene_graph_.AllocateContext();
    geom_context_ = dynamic_cast<GeometryContext<double>*>(context_.get());
    ASSERT_NE(geom_context_, nullptr);
    query_object_ = QOT::MakeQueryObject(geom_context_, &scene_graph_);

    QueryObjectTester::expect_live(*query_object_);
  }

  SceneGraph<double> scene_graph_;
  unique_ptr<Context<double>> context_;
  GeometryContext<double>* geom_context_{nullptr};
  unique_ptr<QueryObject<double>> query_object_;
};

// Confirm copy semantics.
TEST_F(QueryObjectTest, CopySemantics) {
  // Default query object *can* be copied and assigned.
  unique_ptr<QueryObject<double>> default_object =
      QOT::MakeQueryObject<double>();
  QOT::expect_default(*default_object);

  QueryObject<double> from_default{*default_object};
  QOT::expect_default(from_default);

  QueryObject<double> from_live{*query_object_};
  QOT::expect_default(from_live);
}

// NOTE: This doesn't test the specific queries; GeometryQuery simply wraps
// the class (SceneGraph) that actually *performs* those queries. The
// correctness of those queries is handled in geometry_state_test.cc. The
// wrapper merely confirms that the state is correct and that wrapper
// functionality is tested in DefaultQueryThrows.
TEST_F(QueryObjectTest, DefaultQueryThrows) {
  unique_ptr<QueryObject<double>> default_object =
      QOT::MakeQueryObject<double>();
  QOT::expect_default(*default_object);

#define EXPECT_DEFAULT_ERROR(expression) \
  DRAKE_EXPECT_THROWS_MESSAGE(expression, std::runtime_error, \
      "Attempting to perform query on invalid QueryObject.+");

  EXPECT_DEFAULT_ERROR(QOT::ThrowIfDefault(*default_object));

  // Enumerate *all* queries to confirm they throw the proper exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      default_object->ComputePointPairPenetration(), std::logic_error,
      "The development SceneGraph only supports render queries");
  DRAKE_EXPECT_THROWS_MESSAGE(
      default_object->ComputeSignedDistancePairwiseClosestPoints(),
      std::logic_error,
      "The development SceneGraph only supports render queries");
  // I don't need a valid name for the renderer, because we'll never get that
  // far.
  const render::DepthCameraProperties camera_properties(320, 240, M_PI_4,
                                                        "some_name", 0.1, 1.0);
  const FrameId parent_frame = FrameId::get_new_id();
  const Isometry3<double> X_PC = Isometry3<double>::Identity();
  systems::sensors::ImageRgba8U color_image;
  EXPECT_DEFAULT_ERROR(default_object->RenderColorImage(
      camera_properties, parent_frame, X_PC, &color_image, false));
  EXPECT_DEFAULT_ERROR(default_object->RenderColorImage(
      camera_properties, X_PC, &color_image, false));

  systems::sensors::ImageDepth32F depth_image;
  EXPECT_DEFAULT_ERROR(default_object->RenderDepthImage(
      camera_properties, parent_frame, X_PC, &depth_image));
  EXPECT_DEFAULT_ERROR(default_object->RenderDepthImage(
      camera_properties, X_PC, &depth_image));

  systems::sensors::ImageLabel16I label_image;
  EXPECT_DEFAULT_ERROR(default_object->RenderLabelImage(
      camera_properties, parent_frame, X_PC, &label_image, false));
  EXPECT_DEFAULT_ERROR(default_object->RenderLabelImage(
      camera_properties, X_PC, &label_image, false));


#undef EXPECT_DEFAULT_ERROR
}

// Confirms the inspector returned by the QueryObject is "correct" (in that
// it accesses the correct state).
GTEST_TEST(QueryObjectInspectTest, CreateValidInspector) {
  SceneGraph<double> scene_graph;
  SourceId source_id = scene_graph.RegisterSource("source");
  auto identity = Isometry3<double>::Identity();
  FrameId frame_id =
      scene_graph.RegisterFrame(source_id, GeometryFrame("frame", identity));
  GeometryId geometry_id = scene_graph.RegisterGeometry(
      source_id, frame_id, make_unique<GeometryInstance>(
                               identity, make_unique<Sphere>(1.0), "sphere"));
  unique_ptr<Context<double>> context = scene_graph.AllocateContext();
  auto geo_context = dynamic_cast<GeometryContext<double>*>(context.get());
  unique_ptr<QueryObject<double>> query_object =
      QueryObjectTester::MakeQueryObject<double>(geo_context, &scene_graph);

  const SceneGraphInspector<double>& inspector = query_object->inspector();

  // Perform a single query to confirm that the inspector has access to the
  // state uniquely populated above (guaranteed via the uniqueness of frame and
  // geometry identifiers).
  EXPECT_EQ(inspector.GetFrameId(geometry_id), frame_id);
}

}  // namespace
}  // namespace dev
}  // namespace geometry
}  // namespace drake
