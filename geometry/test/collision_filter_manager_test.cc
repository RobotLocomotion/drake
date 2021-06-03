#include "drake/geometry/collision_filter_manager.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_properties.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

// Friend class for accessing SceneGraph protected/private functionality.
class SceneGraphTester {
 public:
  template <typename T>
  static void GetQueryObjectPortValue(const SceneGraph<T>& scene_graph,
                                      const systems::Context<T>& context,
                                      QueryObject<T>* handle) {
    scene_graph.CalcQueryObject(context, handle);
  }
};

namespace {

using math::RigidTransformd;
using std::make_unique;
using std::unique_ptr;

// Convenience function for making a geometry instance.
unique_ptr<GeometryInstance> make_sphere_instance(
    double radius = 1.0) {
  return make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                       make_unique<Sphere>(radius), "sphere");
}

GTEST_TEST(CollisionFilterManagerTest, ExcludeBetween) {
  // Initializes the scene graph and context.
  SceneGraph<double> scene_graph;
  // Simple scene with three frames, each with a sphere which, by default
  // collide.
  SourceId source_id = scene_graph.RegisterSource("source");
  FrameId f_id1 =
      scene_graph.RegisterFrame(source_id, GeometryFrame("frame_1"));
  FrameId f_id2 =
      scene_graph.RegisterFrame(source_id, GeometryFrame("frame_2"));
  FrameId f_id3 =
      scene_graph.RegisterFrame(source_id, GeometryFrame("frame_3"));
  GeometryId g_id1 =
      scene_graph.RegisterGeometry(source_id, f_id1, make_sphere_instance());
  scene_graph.AssignRole(source_id, g_id1, ProximityProperties());
  GeometryId g_id2 =
      scene_graph.RegisterGeometry(source_id, f_id2, make_sphere_instance());
  scene_graph.AssignRole(source_id, g_id2, ProximityProperties());
  GeometryId g_id3 =
      scene_graph.RegisterGeometry(source_id, f_id3, make_sphere_instance());
  scene_graph.AssignRole(source_id, g_id3, ProximityProperties());

  // Confirm that the model reports no filtered pairs.
  const auto& model_inspector = scene_graph.model_inspector();
  EXPECT_FALSE(model_inspector.CollisionFiltered(g_id1, g_id2));
  EXPECT_FALSE(model_inspector.CollisionFiltered(g_id1, g_id3));
  EXPECT_FALSE(model_inspector.CollisionFiltered(g_id2, g_id3));

  auto context = scene_graph.CreateDefaultContext();

  // Confirms the state. NOTE: Because we're not copying the query object or
  // changing context, this query object and inspector are valid for querying
  // the modified context.
  QueryObject<double> query_object;
  SceneGraphTester::GetQueryObjectPortValue(scene_graph, *context,
                                            &query_object);
  const auto& inspector = query_object.inspector();

  // Confirm unfiltered state.
  EXPECT_FALSE(inspector.CollisionFiltered(g_id1, g_id2));
  EXPECT_FALSE(inspector.CollisionFiltered(g_id1, g_id3));
  EXPECT_FALSE(inspector.CollisionFiltered(g_id2, g_id3));

  auto filter_manager = scene_graph.collision_filter_manager(context.get());

  filter_manager.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet({g_id1, g_id2})));
  EXPECT_TRUE(inspector.CollisionFiltered(g_id1, g_id2));
  EXPECT_FALSE(inspector.CollisionFiltered(g_id1, g_id3));
  EXPECT_FALSE(inspector.CollisionFiltered(g_id2, g_id3));

  filter_manager.Apply(CollisionFilterDeclaration().ExcludeBetween(
      GeometrySet({g_id1, g_id2}), GeometrySet({g_id3})));
  EXPECT_TRUE(inspector.CollisionFiltered(g_id1, g_id2));
  EXPECT_TRUE(inspector.CollisionFiltered(g_id1, g_id3));
  EXPECT_TRUE(inspector.CollisionFiltered(g_id2, g_id3));

  // Note that the underlying model *didn't* change.
  EXPECT_FALSE(model_inspector.CollisionFiltered(g_id1, g_id2));
  EXPECT_FALSE(model_inspector.CollisionFiltered(g_id1, g_id3));
  EXPECT_FALSE(model_inspector.CollisionFiltered(g_id2, g_id3));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
