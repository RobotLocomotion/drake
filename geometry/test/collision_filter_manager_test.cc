#include "drake/geometry/collision_filter_manager.h"

#include <array>
#include <memory>
#include <utility>

#include <fmt/format.h>
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
namespace {

using math::RigidTransformd;
using std::array;
using std::make_unique;
using std::move;
using std::unique_ptr;

// Convenience function for making a geometry instance.
unique_ptr<GeometryInstance> make_sphere_instance(double radius = 1.0) {
  return make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                       make_unique<Sphere>(radius), "sphere");
}

/* Populates the given SceneGraph with three spheres with the proximity role
 assigned. Their poses are undefined. Each sphere is affixed to a unique dynamic
 frame. No collision filters are added.

 @returns An array of GeometryIds, one for each added sphere. */
array<GeometryId, 3> PopulateSceneGraph(SceneGraph<double>* scene_graph_ptr) {
  DRAKE_DEMAND(scene_graph_ptr != nullptr);
  SceneGraph<double>& scene_graph = *scene_graph_ptr;

  array<GeometryId, 3> ids;

  SourceId source_id = scene_graph.RegisterSource("source");
  for (int i = 0; i < 3; ++i) {
    const FrameId f_id = scene_graph.RegisterFrame(
        source_id, GeometryFrame(fmt::format("frame_{}", i)));
    const GeometryId g_id =
        scene_graph.RegisterGeometry(source_id, f_id, make_sphere_instance());
    scene_graph.AssignRole(source_id, g_id, ProximityProperties());
    ids[i] = g_id;
  }

  // Confirm that the model reports no filtered pairs.
  const auto& model_inspector = scene_graph.model_inspector();
  if (model_inspector.CollisionFiltered(ids[0], ids[1]) ||
      model_inspector.CollisionFiltered(ids[0], ids[2]) ||
      model_inspector.CollisionFiltered(ids[1], ids[2])) {
    throw std::runtime_error(
        "Initial conditions erroneously has filtered pairs");
  }
  return ids;
}

/* In this test we simply apply a single declaration. We don't worry about the
 various declaration statement types, because that is the responsibility of
 the CollisionFilter class. This just makes sure it gets exercised. */
GTEST_TEST(CollisionFilterManagerTest, Apply) {
  // Initializes the scene graph and context with three geometries and no
  // filtered pairs.
  SceneGraph<double> scene_graph;
  const auto [g_id1, g_id2, g_id3] = PopulateSceneGraph(&scene_graph);
  auto context = scene_graph.CreateDefaultContext();

  // Acquire an inspector for the context geometry data. It has a "live"
  // connection to the data. So, as we change the data below, we can use the
  // same inspector to view the changes.
  const QueryObject<double>& query_object =
      scene_graph.get_query_output_port().Eval<QueryObject<double>>(*context);
  const auto& inspector = query_object.inspector();

  // Confirm unfiltered state was copied into the context..
  EXPECT_FALSE(inspector.CollisionFiltered(g_id1, g_id2));
  EXPECT_FALSE(inspector.CollisionFiltered(g_id1, g_id3));
  EXPECT_FALSE(inspector.CollisionFiltered(g_id2, g_id3));

  auto filter_manager = scene_graph.collision_filter_manager(context.get());

  filter_manager.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet({g_id1, g_id2})));
  EXPECT_TRUE(inspector.CollisionFiltered(g_id1, g_id2));
  EXPECT_FALSE(inspector.CollisionFiltered(g_id1, g_id3));
  EXPECT_FALSE(inspector.CollisionFiltered(g_id2, g_id3));

  EXPECT_FALSE(filter_manager.has_transient_history());

  const FilterId filter_id =
      filter_manager.ApplyTransient(CollisionFilterDeclaration().ExcludeBetween(
          GeometrySet({g_id1, g_id2}), GeometrySet({g_id3})));
  EXPECT_TRUE(inspector.CollisionFiltered(g_id1, g_id2));
  EXPECT_TRUE(inspector.CollisionFiltered(g_id1, g_id3));
  EXPECT_TRUE(inspector.CollisionFiltered(g_id2, g_id3));

  EXPECT_TRUE(filter_manager.has_transient_history());
  EXPECT_TRUE(filter_manager.IsActive(filter_id));
  EXPECT_TRUE(filter_manager.RemoveDeclaration(filter_id));
  EXPECT_TRUE(inspector.CollisionFiltered(g_id1, g_id2));
  EXPECT_FALSE(inspector.CollisionFiltered(g_id1, g_id3));
  EXPECT_FALSE(inspector.CollisionFiltered(g_id2, g_id3));

  // Note that the underlying model *didn't* change.
  const auto& model_inspector = scene_graph.model_inspector();
  EXPECT_FALSE(model_inspector.CollisionFiltered(g_id1, g_id2));
  EXPECT_FALSE(model_inspector.CollisionFiltered(g_id1, g_id3));
  EXPECT_FALSE(model_inspector.CollisionFiltered(g_id2, g_id3));
}

/* Confirm that the CollisionFilterManager can be copy constructed and assigned
 as expected. A copied CollisionFilterManager has the same *live* access as the
 source. */
GTEST_TEST(CollisionFilterManagerTest, CopySemantics) {
  // Initializes the scene graph and context with three geometries and no
  // filtered pairs.
  SceneGraph<double> scene_graph;
  const auto [g_id1, g_id2, g_id3] = PopulateSceneGraph(&scene_graph);
  auto context = scene_graph.CreateDefaultContext();

  // We get inspectors into SceneGraph's model and context's data.
  const auto& model_inspector = scene_graph.model_inspector();
  const QueryObject<double>& query_object =
      scene_graph.get_query_output_port().Eval<QueryObject<double>>(*context);
  const auto& context_inspector = query_object.inspector();
  // We're assuming that the model has no collision filters and the context has
  // successfully copied that trait -- that is explicitly tested in the Apply
  // test.

  // Copy construct a filter from the model. A modification to the copy should
  // be visible in the original model_filter but not the context.
  CollisionFilterManager copy_filter(scene_graph.collision_filter_manager());
  copy_filter.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet{g_id1, g_id2}));
  EXPECT_TRUE(model_inspector.CollisionFiltered(g_id1, g_id2));
  EXPECT_FALSE(context_inspector.CollisionFiltered(g_id1, g_id2));

  // Copy assign the context filter on top of the copy_filter. Now changes will
  // be visible to the context inspector but not model inspector.
  copy_filter = scene_graph.collision_filter_manager(context.get());
  copy_filter.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet{g_id1, g_id3}));
  EXPECT_FALSE(model_inspector.CollisionFiltered(g_id1, g_id3));
  EXPECT_TRUE(context_inspector.CollisionFiltered(g_id1, g_id3));
}

/* Confirm that the CollisionFilterManager can be move constructed and assigned
 as expected. A moved CollisionFilterManager has the same *live* access as the
 source. */
GTEST_TEST(CollisionFilterManagerTest, MoveSemantics) {
  // Initializes the scene graph and context with three geometries and no
  // filtered pairs.
  SceneGraph<double> scene_graph;
  const auto [g_id1, g_id2, g_id3] = PopulateSceneGraph(&scene_graph);
  auto context = scene_graph.CreateDefaultContext();

  // We get inspectors into SceneGraph's model and context's data.
  const auto& model_inspector = scene_graph.model_inspector();
  const QueryObject<double>& query_object =
      scene_graph.get_query_output_port().Eval<QueryObject<double>>(*context);
  const auto& context_inspector = query_object.inspector();
  // We're assuming that the model has no collision filters and the context has
  // successfully copied that trait -- that is explicitly tested in the Apply
  // test.

  // Move construct a filter from the model. A modification to the copy should
  // be visible in the original model_filter but not the context.
  CollisionFilterManager model_filter = scene_graph.collision_filter_manager();
  CollisionFilterManager copy_filter(move(model_filter));
  copy_filter.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet{g_id1, g_id2}));
  EXPECT_TRUE(model_inspector.CollisionFiltered(g_id1, g_id2));
  EXPECT_FALSE(context_inspector.CollisionFiltered(g_id1, g_id2));

  // Move assign the context filter on top of the copy_filter. Now changes will
  // be visible to the context inspector but not model inspector.
  CollisionFilterManager context_filter =
      scene_graph.collision_filter_manager(context.get());
  copy_filter = move(context_filter);
  copy_filter.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet{g_id1, g_id3}));
  EXPECT_FALSE(model_inspector.CollisionFiltered(g_id1, g_id3));
  EXPECT_TRUE(context_inspector.CollisionFiltered(g_id1, g_id3));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
