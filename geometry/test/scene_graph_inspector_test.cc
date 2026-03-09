#include "drake/geometry/scene_graph_inspector.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_state.h"

namespace drake {
namespace geometry {

// Helper class for testing the SceneGraphInspector; specifically to create
// a geometry state and instantiate the inspector.
class SceneGraphInspectorTester {
 public:
  SceneGraphInspectorTester() { inspector_.set(&state_); }

  const SceneGraphInspector<double>& inspector() const { return inspector_; }

  GeometryState<double>& mutable_state() { return state_; }

 private:
  GeometryState<double> state_;
  SceneGraphInspector<double> inspector_;
};

namespace {

using math::RigidTransformd;
using std::make_unique;

// Simply exercises all the methods to confirm there's no build or execution
// problems. NOTE: All methods on SceneGraphInspector should be invoked here.
// Because these are thin wrappers of GeometryState methods, the *correctness*
// of these methods is left to be tested directly on GeometryState. NOTE: if
// SceneGraphInspector adds methods that actually have meaningful internal
// logic, they should be tested explicitly for correctness of that logic.
GTEST_TEST(SceneGraphInspector, ExerciseEverything) {
  SceneGraphInspectorTester tester;
  const SceneGraphInspector<double>& inspector = tester.inspector();

  // Scene-graph wide data methods.
  inspector.num_sources();
  inspector.GetAllSourceIds();

  inspector.num_frames();
  inspector.num_geometries();
  inspector.GetAllGeometryIds();
  inspector.GetGeometryIds(GeometrySet{});
  inspector.NumGeometriesWithRole(Role::kUnassigned);
  inspector.NumDeformableGeometriesWithRole(Role::kUnassigned);
  inspector.NumDynamicGeometries();
  inspector.NumAnchoredGeometries();
  inspector.GetCollisionCandidates();

  // Source and source-related data methods.
  // Register a source to prevent exceptions being thrown.
  const SourceId source_id = tester.mutable_state().RegisterNewSource("name");
  inspector.SourceIsRegistered(source_id);
  inspector.GetName(source_id);
  inspector.NumFramesForSource(source_id);
  inspector.FramesForSource(source_id);

  // Frames and their properties.
  // Register a frame to prevent exceptions being thrown.
  const FrameId frame_id =
      tester.mutable_state().RegisterFrame(source_id, GeometryFrame("frame"));
  inspector.BelongsToSource(frame_id, source_id);
  inspector.GetOwningSourceName(frame_id);
  inspector.GetName(frame_id);
  inspector.GetParentFrame(frame_id);
  inspector.GetFrameGroup(frame_id);
  inspector.NumGeometriesForFrame(frame_id);
  inspector.NumGeometriesForFrameWithRole(frame_id, Role::kUnassigned);
  inspector.GetGeometries(frame_id, Role::kUnassigned);
  // Register a geometry to prevent an exception being thrown.
  const GeometryId geometry_id = tester.mutable_state().RegisterGeometry(
      source_id, frame_id,
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                    make_unique<Sphere>(1.0), "sphere"));
  // Register a deformable geometry.
  auto deformable_instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Sphere>(1.0), "sphere");
  deformable_instance->set_illustration_properties(IllustrationProperties{});
  const GeometryId deformable_geometry_id =
      tester.mutable_state().RegisterDeformableGeometry(
          source_id, internal::InternalFrame::world_frame_id(),
          std::move(deformable_instance), 0.5);
  inspector.GetGeometryIdByName(frame_id, Role::kUnassigned, "sphere");

  // Geometries and their properties.
  inspector.BelongsToSource(geometry_id, source_id);
  inspector.GetOwningSourceName(geometry_id);
  inspector.GetFrameId(geometry_id);
  inspector.GetName(geometry_id);
  inspector.GetShape(geometry_id);
  inspector.GetPoseInFrame(geometry_id);
  inspector.maybe_get_hydroelastic_mesh(geometry_id);
  inspector.GetProximityProperties(geometry_id);
  inspector.GetIllustrationProperties(geometry_id);
  inspector.GetPerceptionProperties(geometry_id);
  inspector.GetReferenceMesh(geometry_id);
  inspector.GetDrivenRenderMeshes(deformable_geometry_id, Role::kIllustration);
  inspector.IsDeformableGeometry(geometry_id);
  inspector.GetAllDeformableGeometryIds();
  inspector.GetConvexHull(geometry_id);
  inspector.GetObbInGeometryFrame(geometry_id);
  inspector.GetReferenceMesh(geometry_id);
  // Register an *additional* geometry and assign proximity properties to both
  // to prevent an exception being thrown.
  const GeometryId geometry_id2 = tester.mutable_state().RegisterGeometry(
      source_id, frame_id,
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                    make_unique<Sphere>(1.0), "sphere2"));
  tester.mutable_state().AssignRole(source_id, geometry_id,
                                    ProximityProperties());
  tester.mutable_state().AssignRole(source_id, geometry_id2,
                                    ProximityProperties());
  inspector.CollisionFiltered(geometry_id, geometry_id2);

  // Cloning geometry instances handled below.

  inspector.geometry_version();
}

// Inspector is uniquely responsible for defining the logic for cloning a
// geometry. As such, merely "exercising" it is insufficient.
GTEST_TEST(SceneGraphInspector, CloneGeometryInstance) {
  SceneGraphInspectorTester tester;
  const SceneGraphInspector<double>& inspector = tester.inspector();

  // Register a geometry to prevent an exception being thrown.
  const SourceId source_id = tester.mutable_state().RegisterNewSource("name");
  const FrameId frame_id =
      tester.mutable_state().RegisterFrame(source_id, GeometryFrame("frame"));

  // Geometry with no properties; confirm the other properties.
  const GeometryInstance original(RigidTransformd(Eigen::Vector3d(1, 2, 3)),
                                  Sphere(1.5), "test_sphere");
  const GeometryId geometry_id = tester.mutable_state().RegisterGeometry(
      source_id, frame_id, make_unique<GeometryInstance>(original));

  // Confirm basic geometry parameters (name, id, etc.) and that if the source
  // doesn't have a role, the clone doesn't either.
  {
    std::unique_ptr<GeometryInstance> clone =
        inspector.CloneGeometryInstance(geometry_id);

    EXPECT_NE(clone->id(), original.id());
    EXPECT_EQ(clone->name(), original.name());
    const auto* shape_clone = dynamic_cast<const Sphere*>(&clone->shape());
    EXPECT_NE(shape_clone, nullptr);
    EXPECT_EQ(shape_clone->radius(), 1.5);
    EXPECT_EQ(clone->proximity_properties(), nullptr);
    EXPECT_EQ(clone->perception_properties(), nullptr);
    EXPECT_EQ(clone->illustration_properties(), nullptr);
  }

  // Now confirm that assigned roles (via their properties) propagate.
  {
    IllustrationProperties illus;
    illus.AddProperty("illus", "value", 1.5);
    PerceptionProperties percep;
    percep.AddProperty("percep", "value", 1.5);
    ProximityProperties prox;
    prox.AddProperty("prox", "value", 1.5);
    tester.mutable_state().AssignRole(source_id, geometry_id, illus);
    tester.mutable_state().AssignRole(source_id, geometry_id, percep);
    tester.mutable_state().AssignRole(source_id, geometry_id, prox);

    std::unique_ptr<GeometryInstance> clone =
        inspector.CloneGeometryInstance(geometry_id);

    ASSERT_NE(clone->illustration_properties(), nullptr);
    EXPECT_TRUE(
        clone->illustration_properties()->HasProperty("illus", "value"));
    ASSERT_NE(clone->perception_properties(), nullptr);
    EXPECT_TRUE(clone->perception_properties()->HasProperty("percep", "value"));
    ASSERT_NE(clone->proximity_properties(), nullptr);
    EXPECT_TRUE(clone->proximity_properties()->HasProperty("prox", "value"));

    // A smoke test to confirm the clone doesn't share data with GeometryState's
    // internals (unlikely as the data most likely to be shared is passed by
    // value). But to hinder regression, we'll peek at some property addresses.
    EXPECT_NE(clone->illustration_properties(),
              inspector.GetIllustrationProperties(geometry_id));
    EXPECT_NE(clone->perception_properties(),
              inspector.GetPerceptionProperties(geometry_id));
    EXPECT_NE(clone->proximity_properties(),
              inspector.GetProximityProperties(geometry_id));
  }
}

// Generally, SceneGraphInspector is a thin wrapper for invoking methods on
// GeometryState. SceneGraphInspector::GetAllFrameIds() is an exception; it does
// transformation work and, as such, needs to be functionally tested.
GTEST_TEST(SceneGraphInspector, AllFrameIds) {
  SceneGraphInspectorTester tester;

  const SourceId source_id = tester.mutable_state().RegisterNewSource("source");

  // Always includes the world frame.
  ASSERT_EQ(tester.inspector().GetAllFrameIds().size(), 1);

  // Add a number of frames (in addition to the world frame which is always
  // included).
  const FrameId world_id = internal::InternalFrame::world_frame_id();
  const FrameId frame_id_1 =
      tester.mutable_state().RegisterFrame(source_id, GeometryFrame("frame1"));
  const FrameId frame_id_2 =
      tester.mutable_state().RegisterFrame(source_id, GeometryFrame("frame2"));
  const FrameId frame_id_3 =
      tester.mutable_state().RegisterFrame(source_id, GeometryFrame("frame3"));

  // We expect the results to be nicely, consistently ordered.
  std::vector<FrameId> expected_ids{frame_id_1, frame_id_2, frame_id_3,
                                    world_id};
  std::sort(expected_ids.begin(), expected_ids.end());

  const std::vector<FrameId> all_frames = tester.inspector().GetAllFrameIds();
  // Same contents and same order.
  EXPECT_EQ(all_frames, expected_ids);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
