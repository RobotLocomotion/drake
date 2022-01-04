#include "drake/geometry/scene_graph_inspector.h"

#include <algorithm>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"

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

  inspector.num_frames();
  inspector.num_geometries();
  inspector.GetAllGeometryIds();
  inspector.GetGeometryIds(GeometrySet{});
  inspector.NumGeometriesWithRole(Role::kUnassigned);
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
  inspector.GetFrameGroup(frame_id);
  inspector.NumGeometriesForFrame(frame_id);
  inspector.NumGeometriesForFrameWithRole(frame_id, Role::kUnassigned);
  inspector.GetGeometries(frame_id, Role::kUnassigned);
  // Register a geometry to prevent an exception being thrown.
  const GeometryId geometry_id =
      tester.mutable_state().RegisterGeometry(
          source_id, frame_id,
          make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                        make_unique<Sphere>(1.0), "sphere"));
  inspector.GetGeometryIdByName(frame_id, Role::kUnassigned, "sphere");

  // Geometries and their properties.
  inspector.BelongsToSource(geometry_id, source_id);
  inspector.GetOwningSourceName(geometry_id);
  inspector.GetFrameId(geometry_id);
  inspector.GetName(geometry_id);
  inspector.GetShape(geometry_id);
  inspector.GetPoseInParent(geometry_id);
  inspector.GetPoseInFrame(geometry_id);
  inspector.maybe_get_hydroelastic_mesh(geometry_id);
  inspector.GetProximityProperties(geometry_id);
  inspector.GetIllustrationProperties(geometry_id);
  inspector.GetPerceptionProperties(geometry_id);
  // Register an *additional* geometry and assign proximity properties to both
  // to prevent an exception being thrown.
  const GeometryId geometry_id2 =
      tester.mutable_state().RegisterGeometry(
          source_id, frame_id,
          make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                        make_unique<Sphere>(1.0), "sphere2"));
  tester.mutable_state().AssignRole(source_id, geometry_id,
                                    ProximityProperties());
  tester.mutable_state().AssignRole(source_id, geometry_id2,
                                    ProximityProperties());
  inspector.CollisionFiltered(geometry_id, geometry_id2);

  std::unique_ptr<GeometryInstance> geometry_instance_clone =
      inspector.CloneGeometryInstance(geometry_id);
  EXPECT_NE(geometry_instance_clone->id(), geometry_id);
  EXPECT_EQ(geometry_instance_clone->name(), "sphere");
  const auto* shape_clone = dynamic_cast<const Sphere*>(
      &geometry_instance_clone->shape());
  EXPECT_NE(shape_clone, nullptr);
  EXPECT_EQ(shape_clone->radius(), 1.0);
  EXPECT_NE(geometry_instance_clone->proximity_properties(), nullptr);
  EXPECT_EQ(geometry_instance_clone->perception_properties(), nullptr);
  EXPECT_EQ(geometry_instance_clone->illustration_properties(), nullptr);
  inspector.geometry_version();
}

// Generally, SceneGraphInspector is a thin wrapper for invoking methods on
// GeometryState. SceneGraphInspector::GetAllFrameIds() is an exception, it does
// transformation work and, as such, needs to be functionally tested.
GTEST_TEST(SceneGraphInspector, AllFrameIds) {
  SceneGraphInspectorTester tester;

  const SourceId source_id = tester.mutable_state().RegisterNewSource("source");

  // Always includes the world frame.
  ASSERT_EQ(tester.inspector().GetAllFrameIds().size(), 1);

  // Add a number of frames.
  const FrameId frame_id_1 =
      tester.mutable_state().RegisterFrame(source_id, GeometryFrame("frame1"));
  const FrameId frame_id_2 =
      tester.mutable_state().RegisterFrame(source_id, GeometryFrame("frame2"));
  const FrameId frame_id_3 =
      tester.mutable_state().RegisterFrame(source_id, GeometryFrame("frame3"));

  const std::vector<FrameId> all_frames = tester.inspector().GetAllFrameIds();
  EXPECT_EQ(std::count(all_frames.begin(), all_frames.end(), frame_id_1), 1);
  EXPECT_EQ(std::count(all_frames.begin(), all_frames.end(), frame_id_2), 1);
  EXPECT_EQ(std::count(all_frames.begin(), all_frames.end(), frame_id_3), 1);
}


}  // namespace
}  // namespace geometry
}  // namespace drake
