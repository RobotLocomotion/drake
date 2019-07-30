#include "drake/geometry/scene_graph_inspector.h"

#include <memory>

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
  inspector.all_frame_ids();
  inspector.num_geometries();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  inspector.all_geometry_ids();
#pragma GCC diagnostic pop
  inspector.GetAllGeometryIds();
  inspector.NumGeometriesWithRole(Role::kUnassigned);
  inspector.GetNumDynamicGeometries();
  inspector.GetNumAnchoredGeometries();
  inspector.GetCollisionCandidates();

  // Source and source-related data methods.
  // Register a source to prevent exceptions being thrown.
  const SourceId source_id = tester.mutable_state().RegisterNewSource("name");
  inspector.SourceIsRegistered(source_id);
  inspector.GetSourceName(source_id);
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  inspector.X_PG(geometry_id);
  inspector.X_FG(geometry_id);
#pragma GCC diagnostic pop
  inspector.GetPoseInParent(geometry_id);
  inspector.GetPoseInFrame(geometry_id);
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
}

}  // namespace
}  // namespace geometry
}  // namespace drake
