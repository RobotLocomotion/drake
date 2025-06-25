#include "drake/geometry/query_object.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/internal_frame.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

using Eigen::Vector3d;
using math::RigidTransformd;
using render::ColorRenderCamera;
using render::DepthRenderCamera;
using std::make_unique;
using std::unique_ptr;
using systems::Context;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

// Friend class to QueryObject -- left in `drake::geometry` to match the friend
// declaration.
class QueryObjectTest : public ::testing::Test {
 protected:
  template <typename T>
  static std::unique_ptr<QueryObject<T>> MakeQueryObject(
      const systems::Context<T>* context, const SceneGraph<T>* scene_graph) {
    auto q = std::unique_ptr<QueryObject<T>>(new QueryObject<T>());
    q->set(context, scene_graph);
    return q;
  }

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
  static ::testing::AssertionResult is_live(const QueryObject<T>& object) {
    if (object.scene_graph_ == nullptr || object.context_ == nullptr ||
        object.state_ != nullptr) {
      return ::testing::AssertionFailure()
             << "A live query object should have non-null scene graph and "
                "context and null state. Has scene_graph: "
             << object.scene_graph_ << ", context: " << object.context_
             << ", state: " << object.state_.get();
    }
    return ::testing::AssertionSuccess();
  }

  template <typename T>
  static ::testing::AssertionResult is_baked(const QueryObject<T>& object) {
    if (object.scene_graph_ != nullptr || object.context_ != nullptr ||
        object.state_ == nullptr) {
      return ::testing::AssertionFailure()
             << "A baked query object should have all null scene graph and "
                "context and non-null state. Has scene_graph: "
             << object.scene_graph_ << ", context: " << object.context_
             << ", state: " << object.state_.get();
    }
    return ::testing::AssertionSuccess();
  }

  template <typename T>
  static ::testing::AssertionResult shared_baked(const QueryObject<T>& o1,
                                                 const QueryObject<T>& o2) {
    auto result = is_baked(o1);
    if (result) {
      result = is_baked(o2);
      if (result) {
        if (o1.state_ != o2.state_) {
          result = ::testing::AssertionFailure()
                   << "The two objects are baked but have different states";
        }
      }
    }
    return result;
  }

  template <typename T>
  void set_live(QueryObject<T>* query_object, Context<T>* context) {
    query_object->set(context, &scene_graph_);
  }

  template <typename T>
  static void ThrowIfNotCallable(const QueryObject<T>& object) {
    object.ThrowIfNotCallable();
  }

  template <typename T>
  static const GeometryState<T>& get_state(const QueryObject<T>& object) {
    return object.geometry_state();
  }

  void FullPoseAndConfigurationUpdate(const QueryObject<double>* query_object) {
    query_object->FullPoseAndConfigurationUpdate();
  }

  SceneGraph<double> scene_graph_;
};

// Confirm copy semantics.
TEST_F(QueryObjectTest, CopySemantics) {
  // Default query object *can* be copied and assigned.
  QueryObject<double> default_object;
  EXPECT_TRUE(is_default(default_object));

  QueryObject<double> from_default{default_object};
  EXPECT_TRUE(is_default(from_default));

  unique_ptr<Context<double>> live_context =
      scene_graph_.CreateDefaultContext();
  unique_ptr<QueryObject<double>> live_query_object =
      MakeQueryObject(live_context.get(), &scene_graph_);
  EXPECT_TRUE(is_live(*live_query_object));

  QueryObject<double> baked_from_live{*live_query_object};
  EXPECT_TRUE(is_baked(baked_from_live));

  QueryObject<double> baked_from_baked{baked_from_live};
  // Simultaneously test from_baked *is* baked and shares state with
  // baked_from_live.
  EXPECT_TRUE(shared_baked(baked_from_live, baked_from_baked));

  // Confirm a baked object can be reset to be a live object.
  set_live(&baked_from_baked, live_context.get());
  EXPECT_TRUE(is_live(baked_from_baked));
}

// NOTE: This doesn't test the specific queries; GeometryQuery simply wraps
// the class (SceneGraph) that actually *performs* those queries. The
// correctness of those queries is handled in geometry_state_test.cc. The
// wrapper merely confirms that the state is correct and that wrapper
// functionality is tested in DefaultQueryThrows. Arguably, the question of
// "do the parameters and return values" get mapped correctly could be under
// test. But, for now, we assume the parameters and return values are propagated
// correctly.
TEST_F(QueryObjectTest, DefaultQueryThrows) {
  QueryObject<double> default_object;
  EXPECT_TRUE(is_default(default_object));

#define EXPECT_DEFAULT_ERROR(expression) \
  DRAKE_EXPECT_THROWS_MESSAGE(           \
      expression, "Attempting to perform query on invalid QueryObject.+");

  EXPECT_DEFAULT_ERROR(ThrowIfNotCallable(default_object));

  // Enumerate *all* queries to confirm they throw the proper exception.

  // Scalar-dependent state queries.
  EXPECT_DEFAULT_ERROR(default_object.GetPoseInWorld(FrameId::get_new_id()));
  EXPECT_DEFAULT_ERROR(default_object.GetPoseInParent(FrameId::get_new_id()));
  EXPECT_DEFAULT_ERROR(default_object.GetPoseInWorld(GeometryId::get_new_id()));
  EXPECT_DEFAULT_ERROR(
      default_object.GetConfigurationsInWorld(GeometryId::get_new_id()));
  EXPECT_DEFAULT_ERROR(default_object.GetDrivenMeshConfigurationsInWorld(
      GeometryId::get_new_id(), Role::kIllustration));

  // Penetration queries.
  EXPECT_DEFAULT_ERROR(default_object.ComputePointPairPenetration());
  const HydroelasticContactRepresentation representation =
      HydroelasticContactRepresentation::kTriangle;
  EXPECT_DEFAULT_ERROR(default_object.ComputeContactSurfaces(representation));
  std::vector<ContactSurface<double>> surfaces;
  std::vector<PenetrationAsPointPair<double>> point_pairs;
  EXPECT_DEFAULT_ERROR(default_object.ComputeContactSurfacesWithFallback(
      representation, &surfaces, &point_pairs));
  internal::DeformableContact<double> deformable_contact;
  EXPECT_DEFAULT_ERROR(
      default_object.ComputeDeformableContact(&deformable_contact));

  // Signed distance queries.
  EXPECT_DEFAULT_ERROR(
      default_object.ComputeSignedDistancePairwiseClosestPoints());
  EXPECT_DEFAULT_ERROR(default_object.ComputeSignedDistancePairClosestPoints(
      GeometryId::get_new_id(), GeometryId::get_new_id()));
  EXPECT_DEFAULT_ERROR(
      default_object.ComputeSignedDistanceToPoint(Vector3<double>::Zero()));
  EXPECT_DEFAULT_ERROR(default_object.ComputeSignedDistanceGeometryToPoint(
      Vector3<double>::Zero(), GeometrySet(GeometryId::get_new_id())));

  EXPECT_DEFAULT_ERROR(default_object.FindCollisionCandidates());
  EXPECT_DEFAULT_ERROR(default_object.HasCollisions());

  // Render queries.
  const ColorRenderCamera color_camera{
      {"n/a", {2, 2, M_PI}, {0.1, 10}, RigidTransformd{}}, false};
  const DepthRenderCamera depth_camera{
      {"n/a", {2, 2, M_PI}, {0.1, 10}, RigidTransformd{}}, {0.2, 0.9}};
  const RigidTransformd X_WC = RigidTransformd::Identity();

  ImageRgba8U color;
  EXPECT_DEFAULT_ERROR(default_object.RenderColorImage(
      color_camera, FrameId::get_new_id(), X_WC, &color));
  ImageDepth32F depth;
  EXPECT_DEFAULT_ERROR(default_object.RenderDepthImage(
      depth_camera, FrameId::get_new_id(), X_WC, &depth));
  ImageLabel16I label;
  EXPECT_DEFAULT_ERROR(default_object.RenderLabelImage(
      color_camera, FrameId::get_new_id(), X_WC, &label));

  EXPECT_DEFAULT_ERROR(default_object.GetRenderEngineByName("dummy"));

#undef EXPECT_DEFAULT_ERROR
}

// Confirms the inspector returned by the QueryObject is "correct" (in that
// it accesses the correct state).
TEST_F(QueryObjectTest, CreateValidInspector) {
  SourceId source_id = scene_graph_.RegisterSource("source");
  auto identity = RigidTransformd::Identity();
  FrameId frame_id =
      scene_graph_.RegisterFrame(source_id, GeometryFrame("frame"));
  GeometryId geometry_id = scene_graph_.RegisterGeometry(
      source_id, frame_id,
      make_unique<GeometryInstance>(identity, make_unique<Sphere>(1.0),
                                    "sphere"));
  unique_ptr<Context<double>> context = scene_graph_.CreateDefaultContext();
  unique_ptr<QueryObject<double>> query_object =
      MakeQueryObject(context.get(), &scene_graph_);

  const SceneGraphInspector<double>& inspector = query_object->inspector();

  // Perform a single query to confirm that the inspector has access to the
  // state uniquely populated above (guaranteed via the uniqueness of frame and
  // geometry identifiers).
  EXPECT_EQ(inspector.GetFrameId(geometry_id), frame_id);
}

// This test confirms that the copied (aka baked) query object has its pose and
// configuration data properly baked. This is confirmed by a great deal of
// convoluted trickery.
TEST_F(QueryObjectTest, BakedCopyHasFullUpdate) {
  SourceId s_id = scene_graph_.RegisterSource("BakeTest");
  FrameId frame_id = scene_graph_.RegisterFrame(s_id, GeometryFrame("frame"));

  auto deformable_geometry = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Sphere>(1.0), "deformable");
  // Make sure the resolution hint is large enough so that we know that the
  // volume mesh is a octahedron.
  GeometryId g_id = scene_graph_.RegisterDeformableGeometry(
      s_id, internal::InternalFrame::world_frame_id(),
      std::move(deformable_geometry), 10.0);
  unique_ptr<Context<double>> context = scene_graph_.CreateDefaultContext();

  RigidTransformd X_WF{Vector3d{1, 2, 3}};
  FramePoseVector<double> poses{{frame_id, X_WF}};
  scene_graph_.get_source_pose_port(s_id).FixValue(context.get(), poses);

  const int kNumVertices = 7;
  const int kNumDofs = kNumVertices * 3;
  const VectorX<double> q_WG = Eigen::VectorXd::LinSpaced(0.0, 1.0, kNumDofs);
  GeometryConfigurationVector<double> configurations{{g_id, q_WG}};
  scene_graph_.get_source_configuration_port(s_id).FixValue(context.get(),
                                                            configurations);

  const auto& query_object =
      scene_graph_.get_query_output_port().Eval<QueryObject<double>>(*context);
  EXPECT_TRUE(is_live(query_object));

  // Here's the convoluted trickery. We examine the state that's embedded in
  // the context of the live query object. It *hasn't* updated
  // poses/configurations at all. So, if we ask for the world pose of frame_id,
  // it will *not* be at X_WF. Similarly, if we ask for the configuration of
  // g_id, it will *not* be q_WG. However, when we copy the query_object, the
  // same query on its state *will* return those values (showing that the copy
  // has the updated poses).
  const GeometryState<double>& state = get_state(query_object);
  const auto& stale_pose = state.get_pose_in_world(frame_id);
  const auto& stale_configuration = state.get_configurations_in_world(g_id);
  // Confirm the live state hasn't been updated yet.
  EXPECT_FALSE(
      CompareMatrices(stale_pose.GetAsMatrix34(), X_WF.GetAsMatrix34()));
  EXPECT_FALSE(CompareMatrices(stale_configuration, q_WG));

  const QueryObject<double> baked(query_object);

  const GeometryState<double>& baked_state = get_state(baked);
  const auto& baked_pose = baked_state.get_pose_in_world(frame_id);
  const auto& baked_configuration =
      baked_state.get_configurations_in_world(g_id);
  EXPECT_TRUE(
      CompareMatrices(baked_pose.GetAsMatrix34(), X_WF.GetAsMatrix34()));
  EXPECT_TRUE(CompareMatrices(baked_configuration, q_WG));
  // And the previously stale pose/configuration is now updated as a pre-cursor
  // to the baked copy.
  EXPECT_TRUE(
      CompareMatrices(stale_pose.GetAsMatrix34(), X_WF.GetAsMatrix34()));
  EXPECT_TRUE(CompareMatrices(stale_configuration, q_WG));

  // These really are different objects.
  EXPECT_NE(&stale_pose, &baked_pose);
  EXPECT_NE(&stale_configuration, &baked_configuration);
}

// This test confirms that queries on a live QueryObject update the cached
// GeometryState. The test sets up a scene with stale configuration and pose
// data. It calls a query and confirms that the state has been updated,
// implying that FullPoseAndConfigurationUpdate() was called.
TEST_F(QueryObjectTest, LiveQueryUpdatesState) {
  const SourceId s_id = scene_graph_.RegisterSource("UpdateTest");
  const FrameId frame_id =
      scene_graph_.RegisterFrame(s_id, GeometryFrame("frame"));

  // Helper function to make a geometry instance with the given name.
  auto make_geometry = [](const std::string& name) {
    auto geometry = make_unique<GeometryInstance>(
        RigidTransformd::Identity(), make_unique<Sphere>(0.5), name);
    geometry->set_proximity_properties(ProximityProperties());
    return geometry;
  };

  // Register a couple of rigid geometries for later use.
  const GeometryId rigid_g_id1 =
      scene_graph_.RegisterGeometry(s_id, frame_id, make_geometry("sphere"));
  const GeometryId rigid_g_id2 = scene_graph_.RegisterGeometry(
      s_id, frame_id, make_geometry("another_sphere"));
  auto deformable_geometry = make_geometry("deformable");
  const GeometryId deformable_g_id = scene_graph_.RegisterDeformableGeometry(
      s_id, internal::InternalFrame::world_frame_id(),
      std::move(deformable_geometry), 10.0);
  unique_ptr<Context<double>> context = scene_graph_.CreateDefaultContext();

  const auto& query_object =
      scene_graph_.get_query_output_port().Eval<QueryObject<double>>(*context);
  EXPECT_TRUE(is_live(query_object));
  const GeometryState<double>& geometry_state = get_state(query_object);

  // A function to introduce stale data into the context and confirm it is
  // stale.
  const RigidTransformd X_WF_stale = geometry_state.get_pose_in_world(frame_id);
  const RigidTransformd X_WF_new{Vector3d{1, 2, 3}};
  const VectorX<double> q_WG_stale =
      geometry_state.get_configurations_in_world(deformable_g_id);
  const VectorX<double> q_WG_new = 2.0 * q_WG_stale;

  FramePoseVector<double> stale_poses{{frame_id, X_WF_stale}};
  GeometryConfigurationVector<double> stale_configurations{
      {deformable_g_id, q_WG_stale}};
  FramePoseVector<double> new_poses{{frame_id, X_WF_new}};
  GeometryConfigurationVector<double> new_configurations{
      {deformable_g_id, q_WG_new}};
  auto set_and_confirm_stale = [&]() {
    // Set the data to be the stale values and flush the update.
    scene_graph_.get_source_pose_port(s_id).FixValue(context.get(),
                                                     stale_poses);
    scene_graph_.get_source_configuration_port(s_id).FixValue(
        context.get(), stale_configurations);
    FullPoseAndConfigurationUpdate(&query_object);
    // Now set the data to be the new values but don't flush the update.
    scene_graph_.get_source_pose_port(s_id).FixValue(context.get(), new_poses);
    scene_graph_.get_source_configuration_port(s_id).FixValue(
        context.get(), new_configurations);
    // Confirm the data is still stale.
    const auto& stale_pose = geometry_state.get_pose_in_world(frame_id);
    const auto& stale_config =
        geometry_state.get_configurations_in_world(deformable_g_id);
    EXPECT_FALSE(
        CompareMatrices(stale_pose.GetAsMatrix34(), X_WF_new.GetAsMatrix34()));
    EXPECT_FALSE(CompareMatrices(stale_config, q_WG_new));
  };

  // A function to confirm the stale data has been updated.
  auto confirm_updated = [&]() {
    const auto& updated_pose = geometry_state.get_pose_in_world(frame_id);
    const auto& updated_config =
        geometry_state.get_configurations_in_world(deformable_g_id);
    EXPECT_TRUE(CompareMatrices(updated_pose.GetAsMatrix34(),
                                X_WF_new.GetAsMatrix34()));
    EXPECT_TRUE(CompareMatrices(updated_config, q_WG_new));
  };

  set_and_confirm_stale();
  query_object.ComputePointPairPenetration();
  confirm_updated();

  set_and_confirm_stale();
  query_object.FindCollisionCandidates();
  confirm_updated();

  set_and_confirm_stale();
  query_object.HasCollisions();
  confirm_updated();

  set_and_confirm_stale();
  internal::DeformableContact<double> deformable_contact;
  query_object.ComputeDeformableContact(&deformable_contact);
  confirm_updated();

  set_and_confirm_stale();
  query_object.ComputeSignedDistancePairwiseClosestPoints();
  confirm_updated();

  set_and_confirm_stale();
  query_object.ComputeSignedDistancePairClosestPoints(rigid_g_id1, rigid_g_id2);
  confirm_updated();

  set_and_confirm_stale();
  query_object.ComputeSignedDistanceToPoint(Vector3d::Zero());
  confirm_updated();

  set_and_confirm_stale();
  GeometrySet geometry_set(rigid_g_id1);
  query_object.ComputeSignedDistanceGeometryToPoint(Vector3d::Zero(),
                                                    geometry_set);
  confirm_updated();

  // Render queries... Since no renderers are registered, calling the render
  // queries will throw, but the state will be updated nonetheless.
  const ColorRenderCamera color_camera{
      {"n/a", {2, 2, M_PI}, {0.1, 10}, RigidTransformd{}}, false};
  const DepthRenderCamera depth_camera{
      {"n/a", {2, 2, M_PI}, {0.1, 10}, RigidTransformd{}}, {0.2, 0.9}};
  const RigidTransformd X_PC = RigidTransformd::Identity();

  set_and_confirm_stale();
  query_object.GetRenderEngineByName("dummy");
  confirm_updated();

  set_and_confirm_stale();
  ImageRgba8U color_image;
  EXPECT_THROW(
      query_object.RenderColorImage(color_camera, frame_id, X_PC, &color_image),
      std::exception);
  confirm_updated();

  set_and_confirm_stale();
  ImageDepth32F depth_image;
  EXPECT_THROW(
      query_object.RenderDepthImage(depth_camera, frame_id, X_PC, &depth_image),
      std::exception);
  confirm_updated();

  set_and_confirm_stale();
  ImageLabel16I label_image;
  EXPECT_THROW(
      query_object.RenderLabelImage(color_camera, frame_id, X_PC, &label_image),
      std::exception);
  confirm_updated();
}

// Ensure that I can construct a QueryObject with the default scalar types.
GTEST_TEST(QueryObjectScalarTest, ScalarTypes) {
  EXPECT_NO_THROW(QueryObject<AutoDiffXd>());
  EXPECT_NO_THROW(QueryObject<symbolic::Expression>());
}

}  // namespace geometry
}  // namespace drake
