#include "drake/geometry/query_object.h"

#include <memory>
#include <utility>
#include <vector>

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

// We'll use SceneGraph's friend to poke at the state of its cache entries.
class SceneGraphTester {
 public:
  static void InvalidateFramePositions(const Context<double>& context,
                                       SceneGraph<double>* sg) {
    sg->get_mutable_cache_entry(sg->pose_update_index_)
        .get_mutable_cache_entry_value(context)
        .mark_out_of_date();
  }
  static void InvalidateDeformablePositions(const Context<double>& context,
                                            SceneGraph<double>* sg) {
    sg->get_mutable_cache_entry(sg->configuration_update_index_)
        .get_mutable_cache_entry_value(context)
        .mark_out_of_date();
  }
  static bool FramePositionsAreUpToDate(const Context<double>& context,
                                        const SceneGraph<double>& sg) {
    return !sg.get_cache_entry(sg.pose_update_index_).is_out_of_date(context);
  }
  static bool DeformablePositionsAreUpToDate(const Context<double>& context,
                                             const SceneGraph<double>& sg) {
    return !sg.get_cache_entry(sg.configuration_update_index_)
                .is_out_of_date(context);
  }
};

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
  EXPECT_DEFAULT_ERROR(
      default_object.ComputeAabbInWorld(GeometryId::get_new_id()));
  EXPECT_DEFAULT_ERROR(
      default_object.ComputeObbInWorld(GeometryId::get_new_id()));

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
// GeometryState. We use the SceneGraphTester above to directly invalidate the
// cache entries and test their state as a result of invoking a query. Some of
// the queries will throw (as noted below); all that matters is that the cache
// entry updates before the throw.
//
// Every function must be exercised with a clear declaration of whether it
// updates rigid poses, deformable poses, or both.
TEST_F(QueryObjectTest, LiveQueryUpdatesState) {
  // Dummy ids for queries that require ids as parameters.
  const GeometryId g_id1 = GeometryId::get_new_id();
  const GeometryId g_id2 = GeometryId::get_new_id();
  const FrameId frame_id = FrameId::get_new_id();

  // Masks for reporting what cache entries we expect get updated.
  const int kPoseOnly = 1;
  const int kDeformOnly = 2;
  const int kFullUpdate = kPoseOnly | kDeformOnly;

  unique_ptr<Context<double>> context = scene_graph_.CreateDefaultContext();

  const auto& qo =
      scene_graph_.get_query_output_port().Eval<QueryObject<double>>(*context);
  ASSERT_TRUE(is_live(qo));

  auto set_and_confirm_stale = [&]() {
    SceneGraphTester::InvalidateFramePositions(*context, &scene_graph_);
    SceneGraphTester::InvalidateDeformablePositions(*context, &scene_graph_);
    EXPECT_FALSE(
        SceneGraphTester::FramePositionsAreUpToDate(*context, scene_graph_));
    EXPECT_FALSE(SceneGraphTester::DeformablePositionsAreUpToDate(
        *context, scene_graph_));
  };

  auto confirm_updated = [&](int update_expectations) {
    // Always expect rigid pose update.
    EXPECT_EQ(
        SceneGraphTester::FramePositionsAreUpToDate(*context, scene_graph_),
        static_cast<bool>(update_expectations & kPoseOnly));
    EXPECT_EQ(SceneGraphTester::DeformablePositionsAreUpToDate(*context,
                                                               scene_graph_),
              static_cast<bool>(update_expectations & kDeformOnly));
  };

#define EXPECT_UPDATES(func, expect_update) \
  {                                         \
    SCOPED_TRACE(#func);                    \
    set_and_confirm_stale();                \
    func;                                   \
    confirm_updated(expect_update);         \
  }

// For the queries that have not been properly prepped (i.e., bad arguments or
// missing state), we allow the throw but still expect that the cache got
// updated before the throw.
#define EXPECT_UPDATES_WITH_THROW(func, expect_update) \
  EXPECT_UPDATES(EXPECT_THROW(func, std::exception), expect_update);

  // Configuration-dependent introspection.
  EXPECT_UPDATES_WITH_THROW(qo.GetPoseInWorld(frame_id), kPoseOnly);
  EXPECT_UPDATES_WITH_THROW(qo.GetPoseInParent(frame_id), kPoseOnly);
  EXPECT_UPDATES_WITH_THROW(qo.GetPoseInWorld(g_id1), kPoseOnly);
  EXPECT_UPDATES_WITH_THROW(qo.GetConfigurationsInWorld(g_id1), kDeformOnly);
  EXPECT_UPDATES_WITH_THROW(
      qo.GetDrivenMeshConfigurationsInWorld(g_id1, Role::kProximity),
      kDeformOnly);

  // Collision queries.
  EXPECT_UPDATES(qo.ComputePointPairPenetration(), kFullUpdate);
  EXPECT_UPDATES(
      qo.ComputeContactSurfaces(HydroelasticContactRepresentation::kTriangle),
      kPoseOnly);
  std::vector<ContactSurface<double>> surfaces;
  std::vector<PenetrationAsPointPair<double>> point_pairs;
  EXPECT_UPDATES(qo.ComputeContactSurfacesWithFallback(
                     HydroelasticContactRepresentation::kTriangle, &surfaces,
                     &point_pairs),
                 kPoseOnly);
  internal::DeformableContact<double> deformable_contact;
  EXPECT_UPDATES(qo.ComputeDeformableContact(&deformable_contact), kFullUpdate);
  EXPECT_UPDATES(qo.FindCollisionCandidates(), kFullUpdate);
  EXPECT_UPDATES(qo.HasCollisions(), kFullUpdate);

  // Signed distance queries.
  EXPECT_UPDATES(qo.ComputeSignedDistancePairwiseClosestPoints(), kFullUpdate);
  EXPECT_UPDATES_WITH_THROW(
      qo.ComputeSignedDistancePairClosestPoints(g_id1, g_id2), kFullUpdate);
  EXPECT_UPDATES(qo.ComputeSignedDistanceToPoint(Vector3d::Zero()),
                 kFullUpdate);
  EXPECT_UPDATES(
      qo.ComputeSignedDistanceGeometryToPoint(Vector3d::Zero(), GeometrySet()),
      kFullUpdate);

  // Render queries.
  const DepthRenderCamera depth_camera{
      {"n/a", {2, 2, M_PI}, {0.1, 10}, RigidTransformd{}}, {0.2, 0.9}};
  const ColorRenderCamera color_camera{depth_camera.core(), false};
  const RigidTransformd X_PC = RigidTransformd::Identity();

  ImageRgba8U color_image;
  EXPECT_UPDATES_WITH_THROW(
      qo.RenderColorImage(color_camera, frame_id, X_PC, &color_image),
      kFullUpdate);
  ImageDepth32F depth_image;
  EXPECT_UPDATES_WITH_THROW(
      qo.RenderDepthImage(depth_camera, frame_id, X_PC, &depth_image),
      kFullUpdate);
  ImageLabel16I label_image;
  EXPECT_UPDATES_WITH_THROW(
      qo.RenderLabelImage(color_camera, frame_id, X_PC, &label_image),
      kFullUpdate);
  EXPECT_UPDATES(qo.GetRenderEngineByName("dummy"), kFullUpdate);

  // Bounding box queries.
  EXPECT_UPDATES_WITH_THROW(qo.ComputeAabbInWorld(g_id1), kDeformOnly);
  EXPECT_UPDATES_WITH_THROW(qo.ComputeObbInWorld(g_id1), kPoseOnly);
}

// Ensure that I can construct a QueryObject with the default scalar types.
GTEST_TEST(QueryObjectScalarTest, ScalarTypes) {
  EXPECT_NO_THROW(QueryObject<AutoDiffXd>());
  EXPECT_NO_THROW(QueryObject<symbolic::Expression>());
}

}  // namespace geometry
}  // namespace drake
