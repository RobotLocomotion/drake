#include "drake/geometry/scene_graph.h"

#include <memory>
#include <utility>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/test_utilities/dummy_render_engine.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

// Test of the unique SceneGraph operations. SceneGraph is mostly a thin
// wrapper around GeometryState. It's purpose is to connect GeometryState to the
// Drake ecosystem. As such, there will be no tests on functional logic but just
// on that wrapping. For examples, queries simply extract a context from the
// QueryObject and pass it to the SceneGraph method. As such, there is
// nothing to test.

namespace drake {
namespace geometry {

using internal::DummyRenderEngine;
using math::RigidTransformd;
using systems::Context;
using systems::System;
using std::make_unique;
using std::unique_ptr;

// Friend class for working with QueryObjects in a test context.
class QueryObjectTest {
 public:
  QueryObjectTest() = delete;

  template <typename T>
  static QueryObject<T> MakeNullQueryObject() {
    return QueryObject<T>();
  }

  template <typename T>
  static void set_query_object(QueryObject<T>* q_object,
                               const SceneGraph<T>* scene_graph,
                               const Context<T>* context) {
    q_object->set(context, scene_graph);
  }
};

// Friend class for accessing SceneGraph protected/private functionality.
class SceneGraphTester {
 public:
  SceneGraphTester() = delete;

  template <typename T>
  static void FullPoseUpdate(const SceneGraph<T>& scene_graph,
                             const Context<T>& context) {
    scene_graph.FullPoseUpdate(context);
  }

  template <typename T>
  static void GetQueryObjectPortValue(const SceneGraph<T>& scene_graph,
                                      const systems::Context<T>& context,
                                      QueryObject<T>* handle) {
    scene_graph.CalcQueryObject(context, handle);
  }

  template <typename T>
  static const GeometryState<T>& GetGeometryState(
      const SceneGraph<T>& scene_graph, const systems::Context<T>& context) {
    return scene_graph.geometry_state(context);
  }

  template <typename T>
  static GeometryState<T>& GetMutableGeometryState(
      const SceneGraph<T>& scene_graph, systems::Context<T>* context) {
    return scene_graph.mutable_geometry_state(context);
  }
};

namespace {

// Convenience function for making a geometry instance.
std::unique_ptr<GeometryInstance> make_sphere_instance(
    double radius = 1.0) {
  return make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                       make_unique<Sphere>(radius), "sphere");
}

// Testing harness to facilitate working with/testing the SceneGraph. Before
// performing *any* queries in tests, `CreateDefaultContext` must be explicitly
// invoked in the test.

class SceneGraphTest : public ::testing::Test {
 public:
  SceneGraphTest()
      : ::testing::Test(),
        query_object_(QueryObjectTest::MakeNullQueryObject<double>()) {}

 protected:
  void CreateDefaultContext() {
    // TODO(SeanCurtis-TRI): This will probably have to be moved into an
    // explicit call so it can be run *after* topology has been set.
    context_ = scene_graph_.CreateDefaultContext();
    QueryObjectTest::set_query_object(&query_object_, &scene_graph_,
                                      context_.get());
  }

  const QueryObject<double>& query_object() const {
    // The `CreateDefaultContext()` method must have been called *prior* to this
    // method.
    if (!context_)
      throw std::runtime_error("Must call CreateDefaultContext() first.");
    return query_object_;
  }

  SceneGraph<double> scene_graph_;
  // Ownership of context.
  unique_ptr<Context<double>> context_;

 private:
  // Keep this private so tests must access it through the getter so we can
  // determine if CreateDefaultContext() has been invoked.
  QueryObject<double> query_object_;
};

// Test sources.

// Tests registration using a default source name. Confirms that the source
// registered.
TEST_F(SceneGraphTest, RegisterSourceDefaultName) {
  SourceId id = scene_graph_.RegisterSource();
  EXPECT_TRUE(id.is_valid());
  EXPECT_TRUE(scene_graph_.SourceIsRegistered(id));
  EXPECT_TRUE(scene_graph_.model_inspector().SourceIsRegistered(id));
}

// Tests registration using a specified source name. Confirms that the source
// registered and that the name is available.
TEST_F(SceneGraphTest, RegisterSourceSpecifiedName) {
  std::string name = "some_unique_name";
  SourceId source_id = scene_graph_.RegisterSource(name);
  EXPECT_TRUE(source_id.is_valid());
  EXPECT_TRUE(scene_graph_.SourceIsRegistered(source_id));
  EXPECT_EQ(scene_graph_.model_inspector().GetName(source_id), name);
}

// Tests that sources can be registered after context allocation; it should be
// considered registered by the scene graph, but *not* the previously
// allocated context.. It also implicitly tests that the model inspector is
// available _after_ allocation.
TEST_F(SceneGraphTest, RegisterSourcePostContext) {
  CreateDefaultContext();
  const std::string new_source_name = "register_source_post_context";
  SourceId new_source = scene_graph_.RegisterSource(new_source_name);
  EXPECT_TRUE(scene_graph_.SourceIsRegistered(new_source));
  // Contained in scene graph.
  EXPECT_EQ(scene_graph_.model_inspector().GetName(new_source),
            new_source_name);
  // Not found in allocated context.
  DRAKE_EXPECT_THROWS_MESSAGE(
      query_object().inspector().GetName(new_source),
      std::logic_error, "Querying source name for an invalid source id.*");
}

// Tests ability to report if a source is registered or not.
TEST_F(SceneGraphTest, SourceIsRegistered) {
  SourceId id = scene_graph_.RegisterSource();
  CreateDefaultContext();
  EXPECT_TRUE(scene_graph_.SourceIsRegistered(id));
  EXPECT_FALSE(scene_graph_.SourceIsRegistered(SourceId::get_new_id()));
}

// Test ports.

// Confirms that attempting to acquire input ports for unregistered sources
// throws exceptions.
TEST_F(SceneGraphTest, InputPortsForInvalidSource) {
  SourceId fake_source = SourceId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph_.get_source_pose_port(fake_source), std::logic_error,
      "Can't acquire pose port for unknown source id: \\d+.");
}

// Confirms that attempting to acquire input ports for valid sources for the
// first time *after* allocation is acceptable.
TEST_F(SceneGraphTest, AcquireInputPortsAfterAllocation) {
  SourceId id = scene_graph_.RegisterSource();
  DRAKE_EXPECT_NO_THROW(scene_graph_.get_source_pose_port(id));
  CreateDefaultContext();
  // Port which *hadn't* been accessed is still accessible.
  DRAKE_EXPECT_NO_THROW(scene_graph_.get_source_pose_port(id));
}

// Tests that topology operations after allocation _are_ allowed. This compares
// the GeometryState instances of the original context and the new context.
// This doesn't check the details of each of the registered members -- just that
// it was registered. It relies on the GeometryState tests to confirm that the
// details are correct.
TEST_F(SceneGraphTest, TopologyAfterAllocation) {
  SourceId id = scene_graph_.RegisterSource();
  FrameId old_frame_id =
      scene_graph_.RegisterFrame(id, GeometryFrame("old_frame"));
  // This geometry will be removed after allocation.
  GeometryId old_geometry_id = scene_graph_.RegisterGeometry(id, old_frame_id,
      make_sphere_instance());

  CreateDefaultContext();

  FrameId parent_frame_id =
      scene_graph_.RegisterFrame(id, GeometryFrame("frame"));
  FrameId child_frame_id =
      scene_graph_.RegisterFrame(id, parent_frame_id, GeometryFrame("frame"));
  GeometryId parent_geometry_id = scene_graph_.RegisterGeometry(
      id, parent_frame_id, make_sphere_instance());
  GeometryId child_geometry_id = scene_graph_.RegisterGeometry(
      id, parent_geometry_id, make_sphere_instance());
  GeometryId anchored_id =
      scene_graph_.RegisterAnchoredGeometry(id, make_sphere_instance());
  scene_graph_.RemoveGeometry(id, old_geometry_id);

  const SceneGraphInspector<double>& model_inspector =
      scene_graph_.model_inspector();
  const SceneGraphInspector<double>& context_inspector =
      query_object().inspector();

  // Now test registration (non-registration) in the new (old) state,
  // respectively.
  EXPECT_TRUE(model_inspector.BelongsToSource(parent_frame_id, id));
  EXPECT_TRUE(model_inspector.BelongsToSource(child_frame_id, id));
  EXPECT_TRUE(model_inspector.BelongsToSource(parent_geometry_id, id));
  EXPECT_TRUE(model_inspector.BelongsToSource(child_geometry_id, id));
  EXPECT_TRUE(model_inspector.BelongsToSource(anchored_id, id));
  // Removed geometry from SceneGraph; "invalid" id throws.
  EXPECT_THROW(model_inspector.BelongsToSource(old_geometry_id, id),
               std::logic_error);

  EXPECT_THROW(context_inspector.BelongsToSource(parent_frame_id, id),
               std::logic_error);
  EXPECT_THROW(context_inspector.BelongsToSource(child_frame_id, id),
               std::logic_error);
  EXPECT_THROW(context_inspector.BelongsToSource(parent_geometry_id, id),
               std::logic_error);
  EXPECT_THROW(context_inspector.BelongsToSource(child_geometry_id, id),
               std::logic_error);
  EXPECT_THROW(context_inspector.BelongsToSource(anchored_id, id),
               std::logic_error);
  EXPECT_TRUE(context_inspector.BelongsToSource(old_geometry_id, id));
}

// Confirms that the direct feedthrough logic is correct -- there is total
// direct feedthrough.
TEST_F(SceneGraphTest, DirectFeedThrough) {
  scene_graph_.RegisterSource();
  EXPECT_EQ(scene_graph_.GetDirectFeedthroughs().size(),
            scene_graph_.num_input_ports() *
                scene_graph_.num_output_ports());
}

// Test the functionality that accumulates the values from the input ports.

// Simple, toy case: there are no geometry sources; evaluate of pose update
// should be, essentially a no op.
TEST_F(SceneGraphTest, FullPoseUpdateEmpty) {
  CreateDefaultContext();
  DRAKE_EXPECT_NO_THROW(
      SceneGraphTester::FullPoseUpdate(scene_graph_, *context_));
}

// Test case where there are only anchored geometries -- same as the empty case;
// no geometry to update.
TEST_F(SceneGraphTest, FullPoseUpdateAnchoredOnly) {
  SourceId s_id = scene_graph_.RegisterSource();
  scene_graph_.RegisterAnchoredGeometry(s_id, make_sphere_instance());
  CreateDefaultContext();
  DRAKE_EXPECT_NO_THROW(
      SceneGraphTester::FullPoseUpdate(scene_graph_, *context_));
}

// Tests operations on a transmogrified SceneGraph. Whether a context has been
// allocated or not, subsequent operations should be allowed.
TEST_F(SceneGraphTest, TransmogrifyWithoutAllocation) {
  SourceId s_id = scene_graph_.RegisterSource();
  // This should allow additional geometry registration.
  std::unique_ptr<systems::System<AutoDiffXd>> system_ad =
      scene_graph_.ToAutoDiffXd();
  SceneGraph<AutoDiffXd>& scene_graph_ad =
      *dynamic_cast<SceneGraph<AutoDiffXd>*>(system_ad.get());
  DRAKE_EXPECT_NO_THROW(
      scene_graph_ad.RegisterAnchoredGeometry(s_id, make_sphere_instance()));

  // After allocation, registration should _still_ be valid.
  CreateDefaultContext();
  system_ad = scene_graph_.ToAutoDiffXd();
  SceneGraph<AutoDiffXd>& scene_graph_ad2 =
      *dynamic_cast<SceneGraph<AutoDiffXd>*>(system_ad.get());
  DRAKE_EXPECT_NO_THROW(
      scene_graph_ad2.RegisterAnchoredGeometry(s_id, make_sphere_instance()));
}

// Tests that the ports are correctly mapped.
TEST_F(SceneGraphTest, TransmogrifyPorts) {
  SourceId s_id = scene_graph_.RegisterSource();
  CreateDefaultContext();
  std::unique_ptr<systems::System<AutoDiffXd>> system_ad =
      scene_graph_.ToAutoDiffXd();
  SceneGraph<AutoDiffXd>& scene_graph_ad =
      *dynamic_cast<SceneGraph<AutoDiffXd>*>(system_ad.get());
  EXPECT_EQ(scene_graph_ad.num_input_ports(),
            scene_graph_.num_input_ports());
  EXPECT_EQ(scene_graph_ad.get_source_pose_port(s_id).get_index(),
            scene_graph_.get_source_pose_port(s_id).get_index());
  EXPECT_NO_THROW(scene_graph_ad.CreateDefaultContext());
}

// Tests that the work to "set" the context values for the transmogrified system
// behaves correctly.
TEST_F(SceneGraphTest, TransmogrifyContext) {
  SceneGraph<double> sg;
  SourceId s_id = sg.RegisterSource();
  // Register geometry that should be successfully transmogrified.
  GeometryId g_id = sg.RegisterAnchoredGeometry(s_id, make_sphere_instance());
  std::unique_ptr<Context<double>> context = sg.CreateDefaultContext();
  // This should transmogrify the internal *model*, so when I allocate the
  // transmogrified context, I should get the "same" values (considering type
  // change).
  std::unique_ptr<System<AutoDiffXd>> system_ad = sg.ToAutoDiffXd();
  SceneGraph<AutoDiffXd>& scene_graph_ad =
      *dynamic_cast<SceneGraph<AutoDiffXd>*>(system_ad.get());
  std::unique_ptr<Context<AutoDiffXd>> context_ad =
      scene_graph_ad.CreateDefaultContext();

  // Extract the GeometryState and query some invariants on it directly.
  const GeometryState<AutoDiffXd>& geo_state_ad =
      SceneGraphTester::GetGeometryState(scene_graph_ad, *context_ad);
  // If the anchored geometry were not ported over, this would throw an
  // exception.
  EXPECT_TRUE(geo_state_ad.BelongsToSource(g_id, s_id));
  EXPECT_THROW(geo_state_ad.BelongsToSource(GeometryId::get_new_id(), s_id),
               std::logic_error);

  // Quick reality check that this is still valid although unnecessary vis a
  // vis the GeometryState.
  DRAKE_EXPECT_NO_THROW(context_ad->SetTimeStateAndParametersFrom(*context));
}

// Tests the model inspector. Exercises a token piece of functionality. The
// inspector is a wrapper on the GeometryState. It is assumed that GeometryState
// confirms the correctness of the underlying functions. This merely tests the
// instantiation, the exercise of a representative function, and the
// post-allocate functionality.
TEST_F(SceneGraphTest, ModelInspector) {
  SourceId source_id = scene_graph_.RegisterSource();
  ASSERT_TRUE(scene_graph_.SourceIsRegistered(source_id));

  FrameId frame_1 = scene_graph_.RegisterFrame(source_id, GeometryFrame{"f1"});
  FrameId frame_2 = scene_graph_.RegisterFrame(source_id, GeometryFrame{"f2"});

  // Note: all these geometries have the same *name* -- but because they are
  // affixed to different nodes, that should be alright.
  GeometryId anchored_id = scene_graph_.RegisterAnchoredGeometry(
      source_id,
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                    make_unique<Sphere>(1.0), "sphere"));
  GeometryId sphere_1 = scene_graph_.RegisterGeometry(
      source_id, frame_1,
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                    make_unique<Sphere>(1.0), "sphere"));
  GeometryId sphere_2 = scene_graph_.RegisterGeometry(
      source_id, frame_2,
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                    make_unique<Sphere>(1.0), "sphere"));

  const SceneGraphInspector<double>& inspector = scene_graph_.model_inspector();

  EXPECT_EQ(inspector.GetGeometryIdByName(frame_1, Role::kUnassigned, "sphere"),
            sphere_1);
  EXPECT_EQ(inspector.GetGeometryIdByName(frame_2, Role::kUnassigned, "sphere"),
            sphere_2);
  EXPECT_EQ(inspector.GetGeometryIdByName(scene_graph_.world_frame_id(),
                                          Role::kUnassigned, "sphere"),
            anchored_id);
}

// SceneGraph provides a thin wrapper on the GeometryState renderer
// configuration/introspection code. These tests are just smoke tests that the
// functions work. It relies on GeometryState to properly unit test the
// full behavior.
TEST_F(SceneGraphTest, RendererSmokeTest) {
  const std::string kRendererName = "bob";

  EXPECT_EQ(scene_graph_.RendererCount(), 0);
  EXPECT_EQ(scene_graph_.RegisteredRendererNames().size(), 0u);
  EXPECT_FALSE(scene_graph_.HasRenderer(kRendererName));

  DRAKE_EXPECT_NO_THROW(scene_graph_.AddRenderer(
      kRendererName, make_unique<DummyRenderEngine>()));

  EXPECT_EQ(scene_graph_.RendererCount(), 1);
  EXPECT_EQ(scene_graph_.RegisteredRendererNames()[0], kRendererName);
  EXPECT_TRUE(scene_graph_.HasRenderer(kRendererName));
}

// SceneGraph provides a thin wrapper on the GeometryState role manipulation
// code. These tests are just smoke tests that the functions work. It relies on
// GeometryState to properly unit test the full behavior.
TEST_F(SceneGraphTest, RoleManagementSmokeTest) {
  SourceId s_id = scene_graph_.RegisterSource("test");
  FrameId f_id = scene_graph_.RegisterFrame(s_id, GeometryFrame("frame"));
  auto instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Sphere>(1.0), "sphere");
  instance->set_illustration_properties(IllustrationProperties());
  instance->set_proximity_properties(ProximityProperties());
  instance->set_perception_properties(PerceptionProperties());

  GeometryId g_id = scene_graph_.RegisterGeometry(s_id, f_id, move(instance));

  const SceneGraphInspector<double>& inspector = scene_graph_.model_inspector();
  EXPECT_NE(inspector.GetProximityProperties(g_id), nullptr);
  EXPECT_NE(inspector.GetIllustrationProperties(g_id), nullptr);
  EXPECT_NE(inspector.GetPerceptionProperties(g_id), nullptr);

  EXPECT_EQ(scene_graph_.RemoveRole(s_id, g_id, Role::kPerception), 1);
  EXPECT_EQ(scene_graph_.RemoveRole(s_id, g_id, Role::kProximity), 1);
  EXPECT_EQ(scene_graph_.RemoveRole(s_id, g_id, Role::kIllustration), 1);
}

// Dummy system to serve as geometry source.
class GeometrySourceSystem : public systems::LeafSystem<double> {
 public:
  explicit GeometrySourceSystem(SceneGraph<double>* scene_graph)
      : systems::LeafSystem<double>(), scene_graph_(scene_graph) {
    // Register with SceneGraph.
    source_id_ = scene_graph->RegisterSource(kRegisteredSourceName);
    FrameId f_id =
        scene_graph->RegisterFrame(source_id_, GeometryFrame("frame"));
    frame_ids_.push_back(f_id);

    // Set up output port now that the frame is registered.
    this->DeclareAbstractOutputPort(
        systems::kUseDefaultName,
        &GeometrySourceSystem::CalcFramePoseOutput);
  }
  SourceId get_source_id() const { return source_id_; }
  const systems::OutputPort<double>& get_pose_output_port() const {
    return systems::System<double>::get_output_port(0);
  }

  // Method used to bring frame ids and poses out of sync. Adds a frame that
  // will *not* automatically get a pose.
  void add_extra_frame(bool add_to_output = true) {
    FrameId frame_id =
        scene_graph_->RegisterFrame(source_id_, GeometryFrame("frame"));
    if (add_to_output) extra_frame_ids_.push_back(frame_id);
  }
  // Method used to bring frame ids and poses out of sync. Adds a pose in
  // addition to all of the default poses.
  void add_extra_pose() { extra_poses_.push_back(RigidTransformd()); }

  static constexpr char kRegisteredSourceName[] = "source_system";

 private:
  // Populate with the pose data.
  void CalcFramePoseOutput(const Context<double>& context,
                           FramePoseVector<double>* poses) const {
    poses->clear();

    const int base_count = static_cast<int>(frame_ids_.size());
    for (int i = 0; i < base_count; ++i) {
      poses->set_value(frame_ids_[i], RigidTransformd::Identity());
    }

    const int extra_count = static_cast<int>(extra_frame_ids_.size());
    for (int i = 0; i < extra_count; ++i) {
      poses->set_value(extra_frame_ids_[i], extra_poses_[i]);
    }
  }

  SceneGraph<double>* scene_graph_{nullptr};
  SourceId source_id_;
  std::vector<FrameId> frame_ids_;
  std::vector<FrameId> extra_frame_ids_;
  std::vector<RigidTransformd> extra_poses_;
};

// Simple test case; system registers frames and provides correct connections.
GTEST_TEST(SceneGraphConnectionTest, FullPoseUpdateConnected) {
  // Build a fully connected system.
  systems::DiagramBuilder<double> builder;
  auto scene_graph = builder.AddSystem<SceneGraph<double>>();
  scene_graph->set_name("scene_graph");
  auto source_system = builder.AddSystem<GeometrySourceSystem>(scene_graph);
  source_system->set_name("source_system");
  SourceId source_id = source_system->get_source_id();
  builder.Connect(source_system->get_pose_output_port(),
                  scene_graph->get_source_pose_port(source_id));
  auto diagram = builder.Build();

  auto diagram_context = diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  const auto& sg_context =
      diagram->GetMutableSubsystemContext(*scene_graph, diagram_context.get());
  DRAKE_EXPECT_NO_THROW(
      SceneGraphTester::FullPoseUpdate(*scene_graph, sg_context));
}

// Adversarial test case: Missing pose port connection.
GTEST_TEST(SceneGraphConnectionTest, FullPoseUpdateDisconnected) {
  // Build a fully connected system.
  systems::DiagramBuilder<double> builder;
  auto scene_graph = builder.AddSystem<SceneGraph<double>>();
  scene_graph->set_name("scene_graph");
  auto source_system = builder.AddSystem<GeometrySourceSystem>(scene_graph);
  source_system->set_name("source_system");
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  const auto& sg_context =
      diagram->GetMutableSubsystemContext(*scene_graph, diagram_context.get());
  DRAKE_EXPECT_THROWS_MESSAGE(
      SceneGraphTester::FullPoseUpdate(*scene_graph, sg_context),
      std::logic_error,
      fmt::format("Source '{}' \\(id: \\d+\\) has registered dynamic frames "
                  "but is not connected .+",
                  GeometrySourceSystem::kRegisteredSourceName));
}

// Adversarial test case: Missing all port connections.
GTEST_TEST(SceneGraphConnectionTest, FullPoseUpdateNoConnections) {
  // Build a fully connected system.
  systems::DiagramBuilder<double> builder;
  auto scene_graph = builder.AddSystem<SceneGraph<double>>();
  scene_graph->set_name("scene_graph");
  auto source_system = builder.AddSystem<GeometrySourceSystem>(scene_graph);
  source_system->set_name("source_system");
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  const auto& sg_context =
      diagram->GetMutableSubsystemContext(*scene_graph, diagram_context.get());
  DRAKE_EXPECT_THROWS_MESSAGE(
      SceneGraphTester::FullPoseUpdate(*scene_graph, sg_context),
      std::logic_error,
      fmt::format("Source '{}' \\(id: \\d+\\) has registered dynamic frames "
                  "but is not connected .+",
                  GeometrySourceSystem::kRegisteredSourceName));
}

// Confirms that the SceneGraph can be instantiated on AutoDiff type.
GTEST_TEST(SceneGraphAutoDiffTest, InstantiateAutoDiff) {
  SceneGraph<AutoDiffXd> scene_graph;
  scene_graph.RegisterSource("dummy_source");
  auto context = scene_graph.CreateDefaultContext();

  QueryObject<AutoDiffXd> handle =
      QueryObjectTest::MakeNullQueryObject<AutoDiffXd>();
  SceneGraphTester::GetQueryObjectPortValue(scene_graph, *context, &handle);
}

// Tests that exercise the Context-modifying API

// Test that geometries can be successfully added to an allocated context.
GTEST_TEST(SceneGraphContextModifier, RegisterGeometry) {
  // Initializes the scene graph and context.
  SceneGraph<double> scene_graph;
  SourceId source_id = scene_graph.RegisterSource("source");
  FrameId frame_id =
      scene_graph.RegisterFrame(source_id, GeometryFrame("frame"));
  auto context = scene_graph.CreateDefaultContext();

  // Confirms the state. NOTE: All subsequent actions modify `context` in place.
  // This allows us to use this same query_object and inspector throughout the
  // test without requiring any updates or changes to them.
  QueryObject<double> query_object;
  SceneGraphTester::GetQueryObjectPortValue(scene_graph, *context,
                                            &query_object);
  const auto& inspector = query_object.inspector();
  EXPECT_EQ(1, inspector.NumFramesForSource(source_id));
  EXPECT_EQ(0, inspector.NumGeometriesForFrame(frame_id));

  // Test registration of geometry onto _frame_.
  GeometryId sphere_id_1 = scene_graph.RegisterGeometry(
      context.get(), source_id, frame_id, make_sphere_instance());
  EXPECT_EQ(1, inspector.NumGeometriesForFrame(frame_id));
  EXPECT_EQ(frame_id, inspector.GetFrameId(sphere_id_1));

  // Test registration of geometry onto _geometry_.
  GeometryId sphere_id_2 = scene_graph.RegisterGeometry(
      context.get(), source_id, sphere_id_1, make_sphere_instance());
  EXPECT_EQ(2, inspector.NumGeometriesForFrame(frame_id));
  EXPECT_EQ(frame_id, inspector.GetFrameId(sphere_id_2));

  // Remove the geometry.
  DRAKE_EXPECT_NO_THROW(
      scene_graph.RemoveGeometry(context.get(), source_id, sphere_id_2));
  EXPECT_EQ(1, inspector.NumGeometriesForFrame(frame_id));
  DRAKE_EXPECT_THROWS_MESSAGE(
      inspector.GetFrameId(sphere_id_2),
      std::logic_error,
      "Referenced geometry \\d+ has not been registered.");
}

// A limited test -- the majority of this functionality is encoded in and tested
// via GeometryState. This is just a regression test to make sure SceneGraph's
// invocation of that function doesn't become corrupt.
GTEST_TEST(SceneGraphRenderTest, AddRenderer) {
  SceneGraph<double> scene_graph;

  DRAKE_EXPECT_NO_THROW(
      scene_graph.AddRenderer("unique", make_unique<DummyRenderEngine>()));

  // Non-unique renderer name.
  // NOTE: The error message is tested in geometry_state_test.cc.
  EXPECT_THROW(
      scene_graph.AddRenderer("unique", make_unique<DummyRenderEngine>()),
      std::logic_error);

  // Adding a renderer _after_ geometry registration. We rely on geometry state
  // tests to confirm that the right thing happens; this just confirms that it's
  // not an error in the SceneGraph API.
  SourceId s_id = scene_graph.RegisterSource("dummy");
  scene_graph.RegisterGeometry(
      s_id, scene_graph.world_frame_id(),
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                    make_unique<Sphere>(1.0), "sphere"));

  DRAKE_EXPECT_NO_THROW(
      scene_graph.AddRenderer("different", make_unique<DummyRenderEngine>()));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
