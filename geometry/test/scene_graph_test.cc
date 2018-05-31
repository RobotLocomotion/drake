#include "drake/geometry/scene_graph.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/shape_specification.h"
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

using systems::Context;
using systems::System;
using std::make_unique;
using std::unique_ptr;

// Friend class for working with QueryObjects in a test context.
class QueryObjectTester {
 public:
  QueryObjectTester() = delete;

  template <typename T>
  static QueryObject<T> MakeNullQueryObject() {
    return QueryObject<T>();
  }

  template <typename T>
  static void set_query_object(QueryObject<T>* q_object,
                               const SceneGraph<T>* scene_graph,
                               const GeometryContext<T>* context) {
    q_object->context_ = context;
    q_object->scene_graph_ = scene_graph;
  }
};

// Friend class for accessing SceneGraph protected/private functionality.
class SceneGraphTester {
 public:
  SceneGraphTester() = delete;

  template <typename T>
  static bool HasDirectFeedthrough(const SceneGraph<T>& scene_graph,
                                   int input_port, int output_port) {
    return scene_graph.DoHasDirectFeedthrough(input_port, output_port)
        .value_or(true);
  }

  template <typename T>
  static void FullPoseUpdate(const SceneGraph<T>& scene_graph,
                             const GeometryContext<T>& context) {
    scene_graph.FullPoseUpdate(context);
  }

  template <typename T>
  static void GetQueryObjectPortValue(const SceneGraph<T>& scene_graph,
                                      const systems::Context<T>& context,
                                      QueryObject<T>* handle) {
    scene_graph.CalcQueryObject(context, handle);
  }
};

namespace {

// Testing harness to facilitate working with/testing the SceneGraph. Before
// performing *any* queries in tests, `AllocateContext` must be explicitly
// invoked in the test.

class SceneGraphTest : public ::testing::Test {
 public:
  SceneGraphTest()
      : ::testing::Test(),
        query_object_(QueryObjectTester::MakeNullQueryObject<double>()) {}

 protected:
  void AllocateContext() {
    // TODO(SeanCurtis-TRI): This will probably have to be moved into an
    // explicit call so it can be run *after* topology has been set.
    context_ = scene_graph_.AllocateContext();
    geom_context_ = dynamic_cast<GeometryContext<double>*>(context_.get());
    ASSERT_NE(geom_context_, nullptr);
    QueryObjectTester::set_query_object(&query_object_, &scene_graph_,
                                        geom_context_);
  }

  const QueryObject<double>& query_object() const {
    // The `AllocateContext()` method must have been called *prior* to this
    // method.
    if (!geom_context_)
      throw std::runtime_error("Must call AllocateContext() first.");
    return query_object_;
  }

  static std::unique_ptr<GeometryInstance> make_sphere_instance(
      double radius = 1.0) {
    return make_unique<GeometryInstance>(Isometry3<double>::Identity(),
                                         make_unique<Sphere>(radius));
  }

  SceneGraph<double> scene_graph_;
  // Ownership of context.
  unique_ptr<Context<double>> context_;
  // Direct access to a pre-cast, geometry-context-typed version of context_.
  GeometryContext<double>* geom_context_{nullptr};

 private:
  // Keep this private so tests must access it through the getter so we can
  // determine if AllocateContext() has been invoked.
  QueryObject<double> query_object_;
};

// Test sources.

// Tests registration using a default source name. Confirms that the source
// registered and that a name is available.
TEST_F(SceneGraphTest, RegisterSourceDefaultName) {
  SourceId id = scene_graph_.RegisterSource();
  EXPECT_TRUE(id.is_valid());
  AllocateContext();
  EXPECT_NO_THROW(query_object().GetSourceName(id));
  EXPECT_TRUE(scene_graph_.SourceIsRegistered(id));
}

// Tests registration using a specified source name. Confirms that the source
// registered and that the name is available.
TEST_F(SceneGraphTest, RegisterSourceSpecifiedName) {
  std::string name = "some_unique_name";
  SourceId id = scene_graph_.RegisterSource(name);
  EXPECT_TRUE(id.is_valid());
  AllocateContext();
  EXPECT_EQ(query_object().GetSourceName(id), name);
  EXPECT_TRUE(scene_graph_.SourceIsRegistered(id));
}

// Tests that sources cannot be registered after context allocation.
TEST_F(SceneGraphTest, PoseContextSourceRegistration) {
  AllocateContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph_.RegisterSource(), std::logic_error,
      "The call to RegisterSource is invalid; a context has already been "
      "allocated.");
}

// Tests ability to report if a source is registered or not.
TEST_F(SceneGraphTest, SourceIsRegistered) {
  SourceId id = scene_graph_.RegisterSource();
  AllocateContext();
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
  EXPECT_NO_THROW(scene_graph_.get_source_pose_port(id));
  AllocateContext();
  // Port which *hadn't* been accessed is still accessible.
  EXPECT_NO_THROW(scene_graph_.get_source_pose_port(id));
}

// Test topology changes (registration, removal, clearing, etc.)

// Tests that topology operations (registration, removal, clearing, etc.) after
// allocation is not allowed -- and an exception with an intelligible message is
// thrown. The underlying handling of the *values* of the parameters is handled
// in the GeometryState unit tests. SceneGraph merely confirms the context
// hasn't been allocated.
TEST_F(SceneGraphTest, TopologyAfterAllocation) {
  SourceId id = scene_graph_.RegisterSource();
  AllocateContext();

  // Attach frame to world.
  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph_.RegisterFrame(
          id, GeometryFrame("frame", Isometry3<double>::Identity())),
      std::logic_error,
      "The call to RegisterFrame is invalid; a context has already been "
      "allocated.");

  // Attach frame to another frame.
  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph_.RegisterFrame(
          id, FrameId::get_new_id(),
          GeometryFrame("frame", Isometry3<double>::Identity())),
      std::logic_error,
      "The call to RegisterFrame is invalid; a context has already been "
      "allocated.");

  // Attach geometry to frame.
  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph_.RegisterGeometry(id, FrameId::get_new_id(),
                                    make_sphere_instance()),
      std::logic_error,
      "The call to RegisterGeometry is invalid; a context has already been "
      "allocated.");

  // Attach geometry to another geometry.
  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph_.RegisterGeometry(id, GeometryId::get_new_id(),
                                    make_sphere_instance()),
      std::logic_error,
      "The call to RegisterGeometry is invalid; a context has already been "
      "allocated.");

  // Attach anchored geometry to world.
  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph_.RegisterAnchoredGeometry(id, make_sphere_instance()),
      std::logic_error,
      "The call to RegisterAnchoredGeometry is invalid; a context has already "
      "been allocated.");
}

// Confirms that the direct feedthrough logic is correct -- there is total
// direct feedthrough.
TEST_F(SceneGraphTest, DirectFeedThrough) {
  SourceId id = scene_graph_.RegisterSource();
  std::vector<int> input_ports{
      scene_graph_.get_source_pose_port(id).get_index()};
  for (int input_port_id : input_ports) {
    EXPECT_TRUE(SceneGraphTester::HasDirectFeedthrough(
        scene_graph_, input_port_id,
        scene_graph_.get_query_output_port().get_index()));
  }
  // TODO(SeanCurtis-TRI): Update when the pose bundle output is added; it has
  // direct feedthrough as well.
}

// Test the functionality that accumulates the values from the input ports.

// Simple, toy case: there are no geometry sources; evaluate of pose update
// should be, essentially a no op.
TEST_F(SceneGraphTest, FullPoseUpdateEmpty) {
  AllocateContext();
  EXPECT_NO_THROW(
      SceneGraphTester::FullPoseUpdate(scene_graph_, *geom_context_));
}

// Test case where there are only anchored geometries -- same as the empty case;
// no geometry to update.
TEST_F(SceneGraphTest, FullPoseUpdateAnchoredOnly) {
  SourceId s_id = scene_graph_.RegisterSource();
  scene_graph_.RegisterAnchoredGeometry(s_id, make_sphere_instance());
  AllocateContext();
  EXPECT_NO_THROW(
      SceneGraphTester::FullPoseUpdate(scene_graph_, *geom_context_));
}

// Tests transmogrification of SceneGraph in the case where a Context has
// *not* been allocated yet. Registration should still be possible.
TEST_F(SceneGraphTest, TransmogrifyWithoutAllocation) {
  SourceId s_id = scene_graph_.RegisterSource();
  // This should allow additional geometry registration.
  std::unique_ptr<systems::System<AutoDiffXd>> system_ad =
      scene_graph_.ToAutoDiffXd();
  SceneGraph<AutoDiffXd>& scene_graph_ad =
      *dynamic_cast<SceneGraph<AutoDiffXd>*>(system_ad.get());
  EXPECT_NO_THROW(
      scene_graph_ad.RegisterAnchoredGeometry(s_id, make_sphere_instance()));

  // After allocation, registration should *not* be valid.
  AllocateContext();
  system_ad = scene_graph_.ToAutoDiffXd();
  SceneGraph<AutoDiffXd>& scene_graph_ad2 =
      *dynamic_cast<SceneGraph<AutoDiffXd>*>(system_ad.get());
  EXPECT_THROW(
      scene_graph_ad2.RegisterAnchoredGeometry(s_id, make_sphere_instance()),
      std::logic_error);
}

// Tests that the ports are correctly mapped.
TEST_F(SceneGraphTest, TransmogrifyPorts) {
  SourceId s_id = scene_graph_.RegisterSource();
  AllocateContext();
  std::unique_ptr<systems::System<AutoDiffXd>> system_ad =
      scene_graph_.ToAutoDiffXd();
  SceneGraph<AutoDiffXd>& scene_graph_ad =
      *dynamic_cast<SceneGraph<AutoDiffXd>*>(system_ad.get());
  EXPECT_EQ(scene_graph_ad.get_num_input_ports(),
            scene_graph_.get_num_input_ports());
  EXPECT_EQ(scene_graph_ad.get_source_pose_port(s_id).get_index(),
            scene_graph_.get_source_pose_port(s_id).get_index());
  std::unique_ptr<systems::Context<AutoDiffXd>> context_ad =
      scene_graph_ad.AllocateContext();
}

// Tests that the work to "set" the context values for the transmogrified system
// behaves correctly.
TEST_F(SceneGraphTest, TransmogrifyContext) {
  SourceId s_id = scene_graph_.RegisterSource();
  // Register geometry that should be successfully transmogrified.
  GeometryId g_id =
      scene_graph_.RegisterAnchoredGeometry(s_id, make_sphere_instance());
  AllocateContext();
  std::unique_ptr<System<AutoDiffXd>> system_ad = scene_graph_.ToAutoDiffXd();
  SceneGraph<AutoDiffXd>& scene_graph_ad =
      *dynamic_cast<SceneGraph<AutoDiffXd>*>(system_ad.get());
  std::unique_ptr<Context<AutoDiffXd>> context_ad =
      scene_graph_ad.AllocateContext();
  context_ad->SetTimeStateAndParametersFrom(*geom_context_);
  GeometryContext<AutoDiffXd>* geo_context_ad =
      dynamic_cast<GeometryContext<AutoDiffXd>*>(context_ad.get());
  ASSERT_NE(geo_context_ad, nullptr);
  // If the anchored geometry were not ported over, this would throw an
  // exception.
  EXPECT_TRUE(geo_context_ad->get_geometry_state().BelongsToSource(g_id, s_id));
  EXPECT_THROW(geo_context_ad->get_geometry_state().BelongsToSource(
                   GeometryId::get_new_id(), s_id),
               std::logic_error);
}

// Tests that exercising the collision filtering logic *after* allocation leads
// to an exception being thrown.
TEST_F(SceneGraphTest, PostAllocationCollisionFiltering) {
  AllocateContext();

  GeometrySet geometry_set1{FrameId::get_new_id()};
  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph_.ExcludeCollisionsWithin(geometry_set1), std::logic_error,
      "The call to ExcludeCollisionsWithin is invalid; a context has already "
      "been allocated.");

  GeometrySet geometry_set2{FrameId::get_new_id()};
  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph_.ExcludeCollisionsBetween(geometry_set1, geometry_set2),
      std::logic_error,
      "The call to ExcludeCollisionsBetween is invalid; a context has already "
      "been allocated.");
}

// Dummy system to serve as geometry source.
class GeometrySourceSystem : public systems::LeafSystem<double> {
 public:
  explicit GeometrySourceSystem(SceneGraph<double>* scene_graph)
      : systems::LeafSystem<double>(), scene_graph_(scene_graph) {
    // Register with SceneGraph.
    source_id_ = scene_graph->RegisterSource();
    FrameId f_id = scene_graph->RegisterFrame(
        source_id_, GeometryFrame("frame", Isometry3<double>::Identity()));
    frame_ids_.push_back(f_id);

    // Set up output port now that the frame is registered.
    std::vector<FrameId> all_ids{f_id};
    all_ids.insert(all_ids.end(), extra_frame_ids_.begin(),
                   extra_frame_ids_.end());
    this->DeclareAbstractOutputPort(
        FramePoseVector<double>(source_id_, all_ids),
        &GeometrySourceSystem::CalcFramePoseOutput);
  }
  SourceId get_source_id() const { return source_id_; }
  const systems::OutputPort<double>& get_pose_output_port() const {
    return systems::System<double>::get_output_port(0);
  }

  // Method used to bring frame ids and poses out of sync. Adds a frame that
  // will *not* automatically get a pose.
  void add_extra_frame(bool add_to_output = true) {
    FrameId frame_id = scene_graph_->RegisterFrame(
        source_id_, GeometryFrame("frame", Isometry3<double>::Identity()));
    if (add_to_output) extra_frame_ids_.push_back(frame_id);
  }
  // Method used to bring frame ids and poses out of sync. Adds a pose in
  // addition to all of the default poses.
  void add_extra_pose() { extra_poses_.push_back(Isometry3<double>()); }

 private:
  // Populate with the pose data.
  void CalcFramePoseOutput(const Context<double>& context,
                           FramePoseVector<double>* poses) const {
    const int frame_count =
        static_cast<int>(frame_ids_.size() + extra_frame_ids_.size());
    DRAKE_DEMAND(poses->size() == frame_count);
    DRAKE_DEMAND(poses->source_id() == source_id_);

    poses->clear();

    const int base_count = static_cast<int>(frame_ids_.size());
    for (int i = 0; i < base_count; ++i) {
      poses->set_value(frame_ids_[i], Isometry3<double>::Identity());
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
  std::vector<Isometry3<double>> extra_poses_;
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

  auto diagram_context = diagram->AllocateContext();
  diagram->SetDefaultContext(diagram_context.get());
  auto& geometry_context = dynamic_cast<GeometryContext<double>&>(
      diagram->GetMutableSubsystemContext(*scene_graph, diagram_context.get()));
  EXPECT_NO_THROW(
      SceneGraphTester::FullPoseUpdate(*scene_graph, geometry_context));
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
  auto diagram_context = diagram->AllocateContext();
  diagram->SetDefaultContext(diagram_context.get());
  auto& geometry_context = dynamic_cast<GeometryContext<double>&>(
      diagram->GetMutableSubsystemContext(*scene_graph, diagram_context.get()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      SceneGraphTester::FullPoseUpdate(*scene_graph, geometry_context),
      std::logic_error,
      "Source \\d+ has registered frames but does not provide pose values on "
      "the input port.");
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
  auto diagram_context = diagram->AllocateContext();
  diagram->SetDefaultContext(diagram_context.get());
  auto& geometry_context = dynamic_cast<GeometryContext<double>&>(
      diagram->GetMutableSubsystemContext(*scene_graph, diagram_context.get()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      SceneGraphTester::FullPoseUpdate(*scene_graph, geometry_context),
      std::logic_error,
      "Source \\d+ has registered frames but does not provide pose values on "
      "the input port.");
}

// Confirms that the SceneGraph can be instantiated on AutoDiff type.
GTEST_TEST(SceneGraphAutoDiffTest, InstantiateAutoDiff) {
  SceneGraph<AutoDiffXd> scene_graph;
  scene_graph.RegisterSource("dummy_source");
  auto context = scene_graph.AllocateContext();
  GeometryContext<AutoDiffXd>* geometry_context =
      dynamic_cast<GeometryContext<AutoDiffXd>*>(context.get());

  ASSERT_NE(geometry_context, nullptr);

  QueryObject<AutoDiffXd> handle =
      QueryObjectTester::MakeNullQueryObject<AutoDiffXd>();
  SceneGraphTester::GetQueryObjectPortValue(scene_graph, *context, &handle);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
