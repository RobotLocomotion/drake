#include "drake/geometry/scene_graph.h"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/make_mesh_for_deformable.h"
#include "drake/geometry/proximity_properties.h"
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
using internal::HydroelasticType;
using internal::kComplianceType;
using internal::kHydroGroup;
using internal::kPointStiffness;
using internal::kRezHint;
using math::RigidTransformd;
using std::make_unique;
using std::unique_ptr;
using symbolic::Expression;
using systems::Context;
using systems::OutputPortIndex;
using systems::System;

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
  static void FullConfigurationUpdate(const SceneGraph<T>& scene_graph,
                                      const Context<T>& context) {
    scene_graph.FullConfigurationUpdate(context);
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

  template <typename T>
  static int64_t ScalarConversionCount() {
    return SceneGraph<T>::scalar_conversion_count_;
  }
};

namespace {

// Convenience function for making a geometry instance.
std::unique_ptr<GeometryInstance> make_sphere_instance(double radius = 1.0) {
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
      "Querying source name for an invalid source id.*");
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
      scene_graph_.get_source_pose_port(fake_source),
      "Can't acquire pose port for unknown source id: \\d+.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph_.get_source_configuration_port(fake_source),
      "Can't acquire configuration port for unknown source id: \\d+.");
}

// Confirms that attempting to acquire input ports for valid sources for the
// first time *after* allocation is acceptable.
TEST_F(SceneGraphTest, AcquireInputPortsAfterAllocation) {
  SourceId id = scene_graph_.RegisterSource();
  DRAKE_EXPECT_NO_THROW(scene_graph_.get_source_pose_port(id));
  DRAKE_EXPECT_NO_THROW(scene_graph_.get_source_configuration_port(id));
  CreateDefaultContext();
  // Ports which *hadn't* been accessed is still accessible.
  DRAKE_EXPECT_NO_THROW(scene_graph_.get_source_pose_port(id));
  DRAKE_EXPECT_NO_THROW(scene_graph_.get_source_configuration_port(id));
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
  GeometryId old_geometry_id =
      scene_graph_.RegisterGeometry(id, old_frame_id, make_sphere_instance());

  CreateDefaultContext();

  FrameId parent_frame_id =
      scene_graph_.RegisterFrame(id, GeometryFrame("parent"));
  FrameId child_frame_id =
      scene_graph_.RegisterFrame(id, parent_frame_id, GeometryFrame("child"));
  GeometryId geometry_id = scene_graph_.RegisterGeometry(
      id, parent_frame_id, make_sphere_instance());
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
  EXPECT_TRUE(model_inspector.BelongsToSource(geometry_id, id));
  EXPECT_TRUE(model_inspector.BelongsToSource(anchored_id, id));
  // Removed geometry from SceneGraph; "invalid" id throws.
  EXPECT_THROW(model_inspector.BelongsToSource(old_geometry_id, id),
               std::logic_error);

  EXPECT_THROW(context_inspector.BelongsToSource(parent_frame_id, id),
               std::logic_error);
  EXPECT_THROW(context_inspector.BelongsToSource(child_frame_id, id),
               std::logic_error);
  EXPECT_THROW(context_inspector.BelongsToSource(geometry_id, id),
               std::logic_error);
  EXPECT_THROW(context_inspector.BelongsToSource(anchored_id, id),
               std::logic_error);
  EXPECT_TRUE(context_inspector.BelongsToSource(old_geometry_id, id));
}

// Confirms that the direct feedthrough logic is correct -- there is total
// direct feedthrough. Also confirm that feedthrough was calculated without the
// need to fall back on SystemSymbolicInspector, and hence populate the
// internal augmented model cache for T=Expression.
TEST_F(SceneGraphTest, DirectFeedThrough) {
  scene_graph_.RegisterSource();
  int64_t tare = SceneGraphTester::ScalarConversionCount<Expression>();
  EXPECT_EQ(scene_graph_.GetDirectFeedthroughs().size(),
            scene_graph_.num_input_ports() * scene_graph_.num_output_ports());
  EXPECT_EQ(SceneGraphTester::ScalarConversionCount<Expression>(), tare);
}

// Test the functionality that accumulates the values from the input ports.

// Simple, toy case: there are no geometry sources; evaluate of pose update
// should be, essentially a no op.
TEST_F(SceneGraphTest, FullPoseUpdateEmpty) {
  CreateDefaultContext();
  DRAKE_EXPECT_NO_THROW(
      SceneGraphTester::FullPoseUpdate(scene_graph_, *context_));
}

// Simple, toy case: there are no deformable geometry sources; evaluate of
// configuration update should be, essentially a no-op.
TEST_F(SceneGraphTest, FullConfigurationUpdateEmpty) {
  CreateDefaultContext();
  DRAKE_EXPECT_NO_THROW(
      SceneGraphTester::FullConfigurationUpdate(scene_graph_, *context_));
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

// Smoke test for registering a deformable geometry
TEST_F(SceneGraphTest, RegisterDeformableGeometry) {
  SourceId s_id = scene_graph_.RegisterSource();
  // Register a rigid and a deformable geometry.
  GeometryId rigid_id = scene_graph_.RegisterGeometry(
      s_id, scene_graph_.world_frame_id(), make_sphere_instance());
  constexpr double kRezHint = 0.5;
  std::unique_ptr<GeometryInstance> geometry_instance = make_sphere_instance();
  GeometryId deformable_id = scene_graph_.RegisterDeformableGeometry(
      s_id, scene_graph_.world_frame_id(), std::move(geometry_instance),
      kRezHint);

  const SceneGraphInspector<double>& inspector = scene_graph_.model_inspector();
  EXPECT_EQ(nullptr, inspector.GetReferenceMesh(rigid_id));
  const VolumeMesh<double>* mesh_ptr =
      inspector.GetReferenceMesh(deformable_id);
  ASSERT_NE(mesh_ptr, nullptr);

  CreateDefaultContext();
  const QueryObject<double>& query_object = this->query_object();
  const VectorX<double> q_WG =
      VectorX<double>::Zero(mesh_ptr->num_vertices() * 3);
  GeometryConfigurationVector<double> configuration_vector;
  configuration_vector.set_value(deformable_id, q_WG);
  scene_graph_.get_source_configuration_port(s_id).FixValue(
      context_.get(), configuration_vector);
  EXPECT_EQ(query_object.GetConfigurationsInWorld(deformable_id), q_WG);

  DRAKE_EXPECT_THROWS_MESSAGE(
      query_object.GetConfigurationsInWorld(rigid_id),
      "Non-deformable geometries.*Use get_pose_in_world().*.");
}

// Smoke test for registering a deformable geometry
TEST_F(SceneGraphTest, RegisterUnsupportedDeformableGeometry) {
  constexpr double kRezHint = 0.5;
  auto geometry_instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Cylinder>(1.0, 2.0), "cylinder");
  SourceId s_id = scene_graph_.RegisterSource();
  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph_.RegisterDeformableGeometry(
          s_id, scene_graph_.world_frame_id(), std::move(geometry_instance),
          kRezHint),
      ".*don't yet generate deformable meshes.+ Cylinder.*");
}

// Test application of defaults during context allocation.
TEST_F(SceneGraphTest, ApplyConfig) {
  EXPECT_EQ(
      scene_graph_.get_config().default_proximity_properties.compliance_type,
      "undefined");
  SourceId s_id = scene_graph_.RegisterSource();
  auto geometry_instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Box>(1.0, 2.0, 3.0), "box");
  geometry_instance->set_proximity_properties(ProximityProperties());
  auto g_id = scene_graph_.RegisterGeometry(s_id, scene_graph_.world_frame_id(),
                                            std::move(geometry_instance));
  // Plain context.
  CreateDefaultContext();
  // No hydroelastic mesh available.
  EXPECT_EQ(
      query_object().inspector().maybe_get_hydroelastic_mesh(g_id).index(), 0);

  SceneGraphConfig config;
  config.default_proximity_properties.compliance_type = "compliant";
  scene_graph_.set_config(config);
  const auto& defaults = scene_graph_.get_config().default_proximity_properties;
  EXPECT_EQ(defaults.compliance_type, "compliant");
  // Hydroelastic-compliant configured context.
  CreateDefaultContext();
  // Volume hydroelastic mesh available.
  EXPECT_EQ(
      query_object().inspector().maybe_get_hydroelastic_mesh(g_id).index(), 2);

  // The tests below demonstrate modifications to geometry directly on the
  // context are subject to applying the default properties.

  // Add a new shape into the context, with empty proximity role. The resulting
  // registered geometry has proximity defaults applied.
  geometry_instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Box>(1.0, 3.0, 5.0), "box2");
  geometry_instance->set_proximity_properties(ProximityProperties());
  g_id = scene_graph_.RegisterGeometry(context_.get(), s_id,
                                       scene_graph_.world_frame_id(),
                                       std::move(geometry_instance));
  // Volume hydroelastic mesh available.
  EXPECT_EQ(
      query_object().inspector().maybe_get_hydroelastic_mesh(g_id).index(), 2);

  // Add a new shape into the context, then assign an empty proximity role. The
  // resulting properties have proximity defaults applied.
  geometry_instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Box>(1.0, 1.0, 1.0), "box3");
  g_id = scene_graph_.RegisterGeometry(context_.get(), s_id,
                                       scene_graph_.world_frame_id(),
                                       std::move(geometry_instance));
  scene_graph_.AssignRole(context_.get(), s_id, g_id, ProximityProperties());
  // Volume hydroelastic mesh available.
  EXPECT_EQ(
      query_object().inspector().maybe_get_hydroelastic_mesh(g_id).index(), 2);

  // Replace the role, with a blank set of properties. The resulting properties
  // are not empty, but have the defaults applied.
  ASSERT_TRUE(scene_graph_.get_config(*context_)
                  .default_proximity_properties.resolution_hint.has_value());
  scene_graph_.AssignRole(context_.get(), s_id, g_id, ProximityProperties(),
                          RoleAssign::kReplace);
  auto* props = query_object().inspector().GetProximityProperties(g_id);
  EXPECT_TRUE(props->HasProperty(kHydroGroup, kRezHint));

  // Replace the role, removing a defaulted property. The property does not
  // get removed, because it is set in the context's scene graph config, and
  // those settings get reapplied during AssignRole().
  ASSERT_TRUE(scene_graph_.get_config(*context_)
                  .default_proximity_properties.resolution_hint.has_value());
  ProximityProperties edit_props(
      *query_object().inspector().GetProximityProperties(g_id));
  edit_props.RemoveProperty(kHydroGroup, kRezHint);
  scene_graph_.AssignRole(context_.get(), s_id, g_id, edit_props,
                          RoleAssign::kReplace);
  props = query_object().inspector().GetProximityProperties(g_id);
  EXPECT_TRUE(props->HasProperty(kHydroGroup, kRezHint));

  // Replace the role, removing a non-defaulted property. The property gets
  // removed, because it is not set in the context's scene graph config, and
  // reapplication during AssignRole() does not affect it.
  ASSERT_FALSE(scene_graph_.get_config(*context_)
                   .default_proximity_properties.slab_thickness.has_value());
  edit_props = *query_object().inspector().GetProximityProperties(g_id);
  edit_props.RemoveProperty(kHydroGroup, kPointStiffness);
  scene_graph_.AssignRole(context_.get(), s_id, g_id, edit_props,
                          RoleAssign::kReplace);
  props = query_object().inspector().GetProximityProperties(g_id);
  // The whole group was not removed.
  EXPECT_TRUE(props->HasGroup(kHydroGroup));
  // The specific non-default property was removed.
  EXPECT_FALSE(props->HasProperty(kHydroGroup, kPointStiffness));
}

template <typename T>
class TypedSceneGraphTest : public SceneGraphTest {
 public:
  TypedSceneGraphTest() = default;
};

TYPED_TEST_SUITE_P(TypedSceneGraphTest);

// Tests operations on a transmogrified SceneGraph. Whether a context has been
// allocated or not, subsequent operations should be allowed.
TYPED_TEST_P(TypedSceneGraphTest, TransmogrifyWithoutAllocation) {
  using U = TypeParam;
  SourceId s_id = this->scene_graph_.RegisterSource();
  // This should allow additional geometry registration.
  auto scene_graph_U = System<double>::ToScalarType<U>(this->scene_graph_);
  DRAKE_EXPECT_NO_THROW(
      scene_graph_U->RegisterAnchoredGeometry(s_id, make_sphere_instance()));

  // After allocation, registration should _still_ be valid.
  this->CreateDefaultContext();
  auto scene_graph_U2 = System<double>::ToScalarType<U>(this->scene_graph_);
  DRAKE_EXPECT_NO_THROW(
      scene_graph_U2->RegisterAnchoredGeometry(s_id, make_sphere_instance()));
}

// Tests that the ports are correctly mapped.
TYPED_TEST_P(TypedSceneGraphTest, TransmogrifyPorts) {
  using U = TypeParam;
  SourceId s_id = this->scene_graph_.RegisterSource();
  this->CreateDefaultContext();
  auto scene_graph_U = System<double>::ToScalarType<U>(this->scene_graph_);
  EXPECT_EQ(scene_graph_U->num_input_ports(),
            this->scene_graph_.num_input_ports());
  EXPECT_EQ(scene_graph_U->get_source_pose_port(s_id).get_index(),
            this->scene_graph_.get_source_pose_port(s_id).get_index());
  EXPECT_EQ(scene_graph_U->get_source_configuration_port(s_id).get_index(),
            this->scene_graph_.get_source_configuration_port(s_id).get_index());
  EXPECT_NO_THROW(scene_graph_U->CreateDefaultContext());
}

// Tests that the work to "set" the context values for the transmogrified system
// behaves correctly.
TYPED_TEST_P(TypedSceneGraphTest, TransmogrifyContext) {
  using U = TypeParam;
  SourceId s_id = this->scene_graph_.RegisterSource();
  // Register geometry that should be successfully transmogrified.
  GeometryId g_id =
      this->scene_graph_.RegisterAnchoredGeometry(s_id, make_sphere_instance());
  this->CreateDefaultContext();
  const Context<double>& context_T = *this->context_;
  // This should transmogrify the internal *model*, so when I allocate the
  // transmogrified context, I should get the "same" values (considering type
  // change).
  auto scene_graph_U = System<double>::ToScalarType<U>(this->scene_graph_);
  std::unique_ptr<Context<U>> context_U = scene_graph_U->CreateDefaultContext();

  // Extract the GeometryState and query some invariants on it directly.
  const GeometryState<U>& geo_state_U =
      SceneGraphTester::GetGeometryState(*scene_graph_U, *context_U);
  // If the anchored geometry were not ported over, this would throw an
  // exception.
  EXPECT_TRUE(geo_state_U.BelongsToSource(g_id, s_id));
  EXPECT_THROW(geo_state_U.BelongsToSource(GeometryId::get_new_id(), s_id),
               std::logic_error);

  // Quick reality check that this is still valid although unnecessary vis a
  // vis the GeometryState.
  DRAKE_EXPECT_NO_THROW(context_U->SetTimeStateAndParametersFrom(context_T));
}

// Tests Clone for a non-double SceneGraph.
// We'll rely on the lack of any exceptions as the success criterion;
// the cloning and conversion code has enough fail-fast checks built in.
TYPED_TEST_P(TypedSceneGraphTest, NonDoubleClone) {
  using U = TypeParam;
  std::unique_ptr<SceneGraph<U>> scene_graph_U =
      System<double>::ToScalarType<U>(this->scene_graph_);
  std::unique_ptr<SceneGraph<U>> copy;
  EXPECT_NO_THROW(copy = System<U>::Clone(*scene_graph_U));
  ASSERT_NE(copy, nullptr);
}

REGISTER_TYPED_TEST_SUITE_P(TypedSceneGraphTest,            //
                            TransmogrifyWithoutAllocation,  //
                            TransmogrifyPorts,              //
                            TransmogrifyContext,            //
                            NonDoubleClone);

using NonDoubleScalarTypes = ::testing::Types<AutoDiffXd, Expression>;
INSTANTIATE_TYPED_TEST_SUITE_P(My, TypedSceneGraphTest, NonDoubleScalarTypes);

// Tests Clone for a non-double SceneGraph.
// We'll rely on the lack of any exceptions as the success criterion;
// the cloning and conversion code has enough fail-fast checks built in.
TEST_F(SceneGraphTest, DoubleClone) {
  std::unique_ptr<SceneGraph<double>> copy;
  EXPECT_NO_THROW(copy = System<double>::Clone(scene_graph_));
  ASSERT_NE(copy, nullptr);
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

  constexpr double kRezHint = 0.5;
  GeometryId deformable_id = scene_graph_.RegisterDeformableGeometry(
      source_id, scene_graph_.world_frame_id(),
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                    make_unique<Sphere>(1.0),
                                    "deformable_sphere"),
      kRezHint);
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
  EXPECT_EQ(
      inspector.GetGeometryIdByName(scene_graph_.world_frame_id(),
                                    Role::kUnassigned, "deformable_sphere"),
      deformable_id);
}

// SceneGraph provides a thin wrapper on the GeometryState renderer
// configuration/introspection code. These tests are just smoke tests that the
// functions work. It relies on GeometryState to properly unit test the
// full behavior.
TEST_F(SceneGraphTest, RendererInSceneGraphSmokeTest) {
  for (bool add_as_unique : {true, false}) {
    SCOPED_TRACE(fmt::format("add_as_unique = {}", add_as_unique));

    // Test the renderer added to the SceneGraph.
    const std::string kRendererName = "bob";

    EXPECT_EQ(scene_graph_.RendererCount(), 0);
    EXPECT_EQ(scene_graph_.RegisteredRendererNames().size(), 0u);
    EXPECT_FALSE(scene_graph_.HasRenderer(kRendererName));

    if (add_as_unique) {
      DRAKE_EXPECT_NO_THROW(scene_graph_.AddRenderer(
          kRendererName, make_unique<DummyRenderEngine>()));
    } else {
      DRAKE_EXPECT_NO_THROW(
          scene_graph_.AddRenderer(kRendererName, DummyRenderEngine()));
    }

    EXPECT_EQ(scene_graph_.RendererCount(), 1);
    EXPECT_EQ(scene_graph_.RegisteredRendererNames()[0], kRendererName);
    EXPECT_TRUE(scene_graph_.HasRenderer(kRendererName));

    DRAKE_EXPECT_NO_THROW(scene_graph_.RemoveRenderer(kRendererName));
    EXPECT_EQ(scene_graph_.RendererCount(), 0);
    EXPECT_FALSE(scene_graph_.HasRenderer(kRendererName));
  }
}

TEST_F(SceneGraphTest, RendererInContextSmokeTest) {
  for (bool add_as_unique : {true, false}) {
    SCOPED_TRACE(fmt::format("add_as_unique = {}", add_as_unique));
    // Test the renderer added to the context
    CreateDefaultContext();
    const std::string kRendererName = "bob";

    EXPECT_EQ(scene_graph_.RendererCount(*context_), 0);
    EXPECT_EQ(scene_graph_.RegisteredRendererNames(*context_).size(), 0u);
    EXPECT_FALSE(scene_graph_.HasRenderer(*context_, kRendererName));

    if (add_as_unique) {
      DRAKE_EXPECT_NO_THROW(scene_graph_.AddRenderer(
          context_.get(), kRendererName, make_unique<DummyRenderEngine>()));
    } else {
      DRAKE_EXPECT_NO_THROW(scene_graph_.AddRenderer(
          context_.get(), kRendererName, DummyRenderEngine()));
    }

    EXPECT_EQ(scene_graph_.RendererCount(*context_), 1);
    // No renderer inside SceneGraph since the renderer is added to the context.
    EXPECT_EQ(scene_graph_.RendererCount(), 0);
    EXPECT_EQ(scene_graph_.RegisteredRendererNames(*context_)[0],
              kRendererName);
    EXPECT_TRUE(scene_graph_.HasRenderer(*context_, kRendererName));

    DRAKE_EXPECT_NO_THROW(
        scene_graph_.RemoveRenderer(context_.get(), kRendererName));
    EXPECT_EQ(scene_graph_.RendererCount(*context_), 0);
    EXPECT_FALSE(scene_graph_.HasRenderer(*context_, kRendererName));
  }
}

// Query the type name of a render engine. This logic is unique to SceneGraph
// so we test it here.
TEST_F(SceneGraphTest, GetRendererTypeName) {
  const std::string kRendererName1 = "bob";
  const std::string kRendererName2 = "alice";

  CreateDefaultContext();
  DRAKE_EXPECT_NO_THROW(scene_graph_.AddRenderer(
      kRendererName1, make_unique<DummyRenderEngine>()));
  DRAKE_EXPECT_NO_THROW(scene_graph_.AddRenderer(
      context_.get(), kRendererName2, make_unique<DummyRenderEngine>()));

  // If no renderer has the name, the type doesn't matter.
  EXPECT_EQ(scene_graph_.GetRendererTypeName("non-existent"), "");
  EXPECT_EQ(scene_graph_.GetRendererTypeName(*context_, "non-existent"), "");

  // Get the expected name.
  EXPECT_EQ(scene_graph_.GetRendererTypeName(kRendererName1),
            NiceTypeName::Get<DummyRenderEngine>());
  EXPECT_EQ(scene_graph_.GetRendererTypeName(*context_, kRendererName2),
            NiceTypeName::Get<DummyRenderEngine>());
}

TEST_F(SceneGraphTest, GetRendererParameterYaml) {
  const std::string kRendererName1 = "bob";
  const std::string kRendererName2 = "alice";

  CreateDefaultContext();
  DRAKE_EXPECT_NO_THROW(scene_graph_.AddRenderer(
      kRendererName1, make_unique<DummyRenderEngine>()));
  DRAKE_EXPECT_NO_THROW(scene_graph_.AddRenderer(
      context_.get(), kRendererName2, make_unique<DummyRenderEngine>()));

  // If no renderer has the name, the parameter string is empty.
  EXPECT_EQ(scene_graph_.GetRendererParameterYaml("non-existent"), "");
  EXPECT_EQ(scene_graph_.GetRendererParameterYaml(*context_, "non-existent"),
            "");

  // Confirm that the string is *not* empty for valid names. We won't worry
  // about the string contents; it has been tested elsewhere.
  EXPECT_FALSE(scene_graph_.GetRendererParameterYaml(kRendererName1).empty());
  EXPECT_FALSE(
      scene_graph_.GetRendererParameterYaml(*context_, kRendererName2).empty());
}

TEST_F(SceneGraphTest, RemoveRenderer) {
  const std::string kRendererName = "bob";

  scene_graph_.AddRenderer(kRendererName, make_unique<DummyRenderEngine>());

  EXPECT_TRUE(scene_graph_.HasRenderer(kRendererName));

  scene_graph_.RemoveRenderer(kRendererName);

  EXPECT_FALSE(scene_graph_.HasRenderer(kRendererName));

  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph_.RemoveRenderer("bad_name"),
      ".* A renderer with the name 'bad_name' does not exist");
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

  GeometryId g_id =
      scene_graph_.RegisterGeometry(s_id, f_id, std::move(instance));

  const SceneGraphInspector<double>& inspector = scene_graph_.model_inspector();
  EXPECT_NE(inspector.GetProximityProperties(g_id), nullptr);
  EXPECT_NE(inspector.GetIllustrationProperties(g_id), nullptr);
  EXPECT_NE(inspector.GetPerceptionProperties(g_id), nullptr);

  EXPECT_EQ(scene_graph_.RemoveRole(s_id, g_id, Role::kPerception), 1);
  EXPECT_EQ(scene_graph_.RemoveRole(s_id, g_id, Role::kProximity), 1);
  EXPECT_EQ(scene_graph_.RemoveRole(s_id, g_id, Role::kIllustration), 1);
}

// For SceneGraph::ChangeShape to be correct, we need to make sure it invokes
// GeometryState correctly. So, we'll invoke it and look for minimum evidence
// that the change has happened (i.e., we report a different shape) and rely
// on GeometryState tests to confirm that all other ancillary details are also
// taken care of.
TEST_F(SceneGraphTest, ChangeShape) {
  const SourceId source_id = scene_graph_.RegisterSource();
  const Sphere sphere(1.0);
  const RigidTransformd X_WG_original(Eigen::Vector3d(1, 2, 3));
  GeometryId g_id = scene_graph_.RegisterAnchoredGeometry(
      source_id, make_unique<GeometryInstance>(
                     X_WG_original, make_unique<Sphere>(sphere), "sphere"));
  CreateDefaultContext();

  const SceneGraphInspector<double>& model_inspector =
      scene_graph_.model_inspector();
  const SceneGraphInspector<double>& context_inspector =
      query_object().inspector();

  // Confirm the shape in the model and context is of the expected type.
  ASSERT_EQ(sphere.type_name(), model_inspector.GetShape(g_id).type_name());
  ASSERT_EQ(sphere.type_name(), context_inspector.GetShape(g_id).type_name());

  // Change shape without changing pose.
  const Box box(1.5, 2.5, 3.5);

  scene_graph_.ChangeShape(source_id, g_id, box);
  EXPECT_EQ(box.type_name(), model_inspector.GetShape(g_id).type_name());
  EXPECT_TRUE(
      CompareMatrices(X_WG_original.GetAsMatrix34(),
                      model_inspector.GetPoseInFrame(g_id).GetAsMatrix34()));

  scene_graph_.ChangeShape(context_.get(), source_id, g_id, box);
  EXPECT_EQ(box.type_name(), context_inspector.GetShape(g_id).type_name());
  EXPECT_TRUE(
      CompareMatrices(X_WG_original.GetAsMatrix34(),
                      context_inspector.GetPoseInFrame(g_id).GetAsMatrix34()));

  // Change shape and pose.
  const Cylinder cylinder(1.5, 2.5);
  // We just need *some* different transformation that isn't the identity. So,
  // we'll transform the original non-identity pose by itself to get something
  // unique. The apparent frame anarchy is unimportant.
  const RigidTransformd X_WG_new = X_WG_original * X_WG_original;

  scene_graph_.ChangeShape(source_id, g_id, cylinder, X_WG_new);
  EXPECT_EQ(cylinder.type_name(), model_inspector.GetShape(g_id).type_name());
  EXPECT_TRUE(
      CompareMatrices(X_WG_new.GetAsMatrix34(),
                      model_inspector.GetPoseInFrame(g_id).GetAsMatrix34()));

  scene_graph_.ChangeShape(context_.get(), source_id, g_id, cylinder, X_WG_new);
  EXPECT_EQ(cylinder.type_name(), context_inspector.GetShape(g_id).type_name());
  EXPECT_TRUE(
      CompareMatrices(X_WG_new.GetAsMatrix34(),
                      context_inspector.GetPoseInFrame(g_id).GetAsMatrix34()));
}

// Dummy system to serve as geometry source.
class GeometrySourceSystem : public systems::LeafSystem<double> {
 public:
  // Constructs a new GeometrySourceSystem, register it as a source with the
  // given `scene_graph`, adds a non-deformable geometry, and optionally adds
  // a deformable geometry.
  explicit GeometrySourceSystem(SceneGraph<double>* scene_graph,
                                bool add_deformable = true)
      : systems::LeafSystem<double>(), scene_graph_(scene_graph) {
    // Register with SceneGraph.
    registered_source_name_ =
        add_deformable ? "mixed_source" : "non-deformable_only_source";
    source_id_ = scene_graph->RegisterSource(registered_source_name_);
    FrameId f_id =
        scene_graph->RegisterFrame(source_id_, GeometryFrame("frame"));
    frame_ids_.push_back(f_id);

    if (add_deformable) {
      constexpr double kRezHint = 0.5;
      GeometryId deformable_id = scene_graph->RegisterDeformableGeometry(
          source_id_, scene_graph->world_frame_id(), make_sphere_instance(),
          kRezHint);

      const SceneGraphInspector<double>& inspector =
          scene_graph->model_inspector();
      const VolumeMesh<double>* mesh_ptr =
          inspector.GetReferenceMesh(deformable_id);
      DRAKE_DEMAND(mesh_ptr != nullptr);
      configurations_.set_value(
          deformable_id, VectorX<double>::Zero(mesh_ptr->num_vertices() * 3));
    }

    // Set up output pose port now that the frame is registered.
    pose_port_index_ =
        this->DeclareAbstractOutputPort(
                "pose", &GeometrySourceSystem::CalcFramePoseOutput)
            .get_index();

    // Set up output configuration port.
    configuration_port_index_ =
        this->DeclareAbstractOutputPort(
                "configuration", &GeometrySourceSystem::CalcConfigurationOutput)
            .get_index();
  }

  SourceId get_source_id() const { return source_id_; }
  const systems::OutputPort<double>& get_pose_output_port() const {
    return systems::System<double>::get_output_port(pose_port_index_);
  }
  const systems::OutputPort<double>& get_configuration_output_port() const {
    return systems::System<double>::get_output_port(configuration_port_index_);
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

  const std::string& registered_source_name() const {
    return registered_source_name_;
  }

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

  // Dummy calc function for configurations.
  void CalcConfigurationOutput(
      const Context<double>& context,
      GeometryConfigurationVector<double>* configurations) const {
    *configurations = configurations_;
  }

  std::string registered_source_name_;
  SceneGraph<double>* scene_graph_{nullptr};
  SourceId source_id_;
  OutputPortIndex pose_port_index_;
  OutputPortIndex configuration_port_index_;
  std::vector<FrameId> frame_ids_;
  std::vector<FrameId> extra_frame_ids_;
  std::vector<RigidTransformd> extra_poses_;
  GeometryConfigurationVector<double> configurations_;
};

// Simple test case; system registers frames/geometries and provides correct
// connections.
GTEST_TEST(SceneGraphConnectionTest, FullPositionUpdateConnected) {
  // Build a fully connected system.
  systems::DiagramBuilder<double> builder;
  auto scene_graph = builder.AddSystem<SceneGraph<double>>();
  scene_graph->set_name("scene_graph");
  auto mixed_source =
      builder.AddSystem<GeometrySourceSystem>(scene_graph, true);
  mixed_source->set_name("mixed source");
  auto nondeformable_source =
      builder.AddSystem<GeometrySourceSystem>(scene_graph, false);
  nondeformable_source->set_name("nondeformable_only_source");
  SourceId mixed_source_id = mixed_source->get_source_id();
  SourceId nondeformable_source_id = nondeformable_source->get_source_id();
  // For source with both deformable and nondeformable geometries, connect both
  // ports.
  builder.Connect(mixed_source->get_pose_output_port(),
                  scene_graph->get_source_pose_port(mixed_source_id));
  builder.Connect(mixed_source->get_configuration_output_port(),
                  scene_graph->get_source_configuration_port(mixed_source_id));
  // For source without deformable geometries, connect only the pose port.
  builder.Connect(nondeformable_source->get_pose_output_port(),
                  scene_graph->get_source_pose_port(nondeformable_source_id));
  auto diagram = builder.Build();

  auto diagram_context = diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  const auto& sg_context =
      diagram->GetMutableSubsystemContext(*scene_graph, diagram_context.get());
  DRAKE_EXPECT_NO_THROW(
      SceneGraphTester::FullPoseUpdate(*scene_graph, sg_context));
  DRAKE_EXPECT_NO_THROW(
      SceneGraphTester::FullConfigurationUpdate(*scene_graph, sg_context));

  // Also check that scene graph methods validate contexts.
  auto& ms_context =
      diagram->GetMutableSubsystemContext(*mixed_source, diagram_context.get());
  // Check an ordinary method; glass-box knowledge shows that these all are
  // implemented on the private mutable_geometry_state() method, which checks
  // its context.
  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph->collision_filter_manager(diagram_context.get()),
      ".*\n.*context-system-mismatch.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      scene_graph->collision_filter_manager(&ms_context),
      ".*\n.*context-system-mismatch.*");
}

// Adversarial test case: Missing port connections.
GTEST_TEST(SceneGraphConnectionTest, FullPoseUpdateDisconnected) {
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
      fmt::format("Source '{}' \\(id: \\d+\\) has registered dynamic frames "
                  "but is not connected .+",
                  source_system->registered_source_name()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      SceneGraphTester::FullConfigurationUpdate(*scene_graph, sg_context),
      fmt::format(
          "Source '{}' \\(id: \\d+\\) has registered deformable geometry "
          "but is not connected .+",
          source_system->registered_source_name()));
}

GTEST_TEST(SceneGraphConnectionTest, NanInPoseInputs) {
  SceneGraph<double> sg;
  const SourceId s_id = sg.RegisterSource("nan_port");
  const FrameId f_id = sg.RegisterFrame(s_id, GeometryFrame("frame"));
  std::unique_ptr<Context<double>> context = sg.CreateDefaultContext();

  FramePoseVector<double> poses;
  RigidTransformd nan_pose(
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
  poses.set_value(f_id, nan_pose);

  sg.get_source_pose_port(s_id).FixValue(context.get(), poses);

  DRAKE_EXPECT_THROWS_MESSAGE(
      SceneGraphTester::FullPoseUpdate(sg, *context),
      ".*non-finite value .* on a pose input port.*'nan_port'.*");
}

GTEST_TEST(SceneGraphConnectionTest, NanInConfigInputs) {
  SceneGraph<double> sg;
  const SourceId s_id = sg.RegisterSource("nan_port");

  // Sphere tessellated as an octahedron.
  GeometryId deformable_id = sg.RegisterDeformableGeometry(
      s_id, sg.world_frame_id(), make_sphere_instance(), 2.0);

  std::unique_ptr<Context<double>> context = sg.CreateDefaultContext();

  const SceneGraphInspector<double>& inspector = sg.model_inspector();
  const VolumeMesh<double>* mesh_ptr =
      inspector.GetReferenceMesh(deformable_id);
  DRAKE_DEMAND(mesh_ptr != nullptr);

  GeometryConfigurationVector<double> configs;
  Eigen::VectorX<double> qs =
      Eigen::VectorX<double>::Zero(mesh_ptr->num_vertices() * 3);
  qs[1] = std::numeric_limits<double>::quiet_NaN();
  configs.set_value(deformable_id, qs);

  sg.get_source_configuration_port(s_id).FixValue(context.get(), configs);

  DRAKE_EXPECT_THROWS_MESSAGE(
      SceneGraphTester::FullConfigurationUpdate(sg, *context),
      ".*non-finite value .* on a deformable configuration input "
      "port.*'nan_port'.*");
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

// Confirms that the SceneGraph can be instantiated on Expression type.
GTEST_TEST(SceneGraphExpressionTest, InstantiateExpression) {
  SceneGraph<Expression> scene_graph;
  scene_graph.RegisterSource("dummy_source");
  auto context = scene_graph.CreateDefaultContext();

  QueryObject<Expression> handle =
      QueryObjectTest::MakeNullQueryObject<Expression>();
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

  // Remove the geometry.
  DRAKE_EXPECT_NO_THROW(
      scene_graph.RemoveGeometry(context.get(), source_id, sphere_id_1));
  EXPECT_EQ(0, inspector.NumGeometriesForFrame(frame_id));
  DRAKE_EXPECT_THROWS_MESSAGE(
      inspector.GetFrameId(sphere_id_1),
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
