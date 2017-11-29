#include "drake/geometry/geometry_system.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/query_handle.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/test_utilities/expect_error_message.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

// Test of the unique GeometrySystem operations. GeometrySystem is mostly a thin
// wrapper around GeometryWorld. It's purpose is to connect GeometryWorld to the
// Drake ecosystem. As such, there will be no tests on functional logic but just
// on that wrapping. For examples, queries simply extract a context from the
// QueryHandle and pass it to the GeometryWorld method. As such, there is
// nothing to test.

namespace drake {
namespace geometry {

using systems::Context;
using systems::System;
using std::make_unique;
using std::unique_ptr;

// Friend class for working with QueryHandles in a test context.
class QueryHandleTester {
 public:
  QueryHandleTester() = delete;

  template <typename T>
  static QueryHandle<T> MakeNullQueryHandle() {
    return QueryHandle<T>(nullptr, 0);
  }

  template <typename T>
  static QueryHandle<T> MakeQueryHandle(
      const GeometryContext<T>* context) {
    return QueryHandle<T>(context, 0);
  }

  template <typename T>
  static void set_context(QueryHandle<T>* handle,
                          const GeometryContext<T>* context) {
    handle->context_ = context;
    // NOTE: This does not set the hash because these tests do not depend on it
    // yet.
    // TODO(SeanCurtis-TRI): Set guard value. *Test* guard value.
  }
};

// Friend class for accessing GeometrySystem protected/private functionality.
class GeometrySystemTester {
 public:
  GeometrySystemTester() = delete;

  template <typename T>
  static bool HasDirectFeedthrough(const GeometrySystem<T>& system,
                                   int input_port, int output_port) {
    return system.DoHasDirectFeedthrough(
        input_port, output_port).value_or(true);
  }

  template <typename T>
  static void FullPoseUpdate(const GeometrySystem<T>& system,
                             const GeometryContext<T>& context) {
    system.FullPoseUpdate(context);
  }

  template <typename T>
  static std::vector<PenetrationAsPointPair<T>> ComputePenetration(
      const GeometrySystem<T>& system, const QueryHandle<T>& handle) {
    return system.ComputePenetration(handle);
  }

  template <typename T>
  static void GetQueryHandlePortValue(const GeometrySystem<T>& system,
                                      const systems::Context<T>& context,
                                      QueryHandle<T>* handle) {
    system.CalcQueryHandle(context, handle);
  }
};

namespace {

// Testing harness to facilitate working with/testing the GeometrySystem. Before
// performing *any* queries in tests, `AllocateContext` must be explicitly
// invoked in the test.

class GeometrySystemTest : public ::testing::Test {
 public:
  GeometrySystemTest()
      : ::testing::Test(),
        query_handle_(QueryHandleTester::MakeNullQueryHandle<double>()) {}

 protected:
  void AllocateContext() {
    // TODO(SeanCurtis-TRI): This will probably have to be moved into an
    // explicit call so it can be run *after* topology has been set.
    context_ = system_.AllocateContext();
    geom_context_ = dynamic_cast<GeometryContext<double>*>(context_.get());
    ASSERT_NE(geom_context_, nullptr);
    QueryHandleTester::set_context(&query_handle_, geom_context_);
  }

  const QueryHandle<double>& get_query_handle() const {
    // The `AllocateContext()` method must have been called *prior* to this
    // method.
    if (!geom_context_)
      throw std::runtime_error("Must call AllocateContext() first.");
    return query_handle_;
  }

  static std::unique_ptr<GeometryInstance> make_sphere_instance(
      double radius = 1.0) {
    return make_unique<GeometryInstance>(Isometry3<double>::Identity(),
                                         make_unique<Sphere>(radius));
  }

  GeometrySystem<double> system_;
  // Ownership of context.
  unique_ptr<Context<double>> context_;
  // Direct access to a pre-cast, geometry-context-typed version of context_.
  GeometryContext<double>* geom_context_{nullptr};

 private:
  // Keep this private so tests must access it through the getter so we can
  // determine if AllocateContext() has been invoked.
  QueryHandle<double> query_handle_;
};

// Test sources.

// Tests registration using a default source name. Confirms that the source
// registered and that a name is available.
TEST_F(GeometrySystemTest, RegisterSourceDefaultName) {
  SourceId id = system_.RegisterSource();
  EXPECT_TRUE(id.is_valid());
  AllocateContext();
  EXPECT_THROW(system_.get_source_name(get_query_handle(), id),
               std::runtime_error);
  EXPECT_TRUE(system_.SourceIsRegistered(id));
}

// Tests registration using a specified source name. Confirms that the source
// registered and that the name is available.
TEST_F(GeometrySystemTest, RegisterSourceSpecifiedName) {
  std::string name = "some_unique_name";
  SourceId id = system_.RegisterSource(name);
  EXPECT_TRUE(id.is_valid());
  AllocateContext();
  EXPECT_THROW(system_.get_source_name(get_query_handle(), id),
               std::runtime_error);
  EXPECT_TRUE(system_.SourceIsRegistered(id));
}

// Tests that sources cannot be registered after context allocation.
TEST_F(GeometrySystemTest, PoseContextSourceRegistration) {
  AllocateContext();
  EXPECT_ERROR_MESSAGE(
      system_.RegisterSource(),
      std::logic_error,
      "The call to RegisterSource is invalid; a context has already been "
      "allocated.");
}

// Tests ability to report if a source is registered or not.
TEST_F(GeometrySystemTest, SourceIsRegistered) {
  SourceId id = system_.RegisterSource();
  AllocateContext();
  EXPECT_TRUE(system_.SourceIsRegistered(id));
  EXPECT_FALSE(system_.SourceIsRegistered(SourceId::get_new_id()));
}

// Test ports.

// Confirms that attempting to acquire input ports for unregistered sources
// throws exceptions.
TEST_F(GeometrySystemTest, InputPortsForInvalidSource) {
  SourceId fake_source = SourceId::get_new_id();
  EXPECT_ERROR_MESSAGE(
      system_.get_source_frame_id_port(fake_source),
      std::logic_error,
      "Can't acquire id port for unknown source id: \\d+.");
  EXPECT_ERROR_MESSAGE(
      system_.get_source_pose_port(fake_source),
      std::logic_error,
      "Can't acquire pose port for unknown source id: \\d+.");
}

// Confirms that attempting to acquire input ports for valid sources for the
// first time *after* allocation is acceptable.
TEST_F(GeometrySystemTest, AcquireInputPortsAfterAllocation) {
  SourceId id = system_.RegisterSource();
  EXPECT_NO_THROW(system_.get_source_frame_id_port(id));
  AllocateContext();
  // Port previously accessed is still accessible.
  EXPECT_NO_THROW(system_.get_source_frame_id_port(id));
  // Port which *hadn't* been accessed is still accessible.
  EXPECT_NO_THROW(system_.get_source_pose_port(id));
}

// Test topology changes (registration, removal, clearing, etc.)

// Tests that topology operations (registration, removal, clearing, etc.) after
// allocation is not allowed -- and an exception with an intelligible message is
// thrown. The underlying handling of the *values* of the parameters is handled
// in the GeometryState unit tests. GeometrySystem merely confirms the context
// hasn't been allocated.
TEST_F(GeometrySystemTest, TopologyAfterAllocation) {
  SourceId id = system_.RegisterSource();
  AllocateContext();

  // Attach frame to world.
  EXPECT_ERROR_MESSAGE(
      system_.RegisterFrame(
          id, GeometryFrame("frame", Isometry3<double>::Identity())),
      std::logic_error,
      "The call to RegisterFrame is invalid; a context has already been "
      "allocated.");

  // Attach frame to another frame.
  EXPECT_ERROR_MESSAGE(
      system_.RegisterFrame(
          id, FrameId::get_new_id(),
          GeometryFrame("frame", Isometry3<double>::Identity())),
      std::logic_error,
      "The call to RegisterFrame is invalid; a context has already been "
      "allocated.");

  // Attach geometry to frame.
  EXPECT_ERROR_MESSAGE(
      system_.RegisterGeometry(
          id, FrameId::get_new_id(), make_sphere_instance()),
      std::logic_error,
      "The call to RegisterGeometry is invalid; a context has already been "
      "allocated.");

  // Attach geometry to another geometry.
  EXPECT_ERROR_MESSAGE(
      system_.RegisterGeometry(
          id, GeometryId::get_new_id(), make_sphere_instance()),
      std::logic_error,
      "The call to RegisterGeometry is invalid; a context has already been "
          "allocated.");

  // Attach anchored geometry to world.
  EXPECT_ERROR_MESSAGE(
      system_.RegisterAnchoredGeometry(id, make_sphere_instance()),
      std::logic_error,
      "The call to RegisterAnchoredGeometry is invalid; a context has already "
      "been allocated.");

  // Clearing a source.
  EXPECT_ERROR_MESSAGE(
      system_.ClearSource(id),
      std::logic_error,
      "The call to ClearSource is invalid; a context has already been "
      "allocated.");

  // Removing a frame.
  EXPECT_ERROR_MESSAGE(
      system_.RemoveFrame(id, FrameId::get_new_id()),
      std::logic_error,
      "The call to RemoveFrame is invalid; a context has already been "
      "allocated.");

  // Removing a geometry.
  EXPECT_ERROR_MESSAGE(
      system_.RemoveGeometry(id, GeometryId::get_new_id()),
      std::logic_error,
      "The call to RemoveGeometry is invalid; a context has already been "
      "allocated.");
}

// Confirms that the direct feedthrough logic is correct -- there is total
// direct feedthrough.
TEST_F(GeometrySystemTest, DirectFeedThrough) {
  SourceId id = system_.RegisterSource();
  std::vector<int> input_ports{
      system_.get_source_frame_id_port(id).get_index(),
      system_.get_source_pose_port(id).get_index()};
  for (int input_port_id : input_ports) {
    EXPECT_TRUE(GeometrySystemTester::HasDirectFeedthrough(
        system_, input_port_id, system_.get_query_output_port().get_index()));
  }
  // TODO(SeanCurtis-TRI): Update when the pose bundle output is added; it has
  // direct feedthrough as well.
}

// NOTE: There are no tests on the query methods: GetFrameId and ComputeContact.
// Ultimately, that functionality will lie in a different class and will be
// tested with *that* class. The tests included here, even those that currently
// only throw exceptions, will change when meaningful functionality is given to
// GeometrySystem.

// Test the functionality that accumulates the values from the input ports.

// Simple, toy case: there are no geometry sources; evaluate of pose update
// should be, essentially a no op.
TEST_F(GeometrySystemTest, FullPoseUpdateEmpty) {
  AllocateContext();
  EXPECT_NO_THROW(
      GeometrySystemTester::FullPoseUpdate(system_, *geom_context_));
}

// Test case where there are only anchored geometries -- same as the empty case;
// no geometry to update.
TEST_F(GeometrySystemTest, FullPoseUpdateAnchoredOnly) {
  SourceId s_id = system_.RegisterSource();
  system_.RegisterAnchoredGeometry(s_id, make_sphere_instance());
  AllocateContext();
  EXPECT_NO_THROW(
      GeometrySystemTester::FullPoseUpdate(system_, *geom_context_));
}

// Tests transmogrification of GeometrySystem in the case where a Context has
// *not* been allocated yet. Registration should still be possible.
TEST_F(GeometrySystemTest, TransmogrifyWithoutAllocation) {
  SourceId s_id = system_.RegisterSource();
  // This should allow additional geometry registration.
  std::unique_ptr<systems::System<AutoDiffXd>> system_ad =
      system_.ToAutoDiffXd();
  GeometrySystem<AutoDiffXd>& geo_system_ad =
      *dynamic_cast<GeometrySystem<AutoDiffXd>*>(system_ad.get());
  EXPECT_NO_THROW(
      geo_system_ad.RegisterAnchoredGeometry(s_id, make_sphere_instance()));

  // After allocation, registration should *not* be valid.
  AllocateContext();
  system_ad = system_.ToAutoDiffXd();
  GeometrySystem<AutoDiffXd>& geo_system_ad2 =
      *dynamic_cast<GeometrySystem<AutoDiffXd>*>(system_ad.get());
  EXPECT_THROW(
      geo_system_ad2.RegisterAnchoredGeometry(s_id, make_sphere_instance()),
      std::logic_error);
}

// Tests that the ports are correctly mapped.
TEST_F(GeometrySystemTest, TransmogrifyPorts) {
  SourceId s_id = system_.RegisterSource();
  AllocateContext();
  std::unique_ptr<systems::System<AutoDiffXd>> system_ad
      = system_.ToAutoDiffXd();
  GeometrySystem<AutoDiffXd>& geo_system_ad =
      *dynamic_cast<GeometrySystem<AutoDiffXd>*>(system_ad.get());
  EXPECT_EQ(geo_system_ad.get_num_input_ports(), system_.get_num_input_ports());
  EXPECT_EQ(geo_system_ad.get_source_frame_id_port(s_id).get_index(),
            system_.get_source_frame_id_port(s_id).get_index());
  EXPECT_EQ(geo_system_ad.get_source_pose_port(s_id).get_index(),
            system_.get_source_pose_port(s_id).get_index());
  std::unique_ptr<systems::Context<AutoDiffXd>> context_ad =
      geo_system_ad.AllocateContext();
}

// Tests that the work to "set" the context values for the transmogrified system
// behaves correctly.
TEST_F(GeometrySystemTest, TransmogrifyContext) {
  SourceId s_id = system_.RegisterSource();
  // Register geometry that should be successfully transmogrified.
  GeometryId g_id = system_.RegisterAnchoredGeometry(s_id,
                                                     make_sphere_instance());
  AllocateContext();
  std::unique_ptr<System<AutoDiffXd>> system_ad = system_.ToAutoDiffXd();
  GeometrySystem<AutoDiffXd>& geo_system_ad =
      *dynamic_cast<GeometrySystem<AutoDiffXd>*>(system_ad.get());
  std::unique_ptr<Context<AutoDiffXd>> context_ad =
      geo_system_ad.AllocateContext();
  context_ad->SetTimeStateAndParametersFrom(*geom_context_);
  GeometryContext<AutoDiffXd>* geo_context_ad =
      dynamic_cast<GeometryContext<AutoDiffXd>*>(context_ad.get());
  ASSERT_NE(geo_context_ad, nullptr);
  // If the anchored geometry were not ported over, this would throw an
  // exception.
  EXPECT_NO_THROW(
      geo_context_ad->get_mutable_geometry_state().RemoveGeometry(s_id, g_id));
  EXPECT_THROW(geo_context_ad->get_mutable_geometry_state().RemoveGeometry(
                   s_id, GeometryId::get_new_id()),
               std::logic_error);
}

// Dummy system to serve as geometry source.
class GeometrySourceSystem : public systems::LeafSystem<double> {
 public:
  explicit GeometrySourceSystem(GeometrySystem<double>* geometry_system)
      : systems::LeafSystem<double>(), geometry_system_(geometry_system) {
    // Register with GeometrySystem.
    source_id_ = geometry_system->RegisterSource();
    FrameId f_id = geometry_system->RegisterFrame(
        source_id_, GeometryFrame("frame", Isometry3<double>::Identity()));
    frame_ids_.push_back(f_id);
    // Set up output ports
    this->DeclareAbstractOutputPort(
        &GeometrySourceSystem::AllocateFrameIdOutput,
        &GeometrySourceSystem::CalcFrameIdOutput);
    this->DeclareAbstractOutputPort(
        &GeometrySourceSystem::AllocateFramePoseOutput,
        &GeometrySourceSystem::CalcFramePoseOutput);
  }
  SourceId get_source_id() const { return source_id_; }
  const systems::OutputPort<double>& get_id_output_port() const {
    return systems::System<double>::get_output_port(0);
  }
  const systems::OutputPort<double>& get_pose_output_port() const {
    return systems::System<double>::get_output_port(1);
  }
  // Method used to bring frame ids and poses out of sync. Adds a frame that
  // will *not* automatically get a pose.
  void add_extra_frame(bool add_to_output = true) {
    FrameId frame_id = geometry_system_->RegisterFrame(
        source_id_, GeometryFrame("frame", Isometry3<double>::Identity()));
    if (add_to_output) extra_frame_ids_.push_back(frame_id);
  }
  // Method used to bring frame ids and poses out of sync. Adds a pose in
  // addition to all of the default poses.
  void add_extra_pose() {
    extra_poses_.push_back(Isometry3<double>());
  }

 private:
  // Frame id output allocation and calculation.
  FrameIdVector AllocateFrameIdOutput(
      const Context<double>& context) const {
    FrameIdVector ids(source_id_, frame_ids_);
    ids.AddFrameIds(extra_frame_ids_);
    return ids;
  }
  // Frame ids never change after allocation.
  void CalcFrameIdOutput(const Context<double> &context,
                                               FrameIdVector *) const {}
  // Frame pose output allocation.
  FramePoseVector<double> AllocateFramePoseOutput(
  const Context<double>& context) const {
    FramePoseVector<double> poses(source_id_);
    for (size_t i = 0; i < frame_ids_.size(); ++i) {
      poses.mutable_vector().push_back(Isometry3<double>::Identity());
    }
    for (const auto& extra_pose : extra_poses_) {
      poses.mutable_vector().push_back(extra_pose);
    }
    return poses;
  }
  // For test purposes, no changes are required.
  void CalcFramePoseOutput(const Context<double>& context,
                           FramePoseVector<double>* pose_set) const {}

  GeometrySystem<double>* geometry_system_{nullptr};
  SourceId source_id_;
  std::vector<FrameId> frame_ids_;
  std::vector<FrameId> extra_frame_ids_;
  std::vector<Isometry3<double>> extra_poses_;
};

// Simple test case; system registers frames and provides correct connections.
GTEST_TEST(GeometrySystemConnectionTest, FullPoseUpdateUnconnectedId) {
  // Build a fully connected system.
  systems::DiagramBuilder<double> builder;
  auto geometry_system = builder.AddSystem<GeometrySystem<double>>();
  geometry_system->set_name("geometry_system");
  auto source_system = builder.AddSystem<GeometrySourceSystem>(geometry_system);
  source_system->set_name("source_system");
  SourceId source_id = source_system->get_source_id();
  builder.Connect(source_system->get_id_output_port(),
                  geometry_system->get_source_frame_id_port(source_id));
  builder.Connect(source_system->get_pose_output_port(),
                  geometry_system->get_source_pose_port(source_id));
  auto diagram = builder.Build();

  auto diagram_context = diagram->AllocateContext();
  diagram->SetDefaultContext(diagram_context.get());
  auto& geometry_context = dynamic_cast<GeometryContext<double>&>(
      diagram->GetMutableSubsystemContext(*geometry_system,
                                          diagram_context.get()));
  EXPECT_NO_THROW(
      GeometrySystemTester::FullPoseUpdate(*geometry_system, geometry_context));
}

// Adversarial test case: Missing id port connection.
GTEST_TEST(GeometrySystemConnectionTest, FullPoseUpdateNoIdConnection) {
  // Build a fully connected system.
  systems::DiagramBuilder<double> builder;
  auto geometry_system = builder.AddSystem<GeometrySystem<double>>();
  geometry_system->set_name("geometry_system");
  auto source_system = builder.AddSystem<GeometrySourceSystem>(geometry_system);
  source_system->set_name("source_system");
  SourceId source_id = source_system->get_source_id();
  builder.Connect(source_system->get_pose_output_port(),
                  geometry_system->get_source_pose_port(source_id));
  auto diagram = builder.Build();
  auto diagram_context = diagram->AllocateContext();
  diagram->SetDefaultContext(diagram_context.get());
  auto& geometry_context = dynamic_cast<GeometryContext<double>&>(
      diagram->GetMutableSubsystemContext(*geometry_system,
                                          diagram_context.get()));
  EXPECT_ERROR_MESSAGE(
      GeometrySystemTester::FullPoseUpdate(*geometry_system, geometry_context),
      std::logic_error,
      "Source \\d+ has registered frames but does not provide id values on "
      "the input port.");
}

// Adversarial test case: Missing pose port connection.
GTEST_TEST(GeometrySystemConnectionTest, FullPoseUpdateNoPoseConnection) {
  // Build a fully connected system.
  systems::DiagramBuilder<double> builder;
  auto geometry_system = builder.AddSystem<GeometrySystem<double>>();
  geometry_system->set_name("geometry_system");
  auto source_system = builder.AddSystem<GeometrySourceSystem>(geometry_system);
  source_system->set_name("source_system");
  SourceId source_id = source_system->get_source_id();
  builder.Connect(source_system->get_id_output_port(),
                  geometry_system->get_source_frame_id_port(source_id));
  auto diagram = builder.Build();
  auto diagram_context = diagram->AllocateContext();
  diagram->SetDefaultContext(diagram_context.get());
  auto& geometry_context = dynamic_cast<GeometryContext<double>&>(
      diagram->GetMutableSubsystemContext(*geometry_system,
                                          diagram_context.get()));
  EXPECT_ERROR_MESSAGE(
      GeometrySystemTester::FullPoseUpdate(*geometry_system, geometry_context),
      std::logic_error,
      "Source \\d+ has registered frames but does not provide pose values on "
          "the input port.");
}

// Adversarial test case: Missing all port connections.
GTEST_TEST(GeometrySystemConnectionTest, FullPoseUpdateNoConnections) {
  // Build a fully connected system.
  systems::DiagramBuilder<double> builder;
  auto geometry_system = builder.AddSystem<GeometrySystem<double>>();
  geometry_system->set_name("geometry_system");
  auto source_system = builder.AddSystem<GeometrySourceSystem>(geometry_system);
  source_system->set_name("source_system");
  auto diagram = builder.Build();
  auto diagram_context = diagram->AllocateContext();
  diagram->SetDefaultContext(diagram_context.get());
  auto& geometry_context = dynamic_cast<GeometryContext<double>&>(
      diagram->GetMutableSubsystemContext(*geometry_system,
                                          diagram_context.get()));
  EXPECT_ERROR_MESSAGE(
      GeometrySystemTester::FullPoseUpdate(*geometry_system, geometry_context),
      std::logic_error,
      "Source \\d+ has registered frames but does not provide id values on "
          "the input port.");
}

// Confirms that the GeometrySystem can be instantiated on AutoDiff type.
GTEST_TEST(GeometrySystemAutoDiffTest, InstantiateAutoDiff) {
  GeometrySystem<AutoDiffXd> geometry_system;
  geometry_system.RegisterSource("dummy_source");
  auto context = geometry_system.AllocateContext();
  GeometryContext<AutoDiffXd>* geometry_context =
      dynamic_cast<GeometryContext<AutoDiffXd>*>(context.get());

  ASSERT_NE(geometry_context, nullptr);

  QueryHandle<AutoDiffXd> handle =
      QueryHandleTester::MakeNullQueryHandle<AutoDiffXd>();
  GeometrySystemTester::GetQueryHandlePortValue(geometry_system, *context,
                                                &handle);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
