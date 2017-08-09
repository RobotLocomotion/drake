#include "drake/geometry/geometry_system.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/query_handle.h"
#include "drake/geometry/test/expect_error_message.h"
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
using std::make_unique;
using std::unique_ptr;

// Friend class for working with QueryHandles in a test context.
class QueryHandleTester {
 public:
  QueryHandleTester() = delete;
  static QueryHandle<double> MakeNullQueryHandle() {
    return QueryHandle<double>(nullptr, 0);
  }
  static void set_context(QueryHandle<double>* handle,
                          const Context<double>* context) {
    handle->context_ = context;
    // NOTE: This does not set the hash because these tests do not depend on it
    // yet.
  }
};

// Friend class for accessing GeometrySystem protected/private functionality.
class GeometrySystemTester {
 public:
  static QueryHandle<double> MakeHandle(
      const GeometrySystem<double>& system,
      const Context<double>* context) {
    return system.MakeQueryHandle(*context);
  }
  static bool HasDirectFeedthrough(const GeometrySystem<double>& system,
                                   int input_port, int output_port) {
    return system.DoHasDirectFeedthrough(nullptr, input_port, output_port);
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
        query_handle_(QueryHandleTester::MakeNullQueryHandle()) {}

 protected:
  void AllocateContext() {
    // TODO(SeanCurtis-TRI): This will probably have to be moved into an
    // explicit call so it can be run *after* topology has been set.
    context_ = system_.AllocateContext();
    QueryHandleTester::set_context(&query_handle_, context_.get());
  }

  const QueryHandle<double>& get_query_handle() const {
    // The `AllocateContext()` method must have been called *prior* to this
    // method.
    if (!context_)
      throw std::runtime_error("Must call AllocateContext() first.");
    return query_handle_;
  }

  static std::unique_ptr<GeometryInstance<double>> make_sphere_instance(
      double radius = 1.0) {
    return make_unique<GeometryInstance<double>>(Isometry3<double>::Identity(),
                                                 make_unique<Sphere>(radius));
  }

  GeometrySystem<double> system_;
  // Ownership of context.
  unique_ptr<Context<double>> context_;

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
          id, GeometryFrame<double>()),
      std::logic_error,
      "The call to RegisterFrame is invalid; a context has already been "
      "allocated.");

  // Attach frame to another frame.
  EXPECT_ERROR_MESSAGE(
      system_.RegisterFrame(
          id, FrameId::get_new_id(),
          GeometryFrame<double>()),
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

}  // namespace
}  // namespace geometry
}  // namespace drake
