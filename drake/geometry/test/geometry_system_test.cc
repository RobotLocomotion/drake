#include "drake/geometry/geometry_system.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
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
using GSystem = GeometrySystem<double>;
using std::make_unique;
using std::unique_ptr;

// Friend class for accessing GeometrySystem protected/private functionality.
class GeometrySystemTester {
 public:
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
 protected:
  void AllocateContext() {
    // TODO(SeanCurtis-TRI): This will probably have to be moved into an
    // explicit call so it can be run *after* topology has been set.
    context_ = system_.AllocateContext();
  }

  static std::unique_ptr<GeometryInstance<double>> make_sphere_instance(
      double radius = 1.0) {
    return make_unique<GeometryInstance<double>>();
  }

  GSystem system_;
  // Ownership of context.
  unique_ptr<Context<double>> context_;
};

// Test sources.

// Tests registration using a default source name. Confirms that the source
// registered and that a name is available.
TEST_F(GeometrySystemTest, RegisterSourceDefaultName) {
  SourceId id;
  EXPECT_NO_THROW(id = system_.RegisterSource());
  EXPECT_TRUE(id.is_valid());
}

// Tests registration using a specified source name. Confirms that the source
// registered and that the name is available.
TEST_F(GeometrySystemTest, RegisterSourceSpecifiedName) {
  std::string name = "some_unique_name";
  SourceId id;
  EXPECT_NO_THROW(id = system_.RegisterSource(name));
  EXPECT_TRUE(id.is_valid());
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

}  // namespace
}  // namespace geometry
}  // namespace drake
