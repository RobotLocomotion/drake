#include "drake/geometry/in_memory_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace {

void ConsumeMeshSource(const MeshSource&) {}

/* We allow implicit conversion from a filename/path to a MeshSource. Successful
 compilation and execution implies test success. */
GTEST_TEST(MeshSourceTest, ImplicitConversion) {
  ConsumeMeshSource("char_pointer");
  ConsumeMeshSource(std::string("string"));
  ConsumeMeshSource(std::string_view("string_view"));
  ConsumeMeshSource(std::filesystem::path("path"));
}
}  // namespace
}  // namespace geometry
}  // namespace drake
