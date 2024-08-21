#include "drake/geometry/in_memory_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(InMemoryMeshTest, Constructor) {
  const InMemoryMesh empty;
  EXPECT_TRUE(empty.empty());

  const InMemoryMesh just_mesh(MemoryFile("body", ".ext", "hint"));
  ASSERT_FALSE(just_mesh.empty());
  EXPECT_EQ(just_mesh.mesh_file().contents(), "body");
}

GTEST_TEST(MeshSourceTest, PathConstructor) {
  const MeshSource s(std::filesystem::path("path"));
  EXPECT_EQ(s.description(), "path");
  ASSERT_TRUE(s.IsPath());
  EXPECT_FALSE(s.IsInMemory());
  EXPECT_EQ(s.path(), "path");
}

GTEST_TEST(MeshSourceTest, InMemoryMeshConstructor) {
  const MeshSource s(InMemoryMesh(MemoryFile("contents", ".ext", "hint")));
  EXPECT_EQ(s.description(), "hint");
  EXPECT_FALSE(s.IsPath());
  ASSERT_TRUE(s.IsInMemory());
  EXPECT_EQ(s.mesh_data().mesh_file().contents(), "contents");
}

GTEST_TEST(MeshSourceTest, StringConstructor) {
  const char c_ptr[] = "path1";
  const std::string_view view("path2");
  const std::string str("path3");

  /* C-style string. */
  {
    const MeshSource s(c_ptr);
    EXPECT_TRUE(s.IsPath());
    EXPECT_FALSE(s.IsInMemory());
  }

  /* std::string_view. */
  {
    const MeshSource s(view);
    EXPECT_TRUE(s.IsPath());
    EXPECT_FALSE(s.IsInMemory());
  }

  /* std::string. */
  {
    const MeshSource s(str);
    EXPECT_TRUE(s.IsPath());
    EXPECT_FALSE(s.IsInMemory());
  }
}

/* Dummy function for testing implicit construction. */
void ConsumeMeshSource(const MeshSource&) {}

/* We allow implicit conversion from a filename/path and in-memory mesh to a
 MeshSource. Successful compilation implies test success. */
GTEST_TEST(MeshSourceTest, ImplicitConversion) {
  ConsumeMeshSource("char_pointer");
  ConsumeMeshSource(std::string("string"));
  ConsumeMeshSource(std::string_view("string_view"));
  ConsumeMeshSource(std::filesystem::path("path"));
  ConsumeMeshSource(InMemoryMesh(MemoryFile("contents", ".ext", "hint")));
}
}  // namespace
}  // namespace geometry
}  // namespace drake
