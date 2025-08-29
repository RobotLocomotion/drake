#include "drake/geometry/mesh_source.h"

#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/geometry/in_memory_mesh.h"

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(MeshSourceTest, PathConstructor) {
  const MeshSource s(std::filesystem::path("path"));
  EXPECT_EQ(s.description(), "path");
  ASSERT_TRUE(s.is_path());
  EXPECT_FALSE(s.is_in_memory());
  EXPECT_EQ(s.path(), "path");
}

GTEST_TEST(MeshSourceTest, InMemoryMeshConstructor) {
  const MeshSource s(InMemoryMesh{MemoryFile("contents", ".ext", "hint")});
  EXPECT_EQ(s.description(), "hint");
  EXPECT_FALSE(s.is_path());
  ASSERT_TRUE(s.is_in_memory());
  EXPECT_EQ(s.in_memory().mesh_file.contents(), "contents");
}

GTEST_TEST(MeshSourceTest, EmptyDescription) {
  EXPECT_EQ(
      MeshSource(InMemoryMesh{MemoryFile("body", ".ext", "")}).description(),
      "<no filename hint given>");
  EXPECT_EQ(MeshSource(std::filesystem::path()).description(), "<empty path>");
}

GTEST_TEST(MeshSourceTest, CopySemantics) {
  const MeshSource source(InMemoryMesh{MemoryFile("body", ".ext", "hint")});
  EXPECT_TRUE(source.is_in_memory());
  EXPECT_EQ(source.description(), "hint");

  const MeshSource copy_ctor(source);
  EXPECT_TRUE(copy_ctor.is_in_memory());
  EXPECT_EQ(copy_ctor.description(), "hint");
  EXPECT_TRUE(source.is_in_memory());
  EXPECT_EQ(source.description(), "hint");

  MeshSource copy_assign(std::filesystem::path("path"));
  EXPECT_TRUE(copy_assign.is_path());
  copy_assign = source;
  EXPECT_TRUE(copy_assign.is_in_memory());
  EXPECT_EQ(copy_assign.description(), "hint");
  EXPECT_TRUE(source.is_in_memory());
  EXPECT_EQ(source.description(), "hint");
}

GTEST_TEST(MeshSourceTest, MovedFrom) {
  MeshSource s(std::filesystem::path("path"));
  ASSERT_TRUE(s.is_path());
  EXPECT_EQ(s.path(), "path");

  const MeshSource d(std::move(s));
  ASSERT_TRUE(d.is_path());
  EXPECT_EQ(d.path(), "path");

  // The moved-from source now reports as an empty in-memory mesh.
  ASSERT_TRUE(s.is_in_memory());
  EXPECT_TRUE(s.in_memory().mesh_file.contents().empty());
  EXPECT_EQ(s.extension(), "");
}

/* Dummy function for testing implicit construction. */
void ConsumeMeshSource(const MeshSource&) {}

/* We allow implicit conversions from paths and in-memory meshes to MeshSource.
Successful compilation implies test success. */
GTEST_TEST(MeshSourceTest, ImplicitConversion) {
  ConsumeMeshSource(std::filesystem::path("path"));
  ConsumeMeshSource(InMemoryMesh{MemoryFile("contents", ".ext", "hint")});
}
}  // namespace
}  // namespace geometry
}  // namespace drake
