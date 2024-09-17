#include "drake/geometry/in_memory_mesh.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(InMemoryMeshTest, BasicApi) {
  const InMemoryMesh empty;
  EXPECT_TRUE(empty.empty());

  const InMemoryMesh just_mesh(MemoryFile("body", ".ext", "hint"));
  ASSERT_FALSE(just_mesh.empty());
  EXPECT_EQ(just_mesh.mesh_file().contents(), "body");
}

GTEST_TEST(InMemorymeshTest, CopySemantics) {
  const InMemoryMesh source(MemoryFile("body", ".ext", "hint"));
  ASSERT_FALSE(source.empty());
  EXPECT_EQ(source.mesh_file().contents(), "body");

  const InMemoryMesh copy_ctor(source);
  ASSERT_FALSE(copy_ctor.empty());
  EXPECT_EQ(copy_ctor.mesh_file().contents(), "body");
  ASSERT_FALSE(source.empty());
  EXPECT_EQ(source.mesh_file().contents(), "body");

  InMemoryMesh copy_assign;
  EXPECT_TRUE(copy_assign.empty());
  copy_assign = source;
  ASSERT_FALSE(copy_assign.empty());
  EXPECT_EQ(copy_assign.mesh_file().contents(), "body");
  ASSERT_FALSE(source.empty());
  EXPECT_EQ(source.mesh_file().contents(), "body");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
