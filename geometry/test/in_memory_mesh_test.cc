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
  EXPECT_FALSE(just_mesh.empty());
  EXPECT_EQ(just_mesh.num_supporting_files(), 0);

  const InMemoryMesh full_mesh(MemoryFile("body", ".ext", "hint"),
                               {{"a", MemoryFile("aa", ".a", "aa")},
                                {"b", MemoryFile("bb", ".b", "bb")}});
  EXPECT_FALSE(full_mesh.empty());
  EXPECT_EQ(full_mesh.num_supporting_files(), 2);
  EXPECT_EQ(full_mesh.file("unknown"), nullptr);
  ASSERT_NE(full_mesh.file("a"), nullptr);
  EXPECT_EQ(full_mesh.file("a")->contents(), "aa");
  ASSERT_NE(full_mesh.file("b"), nullptr);
  EXPECT_EQ(full_mesh.file("b")->contents(), "bb");
}

GTEST_TEST(InMemoryMeshTest, SupportingFiles) {
  InMemoryMesh mesh(MemoryFile("body", ".ext", "hint"));
  EXPECT_EQ(mesh.num_supporting_files(), 0);

  mesh.AddSupportingFile("first", MemoryFile("1", ".1", "1"));
  EXPECT_EQ(mesh.num_supporting_files(), 1);
  ASSERT_NE(mesh.file("first"), nullptr);
  EXPECT_EQ(mesh.file("first")->contents(), "1");

  mesh.AddSupportingFile("alpha", MemoryFile("A", ".A", "A"));
  EXPECT_EQ(mesh.num_supporting_files(), 2);
  ASSERT_NE(mesh.file("alpha"), nullptr);
  EXPECT_EQ(mesh.file("alpha")->contents(), "A");

  std::vector<std::string_view> names = mesh.SupportingFileNames();
  for (const auto name : names) {
    EXPECT_NE(mesh.file(name), nullptr);
  }

  DRAKE_EXPECT_THROWS_MESSAGE(
      mesh.AddSupportingFile("first", MemoryFile("2", ".2", "2")),
      ".*that name has already been used for file '1'.");
}

/* Dummy function for testing implicit construction. */
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
