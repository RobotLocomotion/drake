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
  EXPECT_EQ(just_mesh.num_supporting_files(), 0);

  const InMemoryMesh full_mesh(MemoryFile("body", ".ext", "hint"),
                               {{"a", MemoryFile("aa", ".a", "aa")},
                                {"b", "b/path"}});
  EXPECT_FALSE(full_mesh.empty());
  EXPECT_EQ(full_mesh.num_supporting_files(), 2);
  EXPECT_EQ(full_mesh.supporting_file("unknown"), nullptr);
  ASSERT_NE(full_mesh.supporting_file("a"), nullptr);
  ASSERT_TRUE(full_mesh.supporting_file("a")->is_memory_file());
  EXPECT_EQ(full_mesh.supporting_file("a")->memory_file().contents(), "aa");
  ASSERT_NE(full_mesh.supporting_file("b"), nullptr);
  ASSERT_TRUE(full_mesh.supporting_file("b")->is_path());
  EXPECT_EQ(full_mesh.supporting_file("b")->path(), "b/path");
}

GTEST_TEST(InMemoryMeshTest, SupportingFiles) {
  InMemoryMesh mesh(MemoryFile("body", ".ext", "hint"));
  EXPECT_EQ(mesh.num_supporting_files(), 0);

  mesh.AddSupportingFile("first", MemoryFile("1", ".1", "1"));
  EXPECT_EQ(mesh.num_supporting_files(), 1);
  ASSERT_NE(mesh.supporting_file("first"), nullptr);
  EXPECT_EQ(mesh.supporting_file("first")->memory_file().contents(), "1");

  mesh.AddSupportingFile("alpha", MemoryFile("A", ".A", "A"));
  EXPECT_EQ(mesh.num_supporting_files(), 2);
  ASSERT_NE(mesh.supporting_file("alpha"), nullptr);
  EXPECT_EQ(mesh.supporting_file("alpha")->memory_file().contents(), "A");

  std::vector<std::string_view> names = mesh.SupportingFileNames();
  for (const auto name : names) {
    EXPECT_NE(mesh.supporting_file(name), nullptr);
  }

  DRAKE_EXPECT_THROWS_MESSAGE(
      mesh.AddSupportingFile("first", MemoryFile("2", ".2", "2")),
      ".*that name has already been used for file '1'.");

  // The supporting file names should be returned in a consistent order,
  // regardless of the order they were added.
  InMemoryMesh mesh2(MemoryFile("body", ".ext", "hint"));
  InMemoryMesh mesh3(MemoryFile("body", ".ext", "hint"));
  const std::vector<FileSource> sources{FileSource("one"), FileSource("two"),
                                        FileSource("three")};
  for (int i = 0; i < ssize(sources); ++i) {
    int i2 = i;
    mesh2.AddSupportingFile(sources[i2].description(), sources[i2]);
    int i3 = ssize(sources) - 1 - i;
    mesh3.AddSupportingFile(sources[i3].description(), sources[i3]);
  }
  EXPECT_EQ(mesh2.SupportingFileNames(), mesh3.SupportingFileNames());
}

GTEST_TEST(MeshSourceTest, PathConstructor) {
  const MeshSource s(std::filesystem::path("path"));
  EXPECT_EQ(s.description(), "path");
  ASSERT_TRUE(s.is_path());
  EXPECT_FALSE(s.is_in_memory());
  EXPECT_EQ(s.path(), "path");
}

GTEST_TEST(MeshSourceTest, InMemoryMeshConstructor) {
  const MeshSource s(InMemoryMesh(MemoryFile("contents", ".ext", "hint")));
  EXPECT_EQ(s.description(), "hint");
  EXPECT_FALSE(s.is_path());
  ASSERT_TRUE(s.is_in_memory());
  EXPECT_EQ(s.in_memory().mesh_file().contents(), "contents");
}

GTEST_TEST(MeshSourceTest, StringConstructor) {
  const char c_ptr[] = "path1";
  const std::string_view view("path2");
  const std::string str("path3");

  /* C-style string. */
  {
    const MeshSource s(c_ptr);
    EXPECT_TRUE(s.is_path());
    EXPECT_FALSE(s.is_in_memory());
  }

  /* std::string_view. */
  {
    const MeshSource s(view);
    EXPECT_TRUE(s.is_path());
    EXPECT_FALSE(s.is_in_memory());
  }

  /* std::string. */
  {
    const MeshSource s(str);
    EXPECT_TRUE(s.is_path());
    EXPECT_FALSE(s.is_in_memory());
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
