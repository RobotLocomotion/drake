#include "drake/geometry/in_memory_mesh.h"

#include <filesystem>

#include <gtest/gtest.h>

#include "drake/common/overloaded.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace {

namespace fs = std::filesystem;

GTEST_TEST(InMemoryMeshTest, BasicApi) {
  const InMemoryMesh empty;
  EXPECT_TRUE(empty.empty());

  const InMemoryMesh just_mesh(MemoryFile("body", ".ext", "hint"));
  ASSERT_FALSE(just_mesh.empty());
  EXPECT_EQ(just_mesh.mesh_file().contents(), "body");
  EXPECT_EQ(just_mesh.num_supporting_files(), 0);

  const InMemoryMesh full_mesh(
      MemoryFile("body", ".ext", "hint"),
      {{"a", MemoryFile("aa", ".a", "aa")}, {"b", fs::path("b/path")}});
  EXPECT_FALSE(full_mesh.empty());
  EXPECT_EQ(full_mesh.num_supporting_files(), 2);
  EXPECT_EQ(full_mesh.supporting_file("unknown"), nullptr);
  ASSERT_NE(full_mesh.supporting_file("a"), nullptr);
  ASSERT_TRUE(
      std::holds_alternative<MemoryFile>(*full_mesh.supporting_file("a")));
  EXPECT_EQ(std::get<MemoryFile>(*full_mesh.supporting_file("a")).contents(),
            "aa");
  ASSERT_NE(full_mesh.supporting_file("b"), nullptr);
  ASSERT_TRUE(
      std::holds_alternative<fs::path>(*full_mesh.supporting_file("b")));
  EXPECT_EQ(std::get<fs::path>(*full_mesh.supporting_file("b")), "b/path");
}

GTEST_TEST(InMemoryMeshTest, SupportingFiles) {
  InMemoryMesh mesh(MemoryFile("body", ".ext", "hint"));
  EXPECT_EQ(mesh.num_supporting_files(), 0);

  mesh.AddSupportingFile("first", MemoryFile("1", ".1", "1"));
  EXPECT_EQ(mesh.num_supporting_files(), 1);
  ASSERT_NE(mesh.supporting_file("first"), nullptr);
  EXPECT_EQ(std::get<MemoryFile>(*mesh.supporting_file("first")).contents(),
            "1");

  mesh.AddSupportingFile("alpha", MemoryFile("A", ".A", "A"));
  EXPECT_EQ(mesh.num_supporting_files(), 2);
  ASSERT_NE(mesh.supporting_file("alpha"), nullptr);
  EXPECT_EQ(std::get<MemoryFile>(*mesh.supporting_file("alpha")).contents(),
            "A");

  std::vector<std::string_view> names = mesh.SupportingFileNames();
  for (const auto name : names) {
    EXPECT_NE(mesh.supporting_file(name), nullptr);
  }

  DRAKE_EXPECT_THROWS_MESSAGE(
      mesh.AddSupportingFile("first", MemoryFile("2", ".2", "2")),
      ".*that name has already been used for file '1'.");

  auto description = [](const FileSource& source) {
    return std::visit(overloaded{
      [](const fs::path& path) { return path.string(); },
      [](const MemoryFile& file) { return file.filename_hint(); }
    }, source);
  };

  // The supporting file names should be returned in a consistent order,
  // regardless of the order they were added.
  InMemoryMesh mesh2(MemoryFile("body", ".ext", "hint"));
  InMemoryMesh mesh3(MemoryFile("body", ".ext", "hint"));
  const std::vector<FileSource> sources{fs::path("one"), fs::path("two"),
                                        fs::path("three")};
  for (int i = 0; i < ssize(sources); ++i) {
    int i2 = i;
    mesh2.AddSupportingFile(description(sources[i2]), sources[i2]);
    int i3 = ssize(sources) - 1 - i;
    mesh3.AddSupportingFile(description(sources[i3]), sources[i3]);
  }
  EXPECT_EQ(mesh2.SupportingFileNames(), mesh3.SupportingFileNames());
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
