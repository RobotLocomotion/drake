#include "drake/geometry/mesh_source.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/memory_file.h"
#include "drake/common/temp_directory.h"
#include "drake/geometry/in_memory_mesh.h"

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(MeshSourceTest, EmptySource) {
  const MeshSource s;
  EXPECT_FALSE(s.is_path());
  EXPECT_TRUE(s.is_in_memory());
  EXPECT_TRUE(s.in_memory().mesh_file.contents().empty());
  EXPECT_EQ(s.extension(), "");

  EXPECT_FALSE(MeshSource::Empty().is_path());
  EXPECT_TRUE(MeshSource::Empty().is_in_memory());
  EXPECT_TRUE(MeshSource::Empty().in_memory().mesh_file.contents().empty());
  EXPECT_EQ(MeshSource::Empty().extension(), "");
}

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

// Two OBJ content strings used throughout the GetCacheKey tests.
constexpr char kObjContent[] =
    "v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 1\nf 1//1 2//1 3//1\n";
constexpr char kObjContentAlt[] =
    "v 1 0 0\nv 0 1 0\nv 0 0 1\nvn 1 0 0\nf 1//1 2//1 3//1\n";

namespace fs = std::filesystem;

// Writes content to path and returns path.
fs::path WriteTestFile(const fs::path& path, std::string_view content) {
  std::ofstream f(path);
  f << content;
  return path;
}

InMemoryMesh MakeTestMemoryMesh(const std::string& contents) {
  return InMemoryMesh(MemoryFile(contents, ".obj", "common_name.obj"));
}

/* Test fixture that writes the shared on-disk files once for the entire suite
(via SetUpTestSuite). */
class MeshSourceGetCacheKeyTest : public ::testing::Test {
 public:
  static void SetUpTestSuite() {
    WriteTestFile(file_source_a_.path(), kObjContent);
    WriteTestFile(file_source_b_.path(), kObjContent);
  }

 protected:
  static const fs::path dir_;
  static const MeshSource file_source_a_;
  static const MeshSource file_source_b_;
  static const MeshSource mem_source_a_;
  static const MeshSource mem_source_b_;
};

const fs::path MeshSourceGetCacheKeyTest::dir_(temp_directory());
const MeshSource MeshSourceGetCacheKeyTest::file_source_a_(dir_ / "mesh_a.obj");
const MeshSource MeshSourceGetCacheKeyTest::file_source_b_(dir_ / "mesh_b.obj");
const MeshSource MeshSourceGetCacheKeyTest::mem_source_a_(
    MakeTestMemoryMesh(kObjContent));
const MeshSource MeshSourceGetCacheKeyTest::mem_source_b_(
    MakeTestMemoryMesh(kObjContentAlt));

// The same file used for mesh and convex produce different keys.
TEST_F(MeshSourceGetCacheKeyTest, OnDiskMeshVsConvex) {
  EXPECT_NE(file_source_a_.GetCacheKey(false),
            file_source_a_.GetCacheKey(true));
}

// Two physically different files produce different keys (even if their contents
// are identical).
TEST_F(MeshSourceGetCacheKeyTest, OnDiskDifferentPaths) {
  EXPECT_NE(file_source_a_.GetCacheKey(false),
            file_source_b_.GetCacheKey(false));
}

// A symlink to a file resolves to the same canonical key as the real path.
TEST_F(MeshSourceGetCacheKeyTest, OnDiskSymlinkCanonicalizesToSameKey) {
  const std::string real_key = file_source_a_.GetCacheKey(false);
  const fs::path link = dir_ / "link.obj";
  fs::create_symlink(file_source_a_.path(), link);
  EXPECT_EQ(real_key, MeshSource(link).GetCacheKey(false));
}

// A non-existent path throws std::runtime_error.
TEST_F(MeshSourceGetCacheKeyTest, OnDiskNonExistentPathThrows) {
  EXPECT_THROW(MeshSource(fs::path("/no/such/file.obj")).GetCacheKey(false),
               std::runtime_error);
}

// The same contents used for mesh and convex produce different keys.
TEST_F(MeshSourceGetCacheKeyTest, InMemoryMeshVsConvex) {
  EXPECT_NE(mem_source_a_.GetCacheKey(false), mem_source_a_.GetCacheKey(true));
}

// Two in-memory meshes with different content produce different keys.
TEST_F(MeshSourceGetCacheKeyTest, InMemoryDifferentContentDifferentKey) {
  EXPECT_NE(mem_source_a_.GetCacheKey(false), mem_source_b_.GetCacheKey(false));
}
}  // namespace
}  // namespace geometry
}  // namespace drake
