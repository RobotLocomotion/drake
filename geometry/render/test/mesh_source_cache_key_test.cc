#include "drake/geometry/render/mesh_source_cache_key.h"

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/memory_file.h"
#include "drake/common/temp_directory.h"
#include "drake/geometry/in_memory_mesh.h"
#include "drake/geometry/mesh_source.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {
namespace {

namespace fs = std::filesystem;

// Two OBJ content strings used throughout these tests. They are
// geometrically distinct so their SHA256 hashes differ.
constexpr char kObjContent[] =
    "v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 1\nf 1//1 2//1 3//1\n";
constexpr char kObjContentAlt[] =
    "v 1 0 0\nv 0 1 0\nv 0 0 1\nvn 1 0 0\nf 1//1 2//1 3//1\n";

// Writes content to path and returns path.
fs::path WriteFile(const fs::path& path, std::string_view content) {
  std::ofstream f(path);
  f << content;
  return path;
}

InMemoryMesh MakeMemSource(const std::string& contents) {
  // All memory sources have the same name and description so we can show only
  // the contents alone is sufficient to distinguish.
  return InMemoryMesh(MemoryFile(contents, ".obj", "common_name.obj"));
}

/* Test fixture that writes the shared on-disk files and in-memory MemoryFiles
once for the entire test suite (via SetUpTestSuite). */
class GetMeshSourceCacheKeyTest : public ::testing::Test {
 public:
  static void SetUpTestSuite() {
    // Place the physical files on disk in support of the declared mesh sources.
    WriteFile(file_source_a_.path(), kObjContent);
    WriteFile(file_source_b_.path(), kObjContent);
  }

 protected:
  // Shared on-disk resources (one temp dir, written once).
  static const fs::path dir_;
  static const MeshSource file_source_a_;
  static const MeshSource file_source_b_;

  // Shared in-memory resources.
  static const MeshSource mem_source_a_;
  static const MeshSource mem_source_b_;
};

const fs::path GetMeshSourceCacheKeyTest::dir_(temp_directory());
const MeshSource GetMeshSourceCacheKeyTest::file_source_a_(dir_ / "mesh_a.obj");
const MeshSource GetMeshSourceCacheKeyTest::file_source_b_(dir_ / "mesh_b.obj");
// In-memory meshes used by in-memory tests. They intentionally match in every
// way except for the content, to make sure only the content matters.
const MeshSource GetMeshSourceCacheKeyTest::mem_source_a_(
    MakeMemSource(kObjContent));
const MeshSource GetMeshSourceCacheKeyTest::mem_source_b_(
    MakeMemSource(kObjContentAlt));

// The same file used for mesh and convex produce different keys.
TEST_F(GetMeshSourceCacheKeyTest, OnDiskMeshVsConvex) {
  const std::string mesh_key = GetMeshSourceCacheKey(file_source_a_, false);
  const std::string convex_key = GetMeshSourceCacheKey(file_source_a_, true);

  EXPECT_NE(mesh_key, convex_key);
}

// Two physically different files produce different keys (even if their contents
// are identical).
TEST_F(GetMeshSourceCacheKeyTest, OnDiskDifferentPaths) {
  const std::string key_a = GetMeshSourceCacheKey(file_source_a_, false);
  const std::string key_b = GetMeshSourceCacheKey(file_source_b_, false);

  EXPECT_NE(key_a, key_b);
}

// A symlink to a file resolves to the same canonical key as the real path.
TEST_F(GetMeshSourceCacheKeyTest, OnDiskSymlinkCanonicalizesToSameKey) {
  const std::string real_key = GetMeshSourceCacheKey(file_source_a_, false);
  fs::path link = dir_ / "link.obj";
  fs::create_symlink(file_source_a_.path(), link);
  const std::string link_key = GetMeshSourceCacheKey(MeshSource(link), false);

  EXPECT_EQ(real_key, link_key);
}

// A non-existent path throws std::runtime_error.
TEST_F(GetMeshSourceCacheKeyTest, OnDiskNonExistentPathThrows) {
  EXPECT_THROW(
      GetMeshSourceCacheKey(MeshSource(fs::path("/no/such/file.obj")), false),
      std::runtime_error);
}

// The same contents used for mesh and convex produce different keys.
TEST_F(GetMeshSourceCacheKeyTest, InMemoryMeshVsConvex) {
  const std::string mesh_key = GetMeshSourceCacheKey(mem_source_a_, false);
  const std::string convex_key = GetMeshSourceCacheKey(mem_source_a_, true);

  EXPECT_NE(mesh_key, convex_key);
}

// Two in-memory meshes with different content produce different keys.
TEST_F(GetMeshSourceCacheKeyTest, InMemoryDifferentContentDifferentKey) {
  EXPECT_NE(GetMeshSourceCacheKey(mem_source_a_, false),
            GetMeshSourceCacheKey(mem_source_b_, false));
}

}  // namespace
}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
