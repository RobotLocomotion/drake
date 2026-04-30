#include "drake/geometry/proximity/mesh_sdf_cache.h"

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

// Provides direct access to MeshSdfCache internals for testing.
class MeshSdfCacheTester {
 public:
  using Memoizer = MemoizerCache<MeshSdfCache::Key, MeshDistanceBoundary>;

  static std::shared_ptr<const Memoizer> get_memoizer(
      const MeshSdfCache& cache) {
    return cache.memoizer_;
  }

  // Returns the number of distinct source-file entries in the shared table.
  static int file_entry_count(const MeshSdfCache& cache) {
    if (cache.memoizer_ == nullptr) {
      return 0;
    }
    std::set<std::string> file_keys;
    for (const auto& [key, _] : cache.memoizer_->Dump()) {
      file_keys.insert(key.mesh_key);
    }
    return static_cast<int>(file_keys.size());
  }

  // Returns the number of (file, scale) entries in the table that share the
  // same source file as geometry `id`.
  static int scale_entry_count(const MeshSdfCache& cache, GeometryId id) {
    if (!cache.contains(id)) {
      return 0;
    }
    const std::string& file_key = cache.geometry_entries_.at(id).key.mesh_key;
    int count = 0;
    for (const auto& [key, _] : cache.memoizer_->Dump()) {
      if (key.mesh_key == file_key) ++count;
    }
    return count;
  }

  // Returns the number of geometry registrations (across all MeshSdfCache
  // instances sharing the same table) for the (file, scale) used by `id`.
  // Equivalent to shared_ptr use_count minus the table's own reference.
  // All copies of the cache share the table, so however many copies of the
  // cache exist, the shared table will only ever contribute one reference.
  static int ref_count(const MeshSdfCache& cache, GeometryId id) {
    if (!cache.contains(id)) {
      return 0;
    }
    const auto boundary = cache.geometry_entries_.at(id).boundary.load();
    EXPECT_TRUE(boundary != nullptr);
    if (boundary == nullptr) {
      return 0;
    }
    return static_cast<int>(boundary.use_count()) - 1;
  }

  // Returns true if `a` and `b` share the same underlying shared table.
  static bool shares_table(const MeshSdfCache& a, const MeshSdfCache& b) {
    return get_memoizer(a) == get_memoizer(b);
  }
};

namespace {

using Eigen::Vector3d;

// An OBJ string encoding a minimal, non-convex mesh. It is a tet with one face
// dented inwards. We want to be able to distinguish between when this is used
// as a Mesh vs a Convex.
constexpr char kDentedTet[] = R"""(
v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
v 0.2 0.2 0.2
# Bottom face with dent
f 1 5 2
f 2 5 3
f 3 5 1
# other faces
f 1 2 4
f 1 4 3
f 2 3 4
)""";

// A VTK string encoding two tetrahedra that share face {0,1,2}. After
// ConvertVolumeToSurfaceMesh() that interior face is stripped, leaving exactly
// 6 boundary triangles (3 per tet). A correct implementation produces 6; a
// naive one that simply triangulates all tet faces would produce 8.
constexpr char kTwoTetVtk[] = R"""(# vtk DataFile Version 2.0
two tets
ASCII
DATASET UNSTRUCTURED_GRID
POINTS 5 double
0 0 0
1 0 0
0 1 0
0 0 1
0 0 -1
CELLS 2 10
4 0 1 2 3
4 0 2 1 4
CELL_TYPES 2
10
10
)""";

// An OBJ string encoding a simple tet.
constexpr char kTet[] = R"""(
v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
f 1 3 2
f 1 2 4
f 1 4 3
f 2 3 4
)""";

// Returns a Mesh backed by an in-memory OBJ.
Mesh MakeObjMesh(double scale = 1.0, const char* obj = kDentedTet) {
  return Mesh(InMemoryMesh{MemoryFile(obj, ".obj", "tet")}, scale);
}

// Returns a Convex backed by an in-memory OBJ.
Convex MakeObjConvex(double scale = 1.0, const char* obj = kDentedTet) {
  return Convex(InMemoryMesh{MemoryFile(obj, ".obj", "tet")}, scale);
}

// Returns a Mesh backed by an in-memory VTK (two tets sharing one face).
Mesh MakeVtkMesh(double scale = 1.0) {
  return Mesh(InMemoryMesh{MemoryFile(kTwoTetVtk, ".vtk", "two_tet")}, scale);
}

// Registeres a shape (Mesh or Convex) with the given cache. Returns a geometry
// id for it. Guarantees cache->contains(id) is true on return.
template <typename MeshLike>
GeometryId RegisterShape(const MeshLike& mesh, MeshSdfCache* cache) {
  const GeometryId id = GeometryId::get_new_id();
  cache->Register(id, mesh);
  DRAKE_DEMAND(cache->contains(id));
  return id;
}

// Convenience alias.
using Tester = MeshSdfCacheTester;

// Confirm that copies share the tables but record independent populations.
GTEST_TEST(MeshSdfCacheTest, CopyAssignment) {
  MeshSdfCache cache;

  const GeometryId id = RegisterShape(MakeObjMesh(), &cache);
  const MeshDistanceBoundary& boundary = cache.GetOrCompute(id);
  EXPECT_EQ(Tester::ref_count(cache, id), 1);

  MeshSdfCache copy;
  copy = cache;

  // Confirm that the two caches share the same table and that the copy starts
  // with the same population (id).
  EXPECT_TRUE(Tester::shares_table(cache, copy));
  EXPECT_EQ(Tester::ref_count(cache, id), 2);
  EXPECT_EQ(Tester::ref_count(copy, id), 2);
  EXPECT_TRUE(copy.contains(id));
  EXPECT_EQ(&copy.GetOrCompute(id), &boundary);

  // Adding a new geometry to copy, but referencing the same (file, scale) as
  // what was registered in cache, should produce the same boundary, drawn from
  // the same table.
  const GeometryId copy_id = RegisterShape(MakeObjMesh(), &copy);
  const MeshDistanceBoundary& copy_boundary = copy.GetOrCompute(copy_id);
  EXPECT_EQ(&copy_boundary, &boundary);
  // We picked up another reference, visible from both caches.
  EXPECT_EQ(Tester::ref_count(cache, id), 3);
  EXPECT_EQ(Tester::ref_count(copy, id), 3);

  // Population is different.
  // Geometry added to copy is not in cache.
  EXPECT_FALSE(cache.contains(copy_id));
  cache.Remove(id);
  EXPECT_FALSE(cache.contains(id));
  // Original membership at time of copy is preserved.
  EXPECT_TRUE(copy.contains(id));
  // copy, which still has id, sees the reference count decrement.
  EXPECT_EQ(Tester::ref_count(copy, id), 2);
}

// Confirm that the copy constructor shares the table but records an independent
// population.
GTEST_TEST(MeshSdfCacheTest, CopyConstructor) {
  MeshSdfCache cache;

  const GeometryId id = RegisterShape(MakeObjMesh(), &cache);
  const MeshDistanceBoundary& boundary = cache.GetOrCompute(id);
  EXPECT_EQ(Tester::ref_count(cache, id), 1);

  MeshSdfCache copy{cache};

  // Confirm that the two caches share the same table and that the copy starts
  // with the same population (id).
  EXPECT_TRUE(Tester::shares_table(cache, copy));
  EXPECT_EQ(Tester::ref_count(cache, id), 2);
  EXPECT_EQ(Tester::ref_count(copy, id), 2);
  EXPECT_TRUE(copy.contains(id));
  EXPECT_EQ(&copy.GetOrCompute(id), &boundary);

  // Adding a new geometry to copy, but referencing the same (file, scale) as
  // what was registered in cache, should produce the same boundary, drawn from
  // the same table.
  const GeometryId copy_id = RegisterShape(MakeObjMesh(), &copy);
  const MeshDistanceBoundary& copy_boundary = copy.GetOrCompute(copy_id);
  EXPECT_EQ(&copy_boundary, &boundary);
  // We picked up another reference, visible from both caches.
  EXPECT_EQ(Tester::ref_count(cache, id), 3);
  EXPECT_EQ(Tester::ref_count(copy, id), 3);

  // Population is different.
  // Geometry added to copy is not in cache.
  EXPECT_FALSE(cache.contains(copy_id));
  cache.Remove(id);
  EXPECT_FALSE(cache.contains(id));
  // Original membership at time of copy is preserved.
  EXPECT_TRUE(copy.contains(id));
  // copy, which still has id, sees the reference count decrement.
  EXPECT_EQ(Tester::ref_count(copy, id), 2);
}

// Copy assignment into a non-empty cache flushes the old content first.
GTEST_TEST(MeshSdfCacheTest, CopyAssignmentIntoNonEmpty) {
  MeshSdfCache dest;
  const GeometryId dest_id = RegisterShape(MakeObjMesh(1, kTet), &dest);
  unused(dest.GetOrCompute(dest_id));
  ASSERT_EQ(Tester::file_entry_count(dest), 1);

  MeshSdfCache source;
  const GeometryId src_id = RegisterShape(MakeObjMesh(), &source);
  unused(source.GetOrCompute(src_id));

  dest = source;

  // Old content flushed: dest_id gone and its file entry evicted.
  EXPECT_FALSE(dest.contains(dest_id));
  // New content adopted.
  EXPECT_TRUE(dest.contains(src_id));
  EXPECT_TRUE(Tester::shares_table(dest, source));
  EXPECT_EQ(Tester::ref_count(dest, src_id), 2);
  EXPECT_EQ(Tester::file_entry_count(dest), 1);  // only the source file
}

// Move constructor transfers ownership without changing ref counts.
GTEST_TEST(MeshSdfCacheTest, MoveConstructor) {
  MeshSdfCache original;
  const GeometryId id = RegisterShape(MakeObjMesh(), &original);
  const MeshDistanceBoundary& boundary = original.GetOrCompute(id);
  ASSERT_EQ(Tester::ref_count(original, id), 1);

  MeshSdfCache moved(std::move(original));

  // Moved-to has the geometry and the (already-computed) boundary.
  EXPECT_TRUE(moved.contains(id));
  EXPECT_EQ(&moved.GetOrCompute(id), &boundary);
  // Ownership transferred — ref count unchanged.
  EXPECT_EQ(Tester::ref_count(moved, id), 1);

  // Moved-from is empty and has a fresh independent table.
  EXPECT_FALSE(original.contains(id));
  EXPECT_EQ(Tester::file_entry_count(original), 0);
  EXPECT_FALSE(Tester::shares_table(original, moved));
}

// Move assignment flushes dest's old content and transfers ownership.
GTEST_TEST(MeshSdfCacheTest, MoveAssignment) {
  MeshSdfCache dest;
  const GeometryId dest_id = RegisterShape(MakeObjMesh(1, kTet), &dest);
  unused(dest.GetOrCompute(dest_id));
  ASSERT_EQ(Tester::file_entry_count(dest), 1);

  MeshSdfCache source;
  const GeometryId src_id = RegisterShape(MakeObjMesh(), &source);
  const MeshDistanceBoundary& boundary = source.GetOrCompute(src_id);
  ASSERT_EQ(Tester::ref_count(source, src_id), 1);

  dest = std::move(source);

  // Old dest content flushed and evicted.
  EXPECT_FALSE(dest.contains(dest_id));
  // New content transferred.
  EXPECT_TRUE(dest.contains(src_id));
  EXPECT_EQ(&dest.GetOrCompute(src_id), &boundary);
  EXPECT_EQ(Tester::ref_count(dest, src_id), 1);  // ownership moved, not shared

  // Source is empty and has a fresh independent table.
  EXPECT_FALSE(source.contains(src_id));
  EXPECT_EQ(Tester::file_entry_count(source), 0);
  EXPECT_FALSE(Tester::shares_table(source, dest));
}

// Destroying a cache decrements ref counts for all its entries, but does not
// corrupt other instances..
GTEST_TEST(MeshSdfCacheTest, Destructor) {
  MeshSdfCache cache;

  const GeometryId id1 = RegisterShape(MakeObjMesh(), &cache);
  const GeometryId id2 = RegisterShape(MakeObjMesh(), &cache);
  unused(cache.GetOrCompute(id1));
  unused(cache.GetOrCompute(id2));
  // State of cache.
  EXPECT_TRUE(cache.contains(id1));
  EXPECT_EQ(Tester::ref_count(cache, id1), 2);
  EXPECT_EQ(Tester::file_entry_count(cache), 1);

  {
    MeshSdfCache copy;
    copy = cache;
    // We double the number of references; observable from both caches.
    EXPECT_EQ(Tester::ref_count(cache, id1), 4);
    EXPECT_EQ(Tester::ref_count(copy, id1), 4);
  }  // copy destroyed here; ref count decremented back to 2.

  // Counts restored.
  EXPECT_TRUE(cache.contains(id1));
  EXPECT_EQ(Tester::ref_count(cache, id1), 2);
  EXPECT_EQ(Tester::file_entry_count(cache), 1);
}

// Confirm that "contains" reports as expected.
GTEST_TEST(MeshSdfCacheTest, Contains) {
  MeshSdfCache cache;

  // An Id that hasn't been registered, reports as such.
  EXPECT_FALSE(cache.contains(GeometryId::get_new_id()));

  // Now we register something.
  const GeometryId id1 = GeometryId::get_new_id();
  cache.Register(id1, MakeObjMesh());
  EXPECT_TRUE(cache.contains(id1));

  // After removing it, it's no longer "contained".
  cache.Remove(id1);
  EXPECT_FALSE(cache.contains(id1));
}

GTEST_TEST(MeshSdfCacheTest, Register) {
  MeshSdfCache cache;
  EXPECT_EQ(Tester::file_entry_count(cache), 0);

  // Unsupported extensions are silently ignored.
  const GeometryId id1 = GeometryId::get_new_id();
  const Mesh glb(InMemoryMesh{MemoryFile("", ".fake", "dummy")});
  cache.Register(id1, glb);
  EXPECT_FALSE(cache.contains(id1));
  EXPECT_EQ(Tester::file_entry_count(cache), 0);

  // Supported file creates an entry.
  cache.Register(id1, MakeObjMesh());
  unused(cache.GetOrCompute(id1));
  EXPECT_TRUE(cache.contains(id1));
  EXPECT_EQ(Tester::file_entry_count(cache), 1);
  EXPECT_EQ(Tester::scale_entry_count(cache, id1), 1);
  EXPECT_EQ(Tester::ref_count(cache, id1), 1);

  // Second instance of the same file and scale *only* increases the ref count.
  const GeometryId id2 = GeometryId::get_new_id();
  cache.Register(id2, MakeObjMesh());
  unused(cache.GetOrCompute(id2));
  EXPECT_TRUE(cache.contains(id2));
  EXPECT_EQ(Tester::file_entry_count(cache), 1);
  EXPECT_EQ(Tester::scale_entry_count(cache, id2), 1);
  EXPECT_EQ(Tester::ref_count(cache, id2), 2);

  // Same file, different scale *only* increases scale count.
  const GeometryId id3 = GeometryId::get_new_id();
  cache.Register(id3, MakeObjMesh(2));
  unused(cache.GetOrCompute(id3));
  EXPECT_TRUE(cache.contains(id3));
  EXPECT_EQ(Tester::file_entry_count(cache), 1);
  EXPECT_EQ(Tester::scale_entry_count(cache, id3), 2);
  EXPECT_EQ(Tester::ref_count(cache, id2), 2);
  EXPECT_EQ(Tester::ref_count(cache, id3), 1);
}

GTEST_TEST(MeshSdfCacheTest, RegisterMeshVsConvex) {
  MeshSdfCache cache;
  // Register two files with the same mesh data and scale, this time the
  // difference is between Mesh and Convex. The mesh data is the dented tet.
  // We can tell that the right thing was done for the Convex, because its
  // triangle surface mesh will have fewer triangles.
  const GeometryId mesh_id = RegisterShape(MakeObjMesh(), &cache);
  const GeometryId convex_id = RegisterShape(MakeObjConvex(), &cache);
  unused(cache.GetOrCompute(mesh_id));
  unused(cache.GetOrCompute(convex_id));

  // Distinct file entries.
  EXPECT_EQ(Tester::file_entry_count(cache), 2);

  const MeshDistanceBoundary& mesh_b = cache.GetOrCompute(mesh_id);
  const MeshDistanceBoundary& convex_b = cache.GetOrCompute(convex_id);

  ASSERT_NE(&mesh_b, &convex_b);
  EXPECT_LT(convex_b.tri_mesh().num_triangles(),
            mesh_b.tri_mesh().num_triangles());
}

// All previous tests used OBJ meshes. Confirm VTK volume meshes are supported.
// The mesh is two tets sharing one interior face; ConvertVolumeToSurfaceMesh
// must strip that face, leaving exactly 6 boundary triangles.
GTEST_TEST(MeshSdfCacheTest, RegisterVtk) {
  MeshSdfCache cache;
  const GeometryId id = RegisterShape(MakeVtkMesh(), &cache);
  unused(cache.GetOrCompute(id));
  EXPECT_EQ(Tester::file_entry_count(cache), 1);
  const MeshDistanceBoundary& boundary = cache.GetOrCompute(id);
  EXPECT_EQ(boundary.tri_mesh().num_triangles(), 6);
}

GTEST_TEST(MeshSdfCacheTest, Remove) {
  MeshSdfCache cache;

  // ids a* have the same mesh and scale. b shares the mesh with a, but has a
  // diferent scale. c is different mesh and scale.
  const GeometryId id_a1 = RegisterShape(MakeObjMesh(), &cache);
  const GeometryId id_a2 = RegisterShape(MakeObjMesh(), &cache);
  const GeometryId id_b = RegisterShape(MakeObjMesh(2), &cache);
  const GeometryId id_c = RegisterShape(MakeObjMesh(1, kTet), &cache);
  unused(cache.GetOrCompute(id_a1));
  unused(cache.GetOrCompute(id_a2));
  unused(cache.GetOrCompute(id_b));
  unused(cache.GetOrCompute(id_c));
  ASSERT_EQ(Tester::file_entry_count(cache), 2);
  ASSERT_EQ(Tester::scale_entry_count(cache, id_a1), 2);
  ASSERT_EQ(Tester::ref_count(cache, id_a1), 2);

  // Removing id_a2 evicts nothing both files at alls scales still used.
  cache.Remove(id_a2);
  EXPECT_FALSE(cache.contains(id_a2));
  EXPECT_EQ(Tester::file_entry_count(cache), 2);
  EXPECT_EQ(Tester::scale_entry_count(cache, id_a1), 2);
  EXPECT_EQ(Tester::ref_count(cache, id_a1), 1);  // The only change.

  // Removing the last geometry for a scale, evicts the scale, keeps the file.
  cache.Remove(id_b);
  EXPECT_FALSE(cache.contains(id_b));
  EXPECT_EQ(Tester::file_entry_count(cache), 2);
  EXPECT_EQ(Tester::scale_entry_count(cache, id_a1), 1);  // The change.
  EXPECT_EQ(Tester::ref_count(cache, id_a1), 1);

  // Removing last geometry for a file, evicts the file, other files remain.
  cache.Remove(id_c);
  EXPECT_FALSE(cache.contains(id_c));
  EXPECT_EQ(Tester::file_entry_count(cache), 1);
  EXPECT_EQ(Tester::scale_entry_count(cache, id_a1), 1);  // The change.
  EXPECT_EQ(Tester::ref_count(cache, id_a1), 1);

  // Remove() on an unregistered id is a no-op.
  EXPECT_NO_THROW(cache.Remove(GeometryId::get_new_id()));
  EXPECT_EQ(Tester::file_entry_count(cache), 1);
  EXPECT_EQ(Tester::scale_entry_count(cache, id_a1), 1);
  EXPECT_EQ(Tester::ref_count(cache, id_a1), 1);
}

GTEST_TEST(MeshSdfCacheTest, GetOrCompute) {
  MeshSdfCache cache;

  // ids a* have the same mesh and scale. b shares the mesh with a, but has a
  // diferent scale. c is different mesh and scale.
  const GeometryId id_a1 = RegisterShape(MakeObjMesh(1, kDentedTet), &cache);
  const GeometryId id_a2 = RegisterShape(MakeObjMesh(1, kDentedTet), &cache);
  const GeometryId id_b = RegisterShape(MakeObjMesh(2, kDentedTet), &cache);
  const GeometryId id_c = RegisterShape(MakeObjMesh(1, kTet), &cache);
  const MeshDistanceBoundary& bound_a1 = cache.GetOrCompute(id_a1);
  const MeshDistanceBoundary& bound_a2 = cache.GetOrCompute(id_a2);
  const MeshDistanceBoundary& bound_b = cache.GetOrCompute(id_b);
  const MeshDistanceBoundary& bound_c = cache.GetOrCompute(id_c);
  EXPECT_EQ(Tester::file_entry_count(cache), 2);

  // The a* boundaries should *literally* be the same object.
  EXPECT_EQ(&bound_a1, &bound_a2);
  // b and c should be distinct objects.
  EXPECT_NE(&bound_a1, &bound_b);
  EXPECT_NE(&bound_a1, &bound_c);
  EXPECT_NE(&bound_b, &bound_c);

  // Because id_a* and id_b share the same file, their meshes should match to
  // a scale factor.
  EXPECT_TRUE(bound_b.tri_mesh().Equal(
      bound_a1.tri_mesh().CreateScaledMesh(Vector3d(2, 2, 2))));

  // Repeated calls should keep returning the same reference.
  EXPECT_EQ(&cache.GetOrCompute(id_a1), &bound_a1);
  EXPECT_EQ(&cache.GetOrCompute(id_b), &bound_b);
  EXPECT_EQ(&cache.GetOrCompute(id_c), &bound_c);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
