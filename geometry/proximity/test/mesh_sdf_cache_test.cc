#include "drake/geometry/proximity/mesh_sdf_cache.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
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

// Returns a Mesh backed by an in-memory OBJ.
template <typename MeshLike>
MeshLike MakeObj(double scale = 1.0) {
  return MeshLike(InMemoryMesh{MemoryFile(kDentedTet, ".obj", "tet")}, scale);
}

// Returns a Mesh backed by an in-memory VTK (two tets sharing one face).
template <typename MeshLike>
MeshLike MakeVtk(double scale = 1.0) {
  return MeshLike(InMemoryMesh{MemoryFile(kTwoTetVtk, ".vtk", "two_tet")},
                  scale);
}

// Registers a shape (Mesh or Convex) with the given cache. Returns a geometry
// id for it. Guarantees cache->contains(id) is true on return.
template <typename MeshLike>
GeometryId RegisterShape(const MeshLike& mesh, MeshSdfCache* cache) {
  const GeometryId id = GeometryId::get_new_id();
  cache->Register(id, mesh);
  DRAKE_DEMAND(cache->contains(id));
  return id;
}

GTEST_TEST(MeshSdfCacheTest, DefaultCache) {
  MeshSdfCache cache;
  EXPECT_FALSE(cache.is_computed());
  EXPECT_FALSE(cache.contains(GeometryId::get_new_id()));
  EXPECT_EQ(cache.num_geometries(), 0);

  // Computing an empty cache has no effect.
  cache.ComputeAll();
  EXPECT_FALSE(cache.is_computed());
  EXPECT_FALSE(cache.contains(GeometryId::get_new_id()));
  EXPECT_EQ(cache.num_geometries(), 0);
}

GTEST_TEST(MeshSdfCacheTest, Contains) {
  MeshSdfCache cache;
  const GeometryId id = RegisterShape(MakeObj<Mesh>(), &cache);

  EXPECT_TRUE(cache.contains(id));
  EXPECT_FALSE(cache.contains(GeometryId::get_new_id()));
}

GTEST_TEST(MeshSdfCacheTest, CopySemantics) {
  MeshSdfCache cache;
  const GeometryId id = RegisterShape(MakeObj<Mesh>(), &cache);

  MeshSdfCache pre_compute_copy{cache};
  MeshSdfCache pre_compute_copy_assign;
  pre_compute_copy_assign = cache;

  // They are all still uncomputed and still have a single member.
  EXPECT_FALSE(cache.is_computed());
  EXPECT_FALSE(pre_compute_copy.is_computed());
  EXPECT_FALSE(pre_compute_copy_assign.is_computed());

  EXPECT_TRUE(cache.contains(id));
  EXPECT_TRUE(pre_compute_copy.contains(id));
  EXPECT_TRUE(pre_compute_copy_assign.contains(id));

  EXPECT_EQ(cache.num_geometries(), 1);
  EXPECT_EQ(pre_compute_copy.num_geometries(), 1);
  EXPECT_EQ(pre_compute_copy_assign.num_geometries(), 1);

  cache.ComputeAll();
  EXPECT_TRUE(cache.is_computed());

  MeshSdfCache post_compute_copy{cache};
  MeshSdfCache post_compute_copy_assign;
  post_compute_copy_assign = cache;

  // Copied from a computed cache, makes computed caches.
  EXPECT_TRUE(cache.is_computed());
  EXPECT_TRUE(post_compute_copy.is_computed());
  EXPECT_TRUE(post_compute_copy_assign.is_computed());

  EXPECT_TRUE(cache.contains(id));
  EXPECT_TRUE(post_compute_copy.contains(id));
  EXPECT_TRUE(post_compute_copy_assign.contains(id));

  EXPECT_EQ(cache.num_geometries(), 1);
  EXPECT_EQ(post_compute_copy.num_geometries(), 1);
  EXPECT_EQ(post_compute_copy_assign.num_geometries(), 1);
}

GTEST_TEST(MeshSdfCacheTest, MoveSemantics) {
  MeshSdfCache cache;
  const GeometryId id = RegisterShape(MakeObj<Mesh>(), &cache);

  MeshSdfCache pre_compute_moved(std::move(cache));
  EXPECT_FALSE(pre_compute_moved.is_computed());
  EXPECT_TRUE(pre_compute_moved.contains(id));
  EXPECT_EQ(pre_compute_moved.num_geometries(), 1);

  // Moved from returns to default state.
  EXPECT_FALSE(cache.is_computed());
  EXPECT_FALSE(cache.contains(id));
  EXPECT_EQ(cache.num_geometries(), 0);

  MeshSdfCache pre_compute_move_assigned;
  pre_compute_move_assigned = (std::move(pre_compute_moved));
  EXPECT_FALSE(pre_compute_move_assigned.is_computed());
  EXPECT_TRUE(pre_compute_move_assigned.contains(id));
  EXPECT_EQ(pre_compute_move_assigned.num_geometries(), 1);

  // Moved from returns to default state.
  EXPECT_FALSE(pre_compute_moved.is_computed());
  EXPECT_FALSE(pre_compute_moved.contains(id));
  EXPECT_EQ(pre_compute_moved.num_geometries(), 0);

  // Now test post-computed; we won't keep testing the state of the moved-from.
  // The move logic doesn't care about compute state.

  pre_compute_move_assigned.ComputeAll();

  MeshSdfCache post_compute_moved(std::move(pre_compute_move_assigned));
  EXPECT_TRUE(post_compute_moved.is_computed());
  EXPECT_TRUE(post_compute_moved.contains(id));
  EXPECT_EQ(post_compute_moved.num_geometries(), 1);

  MeshSdfCache post_compute_move_assigned;
  post_compute_move_assigned = std::move(post_compute_moved);
  EXPECT_TRUE(post_compute_move_assigned.is_computed());
  EXPECT_TRUE(post_compute_move_assigned.contains(id));
  EXPECT_EQ(post_compute_move_assigned.num_geometries(), 1);
}

GTEST_TEST(MeshSdfCacheTest, Register) {
  MeshSdfCache cache;

  // Unsupported extensions are silently ignored for both Mesh and Convex.
  const GeometryId id_unsupported = GeometryId::get_new_id();
  const Mesh glb(InMemoryMesh{MemoryFile("", ".fake", "dummy")});
  cache.Register(id_unsupported, glb);
  EXPECT_FALSE(cache.contains(id_unsupported));
  EXPECT_EQ(cache.num_geometries(), 0);

  const Convex glb_convex(InMemoryMesh{MemoryFile("", ".fake", "dummy")});
  cache.Register(id_unsupported, glb_convex);
  EXPECT_FALSE(cache.contains(id_unsupported));
  EXPECT_EQ(cache.num_geometries(), 0);

  // Obj is supported for Mesh and Convex.
  const GeometryId id_mesh_obj = RegisterShape(MakeObj<Mesh>(), &cache);
  EXPECT_TRUE(cache.contains(id_mesh_obj));
  EXPECT_EQ(cache.num_geometries(), 1);

  const GeometryId id_convex_obj = RegisterShape(MakeObj<Convex>(), &cache);
  EXPECT_TRUE(cache.contains(id_convex_obj));
  EXPECT_EQ(cache.num_geometries(), 2);

  // Vtk is supported for Mesh and Convex.
  const GeometryId id_mesh_vtk = RegisterShape(MakeVtk<Mesh>(), &cache);
  EXPECT_TRUE(cache.contains(id_mesh_vtk));
  EXPECT_EQ(cache.num_geometries(), 3);

  const GeometryId id_convex_vtk = RegisterShape(MakeVtk<Convex>(), &cache);
  EXPECT_TRUE(cache.contains(id_convex_vtk));
  EXPECT_EQ(cache.num_geometries(), 4);

  // Repeated registrations of same id throw, regardless of mesh type.
  ASSERT_TRUE(cache.contains(id_mesh_obj));
  EXPECT_THROW(cache.Register(id_mesh_obj, MakeObj<Mesh>()), std::exception);
  EXPECT_THROW(cache.Register(id_mesh_obj, MakeObj<Convex>()), std::exception);

  ASSERT_FALSE(cache.is_computed());
  ASSERT_THROW(cache.GetBoundary(id_mesh_obj), std::exception);

  cache.ComputeAll();

  ASSERT_TRUE(cache.is_computed());
  ASSERT_NO_THROW(cache.GetBoundary(id_mesh_obj));

  // New registration after compute is immediately available.
  const GeometryId id_new_mesh = RegisterShape(MakeObj<Mesh>(), &cache);
  EXPECT_NO_THROW(cache.GetBoundary(id_new_mesh));
  const GeometryId id_new_convex = RegisterShape(MakeObj<Convex>(), &cache);
  EXPECT_NO_THROW(cache.GetBoundary(id_new_convex));
}

GTEST_TEST(MeshSdfCacheTest, Remove) {
  MeshSdfCache cache;

  std::vector<GeometryId> ids;
  constexpr int kNumGeometries = 5;
  for (int i = 0; i < kNumGeometries; ++i) {
    // Mix up the MeshLike types.
    if (i % 2 == 0) {
      ids.push_back(RegisterShape(MakeObj<Mesh>(), &cache));
    } else {
      ids.push_back(RegisterShape(MakeObj<Convex>(), &cache));
    }
  }
  EXPECT_EQ(cache.num_geometries(), kNumGeometries);

  // Removing an unregistered id is a no-op (populated cache).
  EXPECT_NO_THROW(cache.Remove(GeometryId::get_new_id()));

  // We'll remove them in a different order; even indexed and then odd.
  int expected_num = kNumGeometries;
  for (int i = 0; i < kNumGeometries; i += 2) {
    cache.Remove(ids[i]);
    --expected_num;
    EXPECT_FALSE(cache.contains(ids[i]));
  }
  for (int i = 1; i < kNumGeometries; i += 2) {
    cache.Remove(ids[i]);
    --expected_num;
    EXPECT_FALSE(cache.contains(ids[i]));
  }

  // Removing an unregistered id is a no-op (empty cache).
  EXPECT_NO_THROW(cache.Remove(GeometryId::get_new_id()));
}

// We call ComputeAll() all over the tests. We'll consider it will tested and
// account for it here as a matter of form.
GTEST_TEST(MeshSdfCacheTest, ComputeAllAndGetBoundary) {
  MeshSdfCache cache;
  const GeometryId id = RegisterShape(MakeObj<Mesh>(), &cache);

  // Throws for bad ids and uncomputed boundaries.
  EXPECT_THROW(cache.GetBoundary(GeometryId::get_new_id()), std::exception);
  EXPECT_THROW(cache.GetBoundary(id), std::exception);

  cache.ComputeAll();

  // Still throws for bad ids, but existing boundaries are happy.
  EXPECT_THROW(cache.GetBoundary(GeometryId::get_new_id()), std::exception);
  EXPECT_NO_THROW(cache.GetBoundary(id));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
