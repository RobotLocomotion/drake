#include "drake/geometry/proximity/mesh_distance_boundary_cache.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/unused.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

// An obj and vtk both representing a single tet. The details don't matter,
// only that they turn into MeshDistanceBoundary objects (no throws).
constexpr char kTetObj[] = R"""(
v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
f 3 2 1
f 1 2 4
f 1 4 3
f 2 3 4
)""";

constexpr char kTetVtk[] = R"""(# vtk DataFile Version 2.0
one tet
ASCII
DATASET UNSTRUCTURED_GRID
POINTS 4 double
0 0 0
1 0 0
0 1 0
0 0 1
CELLS 1 5
4 0 1 2 3
CELL_TYPES 1
10
)""";

// Returns a Mesh backed by an in-memory OBJ.
template <typename MeshLike>
MeshLike MakeObj(double scale = 1.0) {
  return MeshLike(InMemoryMesh{MemoryFile(kTetObj, ".obj", "tet.obj")}, scale);
}

// Returns a Mesh backed by an in-memory VTK.
template <typename MeshLike>
MeshLike MakeVtk(double scale = 1.0) {
  return MeshLike(InMemoryMesh{MemoryFile(kTetVtk, ".vtk", "tet.vtk")}, scale);
}

// Registers a shape (Mesh or Convex) with the given cache. Returns a geometry
// id for it.
template <typename MeshLike>
GeometryId RegisterShape(const MeshLike& mesh,
                         MeshDistanceBoundaryCache* cache) {
  const GeometryId id = GeometryId::get_new_id();
  cache->Register(id, mesh);
  return id;
}

// Note: We don't have a specific test for GetBoundary(). We call it so much
// as evidence of presence, that it is sufficiently tested.

GTEST_TEST(MeshDistanceBoundaryCacheTesterTest, DefaultCache) {
  MeshDistanceBoundaryCache cache;
  EXPECT_FALSE(cache.is_live());
  EXPECT_EQ(cache.GetBoundary(GeometryId::get_new_id()), nullptr);
  EXPECT_EQ(cache.num_geometries(), 0);

  // Computing an empty cache sets it to be "live".
  cache.ComputeAll();
  EXPECT_TRUE(cache.is_live());
  EXPECT_EQ(cache.GetBoundary(GeometryId::get_new_id()), nullptr);
  EXPECT_EQ(cache.num_geometries(), 0);
}

GTEST_TEST(MeshDistanceBoundaryCacheTesterTest, CopySemantics) {
  MeshDistanceBoundaryCache cache;
  const GeometryId id = RegisterShape(MakeObj<Mesh>(), &cache);

  MeshDistanceBoundaryCache pre_compute_copy{cache};
  MeshDistanceBoundaryCache pre_compute_copy_assign;
  pre_compute_copy_assign = cache;

  // They are all still uncomputed and still have a single member.
  EXPECT_FALSE(cache.is_live());
  EXPECT_FALSE(pre_compute_copy.is_live());
  EXPECT_FALSE(pre_compute_copy_assign.is_live());

  EXPECT_EQ(cache.num_geometries(), 1);
  EXPECT_EQ(pre_compute_copy.num_geometries(), 1);
  EXPECT_EQ(pre_compute_copy_assign.num_geometries(), 1);

  // All copies report the lack of boundary data.
  EXPECT_EQ(cache.GetBoundary(id), nullptr);
  EXPECT_EQ(pre_compute_copy.GetBoundary(id), nullptr);
  EXPECT_EQ(pre_compute_copy_assign.GetBoundary(id), nullptr);

  cache.ComputeAll();
  EXPECT_TRUE(cache.is_live());

  MeshDistanceBoundaryCache post_compute_copy{cache};
  MeshDistanceBoundaryCache post_compute_copy_assign;
  post_compute_copy_assign = cache;

  // Copied from a computed cache, makes computed caches.
  EXPECT_TRUE(cache.is_live());
  EXPECT_TRUE(post_compute_copy.is_live());
  EXPECT_TRUE(post_compute_copy_assign.is_live());

  EXPECT_EQ(cache.num_geometries(), 1);
  EXPECT_EQ(post_compute_copy.num_geometries(), 1);
  EXPECT_EQ(post_compute_copy_assign.num_geometries(), 1);

  // The copies are independent, for the same id, all boundaries are defined
  // but are different objects.
  EXPECT_NE(cache.GetBoundary(id), nullptr);
  EXPECT_NE(post_compute_copy.GetBoundary(id), nullptr);
  EXPECT_NE(post_compute_copy_assign.GetBoundary(id), nullptr);
  EXPECT_NE(cache.GetBoundary(id), post_compute_copy.GetBoundary(id));
  EXPECT_NE(cache.GetBoundary(id), post_compute_copy_assign.GetBoundary(id));
  EXPECT_NE(post_compute_copy.GetBoundary(id),
            post_compute_copy_assign.GetBoundary(id));
}

GTEST_TEST(MeshDistanceBoundaryCacheTesterTest, MoveSemantics) {
  MeshDistanceBoundaryCache cache;
  const GeometryId id = RegisterShape(MakeObj<Mesh>(), &cache);
  ASSERT_EQ(cache.num_geometries(), 1);

  MeshDistanceBoundaryCache pre_compute_moved(std::move(cache));
  EXPECT_FALSE(pre_compute_moved.is_live());
  EXPECT_EQ(pre_compute_moved.num_geometries(), 1);

  // Moved from returns to default state.
  EXPECT_FALSE(cache.is_live());
  EXPECT_EQ(cache.num_geometries(), 0);

  MeshDistanceBoundaryCache pre_compute_move_assigned;
  pre_compute_move_assigned = (std::move(pre_compute_moved));
  EXPECT_FALSE(pre_compute_move_assigned.is_live());
  EXPECT_EQ(pre_compute_move_assigned.num_geometries(), 1);

  // Moved-from returns to default state.
  EXPECT_FALSE(pre_compute_moved.is_live());
  EXPECT_EQ(pre_compute_moved.num_geometries(), 0);

  // Now test post-computed; we won't keep testing the state of the moved-from.
  // The move logic doesn't care about compute state.
  pre_compute_move_assigned.ComputeAll();

  const MeshDistanceBoundary* boundary =
      pre_compute_move_assigned.GetBoundary(id);
  EXPECT_NE(boundary, nullptr);

  MeshDistanceBoundaryCache post_compute_moved(
      std::move(pre_compute_move_assigned));
  EXPECT_TRUE(post_compute_moved.is_live());
  EXPECT_EQ(post_compute_moved.GetBoundary(id), boundary);
  EXPECT_EQ(post_compute_moved.num_geometries(), 1);

  MeshDistanceBoundaryCache post_compute_move_assigned;
  post_compute_move_assigned = std::move(post_compute_moved);
  EXPECT_TRUE(post_compute_move_assigned.is_live());
  EXPECT_EQ(post_compute_move_assigned.GetBoundary(id), boundary);
  EXPECT_EQ(post_compute_move_assigned.num_geometries(), 1);
}

GTEST_TEST(MeshDistanceBoundaryCacheTesterTest, Register) {
  MeshDistanceBoundaryCache cache;

  // Unsupported extensions are silently ignored for both Mesh and Convex.
  const GeometryId id_unsupported = GeometryId::get_new_id();
  const Mesh fake_mesh(InMemoryMesh{MemoryFile("", ".fake", "dummy")});
  cache.Register(id_unsupported, fake_mesh);
  EXPECT_EQ(cache.num_geometries(), 0);

  const Convex fake_convex(InMemoryMesh{MemoryFile("", ".fake", "dummy")});
  cache.Register(id_unsupported, fake_convex);
  EXPECT_EQ(cache.num_geometries(), 0);

  // Obj is supported for Mesh and Convex.
  const GeometryId id_mesh_obj = RegisterShape(MakeObj<Mesh>(), &cache);
  EXPECT_EQ(cache.num_geometries(), 1);

  const GeometryId id_convex_obj = RegisterShape(MakeObj<Convex>(), &cache);
  EXPECT_EQ(cache.num_geometries(), 2);

  // Vtk is supported for Mesh and Convex.
  const GeometryId id_mesh_vtk = RegisterShape(MakeVtk<Mesh>(), &cache);
  EXPECT_EQ(cache.num_geometries(), 3);

  const GeometryId id_convex_vtk = RegisterShape(MakeVtk<Convex>(), &cache);
  EXPECT_EQ(cache.num_geometries(), 4);

  // We don't actually reference these ids anymore.
  unused(id_mesh_vtk, id_convex_obj, id_convex_vtk);

  // Repeated registrations of same id throw, regardless of mesh type.
  EXPECT_THROW(cache.Register(id_mesh_obj, MakeObj<Mesh>()), std::exception);
  EXPECT_THROW(cache.Register(id_mesh_obj, MakeObj<Convex>()), std::exception);

  cache.ComputeAll();
  const MeshDistanceBoundary* boundary = cache.GetBoundary(id_mesh_obj);
  ASSERT_NE(boundary, nullptr);

  // New registration after compute is immediately available.
  const GeometryId id_new_mesh = RegisterShape(MakeObj<Mesh>(), &cache);
  EXPECT_NE(cache.GetBoundary(id_new_mesh), nullptr);
  const GeometryId id_new_convex = RegisterShape(MakeObj<Convex>(), &cache);
  EXPECT_NE(cache.GetBoundary(id_new_convex), nullptr);
  // But it didn't change the old ones.
  EXPECT_EQ(cache.GetBoundary(id_mesh_obj), boundary);
}

GTEST_TEST(MeshDistanceBoundaryCacheTesterTest, Remove) {
  MeshDistanceBoundaryCache cache;

  // Attempting an invalid remove from an uncomputed cache is no problem.
  const GeometryId unregistered_id = GeometryId::get_new_id();
  EXPECT_NO_THROW(cache.Remove(unregistered_id));

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

  // Attempting an invalid remove from an uncomputed cache is no problem.
  EXPECT_NO_THROW(cache.Remove(unregistered_id));

  cache.ComputeAll();

  // We'll remove them in a different order; even indexed and then odd.
  for (int i = 0; i < kNumGeometries; i += 2) {
    EXPECT_NE(cache.GetBoundary(ids[i]), nullptr);
    cache.Remove(ids[i]);
    EXPECT_EQ(cache.GetBoundary(ids[i]), nullptr);
  }

  // Attempting an invalid remove from an *computed* cache is still no problem.
  EXPECT_NO_THROW(cache.Remove(unregistered_id));

  for (int i = 1; i < kNumGeometries; i += 2) {
    EXPECT_NE(cache.GetBoundary(ids[i]), nullptr);
    cache.Remove(ids[i]);
    EXPECT_EQ(cache.GetBoundary(ids[i]), nullptr);
  }

  EXPECT_EQ(cache.num_geometries(), 0);
}

// We call ComputeAll() all over the tests. We'll consider it will tested and
// account for it here as a matter of form. We'll also peek at the boundary
// data to indicate that, indeed, it is populated.
GTEST_TEST(MeshDistanceBoundaryCacheTesterTest, ComputeAllAndGetBoundary) {
  MeshDistanceBoundaryCache cache;
  const GeometryId id = RegisterShape(MakeObj<Mesh>(), &cache);

  // Returns nullptr for bad ids and uncomputed boundaries.
  EXPECT_EQ(cache.GetBoundary(GeometryId::get_new_id()), nullptr);
  EXPECT_EQ(cache.GetBoundary(id), nullptr);

  cache.ComputeAll();

  // Still returns null for bad ids, but existing boundaries are happy.
  EXPECT_EQ(cache.GetBoundary(GeometryId::get_new_id()), nullptr);
  EXPECT_NO_THROW(cache.GetBoundary(id));

  // Sniff at the boundary data, if the mesh has vertices, it's probably fine.
  EXPECT_EQ(cache.GetBoundary(id)->tri_mesh().num_vertices(), 4);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
