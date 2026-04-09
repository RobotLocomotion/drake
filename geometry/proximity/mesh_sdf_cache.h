#pragma once

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include "drake/common/drake_export.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/mesh_distance_boundary.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* Two-level cache for deferred, shared MeshDistanceBoundary computation.

 MeshDistanceBoundary construction (BVH + FeatureNormalSet) is deferred from
 geometry registration time to the first query that needs each
 (source file, scale) combination. Individual MeshDistanceBoundary instances are
 cached and shared across:
  - All geometries that reference the same source file at the same scale.
  - All MeshSdfCache instances that share the same underlying table (i.e.,
    instances that were copy-assigned from the same original).

 Cache structure:
  - A shared table maps file_key → (unit_mesh, scale_key →
    shared_ptr<optional<MeshDistanceBoundary>>). The MeshDistanceBoundary is
    computed lazily on the first GetOrCompute() call for each (file_key,
    scale_key) pair. The shared_ptr use count tracks how many geometry
    registrations — across all instances sharing the table — reference each
    (file, scale) entry; entries are evicted when no registrations remain.
  - A per-instance index maps geometry_id → (Key, shared_ptr), giving direct
    access to the boundary without a second table lookup at query time.

 Thread safety: during the query phase — after all geometry has been
 registered and the set of registered geometries is stable — concurrent
 calls to GetOrCompute() and contains() across any instances sharing the
 same underlying table are safe. GetOrCompute() uses an internal
 shared_mutex to serialize the first computation for each (source, scale)
 pair while allowing subsequent reads to proceed in parallel.

 During the mutation phase — construction, destruction, copy assignment,
 Register(), and Remove() — these operations acquire an exclusive lock on
 the shared table and so are safe with respect to concurrent GetOrCompute()
 calls on *other* instances. However, they must not be called concurrently
 with any other method on the *same* instance, since the per-instance index
 (geometry_entries_) is not protected by any lock. */
class MeshSdfCache {
 public:
  // Copying requires thread coordination (it changes the reference counts), so
  // we require custom implementations. Moving transfers ownership without
  // touching reference counts, but the moved-from must be left with a fresh
  // empty table so its destructor is safe to call.
  MeshSdfCache(const MeshSdfCache&);
  MeshSdfCache(MeshSdfCache&&);
  MeshSdfCache& operator=(const MeshSdfCache& other);
  MeshSdfCache& operator=(MeshSdfCache&&);

  MeshSdfCache();
  ~MeshSdfCache();

  /* Returns true if geometry `id` has been registered. */
  bool contains(GeometryId id) const { return geometry_entries_.contains(id); }

  /* Registers a Mesh geometry. If this is the first registration for `mesh`'s
   source across all instances sharing this table, the unit-scale (1,1,1)
   surface mesh is loaded and stored; otherwise the stored unit mesh is
   reused. Files with .vtk and .obj extensions are supported. Unsupported files
   are silently ignored; such geometry will not be queryable via GetOrCompute().
   */
  void Register(GeometryId id, const Mesh& mesh);

  /* Registers a Convex geometry. If this is the first registration for
   `convex`'s source across all instances sharing this table, the
   triangulated convex hull at unit scale is stored; otherwise the stored
   unit mesh is reused. Convex has wider support for mesh file formats (as
   described in geometry_file_formats_doxygen.h). */
  void Register(GeometryId id, const Convex& convex);

  /* Removes a geometry. If this geometry is the last that refers to a mesh
   boundary, that boundary data will be removed from the cache. */
  void Remove(GeometryId id);

  /* Returns the MeshDistanceBoundary for `id`, computing and caching it on
   the first call for each (source, scale) pair.
   The reference is valid as long as the cache maintains a reference to the
   entry. So, calls to Remove(id) or destruction of the cache will invalidate
   the reference.
   @pre contains(id) is true. */
  const MeshDistanceBoundary& GetOrCompute(GeometryId id) const;

 private:
  friend class MeshSdfCacheTester;

  using ScaleKey = std::array<double, 3>;

  /* Identifies a specific (source file, scale) entry in the shared table.
   Used as the value type of the per-instance geometry index.  */
  struct Key {
    std::string file_key;
    ScaleKey scale_key;
  };

  /* Per-geometry entry: the (file, scale) key used for eviction, plus a
   shared handle to the lazily-computed boundary.  */
  struct GeometryEntry {
    Key key;
    std::shared_ptr<std::optional<MeshDistanceBoundary>> boundary_handle;
  };

  /* All data for one mesh source, for all scales.  */
  struct FileEntry {
    /* Boundary surface mesh at unit scale (1, 1, 1).  */
    TriangleSurfaceMesh<double> unit_mesh;

    /* Per-scale boundary data.

     We require a unique MeshDistanceBoundary for each unique scale. We cannot
     simply apply affine transformation (the scale) to the query point, solve
     it in a canonical space, and then transform the result back. Distance
     queries rely on *orthogonality* and affine transformations do not preserve
     orthogonality.

     std::map is used (not unordered_map) because
     ScaleKey (std::array<double,3>) has no standard hash.  */
    std::map<ScaleKey, std::shared_ptr<std::optional<MeshDistanceBoundary>>>
        scales;
  };

  /* Shared table owned via shared_ptr. All MeshSdfCache instances derived
   from the same original (via copy assignment) share the same instance.  */
  struct SharedTable {
    /* Guards all access to file_entries during lazy computation and during
     mutation (Register, Remove, copy assignment, destruction).  */
    mutable std::shared_mutex mutex;

    std::unordered_map<std::string, FileEntry> file_entries;
  };

  /* Common bookkeeping for Register(). Acquires an exclusive lock, calls
   make_unit_mesh() to obtain the unit-scale surface mesh if and only if this
   is the first registration for file_key, then increments the ref count and
   records the key. If make_unit_mesh() returns nullopt the geometry is
   silently skipped (used to handle unsupported file extensions). */
  void RegisterImpl(GeometryId id, const std::string& file_key,
                    const ScaleKey& scale_key,
                    std::function<std::optional<TriangleSurfaceMesh<double>>()>
                        make_unit_mesh);

  /* Flushes this cache, removing all entries and decrementing all reference
   counts. */
  void Flush();

  /* The table of cached meshes and boundaries. This table is shared across all
   MeshSdfCache instances derived from the same original. Never null. */
  std::shared_ptr<SharedTable> table_;

  /* Mapping of geometry_id → GeometryEntry. Each instance of MeshSdfCache
   maintains a unique index. */
  std::unordered_map<GeometryId, GeometryEntry> geometry_entries_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
