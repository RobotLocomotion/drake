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

/* Flat cache for deferred, shared MeshDistanceBoundary computation.

 MeshDistanceBoundary construction (BVH + FeatureNormalSet) is deferred from
 geometry registration time to the first query that needs each
 (mesh source, scale) combination. Individual MeshDistanceBoundary instances are
 cached and shared across:
  - All geometries that reference the same mesh source at the same scale.
  - All MeshSdfCache instances that share the same underlying table (i.e.,
    instances that were copy-assigned from the same original).

 Cache structure:
  - A shared table maps (mesh_key, scale_key) → shared_ptr<CacheEntry>. Each
    CacheEntry holds either a factory for creating a TriangleSurfaceMesh from
    the specified mesh (from which the mesh boundary gets computed in
    GetOrCompute()) *or* a computed mesh boundary. For each novel (mesh, scale)
    pair, the mesh is read from disk anew. Entries get evicted when the last
    geometry (across all copies of the MeshSdfCache) sharing the cache entry's
    mesh boundary is removed from a cache copy.
  - Each MeshSdfCache contains its own map from GeometryId to GeometryEntry.
    Each geometry entry contains the mesh key (mesh_key, scale_key) for the
    declared geometry and a shared pointer to the mesh boundary stored in the
    table. These independent geometry populations allow for independent mutation
    of different caches while still preserving the shared boundary data.

 Thread safety: during the query phase — after all geometry has been
 registered and the set of registered geometries is stable — concurrent
 calls to GetOrCompute() and contains() across any instances sharing the
 same underlying table are safe. GetOrCompute() uses an internal
 shared_mutex to serialize the first computation for each (mesh source, scale)
 pair while allowing subsequent reads to proceed in parallel.

 During the mutation phase — construction, destruction, copy assignment,
 Register(), and Remove() — these operations acquire an exclusive lock on
 the shared table and so are safe with respect to concurrent GetOrCompute()
 calls on *other* instances. However, they must not be called concurrently
 with any other method on the *same* instance, since the per-cache entry map
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
   reused. Files with .vtk and .obj extensions are supported. Unsupported mesh
   types are silently ignored; such geometry will not be queryable via
   GetOrCompute().
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

  /* Identifies a specific (mesh source, scale) entry in the shared table.
   Used as the value type of the per-instance geometry index. */
  struct Key {
    std::string mesh_key;  // Value of {Mesh|Convex}::GetCacheKey()
    ScaleKey scale_key;
  };

  /* Stores, for one (mesh source, scale) combination, either the deferred
   factory for building the boundary mesh or the computed boundary itself.
   Exactly one of the two is live at any time: the factory is set (and the
   boundary is absent) before the first GetOrCompute() call; afterward the
   boundary holds the result and the factory has been cleared to release any
   captured resources (e.g. mesh source data). */
  class CacheEntry {
   public:
    explicit CacheEntry(std::function<TriangleSurfaceMesh<double>()> make_mesh);

    /* Returns true once ComputeBoundary() has been called. */
    bool has_boundary() const;

    /* Returns the computed boundary.
     @pre has_boundary(). */
    const MeshDistanceBoundary& boundary() const;

    /* Invokes the stored factory, builds the MeshDistanceBoundary, stores the
     result, and clears the factory.
     @pre !has_boundary(). */
    void ComputeBoundary();

   private:
    std::function<TriangleSurfaceMesh<double>()> make_mesh_;
    std::optional<MeshDistanceBoundary> boundary_;
  };

  /* Per-geometry entry: the (mesh, scale) key used for eviction, plus a
   shared handle to the entry data. */
  struct GeometryEntry {
    // The (mesh_key, scale_key) for a registered geometry id; used to
    // efficiently update the shared table upon geometry removal.
    Key key;

    // The handle to the shared cache entry containing the boundary data (or
    // the means to make one).
    std::shared_ptr<CacheEntry> entry_handle;
  };

  /* Shared table owned via shared_ptr. All MeshSdfCache instances derived
   from the same original (via copy assignment) share the same instance.

   std::map is used (not unordered_map) because ScaleKey
   (std::array<double,3>) has no standard hash, making a composite
   unordered_map key awkward. */
  struct SharedTable {
    /* Guards all access to entries during lazy computation and during
     mutation (Register, Remove, copy assignment, destruction). */
    mutable std::shared_mutex mutex;

    std::map<std::pair<std::string, ScaleKey>, std::shared_ptr<CacheEntry>>
        entries;
  };

  /* Introduces a cache-local entry for `id` and optionally updates the shared
   table as necessary.

   For the shared cache table to be robust against arbitrary changes to
   individual cache instances, a cache entry mesh factory needs to contain a
   full copy of the mesh source so it can reliably produce the boundary mesh.
   However, we don't want to create the copy of the mesh source unless we need
   to. To that end, make_factor is a factory for a mesh factory. Only if a new
   cache table entry is necessary, will make_factory() be called and the
   resultant mesh_factory will be stored in the table. That mesh_factory must
   be able to produce a TriangleSurfaceMesh when called, regardless of whether
   the original mesh source is still available. The mesh_factory() only gets
   evaluated in GetOrCompute().

   This work is done with the acquisition of an exclusive lock. */
  void RegisterImpl(
      GeometryId id, const std::string& mesh_key, const ScaleKey& scale_key,
      std::function<std::function<TriangleSurfaceMesh<double>()>()>
          make_factory);

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
