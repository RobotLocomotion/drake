#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_export.h"
#include "drake/common/hash.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/memoizer_cache.h"
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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshSdfCache)

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

  /* Identifies a specific (mesh source, scale) entry in the shared table.
   Used as the value type of the per-instance geometry index. */
  struct Key {
    bool operator==(const Key& other) const = default;

    /* Implements the @ref hash_append concept. */
    template <class HashAlgorithm>
    friend void hash_append(HashAlgorithm& hasher, const Key& key) noexcept {
      using drake::hash_append;
      hash_append(hasher, key.mesh_key);
      hash_append(hasher, key.scale_key[0]);
      hash_append(hasher, key.scale_key[1]);
      hash_append(hasher, key.scale_key[2]);
    }

    std::string mesh_key;  // Value of {Mesh|Convex}::GetCacheKey()
    Eigen::Vector3d scale_key;
  };

  /* Per-geometry entry ... */
  struct GeometryEntry {
    GeometryEntry(Key key_in,
                  std::function<MeshDistanceBoundary()> make_boundary_in)
        : key(key_in), make_boundary(std::move(make_boundary_in)) {}
    GeometryEntry(const GeometryEntry& other);
    GeometryEntry& operator=(const GeometryEntry& other);
    ~GeometryEntry();

    // The handle to the shared cached boundary data (possibly null).
    mutable std::atomic<std::shared_ptr<const MeshDistanceBoundary>> boundary;

    // The (mesh_key, scale_key) for a registered geometry id.
    Key key;

    // A threadsafe factory to construct boundary when it's null.
    std::function<MeshDistanceBoundary()> make_boundary;
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
  void RegisterImpl(GeometryId id, const Key& key,
                    std::function<TriangleSurfaceMesh<double>()> make_tri_mesh);

  std::shared_ptr<const MemoizerCache<Key, MeshDistanceBoundary>> memoizer_{
      std::make_shared<const MemoizerCache<Key, MeshDistanceBoundary>>()};

  /* Mapping of geometry_id → GeometryEntry. Each instance of MeshSdfCache
   maintains a unique mapping (not shared). */
  std::unordered_map<GeometryId, GeometryEntry> geometry_entries_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake

namespace std {
template <>
struct hash<drake::geometry::internal::MeshSdfCache::Key>
    : public drake::DefaultHash {};
}  // namespace std
