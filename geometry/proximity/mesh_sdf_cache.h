#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "drake/common/hash.h"
#include "drake/common/reset_after_move.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/memoizer_cache.h"
#include "drake/geometry/proximity/mesh_distance_boundary.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* Cache mapping GeometryId → MeshDistanceBoundary for point-distance queries.

 MeshDistanceBoundary construction (BVH + FeatureNormalSet) is deferred from
 geometry registration time until ComputeAll() is first called, at which point
 every registered geometry is built in one pass. MeshDistanceBoundary instances
 are shared across:
  - All geometries that reference the same mesh source at the same scale.
  - All MeshSdfCache instances that share the same underlying memoization table
    (i.e., instances that are copies of a common source instance).

 Cache structure:
  - A shared memoization table maps (mesh_key, scale_key) →
    shared_ptr<MeshDistanceBoundary>. For each novel (mesh, scale) pair the
    boundary is computed once; subsequent lookups return the cached result.
    Entries are evicted when the last geometry (across all copies of
    MeshSdfCache) holding a reference to that boundary is removed.
  - Each MeshSdfCache has its own map from GeometryId to GeometryEntry.
    Each entry holds a shared pointer to its boundary plus a factory used to
    build that boundary on demand. These independent per-MeshSdfCache instance
    populations allow each population to change independently, while still
    sharing the underlying boundary data.

 Thread safety::
  - Mutation phase (Register, Remove): single-threaded; must not overlap with
    the query phase on the same instance.
  - Query phase: ComputeAll() builds every registered boundary on first call and
    is a cheap no-op thereafter. GetBoundary() then returns the pre-built
    result. Each ProximityEngine copy owns a distinct MeshSdfCache instance, so
    query methods on different instances can run concurrently without
    coordination; cross-instance deduplication is handled by the shared
    memoization table's own internal mutex.

 Note: When moving *from* a MeshSdfCache instance, the source instance will
 still be a valid, empty cache but will no longer share any data with the
 target (or any of its related instances); it starts a new, independent line. */
class MeshSdfCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshSdfCache);

  MeshSdfCache();
  ~MeshSdfCache();

  /* Returns true if geometry `id` has been registered. */
  bool contains(GeometryId id) const { return geometry_entries_.contains(id); }

  /* Registers a Mesh geometry. The boundary will not be computed until
   ComputeAll() is called. If ComputeAll() has already been called, the boundary
   is computed immediately. Files with .vtk and .obj extensions are supported.
   Unsupported mesh types are silently ignored; such geometry will not be
   queryable via GetBoundary(). */
  void Register(GeometryId id, const Mesh& mesh);

  /* Registers a Convex geometry. The boundary will not be computed until
   ComputeAll() is called. If ComputeAll() has already been called, the boundary
   is computed immediately. Files with .vtk and .obj extensions are supported.
   Unsupported mesh types are silently ignored; such geometry will not be
   queryable via GetBoundary(). */
  void Register(GeometryId id, const Convex& convex);

  /* Removes a geometry. If this geometry is the last that refers to a mesh
   boundary, that boundary data will be removed from the cache. */
  void Remove(GeometryId id);

  /* Computes and caches MeshDistanceBoundary for every registered geometry.
   After this call, GetBoundary() is valid for all registered geometry ids.
   Subsequent calls are no-ops. */
  void ComputeAll() const;

  /* Returns the MeshDistanceBoundary for `id`.
   The reference is valid as long as the cache maintains a reference to the
   entry; calls to Remove(id) or cache destruction invalidate it.
   @pre contains(id) is true.
   @pre ComputeAll() has been called. */
  const MeshDistanceBoundary& GetBoundary(GeometryId id) const;

 private:
  friend class MeshSdfCacheTester;

  /* Identifies a specific (mesh source, scale) entry in the shared table. Each
   geometry is associated with a specific key. */
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

  /* Per-geometry entry. */
  struct GeometryEntry {
    GeometryEntry(Key key_in,
                  std::function<MeshDistanceBoundary()> make_boundary_in)
        : key(std::move(key_in)), make_boundary(std::move(make_boundary_in)) {}

    // Populated by ComputeAll() (or eagerly by Register() after ComputeAll()).
    std::shared_ptr<const MeshDistanceBoundary> boundary;

    Key key;

    // Factory to construct boundary; nulled out after boundary is computed.
    std::function<MeshDistanceBoundary()> make_boundary;
  };

  /* Store the key and mesh factor in *this* instance's geometry table.*/
  void RegisterImpl(GeometryId id, const Key& key,
                    std::function<TriangleSurfaceMesh<double>()> make_tri_mesh);

  // Computes the boundary for a single entry. This is uniquely responsible for
  // confirming the factory method is null when the boundary is populated.
  void ComputeEntry(GeometryEntry* entry) const;

  // A wrapper around the shared memoizer that promises to recreate a valid
  // memoizer after it has been moved from.
  class ResettingMemoizer {
   public:
    using MyMemoizer = MemoizerCache<Key, MeshDistanceBoundary>;

    ResettingMemoizer();
    ResettingMemoizer(const ResettingMemoizer& other);
    ResettingMemoizer& operator=(const ResettingMemoizer& other);
    ResettingMemoizer(ResettingMemoizer&& other) noexcept;
    ResettingMemoizer& operator=(ResettingMemoizer&& other) noexcept;

    // Facilitate pointer-comparisons.
    operator const MyMemoizer*() const { return memoizer_.get(); }
    // Allow it to be referenced as if it were simply the shared_ptr.
    const MyMemoizer* operator->() const { return memoizer_.get(); }
    const MyMemoizer& operator*() const { return *memoizer_; }

    std::shared_ptr<const MyMemoizer> memoizer_{};
  };

  /* Shared table mapping (mesh_key, scale_key) → MeshDistanceBoundary. This is
   shared across all copies of MeshSdfCache that were ultimately copied from the
   same source cache instance. */
  ResettingMemoizer memoizer_{};

  /* True after ComputeAll() has been called; makes ComputeAll() a no-op and
   causes Register() to eagerly compute the boundary for new entries. */
  mutable reset_after_move<bool> is_computed_;

  /* Mapping of geometry_id → GeometryEntry. Each instance of MeshSdfCache
   maintains a unique mapping (not shared). */
  mutable std::unordered_map<GeometryId, GeometryEntry> geometry_entries_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake

namespace std {
template <>
struct hash<drake::geometry::internal::MeshSdfCache::Key>
    : public drake::DefaultHash {};
}  // namespace std
