#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "drake/common/hash.h"
#include "drake/common/reset_after_move.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/mesh_distance_boundary.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* A repository for declared mesh-like Shapes that require MeshDistanceBoundary
 instances for distance-to-point queries. The boundary data will not initially
 be computed; it is only computed upon calling GetBoundary() (at which time
 all registered geometries will have their boundary data computed if possible).

 A unique repository is owned by each ProximityEngine instance. Each proximity
 engine will independently compute mesh boundaries as is needed for that engine.

 In the future, we may introduce a re-use capability where the underlying
 data can be shared across multiple instances for a given mesh at a given scale.
 But for now, we're content with deferring computation and memory footprint
 until we know we need it.

 Note: this is not threadsafe at all. It relies on the fact that one cache
 instance is owned by one ProximityEngine instance, contained in a single
 Context. Drake advises against doing work on a single Context in multiple
 threads. If that advice is followed, the operations on a single cache instance
 will always be serialized. */
class MeshSdfCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshSdfCache);

  MeshSdfCache();
  ~MeshSdfCache();

  /* Registers a Mesh geometry. The boundary will not be computed until
   GetBoundary() is called. If this cache already contains boundary data, then
   the boundary data for this mesh will be computed immediately. Files with .vtk
   and .obj extensions are supported. Unsupported mesh types are silently
   ignored; such geometry will not be queryable via GetBoundary().
   @throws if `contains(id)`. */
  void Register(GeometryId id, const Mesh& mesh);

  /* Registers a Convex geometry. The boundary will not be computed until
   GetBoundary() is called. If this cache already contains boundary data, then
   the boundary data for this convex will be computed immediately. Files with
   .vtk, .obj, and .gltf extensions are supported. Unsupported mesh types are
   silently ignored; such geometry will not be queryable via GetBoundary().
   @throws if `contains(id)`. */
  void Register(GeometryId id, const Convex& convex);

  /* Removes a geometry. */
  void Remove(GeometryId id);

  /* Reports the number of registered geometries. */
  int num_geometries() const { return std::ssize(geometry_entries_); }

  /* Reports if the boundaries have been computed. Note, by convention, an
   empty cache is considered uncomputed. */
  bool is_computed() const {
    return geometry_entries_.size() > 0 && is_computed_.value();
  }

  /* Maybe returns the MeshDistanceBoundary for `id`.
   If `id` has not been registered, a nullptr is always returned.
   If `id` *has* been registered, the boundary is returned (assuming it can be
   computed from the registered Mesh or Convex data).
   @throws if there is an error computing the boundary. */
  const MeshDistanceBoundary* GetBoundary(GeometryId id) const;

 private:
  friend class MeshSdfCacheTester;

  using MeshVariant = std::variant<Mesh, Convex>;
  using GeometryEntry = std::variant<MeshVariant, MeshDistanceBoundary>;

  void RegisterImpl(GeometryId id, MeshVariant mesh_variant);

  /* Computes and caches MeshDistanceBoundary for every registered geometry.
   After this call, assuming no errors reading the Mesh or Convex data, all
   entries will contain a MeshDistanceBoundary. Subsequent calls are no-ops.

   If the cache is empty, this call is likewise a no-op. */
  void ComputeAll() const;

  // Computes the boundary for a single entry, converting the entry from a
  // MeshVariant to a MeshDistanceBoundary
  void ComputeEntry(GeometryEntry* entry) const;

  mutable reset_after_move<bool> is_computed_;

  mutable std::unordered_map<GeometryId, GeometryEntry> geometry_entries_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
