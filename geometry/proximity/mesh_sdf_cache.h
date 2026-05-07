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
 be computed; it is only available upon calling ComputeAll() (at which time
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

  /* Returns true if geometry `id` has been registered. */
  bool contains(GeometryId id) const { return geometry_entries_.contains(id); }

  /* Registers a Mesh geometry. The boundary will not be computed until
   ComputeAll() is called. If this cache already contains boundary data, then
   the boundary data for this mesh will be computed immediately. Files with .vtk
   and .obj extensions are supported. Unsupported mesh types are silently
   ignored; such geometry will not be queryable via GetBoundary().
   @throws if `contains(id)`. */
  void Register(GeometryId id, const Mesh& mesh);

  /* Registers a Convex geometry. The boundary will not be computed until
   ComputeAll() is called. If this cache already contains boundary data, then
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
    return geometry_entries_.size() > 0 &&
           std::holds_alternative<MeshDistanceBoundary>(
               geometry_entries_.begin()->second);
  }

  /* Computes and caches MeshDistanceBoundary for every registered geometry.
   After this call, GetBoundary() is valid for all registered geometry ids.
   Subsequent calls are no-ops.

   If the cache is empty, this call is likewise a no-op. */
  void ComputeAll();

  /* Returns the MeshDistanceBoundary for `id`.
   The reference is valid as long as the cache maintains a reference to the
   entry; calls to Remove(id) or cache destruction invalidate it.
   @pre contains(id) is true.
   @pre ComputeAll() has been called. */
  const MeshDistanceBoundary& GetBoundary(GeometryId id) const;

 private:
  using MeshVariant = std::variant<Mesh, Convex>;
  using GeometryEntry = std::variant<MeshVariant, MeshDistanceBoundary>;

  std::unordered_map<GeometryId, GeometryEntry> geometry_entries_;

  /* Store the key and mesh factor in *this* instance's geometry table.*/
  void RegisterImpl(GeometryId id, MeshVariant mesh_variant);

  // Computes the boundary for a single entry. This is uniquely responsible for
  // confirming the factory method is null when the boundary is populated.
  void ComputeEntry(GeometryEntry* entry);
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
