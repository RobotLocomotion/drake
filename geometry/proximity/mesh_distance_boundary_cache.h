#pragma once

#include <unordered_map>
#include <variant>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/mesh_distance_boundary.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* A repository for declared mesh-like Shapes that require MeshDistanceBoundary
 instances for distance-to-point queries.

 The purpose of the cache is to defer computation of the MeshDistanceBoundary
 until it's actually needed (signaled by a call to ComputeAll()). Prior to that,
 the cache is "lazy", only *remembering* the registrations. After the call to
 ComputeAll(), the cache is "live" and every registration will lead to immediate
 computation of the boundary data for the registered geometry.

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
class MeshDistanceBoundaryCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshDistanceBoundaryCache);

  MeshDistanceBoundaryCache();
  ~MeshDistanceBoundaryCache();

  /* Registers a Mesh geometry. Whether the boundary data gets computed depends
   on whether this cache is "live" or "lazy". Files with .vtk and .obj
   extensions are supported. Unsupported mesh types are silently ignored.
   @throws if `id` has already been registered. */
  void Register(GeometryId id, const Mesh& mesh);

  /* Registers a Convex geometry. Whether the boundary data gets computed
   depends on whether this cache is "live" or "lazy". Files with .vtk, .obj, and
   .gltf extensions are supported. Unsupported mesh types are silently ignored.
   @throws if `id` has already been registered. */
  void Register(GeometryId id, const Convex& convex);

  /* Removes a geometry. */
  void Remove(GeometryId id);

  /* Reports the number of registered geometries. */
  int num_geometries() const { return std::ssize(geometry_entries_); }

  /* Reports if the cache is "live" (registered geometries have boundary data
   and future registrations will immediately compute boundary data). */
  bool is_live() const { return is_live_; }

  /* Computes the MeshDistanceBoundary data for every registered geometry and
   sets the cache as "live". Invocation on a "live" cache is a no-op. */
  void ComputeAll() const;

  /* Maybe returns the MeshDistanceBoundary for `id`. The boundary will only
   be non-null if `id` has been registered and the cache is "live". */
  const MeshDistanceBoundary* GetBoundary(GeometryId id) const;

 private:
  using MeshVariant = std::variant<Mesh, Convex>;
  using GeometryEntry = std::variant<MeshVariant, MeshDistanceBoundary>;

  void RegisterImpl(GeometryId id, MeshVariant mesh_variant);

  /* Computes the boundary for a single entry, converting the entry from a
   MeshVariant to a MeshDistanceBoundary. */
  void ComputeEntry(GeometryEntry* entry) const;

  mutable bool is_live_{};
  mutable std::unordered_map<GeometryId, GeometryEntry> geometry_entries_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
