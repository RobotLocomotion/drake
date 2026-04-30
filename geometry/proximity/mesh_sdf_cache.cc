#include "drake/geometry/proximity/mesh_sdf_cache.h"

#include <mutex>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

MeshSdfCache::GeometryEntry::GeometryEntry(
    const MeshSdfCache::GeometryEntry& other)
    : boundary(other.boundary.load()),
      key(other.key),
      make_boundary(other.make_boundary) {}

MeshSdfCache::GeometryEntry& MeshSdfCache::GeometryEntry::operator=(
    const MeshSdfCache::GeometryEntry& other) {
  if (this != &other) {
    boundary = other.boundary.load();
    key = other.key;
    make_boundary = other.make_boundary;
  }
  return *this;
}

MeshSdfCache::GeometryEntry::~GeometryEntry() = default;

MeshSdfCache::MeshSdfCache() = default;

MeshSdfCache::~MeshSdfCache() = default;

void MeshSdfCache::Register(GeometryId id, const Mesh& mesh) {
  // Extension check requires no disk I/O; unsupported types are silently
  // ignored here so that contains() correctly returns false for them.
  const std::string& ext = mesh.extension();
  if (ext != ".vtk" && ext != ".obj") return;

  // TODO(Seancurtis-TRI) The lambda captures mesh *by value* here. For
  // in-memory-mesh, this can be very expensive. We need to make this *fast*
  // and *cheap* (in a memory sense). Simplest is to change Mesh and Convex
  // to use a std::shared_ptr<MeshSource> instead of simply a MeshSource. This
  // also reduces the cost of keeping these mesh factories around.
  RegisterImpl(
      id, Key{mesh.source().GetCacheKey(/* is_convex= */ false), mesh.scale3()},
      [mesh_copy = mesh]() -> TriangleSurfaceMesh<double> {
        const Vector3d s = mesh_copy.scale3();
        if (mesh_copy.extension() == ".vtk") {
          return ConvertVolumeToSurfaceMesh(
              ReadVtkToVolumeMesh(mesh_copy.source(), s));
        }
        return ReadObjToTriangleSurfaceMesh(mesh_copy.source(), s);
      });
}

void MeshSdfCache::Register(GeometryId id, const Convex& convex) {
  RegisterImpl(
      id,
      Key{convex.source().GetCacheKey(/* is_convex= */ true), convex.scale3()},
      [convex_copy = convex]() -> TriangleSurfaceMesh<double> {
        // GetConvexHull() bakes the convex's scale into the hull.
        return MakeTriangleFromPolygonMesh(convex_copy.GetConvexHull());
      });
}

void MeshSdfCache::Remove(GeometryId id) {
  auto iter = geometry_entries_.find(id);
  if (iter != geometry_entries_.end()) {
    geometry_entries_.erase(iter);
  }
}

const MeshDistanceBoundary& MeshSdfCache::GetOrCompute(GeometryId id) const {
  const GeometryEntry& entry = geometry_entries_.at(id);

  // Check if the boundary already exists.
  std::shared_ptr<const MeshDistanceBoundary> existing = entry.boundary.load();
  if (existing != nullptr) {
    // Even though we return a reference to what `existing` points to, it's okay
    // for `existing` to go out of scope since `entry.boundary` will continue to
    // maintain a strong reference indefinitely.
    return *existing;
  }

  // Try to find an already-computed boundary, or else make one and memoize it.
  std::shared_ptr<const MeshDistanceBoundary> memoized =
      memoizer_->FindOrInsert(entry.key, entry.make_boundary);
  const MeshDistanceBoundary* memoized_raw = memoized.get();
  DRAKE_ASSERT(memoized_raw != nullptr);

  // NOTE: once we've used entry.make_boundary to compute the boundary, we no
  // longer need it. Clearing it would seem a good idea, but would require
  // locking to do so safely. As long as make_boundary has a small memory
  // footprint, it's probably harmless enough to just let it be.

  // Update our GeometryEntry to point to the memoized boundary and return it.
  const bool stored =
      entry.boundary.compare_exchange_strong(existing, std::move(memoized));
  if (stored) {
    return *memoized_raw;
  } else {
    // The entry.boundary was already set by some other thread, and `existing`
    // now refers to that value.
    DRAKE_DEMAND(existing != nullptr);
    return *existing;
  }
}

void MeshSdfCache::RegisterImpl(
    GeometryId id, const Key& key,
    std::function<TriangleSurfaceMesh<double>()> make_tri_mesh) {
  geometry_entries_.emplace(id, GeometryEntry(key, [make_tri_mesh]() {
                              return MeshDistanceBoundary(make_tri_mesh());
                            }));
}

MeshSdfCache::ResettingMemoizer::ResettingMemoizer()
    : memoizer_(std::make_shared<const MyMemoizer>()) {}

MeshSdfCache::ResettingMemoizer::ResettingMemoizer(
    const ResettingMemoizer& other) = default;

MeshSdfCache::ResettingMemoizer& MeshSdfCache::ResettingMemoizer::operator=(
    const ResettingMemoizer& other) = default;

MeshSdfCache::ResettingMemoizer::ResettingMemoizer(
    ResettingMemoizer&& other) noexcept
    : memoizer_(std::move(other.memoizer_)) {
  other.memoizer_ = std::make_shared<const MyMemoizer>();
}

MeshSdfCache::ResettingMemoizer& MeshSdfCache::ResettingMemoizer::operator=(
    ResettingMemoizer&& other) noexcept {
  memoizer_ = std::move(other.memoizer_);
  other.memoizer_ = std::make_shared<const MyMemoizer>();
  return *this;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
