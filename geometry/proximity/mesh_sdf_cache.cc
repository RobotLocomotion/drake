#include "drake/geometry/proximity/mesh_sdf_cache.h"

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

MeshSdfCache::MeshSdfCache() = default;

MeshSdfCache::~MeshSdfCache() = default;

void MeshSdfCache::Register(GeometryId id, const Mesh& mesh) {
  // Extension check requires no disk I/O; unsupported types are silently
  // ignored here so that contains() correctly returns false for them.
  const std::string& ext = mesh.extension();
  if (ext != ".vtk" && ext != ".obj") return;

  // TODO(SeanCurtis-TRI) The lambda captures mesh *by value* here. For
  // in-memory meshes this can be expensive. Mesh and Convex should use a
  // std::shared_ptr<MeshSource> so the factory lambda is cheap to keep around.
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

void MeshSdfCache::ComputeAll() const {
  if (is_computed_) return;
  for (auto& [id, entry] : geometry_entries_) {
    ComputeEntry(&entry);
  }
  is_computed_ = true;
}

const MeshDistanceBoundary& MeshSdfCache::GetBoundary(GeometryId id) const {
  const GeometryEntry& entry = geometry_entries_.at(id);
  DRAKE_DEMAND(entry.boundary != nullptr);
  return *entry.boundary;
}

void MeshSdfCache::RegisterImpl(
    GeometryId id, const Key& key,
    std::function<TriangleSurfaceMesh<double>()> make_tri_mesh) {
  geometry_entries_.emplace(id, GeometryEntry(key, [make_tri_mesh]() {
                              return MeshDistanceBoundary(make_tri_mesh());
                            }));
  if (is_computed_) {
    auto& entry = geometry_entries_.at(id);
    ComputeEntry(&entry);
  }
}

void MeshSdfCache::ComputeEntry(GeometryEntry* entry) const {
  DRAKE_DEMAND(entry != nullptr);
  if (entry->boundary == nullptr) {
    entry->boundary = memoizer_->FindOrInsert(entry->key, entry->make_boundary);
    entry->make_boundary = nullptr;
  }
  DRAKE_DEMAND(entry->make_boundary == nullptr);
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
