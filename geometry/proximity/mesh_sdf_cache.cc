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

MeshSdfCache::CacheEntry::CacheEntry(
    std::function<TriangleSurfaceMesh<double>()> make_mesh)
    : make_mesh_(std::move(make_mesh)) {
  DRAKE_DEMAND(make_mesh_ != nullptr);
}

bool MeshSdfCache::CacheEntry::has_boundary() const {
  DRAKE_DEMAND(boundary_.has_value() != (make_mesh_ != nullptr));
  return boundary_.has_value();
}

const MeshDistanceBoundary& MeshSdfCache::CacheEntry::boundary() const {
  DRAKE_DEMAND(has_boundary());
  return *boundary_;
}

void MeshSdfCache::CacheEntry::ComputeBoundary() {
  DRAKE_DEMAND(!has_boundary());
  TriangleSurfaceMesh<double> scaled_mesh = make_mesh_();
  make_mesh_ = nullptr;
  boundary_.emplace(std::move(scaled_mesh));
}

MeshSdfCache::MeshSdfCache() : table_(std::make_shared<SharedTable>()) {}

MeshSdfCache::MeshSdfCache(const MeshSdfCache& other) : table_(other.table_) {
  // Hold the lock to block concurrent evictions from other instances while we
  // bump use counts by copying the shared_ptr entries.
  std::unique_lock lock(table_->mutex);
  geometry_entries_ = other.geometry_entries_;
}

MeshSdfCache& MeshSdfCache::operator=(const MeshSdfCache& other) {
  if (this == &other) return *this;

  Flush();
  // We should never overwrite a populated table; doing so would mess up
  // reference counts.
  DRAKE_DEMAND(table_->entries.empty());

  // Adopt other's shared table, then hold its lock across both the key copy
  // and the increment so that no concurrent Remove() on `other` can evict an
  // entry between the two steps.
  table_ = other.table_;

  // Hold the lock to block concurrent evictions from other instances while we
  // bump use counts by copying the shared_ptr entries.
  std::unique_lock lock(table_->mutex);
  geometry_entries_ = other.geometry_entries_;
  return *this;
}

MeshSdfCache::MeshSdfCache(MeshSdfCache&& other)
    : table_(std::move(other.table_)),
      geometry_entries_(std::move(other.geometry_entries_)) {
  // Leave the moved-from in a valid empty state so its destructor is safe.
  other.table_ = std::make_shared<SharedTable>();
}

MeshSdfCache& MeshSdfCache::operator=(MeshSdfCache&& other) {
  if (this == &other) return *this;
  // Flush our old content before taking other's.
  Flush();
  table_ = std::move(other.table_);
  geometry_entries_ = std::move(other.geometry_entries_);
  // Leave the moved-from in a valid empty state so its destructor is safe.
  other.table_ = std::make_shared<SharedTable>();
  return *this;
}

MeshSdfCache::~MeshSdfCache() {
  // When this *single* cache gets deleted, it must remove all of its references
  // into the shared table.
  Flush();
}

void MeshSdfCache::Register(GeometryId id, const Mesh& mesh) {
  // Extension check requires no disk I/O; unsupported types are silently
  // ignored here so that contains() correctly returns false for them.
  const std::string& ext = mesh.extension();
  if (ext != ".vtk" && ext != ".obj") return;

  const Vector3d scale = mesh.scale3();
  // The outer lambda captures mesh by reference and is called at most once,
  // synchronously inside RegisterImpl, only when a new table entry is needed.
  // The inner lambda captures by value (copying mesh); that copy is avoided
  // entirely when an existing entry is reused.
  RegisterImpl(id, mesh.source().GetCacheKey(/* is_convex= */ false),
               ScaleKey{scale[0], scale[1], scale[2]}, [&mesh]() {
                 return [mesh_copy = mesh]() -> TriangleSurfaceMesh<double> {
                   const Vector3d s = mesh_copy.scale3();
                   if (mesh_copy.extension() == ".vtk") {
                     return ConvertVolumeToSurfaceMesh(
                         ReadVtkToVolumeMesh(mesh_copy.source(), s));
                   }
                   return ReadObjToTriangleSurfaceMesh(mesh_copy.source(), s);
                 };
               });
}

void MeshSdfCache::Register(GeometryId id, const Convex& convex) {
  const Vector3d scale = convex.scale3();
  // The outer lambda captures convex by reference and is called at most once,
  // synchronously inside RegisterImpl, only when a new table entry is needed.
  // The inner lambda captures by value (copying convex); that copy is avoided
  // entirely when an existing entry is reused.
  RegisterImpl(
      id, convex.source().GetCacheKey(/* is_convex= */ true),
      ScaleKey{scale[0], scale[1], scale[2]}, [&convex]() {
        return [convex_copy = convex]() -> TriangleSurfaceMesh<double> {
          // GetConvexHull() bakes the convex's scale into the hull.
          return MakeTriangleFromPolygonMesh(convex_copy.GetConvexHull());
        };
      });
}

void MeshSdfCache::Remove(GeometryId id) {
  auto it = geometry_entries_.find(id);
  if (it == geometry_entries_.end()) return;

  const Key key = it->second.key;

  // We're about to erase `it`. That will invalidate the iterator and what it
  // references. But, we need access to the shared_ptr to look at its reference
  // count. So, we'll take a local copy of the entry_handle and include it in
  // our accounting below.
  std::shared_ptr<CacheEntry> entry_handle = it->second.entry_handle;
  geometry_entries_.erase(it);

  std::unique_lock lock(table_->mutex);
  // entry_handle.use_count() is now: local(1) + table(1) + other
  // registrations(N). Evict when this was the last registration (N == 0).
  if (entry_handle.use_count() == 2) {
    table_->entries.erase({key.mesh_key, key.scale_key});
  }
}

const MeshDistanceBoundary& MeshSdfCache::GetOrCompute(GeometryId id) const {
  const GeometryEntry& entry = geometry_entries_.at(id);
  const std::shared_ptr<CacheEntry>& entry_handle = entry.entry_handle;

  // Fast path: boundary already computed.
  {
    std::shared_lock read_lock(table_->mutex);
    if (entry_handle->has_boundary()) return entry_handle->boundary();
  }

  // Just in case another thread computed the boundary between the read-lock
  // check and this write_lock, we'll check again.
  std::unique_lock write_lock(table_->mutex);
  if (!entry_handle->has_boundary()) {
    entry_handle->ComputeBoundary();
  }
  return entry_handle->boundary();
}

void MeshSdfCache::RegisterImpl(
    GeometryId id, const std::string& mesh_key, const ScaleKey& scale_key,
    std::function<std::function<TriangleSurfaceMesh<double>()>()>
        make_factory) {
  std::unique_lock lock(table_->mutex);
  const auto composite_key = std::make_pair(mesh_key, scale_key);
  if (!table_->entries.contains(composite_key)) {
    table_->entries.emplace(composite_key,
                            std::make_shared<CacheEntry>(make_factory()));
  }
  geometry_entries_[id] = GeometryEntry{Key{mesh_key, scale_key},
                                        table_->entries.at(composite_key)};
}

void MeshSdfCache::Flush() {
  if (geometry_entries_.empty()) return;

  // Collect one entry_handle per unique (mesh_key, scale_key). Multiple
  // geometry IDs may share the same entry_handle (via a common (mesh_key,
  // scale_key)); try_emplace deduplicates.
  std::map<std::pair<std::string, ScaleKey>, std::shared_ptr<CacheEntry>>
      unique_entries;
  for (const auto& [id, ge] : geometry_entries_) {
    unique_entries.try_emplace({ge.key.mesh_key, ge.key.scale_key},
                               ge.entry_handle);
  }

  geometry_entries_.clear();  // Release all per-geometry shared_ptr references.

  std::unique_lock lock(table_->mutex);
  for (const auto& [composite_key, entry_handle] : unique_entries) {
    // use_count is now: unique_entries(1) + table(1) + other instances(N).
    // Evict when N == 0.
    if (entry_handle.use_count() == 2) {
      table_->entries.erase(composite_key);
    }
  }
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
