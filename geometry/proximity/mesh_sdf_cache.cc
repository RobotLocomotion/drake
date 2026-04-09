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
  DRAKE_DEMAND(table_->file_entries.empty());

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
  const Vector3d scale = mesh.scale3();
  RegisterImpl(id, mesh.source().GetCacheKey(/* is_convex= */ false),
               ScaleKey{scale[0], scale[1], scale[2]},
               [&mesh]() -> std::optional<TriangleSurfaceMesh<double>> {
                 if (mesh.extension() == ".vtk") {
                   return ConvertVolumeToSurfaceMesh(
                       ReadVtkToVolumeMesh(mesh.source(), Vector3d::Ones()));
                 } else if (mesh.extension() == ".obj") {
                   return ReadObjToTriangleSurfaceMesh(mesh.source(),
                                                       Vector3d::Ones());
                 }
                 // We don't yet have general proximity support for other mesh
                 // types (e.g., glTF).
                 return std::nullopt;
               });
}

void MeshSdfCache::Register(GeometryId id, const Convex& convex) {
  const Vector3d scale = convex.scale3();
  RegisterImpl(
      id, convex.source().GetCacheKey(/* is_convex= */ true),
      ScaleKey{scale[0], scale[1], scale[2]},
      [&convex, &scale]() -> std::optional<TriangleSurfaceMesh<double>> {
        TriangleSurfaceMesh<double> scaled_tri =
            MakeTriangleFromPolygonMesh(convex.GetConvexHull());
        // Note: Convex::GetConvexHull() bakes the convex's scale into the hull.
        // We're getting the unit scale by removing the scale factor. If the
        // scale is all ones, it literally makes no difference. If the *first*
        // reference to the mesh source has non-unit scale, then there could
        // be an unimportant, tiny numerical difference in the unit mesh stored
        // (compared to that computed directly from the original mesh). But as
        // most mesh specifications only provide (at most) 32-bit float
        // precision, and we store in double, we're not going to worry about
        // rounding at the limits of double precision.
        const Vector3d inv_scale(1.0 / scale[0], 1.0 / scale[1],
                                 1.0 / scale[2]);
        return scaled_tri.CreateScaledMesh(inv_scale);
      });
}

void MeshSdfCache::Remove(GeometryId id) {
  auto it = geometry_entries_.find(id);
  if (it == geometry_entries_.end()) return;

  const Key key = it->second.key;

  // We're about to erase `it`. That will invalidate the iterator and what it
  // references. But, we need access to the shared_ptr to look at its reference
  // count. So, we'll take a local copy of the boundary_handle and include it in
  // our accounting below.
  std::shared_ptr<std::optional<MeshDistanceBoundary>> boundary_handle =
      it->second.boundary_handle;
  geometry_entries_.erase(it);

  std::unique_lock lock(table_->mutex);
  // boundary_handle.use_count() is now: local(1) + table(1) + other
  // registrations(N). Evict when this was the last registration (N == 0).
  if (boundary_handle.use_count() == 2) {
    auto file_it = table_->file_entries.find(key.file_key);
    if (file_it != table_->file_entries.end()) {
      file_it->second.scales.erase(key.scale_key);
      if (file_it->second.scales.empty()) {
        table_->file_entries.erase(file_it);
      }
    }
  }
}

const MeshDistanceBoundary& MeshSdfCache::GetOrCompute(GeometryId id) const {
  const GeometryEntry& entry = geometry_entries_.at(id);
  const std::shared_ptr<std::optional<MeshDistanceBoundary>>& boundary_handle =
      entry.boundary_handle;

  // Fast path: boundary already computed.
  {
    std::shared_lock read_lock(table_->mutex);
    if (boundary_handle->has_value()) return **boundary_handle;
  }

  // Just in case another thread computed the boundary between simple readlock
  // check and this write_lock, we'll check again. Note: we're checking a
  // *reference* to the GeometryEntry. That is safe because it's stored in
  // a std::map which doesn't invalidate references to entries on mutation.
  std::unique_lock write_lock(table_->mutex);
  if (boundary_handle->has_value()) return **boundary_handle;

  const Key& key = entry.key;
  // Note: this is safe, because the declaration of an entry for a geometry id
  // places the file entry for that file key.
  const FileEntry& file_entry = table_->file_entries.at(key.file_key);
  const ScaleKey& sk = key.scale_key;
  // Note: entry.boundary_handle is already a copy of the shared handle stored
  // in the file_entry.scales. So, overwriting the boundary_handle value in the
  // geometry's boundary_handle is visible through the file_entry as well.
  TriangleSurfaceMesh<double> scaled_mesh =
      file_entry.unit_mesh.CreateScaledMesh(Vector3d(sk[0], sk[1], sk[2]));
  boundary_handle->emplace(std::move(scaled_mesh));
  return **boundary_handle;
}

void MeshSdfCache::RegisterImpl(
    GeometryId id, const std::string& file_key, const ScaleKey& scale_key,
    std::function<std::optional<TriangleSurfaceMesh<double>>()>
        make_unit_mesh) {
  std::unique_lock lock(table_->mutex);
  if (!table_->file_entries.contains(file_key)) {
    auto unit_mesh = make_unit_mesh();
    if (!unit_mesh.has_value()) return;
    table_->file_entries.emplace(file_key,
                                 FileEntry{std::move(*unit_mesh), {}});
  }
  auto& scales = table_->file_entries.at(file_key).scales;
  if (!scales.contains(scale_key)) {
    scales.emplace(scale_key,
                   std::make_shared<std::optional<MeshDistanceBoundary>>());
  }
  geometry_entries_[id] =
      GeometryEntry{Key{file_key, scale_key}, scales.at(scale_key)};
}

void MeshSdfCache::Flush() {
  if (geometry_entries_.empty()) return;

  // Collect one boundary_handle per unique (file_key, scale_key). Multiple
  // geometry IDs may share the same boundary_handle (via a common (file_key,
  // scale_key)); try_emplace deduplicates.
  std::map<std::pair<std::string, ScaleKey>,
           std::shared_ptr<std::optional<MeshDistanceBoundary>>>
      unique_slots;
  for (const auto& [id, ge] : geometry_entries_) {
    unique_slots.try_emplace({ge.key.file_key, ge.key.scale_key},
                             ge.boundary_handle);
  }

  geometry_entries_.clear();  // Release all per-geometry shared_ptr references.

  std::unique_lock lock(table_->mutex);
  for (const auto& [key_pair, boundary_handle] : unique_slots) {
    // use_count is now: unique_slots(1) + table(1) + other instances(N).
    // Evict when N == 0.
    if (boundary_handle.use_count() == 2) {
      auto file_it = table_->file_entries.find(key_pair.first);
      if (file_it != table_->file_entries.end()) {
        file_it->second.scales.erase(key_pair.second);
        if (file_it->second.scales.empty()) {
          table_->file_entries.erase(file_it);
        }
      }
    }
  }
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
