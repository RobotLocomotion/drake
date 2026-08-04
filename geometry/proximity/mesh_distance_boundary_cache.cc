#include "drake/geometry/proximity/mesh_distance_boundary_cache.h"

#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt.h"
#include "drake/common/overloaded.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

MeshDistanceBoundaryCache::MeshDistanceBoundaryCache() = default;

MeshDistanceBoundaryCache::~MeshDistanceBoundaryCache() = default;

void MeshDistanceBoundaryCache::Register(GeometryId id, const Mesh& mesh) {
  const std::string& ext = mesh.extension();
  if (ext != ".vtk" && ext != ".obj") return;

  RegisterImpl(id, mesh);
}

void MeshDistanceBoundaryCache::Register(GeometryId id, const Convex& convex) {
  const std::string& ext = convex.extension();
  if (ext != ".vtk" && ext != ".obj" && ext != ".gltf") return;

  RegisterImpl(id, convex);
}

void MeshDistanceBoundaryCache::Remove(GeometryId id) {
  geometry_entries_.erase(id);
}

void MeshDistanceBoundaryCache::ComputeAll() const {
  if (is_live()) {
    return;
  }
  is_live_ = true;
  for (auto& [id, entry] : geometry_entries_) {
    ComputeEntry(&entry);
  }
}

const MeshDistanceBoundary* MeshDistanceBoundaryCache::GetBoundary(
    GeometryId id) const {
  auto iter = geometry_entries_.find(id);
  if (iter == geometry_entries_.end()) {
    return nullptr;
  }
  const GeometryEntry& entry = iter->second;
  if (!std::holds_alternative<MeshDistanceBoundary>(entry)) {
    return nullptr;
  }
  return &std::get<MeshDistanceBoundary>(entry);
}

void MeshDistanceBoundaryCache::RegisterImpl(GeometryId id,
                                             MeshVariant mesh_variant) {
  auto iter = geometry_entries_.find(id);
  if (iter != geometry_entries_.end()) {
    throw std::logic_error(fmt::format(
        "MeshDistanceBoundaryCache already contains geometry with id {}.", id));
  }
  geometry_entries_.emplace(id, std::move(mesh_variant));
  if (is_live()) {
    auto& entry = geometry_entries_.at(id);
    ComputeEntry(&entry);
  }
}

void MeshDistanceBoundaryCache::ComputeEntry(GeometryEntry* entry) const {
  DRAKE_DEMAND(entry != nullptr);
  DRAKE_ASSERT(!std::holds_alternative<MeshDistanceBoundary>(*entry));

  *entry = MeshDistanceBoundary(std::visit<TriangleSurfaceMesh<double>>(
      overloaded{[](const Mesh& mesh) {
                   const Vector3d& s = mesh.scale3();
                   if (mesh.extension() == ".vtk") {
                     return ConvertVolumeToSurfaceMesh(
                         ReadVtkToVolumeMesh(mesh.source(), s));
                   }
                   return ReadObjToTriangleSurfaceMesh(mesh.source(), s);
                 },
                 [](const Convex& convex) {
                   return MakeTriangleFromPolygonMesh(convex.GetConvexHull());
                 }},
      std::get<MeshVariant>(*entry)));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
