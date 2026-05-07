#include "drake/geometry/proximity/mesh_sdf_cache.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
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

MeshSdfCache::MeshSdfCache() = default;

MeshSdfCache::~MeshSdfCache() = default;

void MeshSdfCache::Register(GeometryId id, const Mesh& mesh) {
  // Extension check requires no disk I/O; unsupported types are silently
  // ignored here so that contains() correctly returns false for them.
  const std::string& ext = mesh.extension();
  if (ext != ".vtk" && ext != ".obj") return;

  RegisterImpl(id, mesh);
}

void MeshSdfCache::Register(GeometryId id, const Convex& convex) {
  const std::string& ext = convex.extension();
  if (ext != ".vtk" && ext != ".obj" && ext != ".gltf") return;

  RegisterImpl(id, convex);
}

void MeshSdfCache::Remove(GeometryId id) {
  geometry_entries_.erase(id);
}

void MeshSdfCache::ComputeAll() {
  if (is_computed()) {
    return;
  }
  for (auto& [id, entry] : geometry_entries_) {
    ComputeEntry(&entry);
  }
}

const MeshDistanceBoundary& MeshSdfCache::GetBoundary(GeometryId id) const {
  const GeometryEntry& entry = geometry_entries_.at(id);
  if (!std::holds_alternative<MeshDistanceBoundary>(entry)) {
    throw std::logic_error(
        fmt::format("MeshSdfCache does not contain boundary data for geometry "
                    "with id {}.",
                    id));
  }
  return std::get<MeshDistanceBoundary>(entry);
}

void MeshSdfCache::RegisterImpl(GeometryId id, MeshVariant mesh_variant) {
  // Because is_computed() is framed in terms of the contents of the map, and
  // emplacing a new entry may change the *first* entry in the map, we need to
  // make sure we check the state *before* we potentially change the sample
  // entry.
  const bool already_computed = is_computed();
  if (contains(id)) {
    throw std::logic_error(
        fmt::format("MeshSdfCache already contains geometry with id {}.", id));
  }
  geometry_entries_.emplace(id, std::move(mesh_variant));
  if (already_computed) {
    auto& entry = geometry_entries_.at(id);
    ComputeEntry(&entry);
  }
}

void MeshSdfCache::ComputeEntry(GeometryEntry* entry) {
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
