#include "drake/geometry/proximity/deformable_contact_internal.h"

#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/tessellation_strategy.h"
#include "drake/multibody/fem/mesh_utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {

using std::make_unique;
using std::move;

DeformableGeometry& DeformableGeometry::operator=(const DeformableGeometry& s) {
  if (this == &s) return *this;
  mesh_ = std::make_unique<internal::DeformableVolumeMesh<double>>(
      s.deformable_volume_mesh());
  reference_mesh_ = std::make_unique<VolumeMesh<double>>(s.reference_mesh());
  /* We can't simply copy the mesh field; the copy must contain a pointer to
   the new reference mesh. So, we use CloneAndSetMesh() instead. */
  signed_distance_ = s.signed_distance().CloneAndSetMesh(reference_mesh_.get());
  return *this;
}

DeformableGeometry::DeformableGeometry(const DeformableGeometry& s) {
  *this = s;
}

void Geometries::RemoveGeometry(GeometryId id) {
  deformable_geometries_.erase(id);
  rigid_geometries_.erase(id);
}

void Geometries::MaybeAddGeometry(const Shape& shape, GeometryId id,
                                  const ProximityProperties& properties) {
  const HydroelasticType type = properties.GetPropertyOrDefault(
      kHydroGroup, kComplianceType, HydroelasticType::kUndefined);
  if (type == HydroelasticType::kRigid) {
    ReifyData data{true, id, properties};
    shape.Reify(this, &data);
  }
}

void Geometries::ImplementGeometry(const Sphere& sphere, void* user_data) {
  MakeShape(sphere, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Cylinder& cylinder, void* user_data) {
  MakeShape(cylinder, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const HalfSpace& half_space,
                                   void* user_data) {
  MakeShape(half_space, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Box& box, void* user_data) {
  MakeShape(box, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Capsule& capsule, void* user_data) {
  MakeShape(capsule, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Ellipsoid& ellipsoid,
                                   void* user_data) {
  MakeShape(ellipsoid, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Mesh& mesh, void* user_data) {
  MakeShape(mesh, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Convex& convex, void* user_data) {
  MakeShape(convex, *static_cast<ReifyData*>(user_data));
}

template <typename ShapeType>
void Geometries::MakeShape(const ShapeType& shape, const ReifyData& data) {
  if (data.is_rigid) {
    auto geometry = MakeRigidRepresentation(shape, data.properties);
    if (geometry) rigid_geometries_.insert({data.id, *geometry});
  } else {
    auto geometry = MakeDeformableRepresentation(shape, data.properties);
    if (geometry) deformable_geometries_.insert({data.id, *geometry});
  }
}

std::optional<DeformableGeometry> MakeDeformableRepresentation(
    const Sphere& sphere, const ProximityProperties& props) {
  const double resolution_hint = props.GetProperty<double>(
      geometry::internal::kHydroGroup, geometry::internal::kRezHint);
  const TessellationStrategy strategy =
      TessellationStrategy::kDenseInteriorVertices;
  auto deformable_volume_mesh = std::make_unique<DeformableVolumeMesh<double>>(
      MakeSphereVolumeMesh<double>(sphere, resolution_hint, strategy));
  // TODO (xuchenhan-tri): Make a mesh field here instead of passing in the
  // nullptr.
  return DeformableGeometry(std::move(deformable_volume_mesh), nullptr);
}

std::optional<DeformableGeometry> MakeDeformableRepresentation(
    const Box& box, const ProximityProperties& props) {
  const double resolution_hint = props.GetProperty<double>(
      geometry::internal::kHydroGroup, geometry::internal::kRezHint);
  auto deformable_volume_mesh = std::make_unique<DeformableVolumeMesh<double>>(
      multibody::fem::MakeDiamondCubicBoxVolumeMesh<double>(box,
                                                            resolution_hint));
  // TODO (xuchenhan-tri): Make a mesh field here instead of passing in the
  // nullptr.
  return DeformableGeometry(std::move(deformable_volume_mesh), nullptr);
}

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
