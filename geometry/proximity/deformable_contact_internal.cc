#include "drake/geometry/proximity/deformable_contact_internal.h"

#include <algorithm>

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {

using std::move;

void Geometries::RemoveGeometry(GeometryId id) {
  deformable_geometries_.erase(id);
  rigid_geometries_.erase(id);
}

void Geometries::MaybeAddGeometry(const Shape& shape, GeometryId id,
                                  const ProximityProperties& properties) {
  // We borrow the hydro tag for now, but eventually deformable contact should
  // have its own properties.
  const HydroelasticType type = properties.GetPropertyOrDefault(
      kHydroGroup, kComplianceType, HydroelasticType::kUndefined);
  ReifyData data{true, id, properties};
  if (type == HydroelasticType::kRigid) {
    shape.Reify(this, &data);
  } else if (type == HydroelasticType::kSoft) {
    data.is_rigid = false;
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
    if (geometry) rigid_geometries_.insert({data.id, move(*geometry)});
  } else {
    auto geometry = MakeDeformableRepresentation(shape, data.properties);
    if (geometry) deformable_geometries_.insert({data.id, move(*geometry)});
  }
}

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
