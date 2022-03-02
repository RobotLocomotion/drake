#include "drake/geometry/proximity/deformable_contact_internal.h"

#include <algorithm>

#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/tessellation_strategy.h"
#include "drake/multibody/fem/mesh_utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {

using std::make_unique;
using std::move;

ReferenceDeformableGeometry::ReferenceDeformableGeometry(
    const Shape& shape, geometry::VolumeMesh<double> mesh)
    : mesh_(std::move(mesh)) {
  std::vector<double> signed_distance = ComputeSignedDistanceOfVertices(shape);
  signed_distance_field_ =
      std::make_unique<geometry::VolumeMeshFieldLinear<double, double>>(
          std::move(signed_distance), &mesh_);
}

ReferenceDeformableGeometry::ReferenceDeformableGeometry(
    const ReferenceDeformableGeometry& s)
    : mesh_(s.mesh_) {
  *this = s;
}

ReferenceDeformableGeometry& ReferenceDeformableGeometry::operator=(
    const ReferenceDeformableGeometry& s) {
  if (this == &s) return *this;

  mesh_ = s.mesh_;
  /* We can't simply copy the mesh field; the copy must contain a pointer to
   the new mesh. So, we use CloneAndSetMesh() instead. */
  signed_distance_field_ = s.signed_distance_field().CloneAndSetMesh(&mesh_);

  return *this;
}

std::vector<double>
ReferenceDeformableGeometry::ComputeSignedDistanceOfVertices(
    const Shape& shape) {
  ReifyData data;
  shape.Reify(this, &data);
  return data.signed_distance;
}

void ReferenceDeformableGeometry::ImplementGeometry(const Sphere& sphere,
                                                    void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  std::vector<double>& signed_distance = data.signed_distance;
  signed_distance.resize(mesh_.num_vertices());
  const double r = sphere.radius();
  for (int v = 0; v < mesh_.num_vertices(); ++v) {
    const Vector3<double>& q_MV = mesh_.vertex(v);
    signed_distance[v] = r - q_MV.norm();
  }
}

void ReferenceDeformableGeometry::ImplementGeometry(const Box& box,
                                                    void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  std::vector<double>& signed_distance = data.signed_distance;
  signed_distance.resize(mesh_.num_vertices());
  for (int v = 0; v < mesh_.num_vertices(); ++v) {
    // Given a point inside a box, find its distance to the surface of the
    // box.
    const Vector3<double>& q_MV = mesh_.vertex(v);
    const double q_MV_x = q_MV(0);
    const double q_MV_y = q_MV(1);
    const double q_MV_z = q_MV(2);
    const double x_distance = box.width() / 2.0 - std::abs(q_MV_x);
    const double y_distance = box.depth() / 2.0 - std::abs(q_MV_y);
    const double z_distance = box.height() / 2.0 - std::abs(q_MV_z);
    DRAKE_DEMAND(x_distance > 0.0);
    DRAKE_DEMAND(y_distance > 0.0);
    DRAKE_DEMAND(z_distance > 0.0);
    signed_distance[v] = std::min({x_distance, y_distance, z_distance});
  }
}

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
    if (geometry) rigid_geometries_.insert({data.id, std::move(*geometry)});
  } else {
    auto geometry = MakeDeformableRepresentation(shape, data.properties);
    if (geometry)
      deformable_geometries_.insert({data.id, std::move(*geometry)});
  }
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const HalfSpace&, const ProximityProperties&) {
  static const logging::Warn log_once(
      "Half spaces are not currently supported for deformable contact; "
      "registration is allowed, but an error will be thrown "
      "during contact.");
  return {};
}

std::optional<DeformableGeometry> MakeDeformableRepresentation(
    const Sphere& sphere, const ProximityProperties& props) {
  const double resolution_hint = props.GetProperty<double>(
      geometry::internal::kHydroGroup, geometry::internal::kRezHint);
  const TessellationStrategy strategy =
      TessellationStrategy::kDenseInteriorVertices;
  VolumeMesh<double> volume_mesh =
      MakeSphereVolumeMesh<double>(sphere, resolution_hint, strategy);
  return DeformableGeometry(sphere, std::move(volume_mesh));
}

std::optional<DeformableGeometry> MakeDeformableRepresentation(
    const Box& box, const ProximityProperties& props) {
  const double resolution_hint = props.GetProperty<double>(
      geometry::internal::kHydroGroup, geometry::internal::kRezHint);
  VolumeMesh<double> volume_mesh =
      multibody::fem::MakeDiamondCubicBoxVolumeMesh<double>(box,
                                                            resolution_hint);
  return DeformableGeometry(box, std::move(volume_mesh));
}

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
