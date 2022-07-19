#include "drake/geometry/proximity/deformable_contact_geometries.h"

#include <algorithm>

#include "drake/geometry/proximity/calc_distance_to_surface_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {
namespace {

using std::make_unique;
using std::move;
using std::vector;

/* Returns a linear field that approximates the signed distance field
 inside the given volume mesh. The signed distance value at each vertex is
 calculated with respect to the surface of the volume mesh and is exact (to
 the accuracy of the distance query algorithm), and the values in
 the interior of the mesh are linearly interpolated from vertex values. All
 values in the field are non-positive.
 @pre mesh != nullptr. */
std::unique_ptr<VolumeMeshFieldLinear<double, double>>
ApproximateSignedDistanceField(const VolumeMesh<double>* mesh) {
  DRAKE_DEMAND(mesh != nullptr);
  vector<double> signed_distance;
  signed_distance.reserve(mesh->num_vertices());
  const TriangleSurfaceMesh<double> surface_mesh =
      ConvertVolumeToSurfaceMesh(*mesh);
  for (const Vector3<double>& vertex : mesh->vertices()) {
    signed_distance.emplace_back(
        -CalcDistanceToSurfaceMesh(vertex, surface_mesh));
  }
  return make_unique<VolumeMeshFieldLinear<double, double>>(
      move(signed_distance), mesh);
}

}  // namespace

DeformableGeometry::DeformableGeometry(const DeformableGeometry& other) {
  *this = other;
}

DeformableGeometry& DeformableGeometry::operator=(
    const DeformableGeometry& other) {
  if (this == &other) return *this;

  deformable_mesh_ =
      std::make_unique<DeformableVolumeMesh<double>>(*other.deformable_mesh_);
  // We can't simply copy the field; the copy must contain a pointer to
  // the new mesh. So, we use CloneAndSetMesh() instead.
  signed_distance_field_ =
      other.signed_distance_field_->CloneAndSetMesh(&deformable_mesh_->mesh());
  return *this;
}

DeformableGeometry::DeformableGeometry(VolumeMesh<double> mesh)
    : deformable_mesh_(
          std::make_unique<DeformableVolumeMesh<double>>(std::move(mesh))),
      signed_distance_field_(
          ApproximateSignedDistanceField(&deformable_mesh_->mesh())) {}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const HalfSpace&, const ProximityProperties&) {
  throw std::logic_error(
      "Half spaces are not currently supported for deformable contact; "
      "registration is allowed, but an error will be thrown "
      "during contact.");
  return {};
}

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
