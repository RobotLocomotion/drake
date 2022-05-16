#include "drake/geometry/proximity/deformable_contact_geometries.h"

#include <algorithm>

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {

using std::make_unique;
using std::move;
using std::vector;

ReferenceDeformableGeometry::ReferenceDeformableGeometry(
    const Shape& shape, VolumeMesh<double> mesh)
    : mesh_(std::make_unique<VolumeMesh<double>>(move(mesh))) {
  vector<double> signed_distance = ComputeSignedDistanceOfVertices(shape);
  signed_distance_field_ = make_unique<VolumeMeshFieldLinear<double, double>>(
      move(signed_distance), mesh_.get());
}

ReferenceDeformableGeometry::ReferenceDeformableGeometry(
    const ReferenceDeformableGeometry& other) {
  *this = other;
}

ReferenceDeformableGeometry& ReferenceDeformableGeometry::operator=(
    const ReferenceDeformableGeometry& other) {
  if (this == &other) return *this;

  mesh_ = make_unique<VolumeMesh<double>>(*(other.mesh_));
  /* We can't simply copy the mesh field; the copy must contain a pointer to
   the new mesh. So, we use CloneAndSetMesh() instead. */
  signed_distance_field_ =
      other.signed_distance_field().CloneAndSetMesh(mesh_.get());

  return *this;
}

vector<double> ReferenceDeformableGeometry::ComputeSignedDistanceOfVertices(
    const Shape& shape) {
  ReifyData data;
  shape.Reify(this, &data);
  return data.signed_distance;
}

void ReferenceDeformableGeometry::ImplementGeometry(const Sphere& sphere,
                                                    void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  vector<double>& signed_distance = data.signed_distance;
  signed_distance.resize(mesh_->num_vertices());
  const double r = sphere.radius();
  for (int v = 0; v < mesh_->num_vertices(); ++v) {
    const Vector3<double>& q_MV = mesh_->vertex(v);
    signed_distance[v] = q_MV.norm() - r;
  }
}

void ReferenceDeformableGeometry::ImplementGeometry(const Box& box,
                                                    void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  vector<double>& signed_distance = data.signed_distance;
  signed_distance.resize(mesh_->num_vertices());
  for (int v = 0; v < mesh_->num_vertices(); ++v) {
    /* Given a point inside a box, find its distance to the surface of the
     box. */
    // TODO(xuchenhan-tri): This assumes the mesh is contained in the box shape.
    // We should switch to an implementation that doesn't rely on this
    // assumption.
    const Vector3<double>& q_MV = mesh_->vertex(v);
    const double q_MV_x = q_MV(0);
    const double q_MV_y = q_MV(1);
    const double q_MV_z = q_MV(2);
    const double x_distance = std::abs(q_MV_x) - box.width() / 2.0;
    const double y_distance = std::abs(q_MV_y) - box.depth() / 2.0;
    const double z_distance = std::abs(q_MV_z) - box.height() / 2.0;
    DRAKE_DEMAND(x_distance <= 0.0);
    DRAKE_DEMAND(y_distance <= 0.0);
    DRAKE_DEMAND(z_distance <= 0.0);
    signed_distance[v] = std::max({x_distance, y_distance, z_distance});
  }
}

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
