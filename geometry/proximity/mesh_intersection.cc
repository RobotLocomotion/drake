#include "drake/geometry/proximity/mesh_intersection.h"

namespace drake {
namespace geometry {
namespace mesh_intersection {

std::unique_ptr<ContactSurface<AutoDiffXd>>
ComputeContactSurfaceFromSoftVolumeRigidSurface(
    const GeometryId, const VolumeMeshField<double, double>&,
    const math::RigidTransform<AutoDiffXd>&, const GeometryId,
    const SurfaceMesh<double>&, const math::RigidTransform<AutoDiffXd>&) {
  throw std::logic_error(
      "ContactSurface for AutoDiff-valued Compliant-Rigid meshes are not "
      "currently supported with AutoDiffXd");
}

}  // namespace mesh_intersection
}  // namespace geometry
}  // namespace drake
