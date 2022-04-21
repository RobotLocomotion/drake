#include "drake/geometry/make_mesh_for_deformable.h"

#include "drake/common/drake_assert.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

VolumeMesh<double> MakeMeshForDeformable(const Sphere& sphere,
                                         double resolution_hint) {
  DRAKE_DEMAND(resolution_hint > 0);
  return MakeSphereVolumeMesh<double>(
      sphere, resolution_hint, TessellationStrategy::kDenseInteriorVertices);
}

// TODO(xuchenhan-tri): Implement these shapes.
VolumeMesh<double> MakeMeshForDeformable(const Cylinder&, double) {
  throw std::logic_error(
      "Cylinder shape is not supported in MakeMeshForDeformable().");
}
VolumeMesh<double> MakeMeshForDeformable(const HalfSpace&, double) {
  throw std::logic_error(
      "Half space shape is not supported in MakeMeshForDeformable().");
}
VolumeMesh<double> MakeMeshForDeformable(const Box&, double) {
  throw std::logic_error(
      "Box shape is not supported in MakeMeshForDeformable().");
}
VolumeMesh<double> MakeMeshForDeformable(const Capsule&, double) {
  throw std::logic_error(
      "Capsule shape is not supported in MakeMeshForDeformable().");
}
VolumeMesh<double> MakeMeshForDeformable(const Ellipsoid&, double) {
  throw std::logic_error(
      "Ellipsoid shape is not supported in MakeMeshForDeformable().");
}
VolumeMesh<double> MakeMeshForDeformable(const Mesh&, double) {
  throw std::logic_error(
      "Mesh shape is not supported in MakeMeshForDeformable().");
}

VolumeMesh<double> MakeMeshForDeformable(const Convex&, double) {
  throw std::logic_error(
      "Convex shape is not supported in MakeMeshForDeformable().");
}

/* Unsupported shapes. */
VolumeMesh<double> MakeMeshForDeformable(const MeshcatCone&, double) {
  throw std::logic_error(
      "MeshcatCone shape is not supported in MakeMeshForDeformable().");
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
