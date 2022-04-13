#include "drake/geometry/make_deformable_mesh.h"

#include <utility>

#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

VolumeMesh<double> MakeDeformableMesh(const Sphere& sphere,
                                      double resolution_hint) {
  DRAKE_DEMAND(resolution_hint > 0);
  return MakeSphereVolumeMesh<double>(
      sphere, resolution_hint, TessellationStrategy::kDenseInteriorVertices);
}

// TODO(xuchenhan-tri): Implement these shapes.
VolumeMesh<double> MakeDeformableMesh(const Cylinder&, double) {
  throw std::logic_error(
      "Cylinder shape is not supported in MakeDeformableMesh().");
}
VolumeMesh<double> MakeDeformableMesh(const HalfSpace&, double) {
  throw std::logic_error(
      "Half space shape is not supported in MakeDeformableMesh().");
}
VolumeMesh<double> MakeDeformableMesh(const Box&, double) {
  throw std::logic_error("Box shape is not supported in MakeDeformableMesh().");
}
VolumeMesh<double> MakeDeformableMesh(const Capsule&, double) {
  throw std::logic_error(
      "Capsule shape is not supported in MakeDeformableMesh().");
}
VolumeMesh<double> MakeDeformableMesh(const Ellipsoid&, double) {
  throw std::logic_error(
      "Ellipsoid shape is not supported in MakeDeformableMesh().");
}
VolumeMesh<double> MakeDeformableMesh(const Mesh&, double) {
  throw std::logic_error(
      "Mesh shape is not supported in MakeDeformableMesh().");
}

VolumeMesh<double> MakeDeformableMesh(const Convex&, double) {
  throw std::logic_error(
      "Convex shape is not supported in MakeDeformableMesh().");
}

/* Unsupported shapes. */
VolumeMesh<double> MakeDeformableMesh(const MeshcatCone&, double) {
  throw std::logic_error(
      "MeshcatCone shape is not supported in MakeDeformableMesh().");
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
