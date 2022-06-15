#include "drake/geometry/make_mesh_for_deformable.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

std::unique_ptr<VolumeMesh<double>> MeshBuilderForDeformable::Build(
    const Shape& shape, double resolution_hint) {
  DRAKE_DEMAND(resolution_hint > 0.0);
  ReifyData data{resolution_hint, nullptr};
  shape.Reify(this, &data);
  return std::move(data.mesh);
}

void MeshBuilderForDeformable::ImplementGeometry(const Sphere& sphere,
                                                 void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  data.mesh = std::make_unique<VolumeMesh<double>>(
      MakeMeshForDeformable(sphere, data.resolution_hint));
}
void MeshBuilderForDeformable::ImplementGeometry(const Cylinder& cylinder,
                                                 void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  data.mesh = std::make_unique<VolumeMesh<double>>(
      MakeMeshForDeformable(cylinder, data.resolution_hint));
}
void MeshBuilderForDeformable::ImplementGeometry(const HalfSpace& half_space,
                                                 void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  data.mesh = std::make_unique<VolumeMesh<double>>(
      MakeMeshForDeformable(half_space, data.resolution_hint));
}
void MeshBuilderForDeformable::ImplementGeometry(const Box& box,
                                                 void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  data.mesh = std::make_unique<VolumeMesh<double>>(
      MakeMeshForDeformable(box, data.resolution_hint));
}
void MeshBuilderForDeformable::ImplementGeometry(const Capsule& capsule,
                                                 void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  data.mesh = std::make_unique<VolumeMesh<double>>(
      MakeMeshForDeformable(capsule, data.resolution_hint));
}
void MeshBuilderForDeformable::ImplementGeometry(const Ellipsoid& ellipsoid,
                                                 void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  data.mesh = std::make_unique<VolumeMesh<double>>(
      MakeMeshForDeformable(ellipsoid, data.resolution_hint));
}
void MeshBuilderForDeformable::ImplementGeometry(const Mesh& mesh,
                                                 void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  data.mesh = std::make_unique<VolumeMesh<double>>(
      MakeMeshForDeformable(mesh, data.resolution_hint));
}
void MeshBuilderForDeformable::ImplementGeometry(const Convex& convex,
                                                 void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  data.mesh = std::make_unique<VolumeMesh<double>>(
      MakeMeshForDeformable(convex, data.resolution_hint));
}
void MeshBuilderForDeformable::ImplementGeometry(const MeshcatCone& cone,
                                                 void* user_data) {
  ReifyData& data = *static_cast<ReifyData*>(user_data);
  data.mesh = std::make_unique<VolumeMesh<double>>(
      MakeMeshForDeformable(cone, data.resolution_hint));
}

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
