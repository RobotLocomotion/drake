#pragma once

#include <memory>

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* See Build() function. */
class MeshBuilderForDeformable : public ShapeReifier {
 public:
  MeshBuilderForDeformable() = default;

  /* Returns a volume mesh suitable for deformable body simulation that
   discretizes the shape with the resolution provided at construction.
   @pre resolution_hint > 0.
   @throws std::exception if `shape` doesn't support a mesh discretization. */
  std::unique_ptr<VolumeMesh<double>> Build(const Shape& shape,
                                            double resolution_hint);

 private:
  /* Data to be used during reification. It is passed as the `user_data`
   parameter in the ImplementGeometry API. */
  struct ReifyData {
    double resolution_hint{};
    std::unique_ptr<VolumeMesh<double>> mesh;
  };

  void ImplementGeometry(const Sphere& sphere, void* user_data) override;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override;
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) override;
  void ImplementGeometry(const Box& box, void* user_data) override;
  void ImplementGeometry(const Capsule& capsule, void* user_data) override;
  void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data) override;
  void ImplementGeometry(const Mesh& mesh, void* user_data) override;
  void ImplementGeometry(const Convex& convex, void* user_data) override;
  void ImplementGeometry(const MeshcatCone& cone, void* user_data) override;
};

/* Returns a volume mesh suitable for deformable body simulation that
 discretizes the shape with the resolution provided at construction. There is a
 seperate function for each concrete shape derived from Shape.
 @pre resolution_hint > 0.
 @throws std::exception if `shape` doesn't support a mesh discretization. */
//@{
VolumeMesh<double> MakeMeshForDeformable(const Sphere& sphere,
                                         double resolution_hint);
VolumeMesh<double> MakeMeshForDeformable(const Cylinder& cylinder,
                                         double resolution_hint);
VolumeMesh<double> MakeMeshForDeformable(const HalfSpace& half_space,
                                         double resolution_hint);
VolumeMesh<double> MakeMeshForDeformable(const Box& box,
                                         double resolution_hint);
VolumeMesh<double> MakeMeshForDeformable(const Capsule& capsule,
                                         double resolution_hint);
VolumeMesh<double> MakeMeshForDeformable(const Ellipsoid& ellipsoid,
                                         double resolution_hint);
VolumeMesh<double> MakeMeshForDeformable(const Mesh& mesh,
                                         double resolution_hint);
VolumeMesh<double> MakeMeshForDeformable(const Convex& convex,
                                         double resolution_hint);
VolumeMesh<double> MakeMeshForDeformable(const MeshcatCone& cone,
                                         double resolution_hint);
//@}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
