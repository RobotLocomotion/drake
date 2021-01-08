#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/*
 Generates a piecewise-linear pressure field inside the given cylinder as
 represented by the given volume mesh. The pressure at a point is defined
 as E * e(x) where e ∈ [0,1] is the extent -- a measure of penetration into
 the volume, and E is the given `elastic_modulus`. The pressure is zero on the
 boundary with maximum E in the interior.

 For hydroelastics, a desirable mesh (`mesh_C` parameter) for this field can
 be created by MakeCylinderMeshWithMa(), which has these properties:
 1. It conforms to the medial axis (MA).
 2. It has no tetrahedra with all four vertices on the boundary.
 3. It tries to minimize the number of tetrahedra without sacrificing the
    accuracy of this piecewise-linear field.

 These two picture files show examples of a pressure field on three
 kinds of cylinders. The first picture shows the pressure field in the
 interior of the mesh. The second picture shows the pressure isosurfaces,
 which are essentially offset surfaces of the cylinder.

 |geometry/proximity/images/cylinder_mesh_medial_axis_pressure.png           |
 | Pressure field on meshes with medial axis of three kinds of cylinders.    |
 | :-----------------------------------------------------------------------: |
 |geometry/proximity/images/cylinder_mesh_medial_axis_pressure_isosurface.png|
 | Pressure isosurfaces on three kinds of cylinders.                         |

 @param[in] cylinder     The cylinder with its canonical frame C.
 @param[in] mesh_C       A pointer to a tetrahedral mesh of the cylinder. It
                         is aliased in the returned pressure field and must
                         remain alive as long as the field. The position
                         vectors of mesh vertices are expressed in the
                         cylinder's frame C.
 @param[in] elastic_modulus  Scale extent to pressure.
 @return                 The pressure field defined on the tetrahedral mesh.
 @pre                    `elastic_modulus` is strictly positive.
                         `mesh_C` represents the cylinder and has enough
                         resolution to represent the pressure field.
 @tparam T               The scalar type for representing the mesh vertex
                         positions and the pressure value.
 */
template <typename T>
VolumeMeshFieldLinear<T, T> MakeCylinderPressureField(
    const Cylinder& cylinder, const VolumeMesh<T>* mesh_C,
    const T elastic_modulus) {
  DRAKE_DEMAND(elastic_modulus > T(0));
  const double radius = cylinder.radius();
  const double length = cylinder.length();
  const double min_half_size = std::min(radius, length / 2.0);

  // TODO(DamrongGuoy): Switch to a better implementation in the future. The
  //  current implementation has a number of limitations:
  //  1. For simplicity, we use a scaling of distance to boundary, which is
  //     not differentiable at points equally far from two or more boundary
  //     points of the cylinder.
  //  2. Implicitly we impose the rigid core on the medial axis (or medial
  //     surfaces for short cylinders).  In the future, we will consider a
  //     rigid core as an offset of the cylinder or other shapes.
  //  3. We do not have a mechanism to define a pressure field using barrier
  //     functions.  One possibility is to generate the mesh in offset
  //     layers, and define linear pressure fields in each offset with
  //     different elastic modulus.

  std::vector<T> pressure_values;
  pressure_values.reserve(mesh_C->num_vertices());
  // TODO(DamrongGuoy): The following three const variables (unused_id,
  //  identity, and fcl_cylinder) are needed for applying DistanceToPoint
  //  functor in the for loop. They are awkward to use here and introduce an
  //  unnecessary dependency on FCL. In the future, we should either refactor
  //  the distance-to-boundary calculation or use a different calculation (for
  //  example, offset-based distance).
  const GeometryId unused_id;
  const auto identity = math::RigidTransform<T>::Identity();
  const fcl::Cylinderd fcl_cylinder(radius, length);
  for (const VolumeVertex<T>& vertex : mesh_C->vertices()) {
    // V is a vertex of the cylinder mesh with frame C.
    const Vector3<T>& r_CV = vertex.r_MV();
    point_distance::DistanceToPoint<T> signed_distance_functor(
        unused_id, identity, r_CV);
    const T signed_distance = signed_distance_functor(fcl_cylinder).distance;
    // Map signed_distance ∈ [-min_half_size, 0] to extent e ∈ [0, 1],
    // -min_half_size ⇝ 1, 0 ⇝ 0.
    const T extent = -signed_distance / T(min_half_size);
    using std::min;
    // Bound the pressure values in [0, E], where E is the elastic modulus.
    pressure_values.push_back(min(elastic_modulus * extent, elastic_modulus));
  }

  // Make sure the boundary vertices have zero pressure. Numerical rounding
  // can cause the boundary vertices to be slightly off the boundary surface
  // of the cylinder.
  std::vector<VolumeVertexIndex> boundary_vertices =
      CollectUniqueVertices(IdentifyBoundaryFaces(mesh_C->tetrahedra()));
  for (VolumeVertexIndex bv : boundary_vertices) {
    pressure_values[bv] = T(0.);
  }

  return VolumeMeshFieldLinear<T, T>("pressure(Pa)",
                                     std::move(pressure_values), mesh_C);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
