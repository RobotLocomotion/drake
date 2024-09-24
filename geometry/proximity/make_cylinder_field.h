#pragma once

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/*
 Generates a piecewise-linear pressure field inside the given cylinder as
 represented by the given volume mesh. The pressure at a point is defined as E *
 e(x) where e is the extent -- a measure of penetration into the volume, and E
 is the given `hydroelastic_modulus`. For a point x inside the `cylinder`, the
 extent is defined as e(x) = (d(x)-δ)/(H-δ), with d(x) the (positive) distance
 to the boundary, δ the `margin` and H = min(R, L/2), where R and L are the
 radius and length of the `cylinder` respectively. With the extent defined, the
 pressure at a point x is computed as p(x) = E⋅e(x). The pressure has maximum
 value E in the interior and minimum value at the boundary p = -E⋅δ/(H-δ).

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
 @param[in] hydroelastic_modulus  Scale extent to pressure.
 @param margin           The magnitude of the margin δ, see @ref hydro_margin.
 @return                 The pressure field defined on the tetrahedral mesh.
 @pre                    `hydroelastic_modulus` is strictly positive.
                         `mesh_C` represents the cylinder and has enough
                         resolution to represent the pressure field.
 @pre                    R > δ and L > 2δ.
 @tparam_nonsymbolic_scalar
 */
template <typename T>
VolumeMeshFieldLinear<T, T> MakeCylinderPressureField(
    const Cylinder& cylinder, const VolumeMesh<T>* mesh_C,
    const T hydroelastic_modulus, double margin = 0);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
