#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fixed_fem/dev/reference_deformable_geometry.h"

namespace drake {
namespace multibody {
namespace fem {
/** Generates a deformable geometry from a given box by subdividing the box
 into _rectangular cells_ (volume bounded by six axis-aligned faces) and
 subdividing each rectangular cell into five tetrahedra. Two adjacent
 rectangular cells (sharing a rectangular face) are subdivided in the patterns
 that are mirrored of each other so that the mesh is conforming.

 The following picture file shows example of the diamond cubic box volume mesh
 and demonstrates the mirrored subdivision pattern in adjacent cells. The file
 is distributed with the source code.

 | multibody/fixed_fem/dev/images/diamond_cubic_box_volume_mesh.png  |
 | (Top) A diamond cubic box volume mesh. (Center and bottom) mirrored
   subdivision patterns to make sure the mesh is conforming.         |

 The output mesh will have these properties:

   1. The generated vertices are unique. There is no repeating vertices in
      the list of vertex coordinates.
   2. The generated tetrahedra are _conforming_. Two tetrahedra intersect in
      their shared face, or shared edge, or shared vertex, or not at all.
      There is no partial overlapping of two tetrahedra.
 @param[in] box
     The box shape specification (see drake::geometry::Box).
 @param[in] resolution_hint
     A measure (in meters) that controls the resolution of the mesh. The length
     of the axis-aligned edges of the mesh will be less than or equal to this
     parameter. The length of non-axis-aligned edges will be less than or equal
     to √3 of this parameter. The coarsest possible mesh can be made by
     providing a resolution hint at least as large as the box's largest
     dimension.
 @param[in] X_WB
     The pose of the rectangular volume mesh in the world frame.
 @tparam_nonsymbolic_scalar */
template <typename T>
internal::ReferenceDeformableGeometry<T> MakeDiamondCubicBoxDeformableGeometry(
    const geometry::Box& box, double resolution_hint,
    const math::RigidTransform<T>& X_WB);

/* Generates a volume mesh of an octahedron comprising of eight tetrahedral
 elements with vertices on the coordinate axes and the origin like this:

                +Z   -X
                 |   /
              v5 ●  ● v3
                 | /
       v4     v0 |/
  -Y----●--------●------●----+Y
                /|      v2
               / |
           v1 ●  ● v6
             /   |
           +X    |
                -Z
 @tparam_nonsymbolic_scalar */
template <typename T>
geometry::VolumeMesh<T> MakeOctahedronVolumeMesh();

/* Generates a ReferenceDeformableGeometry whose underlying mesh is given by
 MakeOctahedronVolumeMesh().
 @tparam_nonsymbolic_scalar */
template <typename T>
internal::ReferenceDeformableGeometry<T> MakeOctahedronDeformableGeometry();

/* Refines each boundary tetrahedron into tetrahedra with at least one
 interior vertex by applying "star" refinement from the centroid of the
 boundary tetrahedron. A boundary tetrahedron is a tetrahedron with
 all four vertices on the boundary surface. "Star" refinement replaces one
 boundary tetrahedron with four non-boundary tetrahedra.

 Schematically this is "star" refinement of a tetrahedron (each triangular
 face of the tetrahedron is drawn schematically as a line segment).

             o                   o
            / \                 /|\
           /   \     star      / | \
          o     o    ===>     o--*--o        --- represent triangular faces
           \   /  refinement   \ | /
            \ /                 \|/
             o                   o

       1 tetrahedron              4 tetrahedra
       4 triangular faces        10 triangular faces
       6 edges                   10 edges
       4 vertices                 5 vertices
 */
template <typename T>
geometry::VolumeMesh<T> StarRefineBoundaryTetrahedra(
    const geometry::VolumeMesh<T>& in);

}  // namespace fem
}  // namespace multibody
}  // namespace drake
