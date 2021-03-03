#pragma once

#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** Generates a tetrahedral volume mesh of a given box by subdividing the box
 into _rectangular cells_ (volume bounded by six axis-aligned faces) and
 subdividing each rectangular cell into five tetrahedra. The output mesh will
 have these properties:

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
     The pose of the rectanglur volume mesh in the world frame.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
geometry::VolumeMesh<T> MakeDiamondCubicBoxVolumeMesh(
    const geometry::Box& box, double resolution_hint,
    const math::RigidTransform<T>& X_WB);
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
