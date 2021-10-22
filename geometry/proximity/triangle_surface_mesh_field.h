#pragma once

#include "drake/geometry/proximity/mesh_field_linear.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {

/** A convenience alias for instantiating a MeshFieldLinear on a
 TriangleSurfaceMesh.
 @tparam FieldValue  A valid Eigen scalar or vector of valid Eigen scalars for
                     the field value.
 @tparam T  A valid Eigen scalar for mesh coordinates.
 */
template <typename FieldValue, typename T>
using TriangleSurfaceMeshFieldLinear =
    MeshFieldLinear<FieldValue, TriangleSurfaceMesh<T>>;

// The homogeneous instances are sufficiently common in Drake, that we'll
// build them once. Types with mixed scalars, or a *vector* mesh field type will
// be compiled as needed, but if they become common, they can be added here.
extern template class MeshFieldLinear<double, TriangleSurfaceMesh<double>>;
extern template class MeshFieldLinear<AutoDiffXd,
                                      TriangleSurfaceMesh<AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake
