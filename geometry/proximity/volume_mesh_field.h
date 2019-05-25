#pragma once

#include "drake/geometry/proximity/mesh_field_linear.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {

/**
 @tparam FieldValue  A valid Eigen scalar or vector of valid Eigen scalars for
                     the field value.
 @tparam T  A valid Eigen scalar for coordinates.
 */
template <typename FieldValue, typename T>
using VolumeMeshFieldLinear = MeshFieldLinear<FieldValue, VolumeMesh<T>>;

}  // namespace geometry
}  // namespace drake
