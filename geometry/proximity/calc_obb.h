#pragma once

#include <optional>

#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

/** Calculates the oriented bounding box (OBB) for the Shape. If a shape does
 not support OBB computation, this function returns `std::nullopt`.

 @throws std::exception if a referenced file cannot be opened.
*/
std::optional<Obb> CalcObb(const Shape& shape);

namespace internal {

/* Creates an oriented bounding box (OBB) for the provided mesh data. The mesh
 data can come from on-disk files or in-memory files. Returns `std::nullopt`
 if the mesh data is in an unsupported format.

 The vertex positions can be scaled uniformly around the origin of the mesh
 data's canonical frame.

 @param mesh_source  The source of the mesh data.
 @param scale        The scale to apply to the vertex data -- the vertex
                     position vectors are scaled relative to the mesh data's
                     canonical frame's origin.

 @throws std::exception if the mesh data is ill formed. */
std::optional<Obb> MakeObb(const MeshSource& mesh_source,
                           const Vector3<double>& scale);

}  // namespace internal

}  // namespace geometry
}  // namespace drake
