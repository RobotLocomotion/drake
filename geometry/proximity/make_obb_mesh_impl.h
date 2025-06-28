#pragma once

#include "drake/geometry/mesh_source.h"
#include "drake/geometry/proximity/obb.h"

namespace drake {
namespace geometry {
namespace internal {

/* Creates an oriented bounding box (OBB) for the provided mesh data. The mesh
 data can come from on-disk files or in-memory files -- but the content must be
 that of a supported Drake mesh type.

 The vertex positions can be scaled uniformly around the origin of the mesh
 data's canonical frame.

 @param mesh_source  The source of the mesh data.
 @param scale        The scale to apply to the vertex data -- the vertex
                     position vectors are scaled relative to the mesh data's
                     canonical frame's origin.

 @throws std::exception if the mesh data is in an unsupported format.
 @throws std::exception if the mesh data is ill formed. */
Obb MakeObb(const MeshSource& mesh_source, const Vector3<double>& scale);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
