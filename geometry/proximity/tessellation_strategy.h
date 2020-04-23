#pragma once

namespace drake {
namespace geometry {
namespace internal {

/* Describes the possible tessellation strategies available when creating a
 VolumeMesh from a Shape specification. Not all tessellation algorithms support
 all strategies. When presented with an unsupported strategy, the algorithm is
 free to choose to select an alternate default algorithm, throw, warn, or
 whatever.  */
enum class TessellationStrategy {
  /* The interior of the volume mesh has a single vertex.  */
  kSingleInteriorVertex,
  /* The interior of the volume mesh has many vertices to maintain well-aspected
   tetrahedra.  */
  kDenseInteriorVertices
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
