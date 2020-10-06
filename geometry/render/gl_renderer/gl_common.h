#pragma once

// TODO(SeanCurtis-TRI): Come up with a better name for this file.

namespace drake {
namespace geometry {
namespace render {
namespace internal {

// TODO(SeanCurtis-TRI): Consider moving this up to RenderEngine; it's useful
//  for multiple RenderEngine types.
/* Rendering types available. Used to index into render-type-dependent data
 structures. Because it serves as an index, we use kTypeCount to declare the
 *number* of index values available (relying on C++'s default behavior of
 assigning sequential values in enumerations).  */
enum RenderType { kColor = 0, kLabel, kDepth, kTypeCount };

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
