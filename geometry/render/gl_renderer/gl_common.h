#pragma once

// TODO(SeanCurtis-TRI): Come up with a better name for this file.

namespace drake {
namespace geometry {
namespace render {
namespace internal {

// TODO(SeanCurtis-TRI): Consider moving this up to RenderEngine; it's useful
//  for multiple RenderEngine types.
/* Rendering types available. Used to index into render-type-dependent data
 structures.  */
enum RenderType { kLabel = 0, kDepth, kTypeCount };

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
