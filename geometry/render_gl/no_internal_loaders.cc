#include <stdexcept>

#include "drake/geometry/render_gl/internal_loaders.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

void GladLoaderLoadEgl() {
  // This is not reachable via Drake's public API.
  throw std::logic_error("Drake's GladLoaderLoadEgl() is unavailable.");
}

void* GladLoaderLoadGlx() {
  // This is not reachable via Drake's public API.
  throw std::logic_error("Drake's GladLoaderLoadGlx() is unavailable.");
}

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
