#pragma once

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

/* Dynamically loads libEGL.so.1 (or libEGL.so) using GLAD and initializes the
glad/egl.h function pointers, or throws an exception if an error occurs.
On Apple platforms, always throws an exception. */
void GladLoaderLoadEgl();

/* Dynamically loads libGL.so.1 (or libGL.so) using GLAD and initializes the
glad/glx.h function pointers, or throws an exception if an error occurs.
On Apple platforms, always throws an exception.

Returns the X11 `Display*` pointer (but as a void pointer, to avoid needing to
include the X11 header file here). */
void* GladLoaderLoadGlx();

}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
