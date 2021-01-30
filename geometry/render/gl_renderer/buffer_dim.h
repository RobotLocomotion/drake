#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/geometry/render/gl_renderer/opengl_includes.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

/* Simple class for recording the dimensions of a render target. Serves as a
 key to access unique RenderTarget instances based on render size.  */
class BufferDim {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BufferDim)

  BufferDim(int width, int height) : width_(width), height_(height) {
    DRAKE_DEMAND(width > 0);
    DRAKE_DEMAND(height > 0);
  }

  int width() const { return width_; }
  int height() const { return height_; }

  /* Implements the @ref hash_append concept.  */
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const BufferDim& dim) noexcept {
    using drake::hash_append;
    hash_append(hasher, dim.width_);
    hash_append(hasher, dim.height_);
  }

  bool operator==(const BufferDim& dim) const {
    return width_ == dim.width_ && height_ == dim.height_;
  }

 private:
  int width_{};
  int height_{};
};

/* The collection of OpenGL objects which define a render target.
 Ultimately, OpenGL's draw commands are sent to this target's `frame_buffer`.
 The `frame_buffer` is configured with two additional objects:

   - the `value_texture` which stores the result of the rendering operations
     (i.e., depth, color, or label values).
   - A z-buffer that OpenGL uses to do hidden surface removal. The values in
     this buffer are strictly for internal OpenGL consumption.
*/
struct RenderTarget {
  GLuint frame_buffer;
  GLuint value_texture;
  GLuint z_buffer;
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake

namespace std {

/* Provides std::hash<BufferDim>.  */
template <>
struct hash<drake::geometry::render::internal::BufferDim>
    : public drake::DefaultHash {};
}  // namespace std
