#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/geometry/render/gl_renderer/opengl_includes.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

/** Simple class for recording the dimensions of a render target. Used store
 unique RenderTarget instances based on render size.  */
class BufferDim {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BufferDim)

  BufferDim(int width, int height) : width_(width), height_(height) {}

  int width() const { return width_; }
  int height() const { return height_; }

  /** Implements the @ref hash_append concept.  */
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
  int width_{-1};
  int height_{-1};
};

/** The collection of OpenGL objects which define a render target --
 essentially, a 2D image that the depth data gets rendered to.  */
struct RenderTarget {
  GLuint frame_buffer;
  GLuint texture;
  GLuint render_buffer;
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake

namespace std {

/** Provides std::hash<BufferDim>.  */
template <>
struct hash<drake::geometry::render::internal::BufferDim>
    : public drake::DefaultHash {};
}  // namespace std
