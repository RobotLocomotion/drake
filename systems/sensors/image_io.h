#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {
namespace sensors {
// TODO(jwnimmer-tri) Move out of internal namespace.
namespace internal {

/* Utility functions for images. TODO(jwnimmer-tri) Implement me. */
class ImageIo {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImageIo);

  /* When loading from memory, this struct denotes a span of raw bytes as
  input. */
  struct ByteSpan {
    /* Pointer to the first byte of the span. */
    const void* data{};
    /* Total number of bytes in the span. */
    size_t size{};
  };

  ImageIo() = delete;
};

}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
