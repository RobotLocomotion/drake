#pragma once

#include <algorithm>
#include <filesystem>
#include <optional>
#include <variant>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkType.h>  // vtkCommonCore

#include "drake/common/drake_assert.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/image_file_format.h"
#include "drake/systems/sensors/image_io.h"

// This file contains low-level ImageIo helper functions that are carved out in
// support of direct unit testing.

namespace drake {
namespace systems {
namespace sensors {
namespace internal {

/* Guesses a file format for the given filename, based on its extension. */
std::optional<ImageFileFormat> FileFormatFromExtension(
    const std::filesystem::path& filename);

/* Guesses a file format for the given input using the initial few bytes of the
file header. If the header is unintelligible, silently returns nullopt. */
std::optional<ImageFileFormat> GuessFileFormat(
    std::variant<const std::filesystem::path*, ImageIo::ByteSpan> input_any);

/* Returns true iff the PixelType's channel ordering is BGR (in which case it
will need to be reversed to match VTK's convention of RGB ordering). */
template <PixelType kPixelType>
constexpr bool NeedsBgrSwizzle() {
  switch (kPixelType) {
    case PixelType::kBgr8U:
    case PixelType::kBgra8U: {
      return true;
    }
    case PixelType::kDepth16U:
    case PixelType::kDepth32F:
    case PixelType::kGrey8U:
    case PixelType::kLabel16I:
    case PixelType::kRgb8U:
    case PixelType::kRgba8U:
      return false;
  }
  DRAKE_UNREACHABLE();
}

/* Returns the VTK scalar type enum for the given PixelType. */
template <PixelType kPixelType>
constexpr int GetVtkScalarType() {
  switch (ImageTraits<kPixelType>::kPixelScalar) {
    case PixelScalar::k8U:
      return VTK_TYPE_UINT8;
    case PixelScalar::k16U:
      return VTK_TYPE_UINT16;
    case PixelScalar::k32F:
      return VTK_TYPE_FLOAT32;
    case PixelScalar::k16I:
      // As a backwards compatibility hack, we save 16I data into 16U.
      return VTK_TYPE_UINT16;
  }
  DRAKE_UNREACHABLE();
}

/* Returns the Drake image scalar enum for the given VTK scalar type enum. */
constexpr std::optional<PixelScalar> GetDrakeScalarType(int vtk_scalar) {
  switch (vtk_scalar) {
    case VTK_TYPE_UINT8:
      return PixelScalar::k8U;
    case VTK_TYPE_UINT16:
      return PixelScalar::k16U;
    case VTK_TYPE_FLOAT32:
      return PixelScalar::k32F;
    default:
      break;
  }
  return std::nullopt;
}

/* Copies pixel data from one memory buffer to another, flipping the image
vertically where the bottom row of `source` becomes the top row of `dest`.
This implementation allows for a differing number of source and dest channels,
but does not allow for anything more complicated (e.g., gaps in any strides).
@tparam T Channel type scalar (typically uint8_t, uint16_t, or float).
@tparam source_channels Number of source channels (typically 1, 3, or 4).
@tparam swizzle Whether to swizzle channels (i.e., changing BGR to RGB).
@tparam dest_channels Number of destination channels (typically 1, 3, or 4).
 This option is intended to either add or remove the alpha channel, so it must
 be within ±1 of source_channels.
@tparam dest_pad Filler value to use when when dest_channels > source_channels.
@param source Source memory buffer.
@param dest Destination memory buffer.
@param width Image width.
@param height Image height. */
template <typename T, int source_channels, bool swizzle,
          int dest_channels = source_channels, uint8_t dest_pad = 0xff>
void CopyAndFlipRaw(const T* source, T* dest, int width, int height) {
  static_assert(dest_channels >= source_channels - 1);
  static_assert(dest_channels <= source_channels + 1);
  constexpr int min_channels = std::min(source_channels, dest_channels);
  // Fast forward `dest` to point to the start of the final row.
  dest += (height - 1) * width * dest_channels;
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      int c = 0;
      for (; c < min_channels; ++c) {
        const int source_channel = swizzle ? (min_channels - c - 1) : c;
        *(dest++) = source[source_channel];
      }
      source += source_channels;
      for (; c < dest_channels; ++c) {
        *(dest++) = dest_pad;
      }
    }
    // Currently `dest` points just past the end of row we just copied.
    // Rewind it two rows worth, so it points to the start of the prior row.
    dest -= 2 * width * dest_channels;
  }
}

}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
