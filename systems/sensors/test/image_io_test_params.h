#pragma once

#include <ostream>

#include <gtest/gtest.h>

#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/image_file_format.h"

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkImageData.h>  // vtkCommonDataModel
#include <vtkNew.h>        // vtkCommonCore
#include <vtkType.h>       // vtkCommonCore

namespace drake {
namespace systems {
namespace sensors {

/* Provides a grouping for value-parameterized test params. */
struct ImageIoTestParams {
  ImageFileFormat format{ImageFileFormat::kJpeg};

  /* Whether to use files or memory buffers. */
  bool write_to_file{false};

  /* Whether to use uint16_t (instead of the natural type). */
  bool force_16u{false};

  /* Whether to use a single-channel image (instead of color).
  When the scalar type is not 8U, then grayscale is already the default. */
  bool force_gray{false};

  /* Whether to use an alpha-channel. */
  bool alpha{false};

  /* Returns the PixelScalar intended for the current params. */
  PixelScalar pixel_scalar() const {
    return force_16u                            ? PixelScalar::k16U
           : (format == ImageFileFormat::kTiff) ? PixelScalar::k32F
                                                : PixelScalar::k8U;
  }

  /* Returns the VTK scalar type intended for the current params. */
  int vtk_scalar() const {
    return force_16u                            ? VTK_TYPE_UINT16
           : (format == ImageFileFormat::kTiff) ? VTK_TYPE_FLOAT32
                                                : VTK_TYPE_UINT8;
  }

  /* The width for Create...Image(). */
  int width() const { return 3; }

  /* The height for Create...Image(). */
  int height() const { return 2; }

  /* The depth for Create...Image(). This will always be 1. */
  int depth() const { return 1; }

  /* The channels for Create...Image(). */
  int channels() const {
    const bool is_byte_channel = pixel_scalar() == PixelScalar::k8U;
    if (force_gray) {
      DRAKE_DEMAND(is_byte_channel);
      return 1;
    }
    if (alpha) {
      DRAKE_DEMAND(is_byte_channel);
      return 4;
    }
    return is_byte_channel ? 3 : 1;
  }

  /* The product of width * height * depth * channels. */
  int total_storage() const {
    return width() * height() * depth() * channels();
  }

  /* Creates a sample image in memory. The image data will be filled with
  incrementing numbers, starting from 1 for the first datum. */
  vtkNew<vtkImageData> CreateIotaVtkImage() const;

  /* Creates a sample image in memory. The image data will be filled with
  incrementing numbers, starting from 1 for the first datum. */
  ImageAny CreateIotaDrakeImage() const;

  /* Given pointers to the pixel data buffer for two images, checks whether
  the round-trip Save() and Load() ended up with a correctly-loaded image.
  @pre the pixel data buffers have the right scalar type and total storage. */
  void CompareImageBuffer(const void* original, const void* readback) const;

  friend std::ostream& operator<<(std::ostream& os,
                                  const ImageIoTestParams& self) {
    os << fmt::format("[format = {}; write_to_file = {}; force_16u = {}]",
                      self.format, self.write_to_file, self.force_16u);
    return os;
  }
};

inline auto GetAllImageIoTestParams() {
  using Params = ImageIoTestParams;
  constexpr ImageFileFormat kJpeg = ImageFileFormat::kJpeg;
  constexpr ImageFileFormat kPng = ImageFileFormat::kPng;
  constexpr ImageFileFormat kTiff = ImageFileFormat::kTiff;
  return testing::Values(
      // VTK JPEG only supports 8-bit channels, so no 16u here.
      Params{.format = kJpeg, .write_to_file = true},
      Params{.format = kJpeg, .write_to_file = false},
      // VTK PNG supports grayscale, an alpha channel, and 16-bit channels.
      Params{.format = kPng, .write_to_file = true},
      Params{.format = kPng, .write_to_file = false},
      Params{.format = kPng, .write_to_file = true, .force_gray = true},
      Params{.format = kPng, .write_to_file = false, .force_gray = true},
      Params{.format = kPng, .write_to_file = true, .alpha = true},
      Params{.format = kPng, .write_to_file = false, .alpha = true},
      Params{.format = kPng, .write_to_file = true, .force_16u = true},
      Params{.format = kPng, .write_to_file = false, .force_16u = true},
      // VTK TIFF doesn't support writing to memory.
      Params{.format = kTiff, .write_to_file = true},
      Params{.format = kTiff, .write_to_file = true, .force_16u = true});
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
