#pragma once

#include <numeric>
#include <ostream>

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

  /* The channels for Create...Image(). */
  int channels() const { return (format == ImageFileFormat::kTiff) ? 1 : 3; }

  /* Creates a sample image in memory. The image data will be filled with
  incrementing numbers, starting from 1 for the first datum. */
  vtkNew<vtkImageData> CreateIotaVtkImage() const {
    const int depth = 1;
    const int total_storage = width() * height() * depth * channels();
    vtkNew<vtkImageData> image;
    image->SetDimensions(width(), height(), depth);
    image->AllocateScalars(vtk_scalar(), channels());
    if (vtk_scalar() == VTK_TYPE_UINT8) {
      auto* dest = reinterpret_cast<uint8_t*>(image->GetScalarPointer());
      std::iota(dest, dest + total_storage, uint8_t{0x01});
    } else if (vtk_scalar() == VTK_TYPE_UINT16) {
      auto* dest = reinterpret_cast<uint16_t*>(image->GetScalarPointer());
      std::iota(dest, dest + total_storage, uint16_t{0x0001});
    } else {
      auto* dest = reinterpret_cast<float*>(image->GetScalarPointer());
      std::iota(dest, dest + total_storage, 1.0f);
    }
    return image;
  }

  /* Creates a sample image in memory. The image data will be filled with
  incrementing numbers, starting from 1 for the first datum. */
  ImageAny CreateIotaDrakeImage() const {
    ImageRgba8U image(width(), height());
    auto* dest = image.at(0, 0);
    std::iota(dest, dest + image.size(), uint8_t{0x01});
    return image;
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  const ImageIoTestParams& self) {
    os << fmt::format("[format = {}; write_to_file = {}; force_16u = {}]",
                      self.format, self.write_to_file, self.force_16u);
    return os;
  }
};

auto GetAllImageIoTestParams() {
  using Params = ImageIoTestParams;
  constexpr ImageFileFormat kJpeg = ImageFileFormat::kJpeg;
  constexpr ImageFileFormat kPng = ImageFileFormat::kPng;
  constexpr ImageFileFormat kTiff = ImageFileFormat::kTiff;
  return testing::Values(
      // VTK JPEG only supports 8-bit channels, so no 16u here.
      Params{.format = kJpeg, .write_to_file = true},
      Params{.format = kJpeg, .write_to_file = false},
      // VTK PNG supports both 8-bit and 16-bit channels.
      Params{.format = kPng, .write_to_file = true},
      Params{.format = kPng, .write_to_file = false},
      Params{.format = kPng, .write_to_file = true, .force_16u = true},
      Params{.format = kPng, .write_to_file = false, .force_16u = true},
      // VTK TIFF doesn't support writing to memory.
      Params{.format = kTiff, .write_to_file = true},
      Params{.format = kTiff, .write_to_file = true, .force_16u = true});
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
