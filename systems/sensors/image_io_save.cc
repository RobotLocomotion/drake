/* clang-format off to disable clang-format-includes */
#include "drake/systems/sensors/image_io.h"
/* clang-format on */

#include <stdexcept>
#include <string>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkImageData.h>     // vtkCommonDataModel
#include <vtkImageWriter.h>   // vtkIOImage
#include <vtkNew.h>           // vtkCommonCore
#include <vtkSmartPointer.h>  // vtkCommonCore

#include "drake/systems/sensors/image_io_internal.h"
#include "drake/systems/sensors/vtk_image_reader_writer.h"

// This file implements half of the class ImageIo (the Save functions).

namespace drake {
namespace systems {
namespace sensors {

namespace {
template <PixelType kPixelType>
void CopyImage(const Image<kPixelType>& source_image,
               vtkImageData* dest_image) {
  // Allocate the VTK storage.
  const int width = source_image.width();
  const int height = source_image.height();
  constexpr int depth = 1;
  constexpr int num_channels = Image<kPixelType>::kNumChannels;
  dest_image->SetDimensions(width, height, depth);
  dest_image->AllocateScalars(internal::GetVtkScalarType<kPixelType>(),
                              num_channels);

  // Copy the bytes.
  using T = typename ImageTraits<kPixelType>::ChannelType;
  const T* const source = source_image.at(0, 0);
  T* dest = reinterpret_cast<T*>(dest_image->GetScalarPointer());
  constexpr bool swizzle = internal::NeedsBgrSwizzle<kPixelType>();
  internal::CopyAndFlipRaw<T, num_channels, swizzle>(source, dest, width,
                                                     height);
}

}  // namespace

void ImageIo::SaveImpl(ImageAnyConstPtr image_any,
                       std::optional<ImageFileFormat> format,
                       OutputAny output_any) const {
  // Choose which file format to use.
  const ImageFileFormat chosen_format = [format, output_any]() {
    if (format.has_value()) {
      return *format;
    } else {
      DRAKE_DEMAND(output_any.index() == 0);
      const std::filesystem::path& path = *std::get<0>(output_any);
      if (std::optional<ImageFileFormat> from_ext =
              internal::FileFormatFromExtension(path)) {
        return *from_ext;
      } else {
        throw std::logic_error(fmt::format(
            "ImageIo::Save(path='{}') requires SetFileFormat() to be called "
            "first because the path does not imply any supported format.",
            path.string()));
      }
    }
  }();

  // Make the VTK writer.
  vtkSmartPointer<vtkImageWriter> writer;
  if (output_any.index() == 0) {
    writer = internal::MakeWriter(chosen_format, *std::get<0>(output_any));
  } else {
    writer = internal::MakeWriter(chosen_format, std::get<1>(output_any));
  }

  // Copy the Drake image buffer to a VTK image buffer. Drake uses (x=0, y=0)
  // as the top left corner, but VTK uses it as the bottom left corner, and
  // neither has an option to switch the representation. Therefore, we always
  // need to eat an extra copy here.
  vtkNew<vtkImageData> vtk_image;
  visit(
      [&vtk_image](const auto* drake_image) {
        CopyImage(*drake_image, vtk_image.Get());
      },
      image_any);

  // Use the writer to encode/compress the image per the file format.
  writer->SetInputData(vtk_image.GetPointer());
  writer->Write();

  // TODO(jwnimmer-tri) We should more carefully check the VTK objects for
  // errors (or warnings), and propagate those up through our diagnostic policy.
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
