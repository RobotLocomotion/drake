/* clang-format off to disable clang-format-includes */
#include "drake/systems/sensors/image_io.h"
/* clang-format on */

#include <string>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkCommand.h>       // vtkCommonCore
#include <vtkImageExport.h>   // vtkIOImage
#include <vtkNew.h>           // vtkCommonCore
#include <vtkSmartPointer.h>  // vtkCommonCore

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/systems/sensors/image_io_internal.h"
#include "drake/systems/sensors/vtk_diagnostic_event_observer.h"
#include "drake/systems/sensors/vtk_image_reader_writer.h"

// This file implements half of the class ImageIo (the Load functions).

namespace drake {
namespace systems {
namespace sensors {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;
using internal::VtkDiagnosticEventObserver;

// These are nested classes within ImageIo. It's convenient to lift them up into
// our namespace for brevity.
using InputAny = std::variant<const std::filesystem::path*, ImageIo::ByteSpan>;
using Metadata = ImageIo::Metadata;

/* When we ask VTK to read an image file, this is what we get back. */
struct DRAKE_NO_EXPORT ImageIo::LoaderTools {
  // This contains borrowed pointers to memory owned elsewhere.
  InputAny input_any;

  // The clearinghouse for I/O errors and warnings.
  DiagnosticPolicy diagnostic;

  // The metadata. Note that `metadata.format` will match `format` when the
  // image has been loaded successfully, but otherwise might be stale.
  ImageFileFormat format{ImageFileFormat::kPng};
  Metadata metadata;

  // The VTK guts.
  vtkNew<VtkDiagnosticEventObserver> reader_observer;
  vtkSmartPointer<vtkImageReader2> reader;
  vtkNew<vtkImageExport> loader;
};

/* Creates the loader infrastucture for the given input. Always returns
something, even if the file is missing or can't be parsed or etc. When the
required format (`format_`) is set, only that image format will be permitted.
Both the `input_any` pointer and the `diagnostic` pointer are aliased so must
outlive the returnu value. */
ImageIo::LoaderTools ImageIo::MakeLoaderTools(
    InputAny input_any, const DiagnosticPolicy* diagnostic) const {
  DRAKE_DEMAND(diagnostic != nullptr);

  // The return value aliases the `input_any` borrowed pointer.
  LoaderTools tools{.input_any = input_any};

  // Wire up the diagnostic policy to incorporate the filename.
  // This aliases the `diagnostic` borrowed pointer.
  // We only wire up errors because VTK doesn't have use warnings while loading.
  const std::string diagnostic_filename =
      (input_any.index() == 0) ? std::get<0>(input_any)->string() : "ImageIo";
  tools.diagnostic.SetActionForErrors(
      [diagnostic, diagnostic_filename](const DiagnosticDetail& detail) {
        DiagnosticDetail updated = detail;
        updated.filename = diagnostic_filename;
        diagnostic->Error(updated);
      });
  tools.reader_observer->set_diagnostic(&tools.diagnostic);

  // Decide which file format to use (with PNG as a last resort).
  if (format_.has_value()) {
    tools.format = *format_;
  } else if (std::optional<ImageFileFormat> guess =
                 internal::GuessFileFormat(input_any)) {
    tools.format = *guess;
  } else {
    tools.format = ImageFileFormat::kPng;
  }

  // Wire up the VTK guts and load the file.
  if (input_any.index() == 0) {
    tools.reader = internal::MakeReader(tools.format, *std::get<0>(input_any));
  } else {
    tools.reader = internal::MakeReader(
        tools.format, std::get<1>(input_any).data, std::get<1>(input_any).size);
  }
  tools.reader->AddObserver(vtkCommand::ErrorEvent, tools.reader_observer);
  tools.reader->AddObserver(vtkCommand::WarningEvent, tools.reader_observer);
  tools.reader->Update();
  // Note that we do NOT call ImageLowerLeftOff() here. That means that the
  // lower left pixel is (0, 0), counting upwards moving up the image.
  tools.loader->SetInputConnection(tools.reader->GetOutputPort(0));
  tools.loader->Update();

  // Set the metadata.
  tools.metadata.format = tools.format;
  const int* const dims = tools.loader->GetDataDimensions();
  tools.metadata.width = dims[0];
  tools.metadata.height = dims[1];
  tools.metadata.depth = dims[2];
  tools.metadata.channels = tools.loader->GetDataNumberOfScalarComponents();
  const int vtk_scalar = tools.loader->GetDataScalarType();
  if (std::optional<PixelScalar> pixel_scalar =
          internal::GetDrakeScalarType(vtk_scalar)) {
    tools.metadata.scalar = *pixel_scalar;
  } else {
    tools.diagnostic.Error(fmt::format(
        "The image uses an unsupported scalar type (VTK type {})", vtk_scalar));
    tools.metadata.channels = 0;
  }

  return tools;
}

namespace {

template <typename LoaderTools, PixelType kPixelType>
void CopyVtkToDrakeImage(const LoaderTools& tools, Image<kPixelType>* image) {
  DRAKE_DEMAND(image != nullptr);
  const Metadata& metadata = tools.metadata;

  // Reject unsupported depths. It doesn't seem possible for this to happen in
  // practice with the formats we support.
  DRAKE_THROW_UNLESS(metadata.depth == 1);

  // Reject mismatched scalars.
  const PixelScalar expected_scalar = ImageTraits<kPixelType>::kPixelScalar;
  if (metadata.scalar != expected_scalar) {
    // As a backwards compatibility hack, we allow loading 16U data into 16I.
    const bool allow_hack = ((metadata.scalar == PixelScalar::k16U) &&
                             (expected_scalar == PixelScalar::k16I));
    if (!allow_hack) {
      tools.diagnostic.Error(
          fmt::format("Can't load image with scalar={} into scalar={}.",
                      metadata.scalar, expected_scalar));
      return;
    }
  }

  // Reject mismatched channels.
  constexpr int num_channels = ImageTraits<kPixelType>::kNumChannels;
  const bool add_alpha = (metadata.channels == 3 && num_channels == 4);
  const bool drop_alpha = (metadata.channels == 4 && num_channels == 3);
  if (!add_alpha && !drop_alpha && (metadata.channels != num_channels)) {
    tools.diagnostic.Error(fmt::format(
        "Can't load image with channels={} into object with channels={}.",
        metadata.channels, num_channels));
    return;
  }

  // Blit everything.
  constexpr bool swizzle = internal::NeedsBgrSwizzle<kPixelType>();
  using T = typename Image<kPixelType>::T;
  const T* const source =
      reinterpret_cast<T*>(tools.loader->GetPointerToData());
  const int width = metadata.width;
  const int height = metadata.height;
  image->resize(width, height);
  T* dest = image->at(0, 0);
  if constexpr (sizeof(T) > 1) {
    DRAKE_DEMAND(!add_alpha);
    DRAKE_DEMAND(!drop_alpha);
    internal::CopyAndFlipRaw<T, num_channels, swizzle>(source, dest, width,
                                                       height);
  } else {
    if (add_alpha) {
      DRAKE_DEMAND(metadata.channels == 3);
      DRAKE_DEMAND(num_channels == 4);
      internal::CopyAndFlipRaw<T, 3, swizzle, 4>(source, dest, width, height);
    } else if (drop_alpha) {
      DRAKE_DEMAND(metadata.channels == 4);
      DRAKE_DEMAND(num_channels == 3);
      internal::CopyAndFlipRaw<T, 4, swizzle, 3>(source, dest, width, height);
    } else {
      DRAKE_DEMAND(metadata.channels == num_channels);
      internal::CopyAndFlipRaw<T, num_channels, swizzle>(source, dest, width,
                                                         height);
    }
  }
}

}  // namespace

std::optional<Metadata> ImageIo::LoadMetadataImpl(InputAny input_any) const {
  // Don't emit any diagnostics to the user when attempting to parse.
  int num_errors = 0;
  DiagnosticPolicy slient;
  slient.SetActionForWarnings([](const auto&) {});
  slient.SetActionForErrors([&num_errors](const auto&) {
    ++num_errors;
  });

  // Attempt to parse.
  LoaderTools tools = MakeLoaderTools(input_any, &slient);

  // Return the metadata (if we have any).
  if (num_errors > 0) {
    return std::nullopt;
  }
  return tools.metadata;
}

ImageAny ImageIo::LoadImpl(InputAny input_any) const {
  // Parse the image.
  LoaderTools tools = MakeLoaderTools(input_any, &diagnostic_);
  const Metadata& metadata = tools.metadata;

  // Construct the image type (chosen based on depth, scalar, and channels),
  // and then call a helper function to copy the data into the image.
  if (metadata.depth == 1) {
    switch (metadata.scalar) {
      case PixelScalar::k8U: {
        if (metadata.channels == 4) {
          ImageRgba8U result;
          CopyVtkToDrakeImage(tools, &result);
          return result;
        } else if (metadata.channels == 3) {
          ImageRgb8U result;
          CopyVtkToDrakeImage(tools, &result);
          return result;
        } else if (metadata.channels == 1) {
          ImageGrey8U result;
          CopyVtkToDrakeImage(tools, &result);
          return result;
        }
        break;
      }
      case PixelScalar::k16U: {
        if (metadata.channels == 1) {
          ImageDepth16U result;
          CopyVtkToDrakeImage(tools, &result);
          return result;
        }
        break;
      }
      case PixelScalar::k32F: {
        if (metadata.channels == 1) {
          ImageDepth32F result;
          CopyVtkToDrakeImage(tools, &result);
          return result;
        }
        break;
      }
      case PixelScalar::k16I:
        DRAKE_UNREACHABLE();
    }
  }

  tools.diagnostic.Error(fmt::format(
      "Can't load image (depth={}, channels={}, scalar={}) into any known "
      "Image<PixelType> template instantiation",
      metadata.depth, metadata.channels, metadata.scalar));
  return {};
}

void ImageIo::LoadImpl(InputAny input_any, ImageAnyMutablePtr image_any) const {
  // Parse the image.
  LoaderTools tools = MakeLoaderTools(input_any, &diagnostic_);

  // Copy out the bytes.
  std::visit(
      [&](auto* image) {
        CopyVtkToDrakeImage(tools, image);
      },
      image_any);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
