#include "drake/systems/sensors/lcm_image_array_to_images.h"

#include <vector>

#include <zlib.h>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkImageExport.h>   // vtkIOImage
#include <vtkImageReader2.h>  // vtkIOImage
#include <vtkNew.h>           // vtkCommonCore

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/lcmt_image_array.hpp"
#include "drake/systems/sensors/lcm_image_traits.h"
#include "drake/systems/sensors/vtk_image_reader_writer.h"

// TODO(jwnimmer-tri) Simplify this code by using "image_io.h" instead of
// "vtk_image_reader_writer.h", and using a DiagnosticPolicy instead of status
// bools flying around everywhere.

namespace drake {
namespace systems {
namespace sensors {
namespace {

bool is_color_image(int8_t type) {
  switch (type) {
    case lcmt_image::PIXEL_FORMAT_RGB:
    case lcmt_image::PIXEL_FORMAT_BGR:
    case lcmt_image::PIXEL_FORMAT_RGBA:
    case lcmt_image::PIXEL_FORMAT_BGRA: {
      return true;
    }
    default: {
      break;
    }
  }
  return false;
}

bool image_has_alpha(int8_t type) {
  switch (type) {
    case lcmt_image::PIXEL_FORMAT_RGBA:
    case lcmt_image::PIXEL_FORMAT_BGRA: {
      return true;
    }
    default: {
      break;
    }
  }
  return false;
}

template <PixelType kPixelType>
bool DecompressVtk(ImageFileFormat format, const lcmt_image* lcm_image,
                   Image<kPixelType>* image) {
  vtkSmartPointer<vtkImageReader2> reader = internal::MakeReader(
      format, lcm_image->data.data(), lcm_image->data.size());
  reader->Update();

  vtkNew<vtkImageExport> exporter;
  exporter->SetInputConnection(reader->GetOutputPort(0));
  exporter->ImageLowerLeftOff();
  exporter->Update();

  if (exporter->GetDataMemorySize() !=
      image->width() * image->height() * image->kPixelSize) {
    drake::log()->error("Malformed output decoding incoming LCM {} image",
                        format);
    return false;
  }
  exporter->Export(image->at(0, 0));
  return true;
}

template <PixelType kPixelType>
bool DecompressZlib(const lcmt_image* lcm_image, Image<kPixelType>* image) {
  // NOLINTNEXTLINE(runtime/int)
  unsigned long dest_len = image->width() * image->height() * image->kPixelSize;
  const int status =
      uncompress(reinterpret_cast<Bytef*>(image->at(0, 0)), &dest_len,
                 lcm_image->data.data(), lcm_image->size);
  if (status != Z_OK) {
    drake::log()->error("zlib decompression failed on incoming LCM image: {}",
                        status);
    *image = Image<kPixelType>();
    return false;
  }
  return true;
}

template <PixelType kPixelType>
bool UnpackLcmImage(const lcmt_image* lcm_image, Image<kPixelType>* image) {
  DRAKE_DEMAND(lcm_image->pixel_format ==
               LcmPixelTraits<Image<kPixelType>::kPixelFormat>::kPixelFormat);
  DRAKE_DEMAND(lcm_image->channel_type ==
               LcmImageTraits<kPixelType>::kChannelType);

  image->resize(lcm_image->width, lcm_image->height);

  switch (lcm_image->compression_method) {
    case lcmt_image::COMPRESSION_METHOD_NOT_COMPRESSED: {
      memcpy(image->at(0, 0), lcm_image->data.data(), image->size());
      return true;
    }
    case lcmt_image::COMPRESSION_METHOD_ZLIB: {
      return DecompressZlib(lcm_image, image);
    }
    case lcmt_image::COMPRESSION_METHOD_JPEG: {
      return DecompressVtk(ImageFileFormat::kJpeg, lcm_image, image);
    }
    case lcmt_image::COMPRESSION_METHOD_PNG: {
      return DecompressVtk(ImageFileFormat::kPng, lcm_image, image);
    }
    default: {
      break;
    }
  }

  drake::log()->error("Unsupported LCM compression method: {}",
                      lcm_image->compression_method);
  *image = Image<kPixelType>();
  return false;
}

}  // namespace

LcmImageArrayToImages::LcmImageArrayToImages()
    : image_array_t_input_port_index_(
          this->DeclareAbstractInputPort("image_array_t",
                                         Value<lcmt_image_array>())
              .get_index()),
      color_image_output_port_index_(
          this->DeclareAbstractOutputPort(
                  "color_image", &LcmImageArrayToImages::CalcColorImage)
              .get_index()),
      depth_image_output_port_index_(
          this->DeclareAbstractOutputPort(
                  "depth_image", &LcmImageArrayToImages::CalcDepthImage)
              .get_index()),
      label_image_output_port_index_(
          this->DeclareAbstractOutputPort(
                  "label_image", &LcmImageArrayToImages::CalcLabelImage)
              .get_index()) {
  // TODO(sammy-tri) Calculating our output ports can be kinda expensive.  We
  // should cache the images.
}

void LcmImageArrayToImages::CalcColorImage(const Context<double>& context,
                                           ImageRgba8U* color_image) const {
  const auto& images =
      image_array_t_input_port().Eval<lcmt_image_array>(context);

  // Look through the image array and just grab the first color image.
  const lcmt_image* lcm_image = nullptr;
  for (int i = 0; i < images.num_images; i++) {
    if (is_color_image(images.images[i].pixel_format)) {
      lcm_image = &images.images[i];
      break;
    }
  }

  if (!lcm_image) {
    *color_image = ImageRgba8U();
    return;
  }

  const bool has_alpha = image_has_alpha(lcm_image->pixel_format);
  if (has_alpha) {
    UnpackLcmImage(lcm_image, color_image);
  } else {
    ImageRgb8U rgb_image;
    if (UnpackLcmImage(lcm_image, &rgb_image)) {
      color_image->resize(lcm_image->width, lcm_image->height);
      for (int x = 0; x < lcm_image->width; x++) {
        for (int y = 0; y < lcm_image->height; y++) {
          color_image->at(x, y)[0] = rgb_image.at(x, y)[0];
          color_image->at(x, y)[1] = rgb_image.at(x, y)[1];
          color_image->at(x, y)[2] = rgb_image.at(x, y)[2];
          color_image->at(x, y)[3] = 0xff;
        }
      }
    } else {
      *color_image = ImageRgba8U();
    }
  }
  // TODO(sam.creasey) Handle BGR images, or at least error.
}

void LcmImageArrayToImages::CalcDepthImage(const Context<double>& context,
                                           ImageDepth32F* depth_image) const {
  const auto& images =
      image_array_t_input_port().Eval<lcmt_image_array>(context);

  // Look through the image array and just grab the first depth image.
  const lcmt_image* lcm_image = nullptr;
  for (int i = 0; i < images.num_images; i++) {
    if (images.images[i].pixel_format == lcmt_image::PIXEL_FORMAT_DEPTH) {
      lcm_image = &images.images[i];
      break;
    }
  }

  if (!lcm_image) {
    *depth_image = ImageDepth32F();
    return;
  }

  bool is_32f = false;
  switch (lcm_image->channel_type) {
    case lcmt_image::CHANNEL_TYPE_UINT16: {
      is_32f = false;
      break;
    }
    case lcmt_image::CHANNEL_TYPE_FLOAT32: {
      is_32f = true;
      break;
    }
    default: {
      drake::log()->error("Unsupported depth image channel type: {}",
                          lcm_image->channel_type);
      *depth_image = ImageDepth32F();
      return;
    }
  }

  if (is_32f) {
    UnpackLcmImage(lcm_image, depth_image);
  } else {
    ImageDepth16U image_16u;
    if (UnpackLcmImage(lcm_image, &image_16u)) {
      ConvertDepth16UTo32F(image_16u, depth_image);
    } else {
      *depth_image = ImageDepth32F();
    }
  }
}

void LcmImageArrayToImages::CalcLabelImage(const Context<double>& context,
                                           ImageLabel16I* label_image) const {
  const auto& images =
      image_array_t_input_port().Eval<lcmt_image_array>(context);

  // Look through the image array and just grab the first label image.
  const lcmt_image* lcm_image = nullptr;
  for (int i = 0; i < images.num_images; i++) {
    if (images.images[i].pixel_format == lcmt_image::PIXEL_FORMAT_LABEL) {
      lcm_image = &images.images[i];
      break;
    }
  }
  if (lcm_image == nullptr) {
    *label_image = {};
    return;
  }
  if (lcm_image->channel_type != lcmt_image::CHANNEL_TYPE_INT16) {
    drake::log()->error("Unsupported label image channel type: {}",
                        lcm_image->channel_type);
    *label_image = {};
    return;
  }

  UnpackLcmImage(lcm_image, label_image);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
