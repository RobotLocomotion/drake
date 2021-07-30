#include "drake/systems/sensors/lcm_image_array_to_images.h"

#include <vector>

#include <png.h>
#include <vtkImageExport.h>
#include <vtkJPEGReader.h>
#include <vtkNew.h>
#include <zlib.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/lcmt_image_array.hpp"
#include "drake/systems/sensors/lcm_image_traits.h"

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

struct PngDecodeData {
  const unsigned char* data{nullptr};
  int size{0};
  int pos{0};
  std::string error_msg;
};

void CopyNextPngBytes(png_structp png_ptr, png_bytep data, size_t length) {
  DRAKE_DEMAND(png_ptr != nullptr);
  PngDecodeData* decode_data = reinterpret_cast<PngDecodeData*>(
      png_get_io_ptr(png_ptr));
  DRAKE_DEMAND(decode_data != nullptr);

  if (decode_data->pos + static_cast<int>(length) > decode_data->size) {
    png_error(png_ptr, "Attempt to read past end of PNG input buffer");
  }

  memcpy(data, decode_data->data + decode_data->pos, length);
  decode_data->pos += length;
}

template <PixelType kPixelType>
bool DecompressPng(const lcmt_image* lcm_image, Image<kPixelType>* image) {
  PngDecodeData decode_data;
  decode_data.data = lcm_image->data.data();
  decode_data.size = lcm_image->size;

  png_structp png_ptr = png_create_read_struct(
      PNG_LIBPNG_VER_STRING, 0, 0, 0);
  DRAKE_DEMAND(png_ptr != nullptr);

  png_infop info_ptr = png_create_info_struct(png_ptr);
  DRAKE_DEMAND(info_ptr != nullptr);

  if (setjmp(png_jmpbuf(png_ptr)) == 0) {
    png_set_read_fn(png_ptr, &decode_data, CopyNextPngBytes);
    png_read_info(png_ptr, info_ptr);
    if ((static_cast<int>(png_get_image_width(png_ptr, info_ptr)) !=
         lcm_image->width) ||
        (static_cast<int>(png_get_image_height(png_ptr, info_ptr)) !=
         lcm_image->height)) {
      drake::log()->error("Decoded PNG parameter mismatch");
      goto out;
    }

    if (png_get_bit_depth(png_ptr, info_ptr) !=
        sizeof(typename ImageTraits<kPixelType>::ChannelType) * 8) {
      drake::log()->error("Decoded PNG bit depth mismatch");
      goto out;
    }

    if (png_get_channels(png_ptr, info_ptr) != image->kNumChannels) {
      drake::log()->error("Decoded PNG bit channel mismatch");
      goto out;
    }

    if (png_get_color_type(png_ptr, info_ptr) == PNG_COLOR_TYPE_PALETTE) {
      png_set_palette_to_rgb(png_ptr);
    }

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    png_set_swap(png_ptr);
#endif

    png_read_update_info(png_ptr, info_ptr);

    std::vector<png_bytep> row_pointers(lcm_image->height);
    for (int i = 0; i < lcm_image->height; ++i) {
      row_pointers[i] = reinterpret_cast<png_bytep>(image->at(0, i));
    }

    png_read_image(png_ptr, row_pointers.data());
    png_destroy_read_struct(&png_ptr, &info_ptr, 0);
    return true;

  } else {
    drake::log()->error("Error during PNG decoding");
  }

 out:
  png_destroy_read_struct(&png_ptr, &info_ptr, 0);
  return false;
}

template <PixelType kPixelType>
bool DecompressJpeg(const lcmt_image* lcm_image, Image<kPixelType>* image) {
  vtkNew<vtkJPEGReader> jpg_reader;
  jpg_reader->SetMemoryBuffer(
      const_cast<unsigned char*>(lcm_image->data.data()));
  jpg_reader->SetMemoryBufferLength(lcm_image->size);
  jpg_reader->Update();

  vtkNew<vtkImageExport> exporter;
  exporter->SetInputConnection(jpg_reader->GetOutputPort(0));
  exporter->ImageLowerLeftOff();
  exporter->Update();

  if (exporter->GetDataMemorySize() !=
      image->width() * image->height() * image->kPixelSize) {
    drake::log()->error("Malformed output decoding incoming LCM JPEG image");
    return false;
  }
  exporter->Export(image->at(0, 0));
  return true;
}

template <PixelType kPixelType>
bool DecompressZlib(const lcmt_image* lcm_image, Image<kPixelType>* image) {
  // NOLINTNEXTLINE(runtime/int)
  unsigned long dest_len =
      image->width() * image->height() * image->kPixelSize;
  const int status = uncompress(
      reinterpret_cast<Bytef*>(image->at(0, 0)), &dest_len,
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
      return DecompressJpeg(lcm_image, image);
    }
    case lcmt_image::COMPRESSION_METHOD_PNG: {
      return DecompressPng(lcm_image, image);
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
          this->DeclareAbstractInputPort(
              "image_array_t", Value<lcmt_image_array>()).get_index()),
      color_image_output_port_index_(
          this->DeclareAbstractOutputPort(
              "color_image", &LcmImageArrayToImages::CalcColorImage)
          .get_index()),
      depth_image_output_port_index_(
          this->DeclareAbstractOutputPort(
              "depth_image", &LcmImageArrayToImages::CalcDepthImage)
          .get_index()) {
  // TODO(sammy-tri) Calculating our output ports can be kinda expensive.  We
  // should cache the images.
}

void LcmImageArrayToImages::CalcColorImage(
    const Context<double>& context, ImageRgba8U* color_image) const {
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

void LcmImageArrayToImages::CalcDepthImage(
    const Context<double>& context, ImageDepth32F* depth_image) const {
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
      depth_image->resize(lcm_image->width, lcm_image->height);
      for (int x = 0; x < lcm_image->width; x++) {
        for (int y = 0; y < lcm_image->height; y++) {
          depth_image->at(x, y)[0] =
              static_cast<float>(image_16u.at(x, y)[0]) / 1e3;
        }
      }
    } else {
      *depth_image = ImageDepth32F();
    }
  }
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
