#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

#include <stdexcept>

#include <zlib.h>

#include "drake/lcmt_image.hpp"
#include "drake/lcmt_image_array.hpp"
#include "drake/systems/sensors/lcm_image_traits.h"

using std::string;

namespace drake {
namespace systems {
namespace sensors {
namespace {

const int64_t kSecToMillisec = 1000000;

// Overwrites the msg's compression_method, size, and data.
template <PixelType kPixelType>
void Compress(const Image<kPixelType>& image, lcmt_image* msg) {
  msg->compression_method = lcmt_image::COMPRESSION_METHOD_ZLIB;

  const int source_size = image.width() * image.height() * image.kPixelSize;
  // The destination buf_size must be slightly larger than the source size.
  // http://refspecs.linuxbase.org/LSB_3.0.0/LSB-PDA/LSB-PDA/zlib-compress2-1.html
  std::vector<uint8_t>& dest = msg->data;
  size_t dest_size = source_size * 1.001 + 12;
  dest.resize(dest_size);

  auto compress_status = compress2(
      dest.data(), &dest_size,
      reinterpret_cast<const Bytef*>(image.at(0, 0)),
      source_size, Z_BEST_SPEED);
  DRAKE_DEMAND(compress_status == Z_OK);

  DRAKE_DEMAND(dest_size <= dest.size());
  dest.resize(dest_size);
  msg->size = dest_size;
}

// Overwrites the msg's compression_method, size, and data.
template <PixelType kPixelType>
void Pack(const Image<kPixelType>& image, lcmt_image* msg) {
  msg->compression_method = lcmt_image::COMPRESSION_METHOD_NOT_COMPRESSED;

  const int size = image.width() * image.height() * image.kPixelSize;
  msg->data.resize(size);
  msg->size = size;
  memcpy(&msg->data[0], image.at(0, 0), size);
}

// Overwrites everything in msg except its header.
template <PixelType kPixelType>
void PackImageToLcmImageT(const Image<kPixelType>& image, lcmt_image* msg,
                          bool do_compress) {
  msg->width = image.width();
  msg->height = image.height();
  msg->row_stride = image.kPixelSize * msg->width;
  msg->bigendian = false;
  msg->pixel_format =
      LcmPixelTraits<ImageTraits<kPixelType>::kPixelFormat>::kPixelFormat;
  msg->channel_type = LcmImageTraits<kPixelType>::kChannelType;

  if (do_compress) {
    Compress(image, msg);
  } else {
    Pack(image, msg);
  }
}

// Overwrites everything in msg except its header.
void PackImageToLcmImageT(const AbstractValue& untyped_image,
                          PixelType pixel_type, lcmt_image* msg,
                          bool do_compress) {
  switch (pixel_type) {
    case PixelType::kRgb8U: {
      const auto& image_value =
          untyped_image.get_value<Image<PixelType::kRgb8U>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kBgr8U: {
      const auto& image_value =
          untyped_image.get_value<Image<PixelType::kBgr8U>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kRgba8U: {
      const auto& image_value =
          untyped_image.get_value<Image<PixelType::kRgba8U>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kBgra8U: {
      const auto& image_value =
          untyped_image.get_value<Image<PixelType::kBgra8U>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kGrey8U: {
      const auto& image_value =
          untyped_image.get_value<Image<PixelType::kGrey8U>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kDepth16U: {
      const auto& image_value =
          untyped_image.get_value<Image<PixelType::kDepth16U>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kDepth32F: {
      const auto& image_value =
          untyped_image.get_value<Image<PixelType::kDepth32F>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kLabel16I: {
      const auto& image_value =
          untyped_image.get_value<Image<PixelType::kLabel16I>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kExpr:
      throw std::domain_error("PixelType::kExpr is not supported.");
  }
}

}  // anonymous namespace

ImageToLcmImageArrayT::ImageToLcmImageArrayT(bool do_compress)
    : do_compress_(do_compress) {
  image_array_t_msg_output_port_index_ = DeclareAbstractOutputPort(
      kUseDefaultName, &ImageToLcmImageArrayT::CalcImageArray)
          .get_index();
}

ImageToLcmImageArrayT::ImageToLcmImageArrayT(const string& color_frame_name,
                                             const string& depth_frame_name,
                                             const string& label_frame_name,
                                             bool do_compress)
    : do_compress_(do_compress) {
  color_image_input_port_index_ =
      DeclareImageInputPort<PixelType::kRgba8U>(color_frame_name).get_index();
  depth_image_input_port_index_ =
      DeclareImageInputPort<PixelType::kDepth32F>(depth_frame_name).get_index();
  label_image_input_port_index_ =
      DeclareImageInputPort<PixelType::kLabel16I>(label_frame_name).get_index();

  image_array_t_msg_output_port_index_ = DeclareAbstractOutputPort(
      kUseDefaultName, &ImageToLcmImageArrayT::CalcImageArray)
          .get_index();
}

const InputPort<double>& ImageToLcmImageArrayT::color_image_input_port() const {
  DRAKE_DEMAND(color_image_input_port_index_ >= 0);
  return this->get_input_port(color_image_input_port_index_);
}

const InputPort<double>& ImageToLcmImageArrayT::depth_image_input_port() const {
  DRAKE_DEMAND(depth_image_input_port_index_ >= 0);
  return this->get_input_port(depth_image_input_port_index_);
}

const InputPort<double>& ImageToLcmImageArrayT::label_image_input_port() const {
  DRAKE_DEMAND(label_image_input_port_index_ >= 0);
  return this->get_input_port(label_image_input_port_index_);
}

const OutputPort<double>& ImageToLcmImageArrayT::image_array_t_msg_output_port()
    const {
  return System<double>::get_output_port(image_array_t_msg_output_port_index_);
}

void ImageToLcmImageArrayT::CalcImageArray(
    const systems::Context<double>& context, lcmt_image_array* msg) const {
  // A best practice for filling in LCM messages is to first value-initialize
  // the entire message to its defaults ("*msg = {}") before setting any new
  // values.  That way, if we happen to skip over any fields, they will be
  // zeroed out instead of leaving behind garbage from whatever the msg memory
  // happened to contain beforehand.
  //
  // In our case though, image data is typically high-bandwidth, so we will
  // carefully work to reuse our message vectors' storage instead of clearing
  // it on every call.  (The headers are small, though, so we'll still clear
  // them.)
  const int64_t utime = static_cast<int64_t>(
      context.get_time() * kSecToMillisec);
  msg->header = {};
  msg->header.utime = utime;
  const int num_inputs = num_input_ports();
  msg->num_images = num_inputs;
  msg->images.resize(num_inputs);
  for (int i = 0; i < num_inputs; i++) {
    const std::string& name = this->get_input_port(i).get_name();
    const PixelType& type = input_port_pixel_type_[i];
    const auto& value = this->get_input_port(i).
        template Eval<AbstractValue>(context);
    lcmt_image& packed = msg->images.at(i);
    packed.header = {};
    packed.header.utime = utime;
    packed.header.frame_name = name;
    PackImageToLcmImageT(value, type, &packed, do_compress_);
  }
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
