#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

#include <memory>
#include <string>
#include <vector>

#include <zlib.h>
#include "robotlocomotion/image_array_t.hpp"
#include "robotlocomotion/image_t.hpp"

#include "drake/systems/sensors/lcm_image_traits.h"

using std::string;
using robotlocomotion::image_t;
using robotlocomotion::image_array_t;

namespace drake {
namespace systems {
namespace sensors {
namespace {

const int64_t kSecToMillisec = 1000000;

template <PixelType kPixelType>
void Compress(const Image<kPixelType>& image, image_t* msg) {
  msg->compression_method = image_t::COMPRESSION_METHOD_ZLIB;

  const int source_size = image.width() * image.height() * image.kPixelSize;
  // The destination buf_size must be slightly larger than the source size.
  // http://refspecs.linuxbase.org/LSB_3.0.0/LSB-PDA/LSB-PDA/zlib-compress2-1.html
  size_t buf_size = source_size * 1.001 + 12;
  std::unique_ptr<uint8_t[]> buf(new uint8_t[buf_size]);

  auto compress_status = compress2(
      buf.get(), &buf_size, reinterpret_cast<const Bytef*>(image.at(0, 0)),
      source_size, Z_BEST_SPEED);

  DRAKE_DEMAND(compress_status == Z_OK);

  msg->data.resize(buf_size);
  msg->size = buf_size;
  memcpy(&msg->data[0], buf.get(), buf_size);
}

template <PixelType kPixelType>
void Pack(const Image<kPixelType>& image, image_t* msg) {
  msg->compression_method = image_t::COMPRESSION_METHOD_NOT_COMPRESSED;

  const int size = image.width() * image.height() * image.kPixelSize;
  msg->data.resize(size);
  msg->size = size;
  memcpy(&msg->data[0], image.at(0, 0), size);
}

template <PixelType kPixelType>
void PackImageToLcmImageT(const Image<kPixelType>& image, image_t* msg,
                          bool do_compress) {
  // TODO(kunimatsu-tri) Fix seq here that is always set to zero.
  msg->header.seq = 0;
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

void PackImageToLcmImageT(const AbstractValue& untyped_image,
                          PixelType pixel_type, int64_t utime,
                          const string& frame_name, image_t* msg,
                          bool do_compress) {
  msg->header.utime = utime;
  msg->header.frame_name = frame_name;

  switch (pixel_type) {
    case PixelType::kRgb8U: {
      const auto& image_value =
          untyped_image.GetValue<Image<PixelType::kRgb8U>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kBgr8U: {
      const auto& image_value =
          untyped_image.GetValue<Image<PixelType::kBgr8U>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kRgba8U: {
      const auto& image_value =
          untyped_image.GetValue<Image<PixelType::kRgba8U>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kBgra8U: {
      const auto& image_value =
          untyped_image.GetValue<Image<PixelType::kBgra8U>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kGrey8U: {
      const auto& image_value =
          untyped_image.GetValue<Image<PixelType::kGrey8U>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kDepth16U: {
      const auto& image_value =
          untyped_image.GetValue<Image<PixelType::kDepth16U>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kDepth32F: {
      const auto& image_value =
          untyped_image.GetValue<Image<PixelType::kDepth32F>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kLabel16I: {
      const auto& image_value =
          untyped_image.GetValue<Image<PixelType::kLabel16I>>();
      PackImageToLcmImageT(image_value, msg, do_compress);
      break;
    }
    case PixelType::kExpr:
      DRAKE_ABORT_MSG("PixelType::kExpr is not supported.");
  }
}

}  // anonymous namespace

ImageToLcmImageArrayT::ImageToLcmImageArrayT(bool do_compress)
    : do_compress_(do_compress) {
  image_array_t_msg_output_port_index_ =
      DeclareAbstractOutputPort(&ImageToLcmImageArrayT::CalcImageArray)
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

  image_array_t_msg_output_port_index_ =
      DeclareAbstractOutputPort(&ImageToLcmImageArrayT::CalcImageArray)
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
    const systems::Context<double>& context, image_array_t* msg) const {
  msg->header.utime = static_cast<int64_t>(context.get_time() * kSecToMillisec);
  msg->header.frame_name.clear();
  msg->num_images = 0;
  msg->images.clear();

  for (int i = 0; i < get_num_input_ports(); i++) {
    const AbstractValue* image_value = this->EvalAbstractInput(context, i);

    image_t image_msg;
    PackImageToLcmImageT(*image_value, input_port_pixel_type_[i],
                         msg->header.utime, this->get_input_port(i).get_name(),
                         &image_msg, do_compress_);
    msg->images.push_back(image_msg);
    msg->num_images++;
  }
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
