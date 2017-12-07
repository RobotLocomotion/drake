#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

#include <memory>
#include <string>
#include <vector>

#include <zlib.h>
#include "robotlocomotion/image_array_t.hpp"
#include "robotlocomotion/image_t.hpp"

#include "drake/systems/sensors/image.h"

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
void PackImageToLcmImageT(const Image<kPixelType>& image, int64_t utime,
                          uint8_t pixel_format, uint8_t channel_type,
                          const string& frame_name, image_t* msg,
                          bool do_compress) {
  // TODO(kunimatsu-tri) Fix seq here that is always set to zero.
  msg->header.seq = 0;
  msg->header.utime = utime;
  msg->header.frame_name = frame_name;
  msg->width = image.width();
  msg->height = image.height();
  msg->row_stride = image.kPixelSize * msg->width;
  msg->bigendian = false;
  msg->pixel_format = pixel_format;
  msg->channel_type = channel_type;

  if (do_compress) {
    Compress(image, msg);
  } else {
    Pack(image, msg);
  }
}

}  // anonymous namespace

ImageToLcmImageArrayT::ImageToLcmImageArrayT(const string& color_frame_name,
                                             const string& depth_frame_name,
                                             const string& label_frame_name,
                                             bool do_compress)
    : color_frame_name_(color_frame_name),
      depth_frame_name_(depth_frame_name),
      label_frame_name_(label_frame_name),
      do_compress_(do_compress) {
  color_image_input_port_index_ =
      DeclareAbstractInputPort(systems::Value<ImageRgba8U>()).get_index();

  depth_image_input_port_index_ =
      DeclareAbstractInputPort(systems::Value<ImageDepth32F>()).get_index();

  label_image_input_port_index_ =
      DeclareAbstractInputPort(systems::Value<ImageLabel16I>()).get_index();

  image_array_t_msg_output_port_index_ =
      DeclareAbstractOutputPort(&ImageToLcmImageArrayT::CalcImageArray)
          .get_index();
}

const InputPortDescriptor<double>&
ImageToLcmImageArrayT::color_image_input_port() const {
  return this->get_input_port(color_image_input_port_index_);
}

const InputPortDescriptor<double>&
ImageToLcmImageArrayT::depth_image_input_port() const {
  return this->get_input_port(depth_image_input_port_index_);
}

const InputPortDescriptor<double>&
ImageToLcmImageArrayT::label_image_input_port() const {
  return this->get_input_port(label_image_input_port_index_);
}

const OutputPort<double>&
ImageToLcmImageArrayT::image_array_t_msg_output_port() const {
  return System<double>::get_output_port(image_array_t_msg_output_port_index_);
}

void ImageToLcmImageArrayT::CalcImageArray(
    const systems::Context<double>& context, image_array_t* msg) const {
  msg->header.utime = static_cast<int64_t>(context.get_time() * kSecToMillisec);
  msg->header.frame_name.clear();
  msg->num_images = 0;
  msg->images.clear();

  const AbstractValue* color_image_value =
      this->EvalAbstractInput(context, color_image_input_port_index_);

  const AbstractValue* depth_image_value =
      this->EvalAbstractInput(context, depth_image_input_port_index_);

  const AbstractValue* label_image_value =
      this->EvalAbstractInput(context, label_image_input_port_index_);

  if (color_image_value) {
    const ImageRgba8U& color_image =
        color_image_value->GetValue<ImageRgba8U>();

    image_t color_image_msg;
    PackImageToLcmImageT(color_image, msg->header.utime,
                         image_t::PIXEL_FORMAT_RGBA,
                         image_t::CHANNEL_TYPE_UINT8, color_frame_name_,
                         &color_image_msg,
                         do_compress_);
    msg->images.push_back(color_image_msg);
    msg->num_images++;
  }

  if (depth_image_value) {
    const ImageDepth32F& depth_image =
        depth_image_value->GetValue<ImageDepth32F>();
    image_t depth_image_msg;
    PackImageToLcmImageT(depth_image, msg->header.utime,
                         image_t::PIXEL_FORMAT_DEPTH,
                         image_t::CHANNEL_TYPE_FLOAT32, depth_frame_name_,
                         &depth_image_msg,
                         do_compress_);
    msg->images.push_back(depth_image_msg);
    msg->num_images++;
  }

  if (label_image_value) {
    const ImageLabel16I& label_image =
        label_image_value->GetValue<ImageLabel16I>();

    image_t label_image_msg;
    PackImageToLcmImageT(label_image, msg->header.utime,
                         image_t::PIXEL_FORMAT_LABEL,
                         image_t::CHANNEL_TYPE_INT16, label_frame_name_,
                         &label_image_msg,
                         do_compress_);
    msg->images.push_back(label_image_msg);
    msg->num_images++;
  }
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
