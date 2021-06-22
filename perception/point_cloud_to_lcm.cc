#include "drake/perception/point_cloud_to_lcm.h"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/lcmt_point_cloud.hpp"
#include "drake/perception/point_cloud.h"

// TODO(jwnimmer-tri) Additional enhancements we could consider here:
//
// - user configuration of the (width, height); the PointCloud itself only
//   stores the total size, not the 2d structure; we could either add the
//   2d structure metadata into PoindCloud directly, or else we could add
//   it as members of this class, with a cross-check vs the cloud size.
//
// - user configuration of whether to discard non-finite points;
//
// - user configuration of which channels to encode / skip;
//
// - user configuration of the relative channel order or storage formats.

namespace drake {
namespace perception {
namespace {

// Populates `message` using the given time, frame_name, and point cloud data.
// (This is the implementation function for our system's output port.)
void Calc(double time, const std::string& frame_name, const PointCloud& cloud,
          lcmt_point_cloud* message) {
  // A best practice for filling in LCM messages is to first value-initialize
  // the entire message to its defaults ("*message = {}") before setting any
  // new values.  That way, if we accidentally skip over any fields, they will
  // be zeroed out instead of leaving behind garbage from whatever the memory
  // happened to contain beforehand.  In our case though, point cloud data is
  // typically high-bandwidth, so we'll carefully work to reuse our message's
  // storage instead of recreating it on every call.

  // Fill in the basic header info.
  message->utime = static_cast<int64_t>(time * 1e6);
  message->frame_name = frame_name;
  message->height = 1;
  message->flags = lcmt_point_cloud::IS_STRICTLY_FINITE;

  // Fill in the field metadata.
  // http://wiki.ros.org/pcl/Overview#Common_PointCloud2_field_names
  const bool has_xyzs = cloud.has_xyzs();
  const bool has_rgbs = cloud.has_rgbs();
  const bool has_normals = cloud.has_normals();
  const int num_fields =
      (has_xyzs ? 3 : 0) +
      (has_rgbs ? 1 : 0) +
      (has_normals ? 3 : 0);
  message->num_fields = num_fields;
  message->fields.resize(num_fields);
  {
    int current_field = 0;
    int current_offset = 0;
    if (has_xyzs) {
      for (int i = 0; i < 3; ++i) {
        auto& field = message->fields[current_field];
        switch (i) {
          case 0: { field.name = "x"; break; }
          case 1: { field.name = "y"; break; }
          case 2: { field.name = "z"; break; }
        }
        field.byte_offset = current_offset;
        field.datatype = lcmt_point_cloud_field::FLOAT32;
        field.count = 1;
        current_field += 1;
        current_offset += 4;
      }
    }
    if (has_rgbs) {
      auto& field = message->fields[current_field];
      field.name = "rgb";
      field.byte_offset = current_offset;
      field.datatype = lcmt_point_cloud_field::UINT32;
      field.count = 1;
      current_field += 1;
      current_offset += 4;
    }
    if (has_normals) {
      for (int i = 0; i < 3; ++i) {
        auto& field = message->fields[current_field];
        switch (i) {
          case 0: { field.name = "normal_x"; break; }
          case 1: { field.name = "normal_y"; break; }
          case 2: { field.name = "normal_z"; break; }
        }
        field.byte_offset = current_offset;
        field.datatype = lcmt_point_cloud_field::FLOAT32;
        field.count = 1;
        current_field += 1;
        current_offset += 4;
      }
    }
    DRAKE_DEMAND(current_field == num_fields);
    message->point_step = current_offset;
  }
  const int point_step = message->point_step;

  // Set the filler size so that the point data aligns.
  message->filler_size = 0;
  message->filler.clear();
  message->data_size = 0;
  message->data.clear();
  {
    const int kDesiredAlign = 16;
    const int current_align = message->getEncodedSize() % kDesiredAlign;
    const int filler_size = (kDesiredAlign - current_align) % kDesiredAlign;
    message->filler_size = filler_size;
    message->filler.resize(filler_size);
  }

  // Grab some shorthand names for the cloud's data.
  const Matrix3X<float> empty_float;
  const Matrix3X<uint8_t> empty_uint8;
  const Eigen::Ref<const Matrix3X<float>> xyzs =
      has_xyzs ? cloud.xyzs() :
      Eigen::Ref<const Matrix3X<float>>(empty_float);
  const Eigen::Ref<const Matrix3X<uint8_t>> rgbs =
      has_rgbs ? cloud.rgbs() :
      Eigen::Ref<const Matrix3X<uint8_t>>(empty_uint8);
  const Eigen::Ref<const Matrix3X<float>> normals =
      has_normals ? cloud.normals() :
      Eigen::Ref<const Matrix3X<float>>(empty_float);

  // Resize our message storage large enough to hold all points, assuming they
  // will all be finite.  If some were non-finite, we'll shrink it down later.
  const int num_points = cloud.size();
  message->data.resize(static_cast<int64_t>(num_points) * point_step);

  // Copy the cloud's data into the message.
  int64_t num_finite_points = 0;
  uint8_t* cursor = message->data.data();
  for (int i = 0; i < num_points; ++i) {
    if (has_xyzs) {
      const float x = xyzs(0, i);
      const float y = xyzs(1, i);
      const float z = xyzs(2, i);
      if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))) {
        continue;
      }
      std::memcpy(cursor, &x, 4); cursor += 4;
      std::memcpy(cursor, &y, 4); cursor += 4;
      std::memcpy(cursor, &z, 4); cursor += 4;
    }
    if (has_rgbs) {
      *cursor = rgbs(0, i); ++cursor;
      *cursor = rgbs(1, i); ++cursor;
      *cursor = rgbs(2, i); ++cursor;
      *cursor = 0;          ++cursor;  // padding
    }
    if (has_normals) {
      const float nx = normals(0, i);
      const float ny = normals(1, i);
      const float nz = normals(2, i);
      std::memcpy(cursor, &nx, 4); cursor += 4;
      std::memcpy(cursor, &ny, 4); cursor += 4;
      std::memcpy(cursor, &nz, 4); cursor += 4;
    }
    ++num_finite_points;
  }

  // Shrink the message down to the actual number of valid points copied.
  const std::ptrdiff_t data_size = cursor - message->data.data();
  DRAKE_DEMAND(data_size == (num_finite_points * point_step));
  message->data_size = data_size;
  message->data.resize(data_size);
  message->width = num_finite_points;
  message->row_step = data_size;
}

}  // anonymous namespace

PointCloudToLcm::PointCloudToLcm(std::string frame_name)
    : frame_name_(std::move(frame_name)) {
  DeclareAbstractInputPort("point_cloud", Value<PointCloud>());
  DeclareAbstractOutputPort(
      "lcmt_point_cloud",
      []() { return AbstractValue::Make<lcmt_point_cloud>(); },
      [this](const systems::Context<double>& context, AbstractValue* value) {
        auto& cloud = this->get_input_port().template Eval<PointCloud>(context);
        auto& message = value->get_mutable_value<lcmt_point_cloud>();
        Calc(context.get_time(), this->frame_name_, cloud, &message);
      });
}

PointCloudToLcm::~PointCloudToLcm() = default;

}  // namespace perception
}  // namespace drake
