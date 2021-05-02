#include "drake/perception/point_cloud_to_lcm.h"

#include <cmath>
#include <string>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/lcmt_point_cloud.hpp"
#include "drake/perception/point_cloud.h"

namespace drake {
namespace perception {
namespace {

void Calc(double time, const PointCloud& cloud, lcmt_point_cloud* message) {
  DRAKE_THROW_UNLESS(cloud.has_xyzs());

  // A best practice for filling in LCM messages is to first value-initialize
  // the entire message to its defaults ("*message = {}") before setting any
  // new values.  That way, if we happen to skip over any fields, they will be
  // zeroed out instead of leaving behind garbage from whatever the memory
  // happened to contain beforehand.
  //
  // In our case though, point cloud data is typically high-bandwidth, so we'll
  // carefully work to reuse our message vectors' storage instead of clearing
  // it on every call.  (The header is small, though, so we'll still clear it.)
  message->header = {};
  message->header.utime = static_cast<int64_t>(time * 1e6);

  // Resize our message storage large enough to hold all points.
  const int num_points = cloud.size();
  const bool has_rgbs = cloud.has_rgbs();
  const int num_channels = has_rgbs ? 3 : 0;
  message->num_points = num_points;
  message->num_channels = num_channels;
  std::vector<std::vector<float>>& points = message->points;
  std::vector<std::vector<float>>& channels = message->channels;
  std::vector<std::string>& channel_names = message->channel_names;
  points.resize(3);
  points[0].resize(num_points);
  points[1].resize(num_points);
  points[2].resize(num_points);
  channels.resize(num_channels);
  channel_names.resize(num_channels);
  if (has_rgbs) {
    channels[0].resize(num_points);
    channels[1].resize(num_points);
    channels[2].resize(num_points);
    channel_names[0] = "r";
    channel_names[1] = "g";
    channel_names[2] = "b";
  }

  // Copy the cloud's data into the message.
  const Eigen::Ref<const Matrix3X<float>> xyzs = cloud.xyzs();
  const Matrix3X<uint8_t> no_rgbs;
  const Eigen::Ref<const Matrix3X<uint8_t>> rgbs =
      has_rgbs ? cloud.rgbs() : Eigen::Ref<const Matrix3X<uint8_t>>(no_rgbs);
  int count = 0;
  for (int i = 0; i < num_points; ++i) {
    const float x = xyzs(0, i);
    const float y = xyzs(1, i);
    const float z = xyzs(2, i);
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      points[0][count] = x;
      points[1][count] = y;
      points[2][count] = z;
      if (has_rgbs) {
        channels[0][count] = rgbs(0, i) / 255.0f;
        channels[1][count] = rgbs(1, i) / 255.0f;
        channels[2][count] = rgbs(2, i) / 255.0f;
      }
      ++count;
    }
  }

  // Shrink the message down to the actual number of valid points copied.
  message->num_points = count;
  points[0].resize(count);
  points[1].resize(count);
  points[2].resize(count);
  if (has_rgbs) {
    channels[0].resize(count);
    channels[1].resize(count);
    channels[2].resize(count);
  }
}

}  // anonymous namespace

PointCloudToLcm::PointCloudToLcm() {
  DeclareAbstractInputPort("point_cloud", Value<PointCloud>());
  DeclareAbstractOutputPort(
      "lcmt_point_cloud",
      []() { return AbstractValue::Make<lcmt_point_cloud>({}); },
      [this](const systems::Context<double>& context, AbstractValue* value) {
        auto& cloud = this->get_input_port().template Eval<PointCloud>(context);
        auto& message = value->get_mutable_value<lcmt_point_cloud>();
        Calc(context.get_time(), cloud, &message);
      });
}

PointCloudToLcm::~PointCloudToLcm() = default;

}  // namespace perception
}  // namespace drake
