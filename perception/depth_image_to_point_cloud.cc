#include "drake/perception/depth_image_to_point_cloud.h"

namespace drake {
namespace perception {

DepthImageToPointCloud::DepthImageToPointCloud(
    const systems::sensors::CameraInfo& camera_info)
    : camera_info_(camera_info) {
  // input port for depth image
  input_port_depth_image_index_ = this->DeclareAbstractInputPort(
      systems::kUseDefaultName,
      systems::Value<systems::sensors::ImageDepth32F>{}).get_index();

  /// output port for filtered point cloud
  this->DeclareAbstractOutputPort(
      &DepthImageToPointCloud::MakeOutputPointCloud,
      &DepthImageToPointCloud::ConvertDepthImageToPointCloud);
}

PointCloud DepthImageToPointCloud::MakeOutputPointCloud() const {
  PointCloud cloud(0);
  return cloud;
}

void DepthImageToPointCloud::ConvertDepthImageToPointCloud(
    const systems::Context<double>& context, PointCloud* output) const {
  const systems::AbstractValue* input_depth_image_ptr =
      this->EvalAbstractInput(context, input_port_depth_image_index_);
  DRAKE_ASSERT(input_depth_image_ptr != nullptr);
  const auto& input_image =
      input_depth_image_ptr->GetValue<systems::sensors::ImageDepth32F>();

  Eigen::Matrix3Xf point_cloud;
  Convert(input_image, camera_info_, &point_cloud);

  const int kNumCols = point_cloud.cols();
  output->resize(kNumCols);
  for (int i = 0; i < kNumCols; i++) {
    output->mutable_xyz(i) = point_cloud.col(i);
  }
}

// Note that if `depth_image` holds any pixels that have NaN, the converted
// points will also become NaN.
void DepthImageToPointCloud::Convert(
    const systems::sensors::ImageDepth32F& depth_image,
    const systems::sensors::CameraInfo& camera_info,
    Eigen::Matrix3Xf* point_cloud) {
  DRAKE_DEMAND(point_cloud != nullptr);
  using InvalidDepth = systems::sensors::InvalidDepth;

  if (depth_image.size() != point_cloud->cols()) {
    point_cloud->resize(3, depth_image.size());
  }

  const int height = depth_image.height();
  const int width = depth_image.width();
  const float cx = camera_info.center_x();
  const float cy = camera_info.center_y();
  const float fx_inv = 1.f / camera_info.focal_x();
  const float fy_inv = 1.f / camera_info.focal_y();

  Eigen::Matrix3Xf& pc = *point_cloud;
  pc = Eigen::Matrix3Xf::Constant(3, height * width, InvalidDepth::kTooFar);

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const float z = depth_image.at(u, v)[0];
      if (z != InvalidDepth::kTooClose &&
          z != InvalidDepth::kTooFar) {
        const int col = v * width + u;
        pc(0, col) = z * (u - cx) * fx_inv;
        pc(1, col) = z * (v - cy) * fy_inv;
        pc(2, col) = z;
      }
    }
  }
}

}  // namespace perception
}  // namespace drake
