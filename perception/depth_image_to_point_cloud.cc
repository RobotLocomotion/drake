#include "drake/perception/depth_image_to_point_cloud.h"

namespace drake {
namespace perception {

DepthImageToPointCloud::DepthImageToPointCloud(
    const systems::sensors::CameraInfo& camera_info)
    : camera_info_(camera_info) {
  // input port for depth image
  input_port_depth_image_index_ = this->DeclareAbstractInputPort().get_index();

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
  systems::sensors::RgbdCamera::ConvertDepthImageToPointCloud(
      input_image, camera_info_, &point_cloud);

  const int kNumCols = point_cloud.cols();
  output->resize(kNumCols);
  for (int i = 0; i < kNumCols; i++) {
    output->mutable_xyz(i) = point_cloud.col(i);
  }
}

}  // namespace perception
}  // namespace drake
