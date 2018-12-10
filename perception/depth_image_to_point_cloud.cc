#include "drake/perception/depth_image_to_point_cloud.h"

#include "drake/math/rigid_transform.h"

namespace drake {
namespace perception {

DepthImageToPointCloud::DepthImageToPointCloud(
    const systems::sensors::CameraInfo& camera_info)
    : camera_info_(camera_info) {
  // Input port for depth image.
  depth_image_input_port_ = this->DeclareAbstractInputPort(
      "depth_image",
      systems::Value<systems::sensors::ImageDepth32F>{}).get_index();

  // Optional input port for camera pose.
  camera_pose_input_port_ = this->DeclareAbstractInputPort("camera_pose",
      systems::Value<math::RigidTransformd>{}).get_index();

  // Output port for filtered point cloud.
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
      this->EvalAbstractInput(context, depth_image_input_port_);
  DRAKE_ASSERT(input_depth_image_ptr != nullptr);
  const auto& input_image =
      input_depth_image_ptr->GetValue<systems::sensors::ImageDepth32F>();

  Eigen::Matrix3Xf point_cloud;
  Convert(input_image, camera_info_, &point_cloud);

  const systems::AbstractValue* camera_pose_ptr = this->EvalAbstractInput
      (context, camera_pose_input_port_);
  // Setup camera pose in parent frame:
  Eigen::Isometry3f X_PC = Eigen::Isometry3f::Identity();
  if (camera_pose_ptr) {
    X_PC = camera_pose_ptr->GetValue<math::RigidTransformd>()
                          .GetAsIsometry3()
                          .cast<float>();
  }

  const int kNumCols = point_cloud.cols();
  output->resize(kNumCols);
  output->mutable_xyzs() = X_PC * point_cloud;

  // Special case for the non-finite points; just pass them through.
  // Note that currently x,y, and z are all non-finite if the depth value is
  // kTooClose or kTooFar.
  for (int i = 0; i < kNumCols; i++) {
    if (!std::isfinite(point_cloud(2, i))) {
      output->mutable_xyz(i) = point_cloud.col(i);
    }
  }
}

// Note that if `depth_image` holds any pixels that have NaN, the converted
// points will also become NaN.
// TODO(russt): Consider dropping NaN/kTooClose/kTooFar points from the point
// cloud output? (This would require adding support for colored point clouds,
// because current implementation assume that an RGB image will still line up).
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
