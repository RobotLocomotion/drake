#include "drake/perception/depth_image_to_point_cloud.h"

#include <limits>

#include "drake/common/drake_optional.h"
#include "drake/common/drake_throw.h"
#include "drake/common/never_destroyed.h"

using Eigen::Isometry3f;
using Eigen::Matrix3Xf;
using Eigen::Vector3f;
using drake::math::RigidTransformd;
using drake::systems::sensors::CameraInfo;
using drake::systems::sensors::PixelType;
using drake::systems::sensors::Image;
using drake::systems::sensors::ImageDepth16U;
using drake::systems::sensors::ImageDepth32F;
using drake::systems::sensors::ImageTraits;
using drake::systems::sensors::InvalidDepth;
using drake::systems::AbstractValue;
using drake::systems::Value;

namespace drake {
namespace perception {
namespace {

using pc_flags::kXYZs;

// Given a PixelType, return a Value<Image<PixelType>> dummy.
const AbstractValue& GetModelValue(PixelType pixel_type) {
  if (pixel_type == PixelType::kDepth32F) {
    static const never_destroyed<Value<ImageDepth32F>> image32f;
    return image32f.access();
  }
  if (pixel_type == PixelType::kDepth16U) {
    static const never_destroyed<Value<ImageDepth16U>> image16u;
    return image16u.access();
  }
  throw std::logic_error("Unsupported pixel_type in DepthImageToPointCloud");
}

// TODO(russt): Consider dropping NaN/kTooClose/kTooFar points from the point
// cloud output? (This would require adding support for colored point clouds,
// because current implementation assume that an RGB image will still line up).
template <PixelType pixel_type>
void DoConvert(
    const optional<pc_flags::BaseFieldT>& exact_base_fields,
    const CameraInfo& camera_info,
    const RigidTransformd* const camera_pose,
    const Image<pixel_type>& depth_image,
    const float scale,
    PointCloud* output) {
  if (exact_base_fields) {
    DRAKE_THROW_UNLESS(output->fields().base_fields() == *exact_base_fields);
  }

  // Reset the output size, if necessary.  We can leave the memory
  // uninitialized iff we are going to fill it in below.
  if (output->size() != depth_image.size()) {
    const bool skip_initialize = (output->fields().base_fields() == kXYZs);
    output->resize(depth_image.size(), skip_initialize);
  }
  Eigen::Ref<Matrix3Xf> output_xyz = output->mutable_xyzs();

  const int height = depth_image.height();
  const int width = depth_image.width();
  const float cx = camera_info.center_x();
  const float cy = camera_info.center_y();
  const float fx_inv = 1.f / camera_info.focal_x();
  const float fy_inv = 1.f / camera_info.focal_y();
  const Isometry3f X_PC = camera_pose ?
      camera_pose->GetAsIsometry3().cast<float>() :
      Isometry3f::Identity();

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const int col = v * width + u;
      const auto z = depth_image.at(u, v)[0];
      if ((z == ImageTraits<pixel_type>::kTooClose) ||
          (z == ImageTraits<pixel_type>::kTooFar)) {
        output_xyz.col(col).array() = std::numeric_limits<float>::infinity();
      } else {
        // N.B. This clause handles both true depths *and* NaNs.
        output_xyz.col(col) = X_PC * Vector3f(
            scale * z * (u - cx) * fx_inv,
            scale * z * (v - cy) * fy_inv,
            scale * z);
      }
    }
  }
}

}  // namespace

DepthImageToPointCloud::DepthImageToPointCloud(
    const CameraInfo& camera_info, PixelType pixel_type, float scale)
    : camera_info_(camera_info),
      pixel_type_(pixel_type),
      scale_(scale) {
  // Input port for depth image.
  depth_image_input_port_ = this->DeclareAbstractInputPort(
      "depth_image", GetModelValue(pixel_type)).get_index();

  // Optional input port for camera pose.
  camera_pose_input_port_ = this->DeclareAbstractInputPort(
      "camera_pose", Value<RigidTransformd>{}).get_index();

  // Output port for filtered point cloud.
  this->DeclareAbstractOutputPort(
      "point_cloud", PointCloud{},
      (pixel_type_ == PixelType::kDepth32F) ?
          &DepthImageToPointCloud::CalcOutput32F :
          &DepthImageToPointCloud::CalcOutput16U);
}

void DepthImageToPointCloud::Convert(
      const systems::sensors::CameraInfo& camera_info,
      const optional<math::RigidTransformd>& camera_pose,
      const systems::sensors::ImageDepth32F& depth_image,
      const optional<float>& scale,
      PointCloud* output) {
  DoConvert(nullopt, camera_info, camera_pose ? &*camera_pose : nullptr,
            depth_image, scale.value_or(1.0f), output);
}

void DepthImageToPointCloud::Convert(
      const systems::sensors::CameraInfo& camera_info,
      const optional<math::RigidTransformd>& camera_pose,
      const systems::sensors::ImageDepth16U& depth_image,
      const optional<float>& scale,
      PointCloud* output) {
  DoConvert(nullopt, camera_info, camera_pose ? &*camera_pose : nullptr,
            depth_image, scale.value_or(1.0f), output);
}

// In the Calc methods (i.e., when using the System framework), it would be
// fairly difficult and unusual to ever see a non-xyz cloud.  The user would
// have to go out of their way to cause that to happen (i.e., not use caching,
// do their own allocation, etc).  Since it's unlikely that doing that could
// ever turn out well, we reject the premise for now -- by passing kXYZs for
// the exact_base_fields below.  If the user wants to go to those lengths, they
// should just use the static methods instead.
void DepthImageToPointCloud::CalcOutput32F(
    const systems::Context<double>& context, PointCloud* output) const {
  const auto* const image = this->EvalInputValue<ImageDepth32F>(
      context, depth_image_input_port_);
  const auto* const pose_or_null = this->EvalInputValue<RigidTransformd>(
      context, camera_pose_input_port_);
  DRAKE_THROW_UNLESS(image != nullptr);
  DoConvert(kXYZs, camera_info_, pose_or_null, *image, scale_, output);
}

void DepthImageToPointCloud::CalcOutput16U(
    const systems::Context<double>& context, PointCloud* output) const {
  const auto* const image = this->EvalInputValue<ImageDepth16U>(
      context, depth_image_input_port_);
  const auto* const pose_or_null = this->EvalInputValue<RigidTransformd>(
      context, camera_pose_input_port_);
  DRAKE_THROW_UNLESS(image != nullptr);
  DoConvert(kXYZs, camera_info_, pose_or_null, *image, scale_, output);
}


}  // namespace perception
}  // namespace drake
