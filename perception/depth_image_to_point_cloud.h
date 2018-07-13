#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/perception/point_cloud.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/rgbd_camera.h"

namespace drake {
namespace perception {

/// Converts a depth image to a point cloud.
/// Basically a system that wraps around ConvertDepthImageToPointCloud in
/// RGBDCamera.
/// The system has a single input port that takes a ImageDepth32F and a single
/// output port that contains a PointCloud.
class DepthImageToPointCloud final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DepthImageToPointCloud)

  /// Constructs the converter.
  ///
  /// @param[in] camera_info The camera info.
  explicit DepthImageToPointCloud(
      const systems::sensors::CameraInfo& camera_info);

  /// Returns the abstract valued input port that contains an ImageDepth32F.
  const systems::InputPort<double>& depth_image_input_port() const {
    return this->get_input_port(input_port_depth_image_index_);
  }

  /// Returns the abstract valued output port that contains a PointCloud.
  const systems::OutputPort<double>& point_cloud_output_port() const {
    return LeafSystem<double>::get_output_port(0);
  }

 private:
  /// Returns an empty point cloud.
  PointCloud MakeOutputPointCloud() const;

  /// Converts the input depth image to a point cloud.
  void ConvertDepthImageToPointCloud(const systems::Context<double>& context,
                                     PointCloud* output) const;

  int input_port_depth_image_index_{-1};

  const systems::sensors::CameraInfo& camera_info_;
};

}  // namespace perception
}  // namespace drake
