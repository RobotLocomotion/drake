#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"
#include "drake/perception/point_cloud.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace perception {

/// Transforms a point cloud.
///
/// Applies a rigid transform to all points in a point cloud. Each point is
/// pre-multiplied by the transform.
///
/// The system has two input ports and one output port. The first input port
/// takes a PointCloud and the second takes a RigidTransform<float>. The output
/// port contains a PointCloud. Notice that because the XYZ values in PointCloud
/// are stored as `float`, the RigidTransform should also be stored as `float`.
class TransformPointCloud final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TransformPointCloud)

  /// Constructs the transformer.
  TransformPointCloud();

  /// Returns the abstract valued input port that contains a PointCloud.
  const systems::InputPort<double>& point_cloud_input_port() const {
    return this->get_input_port(point_cloud_input_port_index_);
  }

  /// Returns the abstract valued input port that contains a
  /// RigidTransform<float>.
  const systems::InputPort<double>& rigid_transform_input_port() const {
    return this->get_input_port(rigid_transform_input_port_index_);
  }

  /// Returns the abstract valued output port that contains a PointCloud.
  const systems::OutputPort<double>& point_cloud_output_port() const {
    return LeafSystem<double>::get_output_port(0);
  }

 private:
  /// Returns an empty point cloud.
  PointCloud MakeOutputPointCloud() const;

  /// Transforms the point cloud by applying the given rigid transform to
  /// each point in the cloud.
  void ApplyTransformToPointCloud(const systems::Context<double>& context,
                                  PointCloud* output) const;

  systems::InputPortIndex point_cloud_input_port_index_;
  systems::InputPortIndex rigid_transform_input_port_index_;
};

}  // namespace perception
}  // namespace drake
