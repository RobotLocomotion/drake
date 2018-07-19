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
/// Left mutiplies all points C in the cloud by a given transform X. The
/// result is D = X * C.
///
/// The system has two input ports and one output port. The first input port
/// takes a PointCloud and the second takes a RigidTransform. The output port
/// contains a PointCloud.
class TransformPointCloud final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TransformPointCloud)

  /// Constructs the transformer.
  TransformPointCloud();

  /// Returns the abstract valued input port that contains a PointCloud.
  const systems::InputPort<double>& point_cloud_input_port() const {
    return this->get_input_port(point_cloud_input_port_index_);
  }

  /// Returns the abstract valued input port that contains a RigidTransform.
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

  /// Transforms the point cloud by left multiplying with the given rigid
  /// transform..
  void ApplyTransformToPointCloud(const systems::Context<double>& context,
                                  PointCloud* output) const;

  int point_cloud_input_port_index_{-1};
  int rigid_transform_input_port_index_{-1};
};

}  // namespace perception
}  // namespace drake
