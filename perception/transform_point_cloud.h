#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/perception/point_cloud.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace perception {

/// Transforms a point cloud to the world frame.
///
/// The system first computes a RigidTransform between a RigidBodyFrame and
/// the world frame based on the state of a RigidBodyTree. Then it applies
/// this transform to a point cloud.
///
/// The system has two input ports and one output port. The first input port
/// takes a PointCloud and the second takes the state of a RigidBodyTree. The
/// output port contains a PointCloud.
class TransformPointCloud final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TransformPointCloud)

  /// Constructs a transformer that transforms from `child_frame` to
  /// `parent_frame`.
  TransformPointCloud(const RigidBodyTree<double>& tree, int parent_frame_index,
                      int child_frame_index);

  /// Constructs a transformer that transforms from `child_frame` to world
  /// frame.
  TransformPointCloud(const RigidBodyTree<double>& tree, int child_frame_index);

  /// Returns the abstract valued input port that contains a PointCloud.
  const systems::InputPort<double>& point_cloud_input_port() const {
    return this->get_input_port(point_cloud_input_port_index_);
  }

  /// Returns the vector valued input port that contains the state associated
  /// with a RigidBodyTree.
  const systems::InputPort<double>& state_input_port() const {
    return this->get_input_port(state_input_port_index_);
  }

  /// Returns the abstract valued output port that contains a PointCloud.
  const systems::OutputPort<double>& point_cloud_output_port() const {
    return LeafSystem<double>::get_output_port(0);
  }

 private:
  /// Returns an empty point cloud.
  PointCloud MakeOutputPointCloud() const;

  /// Transforms the point cloud using a RigidTransform between `frame_` and
  /// the world frame that is calculated based on the state of `tree_`.
  void ApplyTransformToPointCloud(const systems::Context<double>& context,
                                  PointCloud* output) const;

  /// Creates input and output ports for this system.
  void CreatePorts();

  const RigidBodyTree<double>& tree_;
  int parent_frame_index_;
  int child_frame_index_;

  systems::InputPortIndex point_cloud_input_port_index_;
  systems::InputPortIndex state_input_port_index_;
};

}  // namespace perception
}  // namespace drake
