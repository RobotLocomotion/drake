#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/perception/point_cloud.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace perception {

/// Removes known geometries from point clouds.
///
/// Given a RigidBodyTree, the system takes a point cloud and the state of the
/// RigidBodyTree as input, and produces a filtered point cloud as output from
/// which all points expected to belong to the rigid bodies comprising the
/// RigidBodyTree have been removed.
///
/// The system has two input ports and one output port. The first input port
/// consumes a PointCloud and the second takes the state of the RigidBodyTree.
/// The output port contains the filtered PointCloud.
class RigidBodyPointCloudFilter final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyPointCloudFilter)

  /// Constructs the filter given a RigidBodyTree.
  ///
  /// @param[in] tree The RigidBodyTree containing the geometric configuration
  /// of the world. Notice that calculating the filter's output updates the
  /// `tree`'s collision model.
  /// @param[in] collision_threshold The threshold for the collision
  /// detection that determines which points to remove from the point cloud.
  ///
  /// The `tree` object must remain valid for the duration of this object.
  RigidBodyPointCloudFilter(RigidBodyTree<double>* tree,
                            double collision_threshold);

  /// Returns the vector valued input port that contains a vector of `q, v`
  /// corresponding to the positions and velocities associated with a
  /// RigidBodyTree.
  const systems::InputPort<double>& state_input_port() const {
    return this->get_input_port(state_input_port_index_);
  }

  /// Returns the abstract valued input port that contains a PointCloud.
  const systems::InputPort<double>& point_cloud_input_port() const {
    return this->get_input_port(point_cloud_input_port_index_);
  }

  /// Returns the abstract valued output port that contains a PointCloud.
  const systems::OutputPort<double>& point_cloud_output_port() const {
    return LeafSystem<double>::get_output_port(0);
  }

 private:
  // Returns an empty point cloud.
  PointCloud MakeOutputPointCloud() const;

  // Filters the point cloud by removing those geometries that are known
  // from the RigidBodyTree `tree_`.
  void FilterPointCloud(const systems::Context<double>& context,
                        PointCloud* output) const;

  RigidBodyTree<double>* tree_;

  int point_cloud_input_port_index_{-1};
  int state_input_port_index_{-1};

  double collision_threshold_{};
};

}  // namespace perception
}  // namespace drake
