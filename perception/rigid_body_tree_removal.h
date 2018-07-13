#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/perception/point_cloud.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace perception {

/// A filtering algorithm for point clouds that removes known geometries.
/// Given a RigidBodyTree object, the algorithm takes a point cloud and a vector
/// of tree positions as input, and outputs a filtered point cloud from which
/// all points
/// expected to belong to the rigid bodies comprising the RigidBodyTree have
/// been removed.
class RigidBodyTreeRemoval final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyTreeRemoval)

  /// Constructs the filter given a RigidBodyTree.
  ///
  /// @param[in] tree The RigidBodyTree containing the geometric configuration
  /// of the world.
  explicit RigidBodyTreeRemoval(const RigidBodyTree<double>& tree);

  /// Returns the vector valued input port that contains a vector
  /// of `q, v` corresponding to the positions and velocities associated with
  /// a RigidBodyTree.
  const systems::InputPortDescriptor<double>& state_input_port() const {
    return this->get_input_port(state_input_port_index_);
  }

  /// Returns the abstract valued input port that contains a PointCloud.
  const systems::InputPortDescriptor<double>& point_cloud_input_port() const {
    return this->get_input_port(point_cloud_input_port_index_);
  }

  const systems::OutputPort<double>& point_cloud_output_port() const {
    return LeafSystem<double>::get_output_port(0);
  }

 private:
  PointCloud MakeOutputPointCloud() const;

  void FilterPointCloud(const systems::Context<double>& context,
                        PointCloud* output) const;

  const RigidBodyTree<double>& tree_;

  int point_cloud_input_port_index_{-1};
  int state_input_port_index_{-1};

  double collision_threshold_;
};

}  // namespace perception
}  // namespace drake
